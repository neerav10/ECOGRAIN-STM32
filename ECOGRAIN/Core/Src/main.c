


/*
NEERAV DESAI

		
   - DHT11 (PA7)
   - LM35 (PA0 ADC1_CH0)
   - MQ135 (PA2 ADC1_CH2)
   - OLED SSD1306 (I2C PB8/PB9)
   - OLED layout (full clear each loop):
       Page 0 -> Temp(LM35): xx.x C
       Page 1 -> Air(MQ135): xxx PPM
       Page 3 -> Temp(DHT):  xx C
       Page 4 -> Humidity :  xx %
*/

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdio.h>

// ==================== PWM / FUZZY GLOBALS ====================
#define PWM_PSC     83      // 1 MHz
#define PWM_ARR     999     // 1 kHz

// smooth ramp step per loop (in % duty)
#define PWM_STEP    5.0f

// current duty (in %)
static float g_vib_duty_cur  = 0.0f;   // PB0 (TIM3_CH3)
static float g_fan_duty_cur  = 0.0f;   // PB1 (TIM3_CH4)

// target duty (in %), updated by fuzzy logic
static float g_vib_duty_tgt  = 0.0f;
static float g_fan_duty_tgt  = 0.0f;



/* ================= ADC Channels ================= */
#define LM35_PIN   2  // PA0 → ADC1_CH2
#define MQ135_PIN  0  // PA2 → ADC1_CH0

/* DHT11 */
#define DHT11_PORT GPIOA
#define DHT11_PIN  7       // PA7

/* OLED I2C */
#define SSD1306_ADDR    (0x3C << 1)  // 8-bit address
#define I2C_SCL_PIN     8            // PB8
#define I2C_SDA_PIN     9            // PB9

/* ====================== ADC ======================== */
void ADC1_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Analog mode for PA0, PA2
    GPIOA->MODER |= (3U << (LM35_PIN*2)) | (3U << (MQ135_PIN*2));
    GPIOA->PUPDR &= ~((3U << (LM35_PIN*2)) | (3U << (MQ135_PIN*2)));

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    // Right alignment + Single conversion + EOC after each conv
    ADC1->CR2 |= ADC_CR2_EOCS;

    // Sequence length = 1 conversion
    ADC1->SQR1 = 0;

    // Max sampling time for both channels
    ADC1->SMPR2 |= (7U << (LM35_PIN*3)) | (7U << (MQ135_PIN*3));

    // Enable ADC
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t ADC1_Read_Channel(uint8_t channel)
{
    ADC1->SQR3 = channel;

    // Settling delay for mux
    for (volatile int i = 0; i < 2000; i++);

    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait for conversion complete
    while (!(ADC1->SR & ADC_SR_EOC));

    return ADC1->DR; // reading clears EOC
}



float Read_Temperature(void) {
    uint16_t raw = ADC1_Read_Channel(LM35_PIN);
    float voltage = (raw * 3.3f) / 4095.0f;
    return voltage * 100.0f; // LM35: 10mV = 1°C
}

float Read_MQ135_PPM(void) {
    uint16_t raw = ADC1_Read_Channel(MQ135_PIN);
    float Vout = (raw * 4.5f) / 4095.0f;
    float PPM_scale = 1000.0f;
    return (Vout / 4.5f) * PPM_scale;
}

/* ====================== DHT11 ======================== */
#define DHT11_INPUT_MODE()   (DHT11_PORT->MODER &= ~(3U << (DHT11_PIN * 2)))
#define DHT11_OUTPUT_MODE()  (DHT11_PORT->MODER = (DHT11_PORT->MODER & ~(3U << (DHT11_PIN * 2))) | (1U << (DHT11_PIN * 2)))
#define DHT11_PIN_HIGH()     (DHT11_PORT->BSRR = (1U << DHT11_PIN))
#define DHT11_PIN_LOW()      (DHT11_PORT->BSRR = (1U << (DHT11_PIN + 16)))
#define DHT11_READ_PIN()     ((DHT11_PORT->IDR >> DHT11_PIN) & 1U)

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

void DHT11_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    DHT11_OUTPUT_MODE();
    DHT11_PIN_HIGH();

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint8_t DHT11_ReadByte(void) {
    uint8_t i, byte = 0;
    for (i = 0; i < 8; i++) {
        while (!DHT11_READ_PIN());
        delay_us(35);
        if (DHT11_READ_PIN()) byte |= (1 << (7 - i));
        while (DHT11_READ_PIN());
    }
    return byte;
}

uint8_t DHT11_ReadData(uint8_t *humidity, uint8_t *temperature) {
    uint8_t h_int, h_dec, t_int, t_dec, checksum;
    uint32_t timeout = 0;

    DHT11_OUTPUT_MODE();
    DHT11_PIN_LOW();
    HAL_Delay(20);
    DHT11_PIN_HIGH();
    delay_us(50);
    DHT11_INPUT_MODE();

    timeout = 0;
    while (DHT11_READ_PIN()) if (++timeout > 20000) return 1;
    timeout = 0;
    while (!DHT11_READ_PIN()) if (++timeout > 20000) return 1;
    timeout = 0;
    while (DHT11_READ_PIN()) if (++timeout > 20000) return 1;

    h_int = DHT11_ReadByte();
    h_dec = DHT11_ReadByte();
    t_int = DHT11_ReadByte();
    t_dec = DHT11_ReadByte();
    checksum = DHT11_ReadByte();

    if (((uint8_t)(h_int + h_dec + t_int + t_dec)) != checksum)
        return 2;

    *humidity = h_int;
    *temperature = t_int;
    return 0;
}

/* ====================== I2C (CMSIS) ================= */
void I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~((3<<16)|(3<<18));
    GPIOB->MODER |= (2<<16)|(2<<18);
    GPIOB->OTYPER |= (1<<8)|(1<<9);
    GPIOB->PUPDR |= (1<<16)|(1<<18);
    GPIOB->AFR[1] |= (4<<0)|(4<<4);

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    I2C1->CR2 = 42;
    I2C1->CCR = 210;
    I2C1->TRISE = 43;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_Start(void) { I2C1->CR1 |= I2C_CR1_START; while (!(I2C1->SR1 & I2C_SR1_SB)); }
void I2C1_Stop(void)  { I2C1->CR1 |= I2C_CR1_STOP; }
void I2C1_SendAddress(uint8_t addr) { I2C1->DR = addr; while (!(I2C1->SR1 & I2C_SR1_ADDR)); (void)I2C1->SR2; }
void I2C1_SendByte(uint8_t data) { while (!(I2C1->SR1 & I2C_SR1_TXE)); I2C1->DR = data; while (!(I2C1->SR1 & I2C_SR1_BTF)); }

/* ====================== OLED ====================== */
static const uint8_t font6x8[96][6] = {
    /* KEEP YOUR FULL FONT ARRAY EXACTLY AS BEFORE */
		{0x00,0x00,0x00,0x00,0x00,0x00}, // space
		{0x00,0x00,0x5F,0x00,0x00,0x00}, // !
		{0x00,0x07,0x00,0x07,0x00,0x00}, // "
		{0x14,0x7F,0x14,0x7F,0x14,0x00}, // #
		{0x24,0x2A,0x7F,0x2A,0x12,0x00}, // $
		{0x23,0x13,0x08,0x64,0x62,0x00}, // %
		{0x36,0x49,0x55,0x22,0x50,0x00}, // &
		{0x00,0x05,0x03,0x00,0x00,0x00}, // '
		{0x00,0x1C,0x22,0x41,0x00,0x00}, // (
		{0x00,0x41,0x22,0x1C,0x00,0x00}, // )
		{0x14,0x08,0x3E,0x08,0x14,0x00}, // *
		{0x08,0x08,0x3E,0x08,0x08,0x00}, // +
		{0x00,0x50,0x30,0x00,0x00,0x00}, // ,
		{0x08,0x08,0x08,0x08,0x08,0x00}, // -
		{0x00,0x60,0x60,0x00,0x00,0x00}, // .
		{0x20,0x10,0x08,0x04,0x02,0x00}, // /
		{0x3E,0x51,0x49,0x45,0x3E,0x00}, // 0
		{0x00,0x42,0x7F,0x40,0x00,0x00}, // 1
		{0x42,0x61,0x51,0x49,0x46,0x00}, // 2
		{0x21,0x41,0x45,0x4B,0x31,0x00}, // 3
		{0x18,0x14,0x12,0x7F,0x10,0x00}, // 4
		{0x27,0x45,0x45,0x45,0x39,0x00}, // 5
		{0x3C,0x4A,0x49,0x49,0x30,0x00}, // 6
		{0x01,0x71,0x09,0x05,0x03,0x00}, // 7
		{0x36,0x49,0x49,0x49,0x36,0x00}, // 8
		{0x06,0x49,0x49,0x29,0x1E,0x00}, // 9
		{0x00,0x36,0x36,0x00,0x00,0x00}, // :
		{0x00,0x56,0x36,0x00,0x00,0x00}, // ;
		{0x08,0x14,0x22,0x41,0x00,0x00}, // <
		{0x14,0x14,0x14,0x14,0x14,0x00}, // =
		{0x00,0x41,0x22,0x14,0x08,0x00}, // >
		{0x02,0x01,0x51,0x09,0x06,0x00}, // ?
		{0x32,0x49,0x79,0x41,0x3E,0x00}, // @
		{0x7E,0x11,0x11,0x11,0x7E,0x00}, // A
		{0x7F,0x49,0x49,0x49,0x36,0x00}, // B
		{0x3E,0x41,0x41,0x41,0x22,0x00}, // C
		{0x7F,0x41,0x41,0x22,0x1C,0x00}, // D
		{0x7F,0x49,0x49,0x49,0x41,0x00}, // E
		{0x7F,0x09,0x09,0x09,0x01,0x00}, // F
		{0x3E,0x41,0x49,0x49,0x7A,0x00}, // G
		{0x7F,0x08,0x08,0x08,0x7F,0x00}, // H
		{0x41,0x7F,0x41,0x00,0x00,0x00}, // I
		{0x20,0x40,0x41,0x3F,0x01,0x00}, // J
		{0x7F,0x08,0x14,0x22,0x41,0x00}, // K
		{0x7F,0x40,0x40,0x40,0x40,0x00}, // L
		{0x7F,0x02,0x0C,0x02,0x7F,0x00}, // M
		{0x7F,0x04,0x08,0x10,0x7F,0x00}, // N
		{0x3E,0x41,0x41,0x41,0x3E,0x00}, // O
		{0x7F,0x09,0x09,0x09,0x06,0x00}, // P
		{0x3E,0x41,0x51,0x21,0x5E,0x00}, // Q
		{0x7F,0x09,0x19,0x29,0x46,0x00}, // R
		{0x46,0x49,0x49,0x49,0x31,0x00}, // S
		{0x01,0x01,0x7F,0x01,0x01,0x00}, // T
		{0x3F,0x40,0x40,0x40,0x3F,0x00}, // U
		{0x1F,0x20,0x40,0x20,0x1F,0x00}, // V
		{0x7F,0x20,0x18,0x20,0x7F,0x00}, // W
		{0x63,0x14,0x08,0x14,0x63,0x00}, // X
		{0x03,0x04,0x78,0x04,0x03,0x00}, // Y
		{0x61,0x51,0x49,0x45,0x43,0x00}, // Z
		{0x00,0x7F,0x41,0x41,0x00,0x00}, // [
		{0x02,0x04,0x08,0x10,0x20,0x00}, // backslash
		{0x00,0x41,0x41,0x7F,0x00,0x00}, // ]
		{0x04,0x02,0x01,0x02,0x04,0x00}, // ^
		{0x80,0x80,0x80,0x80,0x80,0x00}, // _
		{0x00,0x03,0x05,0x00,0x00,0x00}, // `
		{0x20,0x54,0x54,0x54,0x78,0x00}, // a
		{0x7F,0x48,0x44,0x44,0x38,0x00}, // b
		{0x38,0x44,0x44,0x44,0x20,0x00}, // c
		{0x38,0x44,0x44,0x48,0x7F,0x00}, // d
		{0x38,0x54,0x54,0x54,0x18,0x00}, // e
		{0x08,0x7E,0x09,0x01,0x02,0x00}, // f
		{0x0C,0x52,0x52,0x52,0x3E,0x00}, // g
		{0x7F,0x08,0x04,0x04,0x78,0x00}, // h
		{0x00,0x44,0x7D,0x40,0x00,0x00}, // i
		{0x20,0x40,0x44,0x3D,0x00,0x00}, // j
		{0x7F,0x10,0x28,0x44,0x00,0x00}, // k
		{0x00,0x41,0x7F,0x40,0x00,0x00}, // l
		{0x7C,0x04,0x18,0x04,0x7C,0x00}, // m
		{0x7C,0x08,0x04,0x04,0x78,0x00}, // n
		{0x38,0x44,0x44,0x44,0x38,0x00}, // o
		{0x7C,0x14,0x14,0x14,0x08,0x00}, // p
		{0x08,0x14,0x14,0x18,0x7C,0x00}, // q
		{0x7C,0x08,0x04,0x04,0x08,0x00}, // r
		{0x48,0x54,0x54,0x54,0x20,0x00}, // s
		{0x04,0x3F,0x44,0x40,0x20,0x00}, // t
		{0x3C,0x40,0x40,0x20,0x7C,0x00}, // u
		{0x1C,0x20,0x40,0x20,0x1C,0x00}, // v
		{0x3C,0x60,0x30,0x60,0x3C,0x00}, // w
		{0x44,0x28,0x10,0x28,0x44,0x00}, // x
		{0x0C,0x50,0x50,0x50,0x3C,0x00}, // y
		{0x44,0x64,0x54,0x4C,0x44,0x00}, // z
		{0x00,0x08,0x36,0x41,0x00,0x00}, // {
		{0x00,0x00,0x7F,0x00,0x00,0x00}, // |
		{0x00,0x41,0x36,0x08,0x00,0x00}, // }
		{0x02,0x01,0x02,0x04,0x02,0x00}, // ~

};

void OLED_WriteCmd(uint8_t cmd) {
    I2C1_Start();
    I2C1_SendAddress(SSD1306_ADDR);
    I2C1_SendByte(0x00);
    I2C1_SendByte(cmd);
    I2C1_Stop();
}

void OLED_WriteDataStream(uint8_t *data, uint16_t len) {
    I2C1_Start();
    I2C1_SendAddress(SSD1306_ADDR);
    I2C1_SendByte(0x40);
    for(uint16_t i=0;i<len;i++) I2C1_SendByte(data[i]);
    I2C1_Stop();
}

void OLED_SetCursor(uint8_t x, uint8_t page) {
    OLED_WriteCmd(0xB0+page);
    OLED_WriteCmd(0x00+(x&0x0F));
    OLED_WriteCmd(0x10+((x>>4)&0x0F));
}

void OLED_Clear(void) {
    uint8_t zeros[128]={0};
    for(uint8_t i=0;i<8;i++){
        OLED_SetCursor(0,i);
        OLED_WriteDataStream(zeros,128);
    }
}

void OLED_Init(void) {
    HAL_Delay(100);
    OLED_WriteCmd(0xAE);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xB0); OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0x00); OLED_WriteCmd(0x10);
    OLED_WriteCmd(0x40); OLED_WriteCmd(0x81); OLED_WriteCmd(0x7F);
    OLED_WriteCmd(0xA1); OLED_WriteCmd(0xA6);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F); OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0xF0);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0x22);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x20);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0xAF);
}

void OLED_PrintChar(char c) {
    if(c < 32 || c > 126) c = ' ';
    OLED_WriteDataStream((uint8_t*)font6x8[c-32],6);
}

void OLED_PrintString(const char* s) {
    while(*s) OLED_PrintChar(*s++);
}

// ==================== PWM ON PB0 (CH3) + PB1 (CH4) ====================
void PWM_TIM3_Init(void)
{
    // Enable GPIOB + TIM3
    RCC->AHB1ENR |= (1 << 1);   // GPIOB
    RCC->APB1ENR |= (1 << 1);   // TIM3

    // PB0 → TIM3_CH3 (AF2)
    GPIOB->MODER &= ~(3U << (0*2));
    GPIOB->MODER |=  (2U << (0*2));      // AF
    GPIOB->AFR[0] &= ~(0xF << (0*4));
    GPIOB->AFR[0] |=  (2U << (0*4));     // AF2 = TIM3

    // PB1 → TIM3_CH4 (AF2)
    GPIOB->MODER &= ~(3U << (1*2));
    GPIOB->MODER |=  (2U << (1*2));      // AF
    GPIOB->AFR[0] &= ~(0xF << (1*4));
    GPIOB->AFR[0] |=  (2U << (1*4));     // AF2 = TIM3

    // Timer3 Base Config
    TIM3->PSC = PWM_PSC;
    TIM3->ARR = PWM_ARR;

    // start with 0% duty
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;

    // CH3 PWM
    TIM3->CCMR2 &= ~(7U << 4);
    TIM3->CCMR2 |=  (6U << 4);  // PWM1
    TIM3->CCMR2 |=  (1U << 3);  // preload

    // CH4 PWM
    TIM3->CCMR2 &= ~(7U << 12);
    TIM3->CCMR2 |=  (6U << 12); // PWM1
    TIM3->CCMR2 |=  (1U << 11); // preload

    // Enable outputs
    TIM3->CCER |= (1 << 8);   // CH3
    TIM3->CCER |= (1 << 12);  // CH4

    // Enable timer
    TIM3->CR1 |= (1 << 7);    // ARPE
    TIM3->CR1 |= (1 << 0);    // CEN
}


// ==================== SIMPLE 3-INPUT "FUZZY" EVALUATION ====================
// We use a soft "fuzzy-like" scoring based on normalized inputs and thresholds.

static float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Map raw values to [0,1] severity
static float compute_severity(float temp_c, float ppm, float hum)
{
    // temp: 25°C => 0, 40°C => 1
    float t_norm = (temp_c - 25.0f) / (40.0f - 25.0f);
    t_norm = clamp01(t_norm);

    // ppm: 100 => 0, 800 => 1
    float p_norm = (ppm - 100.0f) / (800.0f - 100.0f);
    p_norm = clamp01(p_norm);

    // humidity: 30% => 0, 80% => 1
    float h_norm = (hum - 30.0f) / (80.0f - 30.0f);
    h_norm = clamp01(h_norm);

    // aggregate severity = max of the three
    float sev = t_norm;
    if (p_norm > sev) sev = p_norm;
    if (h_norm > sev) sev = h_norm;
    return sev;
}

// Decide LOW/MED/HIGH based on severity
static void fuzzy_compute_outputs(float temp_c, float ppm, float hum,
                                  float *fan_duty_out, float *vib_duty_out)
{
    float sev = compute_severity(temp_c, ppm, hum);

    // Fuzzy levels:
    // 0.0 - 0.33 => LOW
    // 0.33 - 0.66 => MED
    // 0.66 - 1.0 => HIGH

    if (sev < 0.33f) {
        // LOW
        *fan_duty_out = 30.0f;   // PB1
        *vib_duty_out = 20.0f;   // PB0
    } else if (sev < 0.66f) {
        // MED
        *fan_duty_out = 60.0f;
        *vib_duty_out = 50.0f;
    } else {
        // HIGH
        *fan_duty_out = 90.0f;
        *vib_duty_out = 80.0f;
    }
}

// Smoothly move current duty toward target
static void pwm_smooth_update(void)
{
    // PB0 (vib)
    if (g_vib_duty_cur < g_vib_duty_tgt) {
        g_vib_duty_cur += PWM_STEP;
        if (g_vib_duty_cur > g_vib_duty_tgt) g_vib_duty_cur = g_vib_duty_tgt;
    } else if (g_vib_duty_cur > g_vib_duty_tgt) {
        g_vib_duty_cur -= PWM_STEP;
        if (g_vib_duty_cur < g_vib_duty_tgt) g_vib_duty_cur = g_vib_duty_tgt;
    }

    // PB1 (fan)
    if (g_fan_duty_cur < g_fan_duty_tgt) {
        g_fan_duty_cur += PWM_STEP;
        if (g_fan_duty_cur > g_fan_duty_tgt) g_fan_duty_cur = g_fan_duty_tgt;
    } else if (g_fan_duty_cur > g_fan_duty_tgt) {
        g_fan_duty_cur -= PWM_STEP;
        if (g_fan_duty_cur < g_fan_duty_tgt) g_fan_duty_cur = g_fan_duty_tgt;
    }

    // Convert % to CCR values (0–ARR)
    uint32_t ccr3 = 0, ccr4 = 0;

    if (g_vib_duty_cur > 0.0f) {
        ccr3 = (uint32_t)((g_vib_duty_cur * (PWM_ARR + 1)) / 100.0f);
        if (ccr3 > PWM_ARR) ccr3 = PWM_ARR;
    }

    if (g_fan_duty_cur > 0.0f) {
        ccr4 = (uint32_t)((g_fan_duty_cur * (PWM_ARR + 1)) / 100.0f);
        if (ccr4 > PWM_ARR) ccr4 = PWM_ARR;
    }

    TIM3->CCR3 = ccr3;
    TIM3->CCR4 = ccr4;
}

// One call per loop: compute fuzzy targets and apply smooth ramp
void Fuzzy_UpdatePWM(float temp_c, float ppm, float hum)
{
    fuzzy_compute_outputs(temp_c, ppm, hum, &g_fan_duty_tgt, &g_vib_duty_tgt);
    pwm_smooth_update();
}




/* ====================== SYSTEM CLOCK =================== */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct={0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct={0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK) while(1);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|
                                  RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV1;
    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2)!=HAL_OK) while(1);
}





/* ====================== MAIN ====================== */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    ADC1_Init();
    DHT11_Init();
    I2C1_Init();
    OLED_Init();
    OLED_Clear();
    PWM_TIM3_Init();

    char buf[32];
    float temp_lm35 = 0.0f, ppm = 0.0f;
    uint8_t dht_temp = 0, dht_hum = 0;
    uint16_t t_whole, t_frac, ppm_val;

    while(1) {
        /* Read analog sensors */
        ppm = Read_MQ135_PPM();

        /* Read DHT11 */
        uint8_t status = DHT11_ReadData(&dht_hum, &dht_temp);

        /* Format for display */
//        t_whole = (uint16_t)temp_lm35;
//        t_frac  = (uint16_t)((temp_lm35 - t_whole) * 10);
        ppm_val = (uint16_t)ppm;

        OLED_Clear();


        OLED_SetCursor(2,1);
        sprintf(buf,"Air(MQ135): %d PPM", ppm_val);
        OLED_PrintString(buf);

        if(status == 0) {
            OLED_SetCursor(2,2);
            sprintf(buf,"Temp(DHT): %d C", dht_temp);
            OLED_PrintString(buf);

            OLED_SetCursor(2,3);
            sprintf(buf,"Humidity : %d %%", dht_hum);
            OLED_PrintString(buf);

             OLED_SetCursor(2,4);
            sprintf(buf,"ADC: %04d", ADC1_Read_Channel(0));
            OLED_PrintString(buf);

        }
        else if (status == 1) {
            OLED_SetCursor(2,3);
            OLED_PrintString("DHT: No Resp");
        }
        else {
            OLED_SetCursor(2,3);
            OLED_PrintString("DHT: Checksum");
        }

        Fuzzy_UpdatePWM(dht_temp, ppm, (float)dht_hum);

        HAL_Delay(1000);
    }
}
