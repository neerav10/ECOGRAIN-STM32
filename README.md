## ***STM32 F401RE***
<img width="615" height="537" alt="F401_diagram" src="https://github.com/user-attachments/assets/5584332d-656d-48dc-bf55-c100ee41cb86" />




## ***PIN CONNECTIONS***


| Category | Component | Function | STM32 Pin | Peripheral Used |
|---------|-----------|----------|----------|----------------|
| **Sensors** | DHT11 | Temperature & Humidity | PA7 | GPIO (1-Wire Timing) |
| | MQ135 | Gas/ Air Quality Sensor | PA0 | ADC1 Channel 0 |
| **Display** | SSD1306 OLED | SDA | PB9 | I2C1 SDA |
| | | SCL | PB8 | I2C1 SCL |
| **Actuators** | Fan | PWM Output | PB1 | TIM4 Channel 4 *(update as per config)* |
| | Vibration Motor | PWM Output | PB0 | TIM3 Channel 3 *(update as per config)* |

---

### 🧩 Features
| Feature |
|--------|
| Real-time humidity and temperature monitoring |
| Air quality sensing |
| OLED graphical output |
| PWM-based fan & vibration motor control |

---

### 🛠 Tools & Framework
| Tool / Library | Usage |
|---------------|-------|
| STM32F4xx MCU | Main Controller |
| HAL Drivers | Peripheral Control |
| Keil uVision / STM32CubeIDE | Development IDE |
| Interfaces Used | GPIO, I2C, ADC, PWM |

---

### 📎 Notes
| Note |
|------|
| Ensure pull-ups on I2C lines PB8 & PB9 (usually on OLED module) |
| MQ135 requires warm-up for accurate readings |
| Confirm selected Timers & Channels for PB0 & PB1 PWM |



## ***PROJECT VISUALS***

**Project setup:)**
![alt text](<project demos/image1.jpg>)






***Display values***
![alt text](<project demos/WhatsApp Image 2025-11-22 at 01.03.40_b871b40f.jpg>)



***Fuzzy Logic***
<img width="831" height="808" alt="image" src="https://github.com/user-attachments/assets/6d6a87aa-a86b-4497-aed5-61d98c069b04" />
                                
