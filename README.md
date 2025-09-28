# 🚗 Advanced Power Window Control System  


![License](https://img.shields.io/badge/License-MIT-blue.svg)  
![Platform](https://img.shields.io/badge/Platform-TM4C123GH6PM-green.svg)  
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-orange.svg)  

---

## 📖 Overview  
This project implements an **Advanced Power Window Control System** using the **Tiva C Series TM4C123GH6PM** microcontroller and **FreeRTOS** for real-time task management.  

It provides:  
- Manual & automatic window operation  
- Safety features (IR obstacle detection, limit switches, emergency reverse)  
- Driver lock override  
- LCD + Buzzer notifications  
- Low-power operation (WFI instruction)  

---

## ✨ Features  

- **Manual & Automatic Control** 🚦  
- **Obstacle Detection & Emergency Reverse** 🛑  
- **Precise Position Tracking (QEI)** 📏  
- **Driver Lock** 🔒  
- **LCD Status + Buzzer Alerts** 📺🔔  
- **FreeRTOS Multi-tasking** ⚙️  

---
## 🏗️ System Architecture  

## 🔧 Hardware Components  

| Component                | Description                           | Pins Used         |
|---------------------------|---------------------------------------|------------------|
| **DC Motor + H-Bridge**  | Drives the window                     | PF2, PF3         |
| **IR Sensor**            | Obstacle detection                    | PB7              |
| **Incremental Encoder**  | Position feedback                     | PD6, PD7 (QEI0)  |
| **Limit Switches**       | Upper/lower bounds                    | PB0, PB1         |
| **Driver Buttons**       | Manual/Auto up & down                 | PB4, PB5         |
| **Passenger Buttons**    | Manual/Auto up & down                 | PA6, PA7         |
| **Lock Switch**          | Disables passenger control            | PB6              |
| **Buzzer**               | Audible alert                         | PA5              |
| **LCD (I²C)**            | Status display                        | I²C bus          |

---

## 💻 Software & FreeRTOS Tasks  

- `DriverControlTask` → manages driver input (manual/auto).  
- `PassengerControlTask` → manages passenger input, respects lock.  
- `EmergencyTask` → handles obstacle detection events from queue.  

**Synchronization:**  
- `xMotorSemaphore` → ensures exclusive motor control.  
- `xEmergencyQueue` → delivers safety events to `EmergencyTask`.  

---

## 📂 Repository Structure  

├── src/

│ ├── main.c

│ ├── LCD_I2C.c

│ ├── gpio.c

│ ├── i2c.c

│ ├── qei.c

│ ├── uart.c

│ ├── sysctl.c

│ └── watchdog.c


├── include/

│ ├── LCD_I2C.h

│ ├── gpio.h

│ ├── i2c.h

│ ├── qei.h

│ ├── uart.h

│ ├── sysctl.h

│ ├── watchdog.h

│ ├── pin_map.h

│ └── fpu.h


├── docs/

│ ├── Advanced Power Window Control System.pdf



└── README.md

yaml
Copy code

---

## 🔮 Future Improvements  

- Add **CAN bus** support for multi-window systems.  
- Integrate **mobile app** for remote monitoring.  
- Implement **fault logging** with UART/SD card.  
- Use **PWM speed control** for smoother motion.  
