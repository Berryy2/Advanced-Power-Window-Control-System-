🚗 Advanced Power Window Control System




📖 Overview

This project implements an Advanced Power Window Control System using the Tiva C Series TM4C123GH6PM microcontroller and FreeRTOS for real-time task management.

The system controls a front passenger door window, providing manual and automatic operation, safety features like obstacle detection and limit switches, driver lock functionality, and low-power mode.

This design emphasizes safety, efficiency, and real-time reliability — key requirements in modern automotive embedded systems.

✨ Key Features

Manual & Automatic Window Operation

Manual: Window moves while switch is pressed.

One-touch Auto: Short press fully opens/closes the window.

Safety & Obstacle Detection

IR sensor detects obstructions during auto-close.

Emergency stop + reverse for 0.5s if an obstacle is detected.

Limit switches prevent over-travel.

Precise Position Control

Incremental encoder tracks window position.

LCD displays window position in cm.

Driver Lock Functionality

Lock switch disables passenger control.

Driver always retains full control.

Audible & Visual Alerts

Buzzer + LCD notifications for safety events.

Power Management

MCU enters low-power sleep mode (WFI) when idle.

FreeRTOS Integration

Independent tasks for driver control, passenger control, emergency handling.

Synchronization via semaphores and message queues.

🏗️ System Architecture
 ┌────────────┐       ┌────────────┐
 │   Driver   │──────▶│  Window    │
 │  Controls  │       │   Motor    │
 └────────────┘       └─────▲──────┘
                            │
 ┌────────────┐       ┌─────┴──────┐       ┌────────────┐
 │ Passenger  │──────▶│   Motor    │──────▶│   LCD +    │
 │  Controls  │       │   Driver   │       │   Buzzer   │
 └────────────┘       └─────▲──────┘       └────────────┘
                            │
                 ┌──────────┴──────────┐
                 │  Sensors & Feedback │
                 │ (IR, Encoder, Limit)│
                 └──────────┬──────────┘
                            │
                     ┌──────┴───────┐
                     │  TM4C123 MCU │
                     │  + FreeRTOS  │
                     └──────────────┘

🔧 Hardware Components
Component	Description	Pins Used
TM4C123GH6PM	Tiva C Series MCU	—
DC Motor + H-Bridge	Drives the window	PF2, PF3
IR Sensor	Obstacle detection	PB7
Incremental Encoder	Position feedback	PD6, PD7 (QEI0)
Limit Switches	Upper/lower bounds	PB0, PB1
Driver Buttons	Manual/Auto up & down	PB4, PB5
Passenger Buttons	Manual/Auto up & down	PA6, PA7
Lock Switch	Disables passenger control	PB6
Buzzer	Audible alert	PA5
LCD (I²C)	Status display	I²C bus
💻 Software & FreeRTOS Implementation

Tasks

DriverControlTask → handles driver buttons (manual/auto).

PassengerControlTask → handles passenger buttons, obeying lock.

EmergencyTask → reacts to IR obstacle detection using a queue.

Synchronization

Semaphore (xMotorSemaphore) → ensures safe motor access (prevents race conditions).

Queue (xEmergencyQueue) → passes obstacle detection events to EmergencyTask.

Idle Hook

Implements WFI instruction → MCU enters low-power sleep mode until next interrupt.

📂 Repository Structure
Advanced-Power-Window-Control-System-/
│── src/           
│    ├── main.c
│    ├── LCD_I2C.c
│    ├── gpio.c
│    ├── i2c.c
│    ├── qei.c
│    ├── uart.c
│    ├── sysctl.c
│    └── watchdog.c
│
│── include/       
│    ├── LCD_I2C.h
│    ├── gpio.h
│    ├── i2c.h
│    ├── qei.h
│    ├── uart.h
│    ├── sysctl.h
│    ├── watchdog.h
│    ├── pin_map.h
│    └── fpu.h
│
│── docs/          
│    └── Advanced Power Window Control System.pdf
│
│── README.md

🚀 Setup & Usage
Prerequisites

Keil µVision or Code Composer Studio

TivaWare driverlib

FreeRTOS

Build & Flash

Clone repo:

git clone https://github.com/<your-username>/Advanced-Power-Window-Control-System-.git
cd Advanced-Power-Window-Control-System-


Open project in IDE.

Build and flash to TM4C123GH6PM LaunchPad.

⚡ Challenges & Solutions

Race conditions → solved with semaphores around motor control.

Task communication → implemented queues for emergency events.

Energy efficiency → used FreeRTOS Idle Hook (WFI) for sleep mode.

Debouncing & auto/manual detection → threshold-based timing logic.

🔮 Future Improvements

Integrate CAN bus for multi-window control.

Add mobile app for remote monitoring.

Implement fault logging via UART or SD card.

Use PWM motor speed control for smoother operation.

📜 License

This project is licensed under the MIT License.

✨ Developed by Mohamed Maged El Sayed Ahmed Elberry
🎓 CSE411: Real-Time and Embedded Systems Design (Spring 2025)
