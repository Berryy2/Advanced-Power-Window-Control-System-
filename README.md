Advanced Power Window Control System

📖 Overview
This project implements an advanced power window control system for automotive applications using FreeRTOS on the TM4C123GH6PM microcontroller. The system features:

Dual control interfaces (driver and passenger)

Auto-up/auto-down functionality

Obstacle detection with emergency reversal

Window lock feature

Position tracking via quadrature encoder

LCD status display

Safety limit switches

🛠 Hardware Components
Microcontroller: TM4C123GH6PM (ARM Cortex-M4)

Motor Driver: L298N or similar H-bridge

Sensors:

Quadrature encoder for position feedback

IR obstacle sensor

Mechanical limit switches

Inputs:

Driver up/down buttons

Passenger up/down buttons

Window lock switch

Outputs:

DC window motor

Buzzer for alerts

I2C LCD display (16x2)

📋 Features
Control Modes
Manual Mode: Press and hold buttons for continuous movement

Auto Mode: Quick press for automatic full open/close

Emergency Stop: Immediate stop and reversal on obstacle detection

Safety Features
Mechanical limit switches for end positions

IR obstacle detection

500ms emergency reverse on obstacle detection

Audible buzzer alert for emergencies

Passenger window lock

User Interface
Real-time position display (0-40cm range)

Status messages (opening/closing/obstacle/etc.)

Visual feedback for lock status

📂 Project Structure
text
power-window-control/
├── Inc/                 # Header files

│   ├── LCD_I2C.h        # LCD interface

│   └── tm4c123gh6pm.h   # MCU definitions

├── Src/
│   └── main.c           # Main application code

├── Drivers/             # Hardware abstraction layer

├── README.md            # This file

└── Documentation/       # Schematics and diagrams



🔌 Pin Configuration
Function	Pin	Description

Motor Up	PF2	Motor control signal

Motor Down	PF3	Motor control signal

Driver Up	PB4	Driver window up button

Driver Down	PB5	Driver window down button

Passenger Up	PA6	Passenger window up button

Passenger Down	PA7	Passenger window down button

Window Lock	PB6	Passenger disable switch

Upper Limit	PB0	Window fully closed switch

Lower Limit	PB1	Window fully open switch

IR Sensor	PB7	Obstacle detection

Buzzer	PA5	Audible alarm

QEI A	PD6	Encoder phase A

QEI B	PD7	Encoder phase B

🚀 Getting Started
Prerequisites
Code Composer Studio or Keil uVision

TM4C123G LaunchPad

Power window hardware setup

I2C LCD display (16x2)

Building and Flashing
Clone the repository

Import project into your IDE

Build the project

Flash to TM4C123G microcontroller

Connect hardware according to pin configuration

� FreeRTOS Configuration
Tasks:

EmergencyTask (Priority: 3)

DriverControlTask (Priority: 2)

PassengerControlTask (Priority: 1)

Synchronization:

Mutex semaphore for motor control

Queue for emergency events

📊 Performance Metrics
Position resolution: ±1cm

Emergency response time: <50ms

Auto mode speed: 15cm/second

Debounce time: 1000ms for auto mode detection

📝 License
This project is licensed under the MIT License - see the LICENSE file for details.

🤝 Contributing
Contributions are welcome! Please open an issue or submit a pull request.

✉ Contact
For questions or support, please contact:
Mohamed ElBerry - Mohamed_berry210@hotmail.com
