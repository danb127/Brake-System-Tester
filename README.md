# ZF-Brake-System-Tester
This repository contains the STM32CubeIDE project for the Cortex-M4 core of the STM32MP157F-DK2 microcontroller used in the ZF Brake System Test Device. It handles the low-level I/O operations for testing various ZF brake system components including the Brake Signal Transmitter, Pressure Sensor, Continuous Wear Sensor, and Electronic Stability Control Module. The code manages PWM, analog voltage, pulsed signals, and CAN communication for device testing.
Key Features:

- ADC configurations for analog sensor readings
- Timer setups for PWM and pulsed signal generation
- CAN peripheral initialization and message handling
- Inter-processor communication (IPC) with the Cortex-A7 core using OpenAMP
- Peripheral I/O management for various sensor interfaces
- Real-time signal processing and basic test result computation
