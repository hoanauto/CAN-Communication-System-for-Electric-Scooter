# CAN-Communication-System-for-Electric-Scooter
Overview Designed and implemented a CAN-based communication system for an electric scooter to enable reliable and efficient data exchange between multiple electronic control units (ECUs) such as motor controller, throttle unit, signals for lights and vehicle accessories and dashboard display.
## Key Features
- STM32F103 microcontroller used as the primary ECU platform.
- Configured bxCAN peripheral on STM32 using STM32CubeMX and HAL libraries.
- Developed drivers for CAN initialization, transmit, receive, and filter configuration.
- Implemented custom CAN message protocol for throttle position, motor speed, signals for lights and vehicle accessories and dashboard display.
## Echnologies Used
- STM32F1 (STM32F103C8T6)
- STM32CubeMX, Keil uVision
- C programming (embedded)
- SPL
- Can bus MCU230 SN65HVD230
- CAN MCP2515
- Raspberry pi 5
