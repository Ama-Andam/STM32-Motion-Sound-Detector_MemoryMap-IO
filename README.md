# STM32 Motion-Sound Detector with RGB LED Status

This project demonstrates a motion and sound detection system for the STM32F446RE Nucleo board using RGB LED status indicators to provide visual feedback.

## Overview

This project implements a comprehensive sensor monitoring system that:
- Detects motion using PIR sensors
- Monitors sound levels through microphone input
- Provides real-time status feedback via RGB LED
- Uses only direct register access (memory-mapped I/O) for peripheral control (no STM32 HAL functions)

This is part of my journey to become a software embedded engineer, building upon and enhancing concepts from my undergraduate embedded systems studies.

## Hardware

• **Board**: STM32F446RE Nucleo
• **Motion Sensor**: PIR (Passive Infrared) sensor
• **Sound Sensor**: Microphone module with analog output
• **Status Indicator**: RGB LED
• **Additional**: Connecting wires and breadboard

## Approach

This implementation uses direct register manipulation which provides:
- Low-level hardware control
- Better understanding of microcontroller internals
- Smaller code footprint
- Direct access to peripheral registers
- No abstraction layer dependencies

### Key Register Operations Used

• **RCC registers** - Clock enable for GPIO and ADC peripherals
• **GPIO MODER** - Pin mode configuration (input/output/analog)
• **GPIO ODR** - Digital output control for LED states
• **ADC registers** - Direct analog-to-digital conversion setup and reading
• **USART registers** - Serial communication for debugging output

## Features

• **Motion Detection**: Real-time PIR sensor monitoring
• **Sound Level Monitoring**: Analog sound level detection with threshold settings
• **RGB LED Status Indication**:
  - **Green**: Normal operation (no motion/sound detected)
  - **Red**: Motion detected
  - **Blue**: Sound detected above threshold
  - **System feedback**: Real-time UART debugging output

## Project Structure

```
├── Core/
│   ├── Inc/           # Header files
│   │   ├── main.h
│   │   ├── main.c     # Main application logic
│   └── Startup/       # Startup files
│   └── Startup/       # Startup files
├── Drivers/           # STM32 HAL drivers
├── Debug/            # Compiled binaries and debug files
└── *.ioc            # STM32CubeMX configuration
```

## Implementation Details

This project utilizes:
- **ADC**: For analog sound level reading with direct register access (no HAL)
- **GPIO**: For PIR sensor digital input and RGB LED control (no HAL)
- **USART**: For real-time debugging output over serial (no HAL)
- **Custom delay**: CPU-cycle based timing without timer peripherals

## Learning Objectives

• Understanding multi-sensor integration with STM32
• Direct register manipulation for ADC configuration
• GPIO control without abstraction layers
• USART configuration for debugging output
• Real-time embedded system design principles

## Demo

[Demo Video Link will be added here]

## Building and Running

1. Open the project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to the STM32F446RE board
4. Connect sensors according to pin configuration in main.h
5. Monitor the RGB LED for real-time status feedback

