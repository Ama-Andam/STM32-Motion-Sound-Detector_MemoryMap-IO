# STM32 Motion-Sound Detector with RGB LED Status

This project demonstrates a motion and sound detection system for the STM32F446RE Nucleo board using RGB LED status indicators to provide visual feedback.

## Overview

This project implements a comprehensive sensor monitoring system that:
- Detects motion using PIR sensors
- Monitors sound levels through microphone input
- Provides real-time status feedback via RGB LED
- Utilizes STM32 HAL functions for peripheral control

This is part of my journey to become a software embedded engineer, building upon and enhancing concepts from my undergraduate embedded systems studies.

## Hardware

• **Board**: STM32F446RE Nucleo
• **Motion Sensor**: PIR (Passive Infrared) sensor
• **Sound Sensor**: Microphone module with analog output
• **Status Indicator**: RGB LED
• **Additional**: Connecting wires and breadboard

## Approach

This implementation uses STM32 HAL functions which provide:
- High-level abstraction for ADC and GPIO operations
- Interrupt-driven sensor reading
- Timer-based RGB LED control
- Built-in error handling and safety checks

### Key HAL Functions Used

• `HAL_ADC_Start_IT()` - For sound level analog reading
• `HAL_GPIO_ReadPin()` - For PIR motion detection
• `HAL_TIM_PWM_Start()` - For RGB LED color control
• `HAL_GPIO_WritePin()` - For digital output control

## Features

• **Motion Detection**: Real-time PIR sensor monitoring
• **Sound Level Monitoring**: Analog sound level detection with threshold settings
• **RGB LED Status Indication**:
  - **Green**: Normal operation (no motion/sound detected)
  - **Yellow**: Motion detected only
  - **Red**: Sound detected only
  - **Blue**: Both motion and sound detected
  - **Purple**: System in calibration mode

## Project Structure

```
├── Core/
│   ├── Inc/           # Header files
│   │   ├── main.h
│   │   ├── stm32f4xx_hal_conf.h
│   │   └── stm32f4xx_it.h
│   ├── Src/           # Source files
│   │   ├── main.c     # Main application logic
│   │   ├── stm32f4xx_hal_msp.c
│   │   └── stm32f4xx_it.c
│   └── Startup/       # Startup files
├── Drivers/           # STM32 HAL drivers
├── Debug/            # Compiled binaries and debug files
└── *.ioc            # STM32CubeMX configuration
```

## Implementation Details

This project utilizes:
- **ADC**: For analog sound level reading with interrupt-based conversion
- **GPIO**: For PIR sensor digital input and RGB LED control
- **Timers**: For PWM generation for smooth RGB color transitions
- **Interrupts**: For real-time sensor response

## Learning Objectives

• Understanding multi-sensor integration with STM32
• ADC configuration for analog sensor reading
• GPIO interrupt handling for digital sensors
• PWM control for RGB LED color mixing
• Real-time embedded system design principles

## Demo

[Demo Video Link will be added here]

## Building and Running

1. Open the project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to the STM32F446RE board
4. Connect sensors according to pin configuration in main.h
5. Monitor the RGB LED for real-time status feedback

