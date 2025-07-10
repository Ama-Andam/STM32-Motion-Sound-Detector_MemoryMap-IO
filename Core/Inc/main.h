#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

// Define PIR pin (PA0)
#define PIR_PIN 0
#define PIR_PORT GPIOA

// Define Sound pin (PA1)
#define SOUND_PIN 1
#define SOUND_PORT GPIOA

// Define RED pin (PA5)
#define RED_PIN 5
#define RED_PORT GPIOA

// Define GREEN pin (PA6)
#define GREEN_PIN 6
#define GREEN_PORT GPIOA

// Define BLUE pin (PA7)
#define BLUE_PIN 7
#define BLUE_PORT GPIOA

// Functions
void delay(volatile uint32_t count);
void gpio_init(void);

#endif /* __MAIN_H */
