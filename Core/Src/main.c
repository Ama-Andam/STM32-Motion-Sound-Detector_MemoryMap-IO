#include "main.h"
#include <stdio.h>

#define SOUND_THRESHOLD 1000  // The sounds sensors range is
#define SOUND_HYSTERESIS   400    // Buffer to avoid false triggers

//Creating a delay function in case I need it
void delay(volatile uint32_t count){
    while (count--) {
        __asm("nop"); // No Operation
    }
}


void adc_init(void){
    // 1. Enable clock for ADC1 (on APB2 bus)
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // 2. Make sure ADC is off before configuring
    if (ADC1->CR2 & ADC_CR2_ADON) {
        ADC1->CR2 &= ~ADC_CR2_ADON;
    }

    // 3. Select channel 1 (which maps to PA1) for the first conversion
    ADC1->SQR3 = 1;  // Sequence register 3 — 1st conversion is channel 1

    // 4. Set a long sample time for channel 1 (PA1)
    ADC1->SMPR2 |= (7UL << 3);  // Channel 1 sample time = 480 cycles

    // 5. Enable ADC1 again
    ADC1->CR2 |= ADC_CR2_ADON;
}

uint16_t read_adc(void) {
    // Start conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;

    // Wait until conversion complete
    while (!(ADC1->SR & ADC_SR_EOC)) {}

    // Read conversion result
    return ADC1->DR;
}

void uart2_init(void) {
    // 1. Enable GPIOA and USART2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // 2. Configure PA2 as alternate function (AF7 = USART2_TX)
    GPIOA->MODER &= ~(3UL << (2 * 2));
    GPIOA->MODER |=  (2UL << (2 * 2));     // Alternate function
    GPIOA->AFR[0] |= (7UL << (4 * 2));     // AF7 on PA2

    // 3. Configure USART2
    USART2->BRR = SystemCoreClock / 115200;  // Assuming APB1 prescaler = 4 → 16 MHz
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;   // Enable TX, USART
}

void USART2_write(int ch) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (ch & 0xFF);
}

int __io_putchar(int ch) {
    USART2_write(ch);
    return ch;
}


void gpio_init(void) {
    // 1. Enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // --- Configure PA0 (PIR) as input ---
    GPIOA->MODER &= ~(3UL << (2 * PIR_PIN));  // Input mode
    GPIOA->PUPDR &= ~(3UL << (2 * PIR_PIN));  // No pull-up/pull-down

    // --- Configure PA1 (Sound sensor input, ADC) ---
    GPIOA->MODER &= ~(3UL << (2 * SOUND_PIN));  // Clear PA1 mode bits
    GPIOA->MODER |=  (3UL << (2 * SOUND_PIN));  // Set PA1 to analog (11)
    GPIOA->PUPDR &= ~(3UL << (2 * SOUND_PIN));  // No pull-up/pull-down

    // --- Configure PA5 (Red LED) as output ---
    GPIOA->MODER &= ~(3UL << (2 * RED_PIN));
    GPIOA->MODER |=  (1UL << (2 * RED_PIN));
    GPIOA->OTYPER &= ~(1UL << RED_PIN);         // Push-pull
    GPIOA->OSPEEDR |= (3UL << (2 * RED_PIN));   // High speed
    GPIOA->PUPDR &= ~(3UL << (2 * RED_PIN));    // No pull-up/pull-down

    // --- Configure PA6 (Green LED) as output ---
    GPIOA->MODER &= ~(3UL << (2 * GREEN_PIN));
    GPIOA->MODER |=  (1UL << (2 * GREEN_PIN));
    GPIOA->OTYPER &= ~(1UL << GREEN_PIN);
    GPIOA->OSPEEDR |= (3UL << (2 * GREEN_PIN));
    GPIOA->PUPDR &= ~(3UL << (2 * GREEN_PIN));

    // --- Configure PA7 (Blue LED) as output ---
    GPIOA->MODER &= ~(3UL << (2 * BLUE_PIN));
    GPIOA->MODER |=  (1UL << (2 * BLUE_PIN));
    GPIOA->OTYPER &= ~(1UL << BLUE_PIN);
    GPIOA->OSPEEDR |= (3UL << (2 * BLUE_PIN));
    GPIOA->PUPDR &= ~(3UL << (2 * BLUE_PIN));

    // Initialize all LEDs OFF at startup
    GPIOA->ODR &= ~((1 << RED_PIN) | (1 << GREEN_PIN) | (1 << BLUE_PIN));
}

int main(void) {
    gpio_init();
    adc_init();
    uart2_init();


    uint32_t motion_idle_counter = 0;
    uint32_t sound_idle_counter = 0;

    const uint32_t motion_threshold = 10;  // 2s
    const uint32_t sound_threshold = 10;   // 2s

    enum {IDLE, SOUND_DETECTED, MOTION_DETECTED} state = IDLE;

    while (1) {

        int motion = (PIR_PORT->IDR & (1 << PIR_PIN));
        uint16_t sound_value = read_adc();
        // Check for motion
        if (motion) {
            state = MOTION_DETECTED;
            motion_idle_counter = 0;
        } else if (state == MOTION_DETECTED) {
            motion_idle_counter++;
            if (motion_idle_counter > motion_threshold) {
                motion_idle_counter = motion_threshold;
                state = IDLE;
            }
        }

        // Check for sound
        if (sound_value > (SOUND_THRESHOLD + SOUND_HYSTERESIS)) {
            state = SOUND_DETECTED;
            sound_idle_counter = 0;
        } else if (state == SOUND_DETECTED) {
            sound_idle_counter++;
            if (sound_idle_counter > sound_threshold) {
                sound_idle_counter = sound_threshold;
                state = IDLE;
            }
        }

        // If no sound or motion for a while, be idle
        if (motion_idle_counter >= motion_threshold && sound_idle_counter >= sound_threshold) {
            state = IDLE;
        }

        // Clear all LEDs
        GPIOA->ODR &= ~((1 << RED_PIN) | (1 << GREEN_PIN) | (1 << BLUE_PIN));
        printf("ADC: %d | State: %d\n", sound_value, state);

        // Set LED based on state
        switch (state) {
            case MOTION_DETECTED:
                RED_PORT->ODR |= (1 << RED_PIN);
                break;
            case SOUND_DETECTED:
                BLUE_PORT->ODR |= (1 << BLUE_PIN);
                break;
            case IDLE:
                GREEN_PORT->ODR |= (1 << GREEN_PIN);
                break;
        }

        delay(200000);  // 200ms
    }
}
