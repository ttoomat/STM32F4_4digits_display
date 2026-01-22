/* CREATION DATE: 22.01.2026
 * FILE NAME:     main.c
 * PROGRAMMER:    @ttoomat
 * PURPOSE:       Main file of the project. Connects everything together.
 */
#include "stm32f446xx.h"

int main() {
	// LED on PA5
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// mode: General purpose output = 01
	GPIOA->MODER |= GPIO_MODER_MODE5_0;
	GPIOA->MODER &= ~GPIO_MODER_MODE5_1;
	// LED on
	GPIOA->ODR |= (1U << 5);
	return 0;
}
