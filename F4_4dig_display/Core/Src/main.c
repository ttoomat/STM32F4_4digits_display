/* CREATION DATE: 23.01.2026
 * FILE NAME:     main.c
 * PROGRAMMER:    @ttoomat
 * PURPOSE:       4 digits display TM1637 protocol.
 */
#include "stm32f4xx.h"

#define CLK_PORT GPIOB
#define CLK_PIN  8
#define DIO_PORT GPIOB
#define DIO_PIN  9

// CLK / DIO: HIGH / LOW
#define CLK_HIGH()  (CLK_PORT->BSRR = (1U << CLK_PIN))
#define CLK_LOW()   (CLK_PORT->BSRR = (1U << (CLK_PIN + 16)))
#define DIO_HIGH() (DIO_PORT->BSRR = (1U << DIO_PIN))
#define DIO_LOW()  (DIO_PORT->BSRR = (1U << (DIO_PIN + 16)))

// delay
// TODO: SysTick
void delay_us(volatile uint32_t us) {
    // При 16 МГц
    us *= 16;
    while (us--);
}

// start condition
void tm1637_start(void) {
    DIO_HIGH();
    CLK_HIGH();
    delay_us(2);
    DIO_LOW();
    delay_us(2);
    CLK_LOW();
}

// stop condition
void tm1637_stop(void) {
    CLK_LOW();
    DIO_LOW();
    delay_us(2);
    CLK_HIGH();
    delay_us(2);
    DIO_HIGH();
    delay_us(2);
}

// send 1 byte
void tm1637_write_byte(uint8_t data) {
    // sends one bit at a time
    for (uint8_t i = 0; i < 8; i++) {
        CLK_LOW();
        // show 1-st bit
        if (data & 0x01) {
            DIO_HIGH();
        } else {
        	DIO_LOW();
        }
        delay_us(2);
        CLK_HIGH();
        delay_us(2);
        // shift so that the next bit is first
        data >>= 1;
    }
    // in perfect world here we must check ACK...
    CLK_LOW();
    delay_us(2);
    CLK_HIGH();
    delay_us(2);
    CLK_LOW();
}

// semi-column = DP for DIG1, for other digits DP doesn't work.
// number codes (DP=0):
const uint8_t digit_code[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// format: DIG0 - left, DIG3 - right
void tm1637_display(uint8_t dig0, uint8_t dig1, uint8_t dig2, uint8_t dig3) {
	// digit codes array
    uint8_t data[4] = {
        digit_code[dig0],
        digit_code[dig1] | 0x80, // DIG1 - с двоеточием
        digit_code[dig2],
        digit_code[dig3]
    };
    // 1. Command 1: Auto increment address
    tm1637_start();
    tm1637_write_byte(0x40); // 0100_0000
    tm1637_stop();
    // 2. Command 2: Address set 00H
    tm1637_start();
    tm1637_write_byte(0xC0); // 1100_0000
    // 3. Write Data 0-3
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(data[i]);
    }
    tm1637_stop();
    // 3. Command 3: Show on, minimum brightness
    tm1637_start();
    tm1637_write_byte(0x88); // 1000_1000
    tm1637_stop();
}

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // PB8, PB9 output mode = 01
    DIO_PORT->MODER |= (1U << (DATA_PIN * 2));
    DIO_PORT->MODER &= ~(1U << (DIO_PIN * 2 + 1));
    CLK_PORT->MODER |= (1U << (CLK_PIN * 2));
    CLK_PORT->MODER &= ~(1U << (CLK_PIN * 2 + 1));
    // show 21:23
    tm1637_display(2, 1, 2, 3);
    while (1) {}
}
