/* CREATION DATE: 23.01.2026
 * FILE NAME:     main.c
 * PROGRAMMER:    @ttoomat
 * PURPOSE:       4 digits display TM1637 protocol.
 */
#include "stm32f4xx.h"
/*
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
// TODO:
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
    0b10000110, // 1
    0b11011011, // 2
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
    tm1637_write_byte(0x8A); // 1000_1000
    tm1637_stop();
}

*/

void delay_us(volatile uint32_t us) {
    // При 16 МГц
    us *= 16;
    while (us--);
}

void SPI1_Init() {
	// 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.
	// PA5 - CLK, PA7 - MOSI. AF5. MODER = 10
	// MODER:AF = 10
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~GPIO_MODER_MODE5_0;
    GPIOA->MODER |= GPIO_MODER_MODE5_1;
    GPIOA->MODER &= ~GPIO_MODER_MODE7_0;
    GPIOA->MODER |= GPIO_MODER_MODE7_1;
    // AFR5 = 0101
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_0;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_1;
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL5_2;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL5_3;
    // AFR7 = 0101
    GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_0;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_1;
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFRL7_3;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
}

void SPI1_Config() {
	// 2. Write to the SPI_CR1 register:
	// a) Configure the serial clock baud rate using the BR[2:0] bits
	// 16 MHz -> 125 KHz, 128 = 110
	SPI1->CR1 |= SPI_CR1_BR_1 | SPI_CR1_BR_2;
	SPI1->CR1 &= ~SPI_CR1_BR_0;
	// b) Configure the CPOL=0 and CPHA=1 bits combination.
	SPI1->CR1 &= ~SPI_CR1_CPOL;
	SPI1->CR1 |= SPI_CR1_CPHA;
	// c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
	// BIDIOE (RXONLY and BIDIMODE can't be set at the same time)
	//SPI1->CR1 |= SPI_CR1_BIDIOE; // output enabled
	// d) Configure the LSBFIRST bit to define the frame format
	// MSB first by default
	// e) Configure the CRCEN and CRCEN bits if CRC is needed -- not needed I guess
	// f) Configure SSM and SSI -- we don't use NSS pin
	// g) Configure the MSTR bit (Master=1)
	SPI1->CR1 |= SPI_CR1_MSTR;
	// h) Set the DFF bit to configure the data frame format (8 or 16 bits).
	// 8 bit by default = 1 command
	// 3. Write to SPI_CR2 register:
	// a) Configure SSOE
	// SS=0 by default
	// SPI enable
	SPI1->CR1 |= SPI_CR1_SPE;
}

// ...
void spi1_transmit(uint8_t *data, uint32_t size) {
	uint32_t i = 0;
	uint8_t temp;
	while (i < size) {
		// wait for TXE
		// на втором байте тут зависает
		while (!(SPI1->SR & SPI_SR_TXE)) {}
		// write data to the data register
		SPI1->DR = data[i];
		i++;
	}
	// wait for TXE
	//while (!(SPI1->SR & (SPI_SR_TXE))) {}
	// wait for BUSY flag to reset
	while (SPI1->SR & SPI_SR_BSY) {}
	SPI1->SR &= ~SPI_SR_TXE;
}


int main(void) {
    /*
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // PB8, PB9 output mode = 01
    DIO_PORT->MODER |= (1U << (DIO_PIN * 2));
    DIO_PORT->MODER &= ~(1U << (DIO_PIN * 2 + 1));
    CLK_PORT->MODER |= (1U << (CLK_PIN * 2));
    CLK_PORT->MODER &= ~(1U << (CLK_PIN * 2 + 1));
    // show 21:23
    tm1637_display(2, 1, 2, 3);
    */
	SPI1_Init();
	SPI1_Config();
	uint8_t data[7]; // 2 commands, 4 bytes, 1 command
	data[0] = 0x40;
	data[1] = 0xC0;
	data[2] = 0xFF; // 8
	data[3] = 0xFF; // 8
	data[4] = 0xFF; // 8
	data[5] = 0xFF; // 8
	data[6] = 0x88;
	for (uint8_t i = 0; i < 7; ++i) {
		spi1_transmit(&(data[i]), 1);
		delay_us(5);
	}
	//spi1_transmit(data, 7);
    while (1) {}
}
