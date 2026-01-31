/* CREATION DATE: 23.01.2026
 * FILE NAME:     main.c
 * PROGRAMMER:    @ttoomat
 * PURPOSE:       4 digits display TM1637 protocol.
 */
#include "stm32f4xx.h"

#define APB1        ((uint32_t)16000000)
#define I2C_F		((uint32_t)APB1/1000000) // F in MHz
#define I2C_TPCLK1	((uint32_t)1000/I2C_F) // период шины APB = 1/Fapb. ~22ns
#define I2C_CCR		((uint32_t)100/I2C_TPCLK1)
#define I2C_TRISE	((uint32_t)(300/I2C_TPCLK1)+1)
#define I2C_100KHZ	80		//0B 0101 0000 = Decimal = 80

#define I2C1_SCL_PIN 8
#define I2C1_SDA_PIN 9
#define I2C1_SCL_BIT (1U << I2C1_SCL_PIN)
#define I2C1_SDA_BIT (1U << I2C1_SDA_PIN)

static uint8_t gpio_read_pin(GPIO_TypeDef* port, uint8_t pin) {
    return (port->IDR & (1U << pin)) ? 1 : 0;
}

/* Разблокировка шины.
 */
static void I2C1_Unstick(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // 1. Сохраняем текущие настройки.
    uint32_t moder_backup = GPIOB->MODER;
    uint32_t otyper_backup = GPIOB->OTYPER;
    // 2. GPIO Output Open-Drain
    // PB8 (SCL)
    GPIOB->MODER &= ~(3U << (2*I2C1_SCL_PIN));
    GPIOB->MODER |=  (1U << (2*I2C1_SCL_PIN));
    // PB9 (SDA)
    GPIOB->MODER &= ~(3U << (2*I2C1_SDA_PIN));
    GPIOB->MODER |=  (1U << (2*I2C1_SDA_PIN));
    // Open-Drain
    GPIOB->OTYPER |= (I2C1_SCL_BIT | I2C1_SDA_BIT);
    // 3. Проверяем текущее состояние шины
    // Если обе линии уже HIGH  разблокировка не нужна
    if (gpio_read_pin(GPIOB, I2C1_SCL_PIN) && gpio_read_pin(GPIOB, I2C1_SDA_PIN)) {
        // Восстанавливаем настройки
        GPIOB->MODER = moder_backup;
        GPIOB->OTYPER = otyper_backup;
        return;
    }
    // 4. Формируем 9 тактовых импульсов на SCL (стандартная процедура разблокировки)
    for (uint8_t i = 0; i < 9; i++) {
        GPIOB->BSRR = I2C1_SCL_BIT;          // SCL = HIGH
        for (volatile uint32_t d = 0; d < 100; d++);
        GPIOB->BSRR = I2C1_SCL_BIT << 16;    // SCL = LOW
        for (volatile uint32_t d = 0; d < 100; d++);
    }
    // 5. Формируем STOP условие.
    GPIOB->BSRR = I2C1_SDA_BIT << 16;  // SDA = LOW
    GPIOB->BSRR = I2C1_SCL_BIT;        // SCL = HIGH
    for (volatile uint32_t d = 0; d < 100; d++);
    GPIOB->BSRR = I2C1_SDA_BIT;        // SDA = HIGH
    // 6. Восстанавливаем настройки GPIO.
    GPIOB->MODER = moder_backup;
    GPIOB->OTYPER = otyper_backup;
}

/* I2C GPIO MODE & AFR register set.
 * PB8 = SCL, PB9 = SDA.
 */
void _I2C1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    // PB8, PB9 → Alternate Function (AF4)
    GPIOB->MODER &= ~(3U << 16); GPIOB->MODER |= (2U << 16); // PB8
    GPIOB->MODER &= ~(3U << 18); GPIOB->MODER |= (2U << 18); // PB9
    // open-drain
    GPIOB->OTYPER |= (1U << 8) | (1U << 9);
    // pull-up
    GPIOB->PUPDR &= ~(3U << 16);
    GPIOB->PUPDR |=  (1U << 16);
    GPIOB->PUPDR &= ~(3U << 18);
    GPIOB->PUPDR |=  (1U << 18);
    // AF4 for PB8, PB9
    GPIOB->AFR[1] &= ~(0xF << 0); GPIOB->AFR[1] |= (4U << 0); // PB8
    GPIOB->AFR[1] &= ~(0xF << 4); GPIOB->AFR[1] |= (4U << 4); // PB9
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
}

/* I2C setup.
 */
void I2C1_Setup() {
  _I2C1_Init();
  // reset
  I2C1->CR1 = I2C_CR1_SWRST;
  for (uint32_t i = 0; i < 1000; i++);
  I2C1->CR1 &= ~I2C_CR1_SWRST;
  // CR2 bits 0-5: APB frequency = 16
  I2C1->CR2 = (1U << 4);
  // Configure the clock control registers
  I2C1->CCR = I2C_100KHZ;
  I2C1->TRISE = I2C_TRISE;
  I2C1->CR1 |= I2C_CR1_PE;
}

/* Send byte using I2C.
 * Parameters: 4 decimal numbers to send?
 */
/*
void I2C1_transmit() {
	volatile int tmp;
	// write as address
	// 0x40 = 0b0100_0000
	// для адреса 0-й бит -- R/W. W=0.
	// Wait until bus not busy
	while (I2C1->SR2 & (I2C_SR2_BUSY)) {}
	// Start
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB)) {}
	// Command1 0x40 but lsb first. 0b0000_0010
	I2C1->DR = 0x02;
	// wait until address is sent
	//while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
	// Clear addr flag
	//tmp = I2C1->SR2;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// Command2 0xC0-> 0b1100_0000 -> 0x03
	I2C1->DR = 0x03;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// number1
	I2C1->DR = 0xFF;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// number2
	I2C1->DR = 0xFF;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// number3
	I2C1->DR = 0xFF;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// number4
	I2C1->DR = 0xFF;
	// Wait until data register empty
	while (!(I2C1->SR1 & (I2C_SR1_TXE))) {}
	// Command 4 0x88 -> 0b1000_1000->0x11;
	I2C1->DR = 0x11;
	// Wait until transfer finished
	while (!(I2C1->SR1 & (I2C_SR1_BTF))){}
	I2C1->CR1 |= I2C_CR1_STOP;
}
*/

// ===== ОТПРАВКА ДАННЫХ (с фиктивным адресом) =====
void I2C1_transmit(void) {
    // 1. START
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {}

    // 2. Фиктивный адрес (0x00) + W=0
	I2C1->DR = 0x00;
	//while (!(I2C1->SR1 & I2C_SR1_ADDR)) {}
	(void)I2C1->SR2;  // Сброс ADDR

	// 3. Команда 0x40 → 0x02 (LSB first)
	I2C1->DR = 0x02;
	while (!(I2C1->SR1 & I2C_SR1_TXE)) {}

	// 4. Команда 0xC0 → 0x03 (LSB first)
	I2C1->DR = 0x03;
	while (!(I2C1->SR1 & I2C_SR1_TXE)) {}

	// 5. Данные (4 цифры)
	for (uint8_t i = 0; i < 4; i++) {
		I2C1->DR = 0xFF;
		while (!(I2C1->SR1 & I2C_SR1_TXE)) {}
	}

	// 6. Команда 0x88 → 0x11 (LSB first)
	I2C1->DR = 0x11;
	while (!(I2C1->SR1 & I2C_SR1_BTF)) {}

	// 7. STOP
	I2C1->CR1 |= I2C_CR1_STOP;
}

int main(void) {
	I2C1_Unstick();
	I2C1_Setup();
	I2C1_transmit();
	while (1) {}
}

