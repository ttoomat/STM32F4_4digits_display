#include "stm32f4xx.h"

// Пины TM1637
#define CLK_PIN   8
#define DATA_PIN  9
#define CLK_PORT  GPIOB
#define DATA_PORT GPIOB

// Макросы для управления пинами
#define CLK_HIGH()  (CLK_PORT->BSRR = (1U << CLK_PIN))
#define CLK_LOW()   (CLK_PORT->BSRR = (1U << (CLK_PIN + 16)))
#define DATA_HIGH() (DATA_PORT->BSRR = (1U << DATA_PIN))
#define DATA_LOW()  (DATA_PORT->BSRR = (1U << (DATA_PIN + 16)))

// режим линии DIO: чтение (для ACK) и запись (для всего остального)
#define DATA_INPUT()  (DATA_PORT->MODER &= ~(3U << (DATA_PIN * 2)))
#define DATA_OUTPUT() (DATA_PORT->MODER |=  (1U << (DATA_PIN * 2)))

// задержка
void delay_us(volatile uint32_t us) {
    // При 16 МГц
    us *= 16;
    while (us--);
}

// условие start
void tm1637_start(void) {
    DATA_OUTPUT();
    DATA_HIGH();
    CLK_HIGH();
    delay_us(2);
    DATA_LOW();
    delay_us(2);
    CLK_LOW();
}

// условие stop
void tm1637_stop(void) {
    DATA_OUTPUT();
    CLK_LOW();
    DATA_LOW();
    delay_us(2);
    CLK_HIGH();
    delay_us(2);
    DATA_HIGH();
    delay_us(2);
}

// Отправка одного байта
void tm1637_write_byte(uint8_t data) {
    DATA_OUTPUT();
    // отправляет по одному биту
    for (uint8_t i = 0; i < 8; i++) {
        CLK_LOW();
        // 1 бит отображаем
        if (data & 0x01) {
            DATA_HIGH();
        } else {
            DATA_LOW();
        }
        delay_us(2);
        CLK_HIGH();
        delay_us(2);
        // переход к следующему биту
        data >>= 1;
    }
    // Ожидаем ACK (TM1637 тянет DIO вниз)
    CLK_LOW();
    DATA_INPUT();          // Переключаем DIO на вход
    delay_us(2);
    // (опционально: можно проверить, что линия LOW)
    CLK_HIGH();
    delay_us(2);
    CLK_LOW();
    DATA_OUTPUT();         // Возвращаем в режим вывода
}

// Отображение "11:00"
// Формат: DIG0 = правая цифра, DIG3 = левая
// Обычно двоеточие управляется через старший бит (бит 7) в соответствующем разряде
// Сегменты: A,B,C,D,E,F,G,DP (биты 0–7)
// Коды цифр (без DP):
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

void tm1637_display(uint8_t dig0, uint8_t dig1, uint8_t dig2, uint8_t dig3) {
	// создаём массив кодов разрядов
    uint8_t data[4] = {
        digit_code[dig0],
        digit_code[dig1] | 0x80, // DIG1 - с двоеточием
        digit_code[dig2],
        digit_code[dig3]
    };

    // 1. Режим автоинкремента адреса
    tm1637_start();
    tm1637_write_byte(0x40); // 01000000: автоадрес, запись данных
    tm1637_stop();

    // 2. Запись данных, начиная с адреса 0xC0 (00H)
    tm1637_start();
    tm1637_write_byte(0xC0); // команда установки адреса 00H
    for (int i = 0; i < 4; i++) {
        tm1637_write_byte(data[i]);
    }
    tm1637_stop();

    // 3. Включить дисплей с минимальной яркостью
    tm1637_start();
    tm1637_write_byte(0x88); // 10001000: включён, яркость = 1/16 (минимум)
    tm1637_stop();
}

int main(void) {
    // Включаем тактирование порта B
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // Настраиваем PB8 и PB9 как выход с Open-Drain (как в I2C)
    // Но TM1637 требует push-pull? На практике — open-drain с подтяжкой
    // Однако TM1637 втягивает линию сам, поэтому можно использовать push-pull
    // Установим как выход общего назначения, push-pull, low speed
    GPIOB->MODER &= ~((3U << (8*2)) | (3U << (9*2)));
    GPIOB->MODER |=  ((1U << (8*2)) | (1U << (9*2))); // Output mode

    GPIOB->OTYPER &= ~((1U << 8) | (1U << 9)); // Push-pull
    GPIOB->PUPDR &= ~((3U << (8*2)) | (3U << (9*2)));   // No pull

    // Инициализация TM1637
    tm1637_display(0, 5, 2, 3);

    while (1) {
    }
}
