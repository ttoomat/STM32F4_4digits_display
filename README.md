# 4 digit display project

TM1637 is a 4 digit display with a 2 wire protocol, different from I2C. For Russian tutorial check TM1637_tutorial_ru.pdf in this repository.

The project is in folder F4_4dig_display. It is for STM32 F446RET6 (NUCLEO board). CMSIS, STM32 Cube IDE project.

The final project would be: two 4 digit displays, one is counting down from 100h (hh:mm), second display is simple stopwatch (mm:ss). Plus there'd be a stop button. Later I'll add writing data to FLASH every second or only when power is gone (then a battery is needed).

Clock Config project is there to use .IOC for prescaler and bus frequencies check.

Idea: send data in I2C format, where instead of address we send the command.