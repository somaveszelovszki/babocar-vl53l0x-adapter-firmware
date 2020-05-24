#pragma once

#include <micro/port/gpio.hpp>
#include <micro/port/i2c.hpp>
#include <micro/port/timer.hpp>
#include <micro/port/uart.hpp>

extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim1;
extern UART_HandleTypeDef huart1;

#define gpio_Led                micro::gpio_t{ GPIOA, GPIO_PIN_5 }

#define i2c_Sensor              micro::i2c_t{ &hi2c1 }

#define tim_System              micro::timer_t{ &htim1 }

#define uart_PanelLink          micro::uart_t{ &huart1 }

#define PANEL_VERSION           0x0c
