
#ifndef MY_DHT11_H
#define MY_DHT11_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include <math.h>

//void DHT11_Init(GPIO_TypeDef* DataPort, uint16_t DataPin);

void DHT11_set_gpio_output (void);

void DHT11_set_gpio_input (void);

void DHT11_start (void);

void DHT11_check_response (void);

uint8_t DHT11_read_data (void);

char DHT11_GetTemp_Humidity(uint8_t *Temp, uint8_t *Humidity);

#endif