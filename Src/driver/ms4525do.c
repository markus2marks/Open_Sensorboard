/*
 * ms4525do.c
 *
 *  Created on: 08.08.2019
 *      Author: markus
 */

#include <stdint.h>
#include <unistd.h>
#include "main.h"
#include "ms4525do.h"


/*
 * Public API implementation.
 */

void ms4525do_read_data(struct ms4525do_dev *dev,  uint8_t *data, uint8_t length)
{
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_RESET);
	HAL_SPI_Receive(dev->hspi,(uint8_t*) data, length, 100);
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_SET);
}
