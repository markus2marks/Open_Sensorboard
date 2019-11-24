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

static uint16_t temperature;
static int16_t pressure;
static uint8_t status;
static uint8_t data[4];
/*
 * Public API implementation.
 */

void ms4525do_read_data(struct ms4525do_dev *dev)
{
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_RESET);
	HAL_SPI_Receive(dev->hspi,(uint8_t*) data, 4, 100);
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_SET);

	status = (data[0] & 0xC0) >> 6;
	pressure = (int16_t) (((data[0] << 8) | data[1]) & 0x3fff) - 0x1FFF + 94;
	temperature =  (((data[2] << 8) | data[3]) >>  5) & 0x7ff;
	temperature = (temperature * 200 / 2047) - 50;
}

/*
 *
 */
uint16_t ms4525do_get_temperature(void)
{
	return temperature;
}

/*
 *
 */
int16_t ms4525do_get_pressure(void)
{
	return pressure;
}

/*
 *
 */
uint16_t ms4525do_get_status(void)
{
	return status;
}
