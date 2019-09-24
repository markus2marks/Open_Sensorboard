/*
 * ms4525do.h
 *
 *  Created on: 08.08.2019
 *      Author: markus
 */

#ifndef MS4525DO_H_
#define MS4525DO_H_

/*
 * Device structure
 */
struct ms4525do_dev
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef  *gpio_port;
	uint16_t pin_cs;
};

void ms4525do_read_data(struct ms4525do_dev *dev,  uint8_t *data, uint8_t length);

#endif /* MS4525DO_H_ */
