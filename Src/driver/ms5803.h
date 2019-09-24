/*
 * libms5803 - MS5803 pressure sensor library
 *
 * Copyright (C) 2014-2016 by Artur Wroblewski <wrobell@pld-linux.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MS5803_H_
#define _MS5803_H_



// registers of the device
#define MS5803_D1 0x40
#define MS5803_D2 0x50
#define MS5803_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5803_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5803_OSR_256 0x00
#define MS5803_OSR_512 0x02
#define MS5803_OSR_1024 0x04
#define MS5803_OSR_2048 0x06
#define MS5803_OSR_4096 0x08

#define MS5803_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
/*
 * Device structure
 */
struct ms5803_dev
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef  *gpio_port;
	uint16_t pin_cs;
};

/*!
 * \brief Initialize MS5803 sensor.
 *
 * I2C bus is initialized and sensor calibration coefficients are read.
 *
 * \param f_dev I2C device filename, i.e. /dev/i2c-0.
 * \param address I2C device address, i.e. 0x77.
 */
int ms5803_init(struct ms5803_dev *dev);

void ms5803_write_command(struct ms5803_dev *dev, uint8_t command);

void ms5803_read_data(struct ms5803_dev *dev,  uint8_t *data, uint8_t length);

void ms5803_start_conv_press(struct ms5803_dev *dev);

void ms5803_start_conv_temp(struct ms5803_dev *dev);

/*!
 * \brief Read pressure and temperature from MS5803 sensor.
 *
 * \param pressure Pressure - 1/10000 of bar or 1/10 of millibar.
 * \param temperature Temperature - 1/100 of Celsius.
 */
int ms5803_read(int32_t *pressure, int32_t *temperature);

#endif /* _MS5803_H_ */

/*
 * vim: sw=4:et:ai
 */
