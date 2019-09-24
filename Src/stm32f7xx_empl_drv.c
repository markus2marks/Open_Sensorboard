#include "stm32f7xx_empl_drv.h"
#include "main.h"

#include <string.h>

static unsigned char stm32f4xx_empl_i2c_wb[MAX_WRITE_SIZE];
static struct mpu9250_dev *dev;

void stm32f7xx_empl_drv_init(struct mpu9250_dev *init_dev)
{
	dev = init_dev;
}


int stm32f4xx_i2c_write (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data)
{	

	
	return 0;
}

													
int stm32f4xx_i2c_read  (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data)
{

	
	return 0;
}


int stm32f4xx_spi_write (unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data)
{	
	if (length + 1 > MAX_WRITE_SIZE)
		return -1;
	
	stm32f4xx_empl_i2c_wb[0] = reg_addr;
	memcpy (stm32f4xx_empl_i2c_wb + 1, data, length);
	
	
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi,stm32f4xx_empl_i2c_wb, length + 1, 1000);
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_SET);

	return 0;
}

int stm32f4xx_spi_read  (unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data)
{
	uint8_t i = 0;
	reg_addr =  0x80 | reg_addr;
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, &reg_addr, 1, 1000);
	HAL_SPI_Receive(dev->hspi, data, length, 1000);
	//HAL_SPI_TransmitReceive(&hspi3, &reg_addr, data, length, 1000);
	
/*
	SPI_send(reg_addr);
	for(i = 0; i < length; i++) 
		data[i] = SPI_send(0x00);
*/
	HAL_GPIO_WritePin(dev->gpio_port, dev->pin_cs, GPIO_PIN_SET);
	return 0;
}


void stm32f4xx_delay_ms  (unsigned long num_ms)
{
	HAL_Delay(num_ms);
}


void stm32f4xx_get_ms    (unsigned long *count)
{
	(*count) = HAL_GetTick();
}


void stm32f4xx_log_i (char const *s, ...)
{
	return;
}


void stm32f4xx_log_e (char const *s, ...)
{
	return;
}
