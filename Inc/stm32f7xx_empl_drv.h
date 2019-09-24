#ifndef __STM32f7xX_EMPL_DRV_H
#define __STM32f7xX_EMPL_DRV_H

#include "stm32f7xx_hal.h"

#define MAX_WRITE_SIZE     128

/*
 * Device structure
 */
struct mpu9250_dev
{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef  *gpio_port;
	uint16_t pin_cs;
};



int stm32f7xx_i2c_write (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char const *data);
													
int stm32f7xx_i2c_read  (unsigned char slave_addr,
                         unsigned char reg_addr, 
                         unsigned char length, 
                         unsigned char *data);

int stm32f7xx_spi_write (unsigned char reg_addr,
                         unsigned char length, 
                         unsigned char const *data);												 
												
int stm32f7xx_spi_read  (unsigned char reg_addr,
                         unsigned char length, 
                         unsigned char *data);				 


uint16_t SPI_send(uint8_t data);
												 
void stm32f7xx_delay_ms (unsigned long num_ms);
void stm32f7xx_get_ms   (unsigned long *count);

void stm32f7xx_log_i    (char const *s, ...);
void stm32f7xx_log_e    (char const *s, ...);

#endif /* __STM32f7xX_EMPL_DRV_H */
