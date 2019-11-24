#ifndef __STM32f7xX_EMPL_DRV_H
#define __STM32f7xX_EMPL_DRV_H

#include "stm32f7xx_hal.h"
#include "mpu9250.h"

#define MAX_WRITE_SIZE     128


void stm32f7xx_empl_drv_init_i2c(struct mpu6050_dev *init_dev);
void stm32f7xx_empl_drv_init(struct mpu9250_dev *init_dev);

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
