#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "main.h"
#include "ms5803.h"
#include "sensortask.h"


StaticTask_t xTaskBuffer;
StackType_t xStack[ STACK_SIZE ];
TaskHandle_t xHandle;
uint8_t data[4];
/*
 *
 */
void SensorTaskInit(void)
{
	BaseType_t ret;


	/* Create the task, storing the handle. */
	ret = xTaskCreate(
					SensorTask,       /* Function that implements the task. */
					"SensorTask",          /* Text name for the task. */
					200,      /* Stack size in words, not bytes. */
					NULL,    /* Parameter passed into the task. */
					tskIDLE_PRIORITY + 1,/* Priority at which the task is created. */
					&xHandle );      /* Used to pass out the created task's handle. */

}
/*
 *
 */
void SensorTask( void *pvParameters )
{
	uint32_t value = 0;
	while(1)
	{
		ms5803_write_command(0x48);
		vTaskDelay(20);
		data[0] = 0x00;
		ms5803_read_data(data, 4);
		value = (data[1] << 16) | (data[2] << 8) | data[3];
		printf("0x%x\r", value);
		vTaskDelay(100);
	}
}
