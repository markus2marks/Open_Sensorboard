#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "main.h"
#include "ms5803.h"
#include "ms4525do.h"
#include "sensortask.h"


StaticTask_t xTaskBuffer;
StackType_t xStack[ STACK_SIZE ];
TaskHandle_t xHandle;
uint8_t data[4];
uint8_t data2[4];

struct ms5803_dev *m_pressure_sensor1;
struct ms5803_dev *m_pressure_sensor2;
struct ms4525do_dev *m_speed_sensor;

/*
 *
 */
void SensorTaskInit(struct ms5803_dev *pressure_sensor1, struct ms5803_dev *pressure_sensor2, struct ms4525do_dev *speed_sensor)
{
	BaseType_t ret;

	m_pressure_sensor1 = pressure_sensor1;
	m_pressure_sensor2 = pressure_sensor2;
	m_speed_sensor = speed_sensor;
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
	uint32_t value2 = 0;
	while(1)
	{
		ms5803_write_command(m_pressure_sensor1, 0x48);
		ms4525do_read_data(m_speed_sensor);
		printf("0x%x %i %i°C\r", ms4525do_get_status(), (int16_t) ms4525do_get_pressure(), ms4525do_get_temperature());
		vTaskDelay(40);
		data[0] = 0x00;
		data[1] = 0x00;
		data[2] = 0x00;
		data[3] = 0x00;
		ms5803_read_data(m_pressure_sensor1, data, 4);
		//ms5803_read_data(m_pressure_sensor2, data2, 4);
		value = (data[1] << 16) | (data[2] << 8) | data[3];
		//value2 = (data2[0] << 24) | (data2[1] << 16) | (data2[2] << 8) | data2[3];
		printf("0x%x\r", value);

		//CDC_Transmit_FS("Hello!\n\r",8);
		vTaskDelay(40);
	}
}
