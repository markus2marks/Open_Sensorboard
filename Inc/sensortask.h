/*
 * sensortask.h
 *
 *  Created on: 09.07.2019
 *      Author: markus
 */

#ifndef SRC_SENSORTASK_H_
#define SRC_SENSORTASK_H_

#define STACK_SIZE 200

void SensorTaskInit(struct ms5803_dev *pressure_sensor1, struct ms5803_dev *pressure_sensor2, struct ms4525do_dev *speed_sensor);
void SensorTask( void *pvParameters );

#endif /* SRC_SENSORTASK_H_ */
