/*
 * sensortask.h
 *
 *  Created on: 09.07.2019
 *      Author: markus
 */

#ifndef SRC_SENSORTASK_H_
#define SRC_SENSORTASK_H_

#define STACK_SIZE 200

void SensorTaskInit(void);
void SensorTask( void *pvParameters );

#endif /* SRC_SENSORTASK_H_ */
