/*
 * app_freertos.h
 *
 *  Created on: Feb 24, 2022
 *      Author: ecastanedat
 */

#ifndef INC_APP_FREERTOS_H_
#define INC_APP_FREERTOS_H_

/* USER CODE BEGIN 1 */
/*These are the task handle references. They were declared here
  to allow gpio.c access freeRTOS functions.*/
osThreadId_t CAN_CommHandle;
osThreadId_t led_greenHandle;
osThreadId_t led_yellowHandle;
osThreadId_t led_redHandle;
osThreadId_t ultra_sensor_trHandle;

/* USER CODE END 1 */

#endif /* INC_APP_FREERTOS_H_ */

