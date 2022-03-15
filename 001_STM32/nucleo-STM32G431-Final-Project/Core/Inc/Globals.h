/*
 * Globals.h
 *
 *  Created on: Nov 29, 2021
 *      Author: Luis Castaneda-Trejo
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"

typedef enum{
	INIT,
	MAIN,
	ERR_h,
	EXIT
}SM_STATES;

struct distance{
	uint8_t danger;
	uint8_t warning;
}distance_thresholds;

osThreadId_t ControllerHandle;
osThreadId_t led_greenHandle;
osThreadId_t led_yellowHandle;
osThreadId_t led_redHandle;
osThreadId_t ultra_sensor_trHandle;
osThreadId_t CAN_Rx_CtrlrHandle;
osThreadId_t CAN_Tx_CtrlrHandle;

/* USER CODE BEGIN Prototypes */
void print_to_console(char *myString);

/* USER CODE END Prototypes */

#endif /* INC_GLOBALS_H_ */
