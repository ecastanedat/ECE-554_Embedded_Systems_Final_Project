/*
 * Globals.h
 *
 *  Created on: Nov 29, 2021
 *      Author: uib01493
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_hal.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"

/* Data Overhead--------------------------------------------------------------*/
/* USER CODE BEGIN Data_Overhead */
struct dataOverhead
{
	char myString[100];
	uint16_t btn1_flag;

}globalCluster;

typedef enum{
	INIT,
	MAIN,
	DANGER,
	WARNING,
	ERROR_handling,
	EXIT
}SM_STATES;

/* USER CODE END Data_Overhead */

uint8_t distance_danger_thershold;
uint8_t distance_warning_thershold;

osThreadId_t ControllerHandle;
osThreadId_t led_greenHandle;
osThreadId_t led_yellowHandle;
osThreadId_t led_redHandle;
osThreadId_t ultra_sensor_trHandle;

/* USER CODE BEGIN Prototypes */
void print_to_console(char *myString);

/* USER CODE END Prototypes */

#endif /* INC_GLOBALS_H_ */
