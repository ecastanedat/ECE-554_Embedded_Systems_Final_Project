/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Globals.h"
#include "fdcan.h"
#include "tim.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct memory_buffer
{
	uint32_t downcounter;
	uint8_t enable;
	uint32_t ticks;
}mBuff;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPEED_OF_SOUND    34300          // Speed of sound in cm/s.
#define TIMER_PERIOD      0.0000000125   //80 Mhz clock. Period = 0.0125 us.
#define OFF               0
#define ON                1
#define TOGGLE            2
#define ADJUSTMENT_FACTOR 5
#define RESTART           100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
CANobject *CAN_Message1;

/* USER CODE END Variables */
/* Definitions for Controller */
//osThreadId_t ControllerHandle;
const osThreadAttr_t Controller_attributes = {
  .name = "Controller",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 300 * 4
};
/* Definitions for led_green */
//osThreadId_t led_greenHandle;
const osThreadAttr_t led_green_attributes = {
  .name = "led_green",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ultra_sensor_tr */
//osThreadId_t ultra_sensor_trHandle;
const osThreadAttr_t ultra_sensor_tr_attributes = {
  .name = "ultra_sensor_tr",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for led_yellow */
//osThreadId_t led_yellowHandle;
const osThreadAttr_t led_yellow_attributes = {
  .name = "led_yellow",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for led_red */
//osThreadId_t led_redHandle;
const osThreadAttr_t led_red_attributes = {
  .name = "led_red",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for CAN_Rx_Ctrlr */
//osThreadId_t CAN_Rx_CtrlrHandle;
const osThreadAttr_t CAN_Rx_Ctrlr_attributes = {
  .name = "CAN_Rx_Ctrlr",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 300 * 4
};
/* Definitions for CAN_Tx_Ctrlr */
//osThreadId_t CAN_Tx_CtrlrHandle;
const osThreadAttr_t CAN_Tx_Ctrlr_attributes = {
  .name = "CAN_Tx_Ctrlr",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
uint8_t number_to_byte_arr(uint8_t *byte_array, uint32_t number_to_convert);

/* USER CODE END FunctionPrototypes */

void Controller_handler(void *argument);
void led_green_handler(void *argument);
void Start_ultra_sensor_tr(void *argument);
void led_yellow_handler(void *argument);
void led_red_handler(void *argument);
void CAN_Rx_Ctrlr_handler(void *argument);
void CAN_Tx_Ctrlr_handler(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Controller */
  ControllerHandle = osThreadNew(Controller_handler, NULL, &Controller_attributes);

  /* creation of led_green */
  led_greenHandle = osThreadNew(led_green_handler, NULL, &led_green_attributes);

  /* creation of ultra_sensor_tr */
  ultra_sensor_trHandle = osThreadNew(Start_ultra_sensor_tr, NULL, &ultra_sensor_tr_attributes);

  /* creation of led_yellow */
  led_yellowHandle = osThreadNew(led_yellow_handler, NULL, &led_yellow_attributes);

  /* creation of led_red */
  led_redHandle = osThreadNew(led_red_handler, NULL, &led_red_attributes);

  /* creation of CAN_Rx_Ctrlr */
  CAN_Rx_CtrlrHandle = osThreadNew(CAN_Rx_Ctrlr_handler, NULL, &CAN_Rx_Ctrlr_attributes);

  /* creation of CAN_Tx_Ctrlr */
  CAN_Tx_CtrlrHandle = osThreadNew(CAN_Tx_Ctrlr_handler, NULL, &CAN_Tx_Ctrlr_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Controller_handler */
/**
  * @brief  Function implementing the Controller thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Controller_handler */
void Controller_handler(void *argument)
{
  /* USER CODE BEGIN Controller_handler */
  uint32_t timer_ticks = 0;
  uint8_t reversed_array[8], reversed_array_size = 0, reversed_array_elem = 0;
  float distance;
  char string_number[20];
  SM_STATES state = INIT;
  BaseType_t status;
  mBuff debounce;

  /* Infinite loop */
  for(;;)
  {
	  /* USER CODE BEGIN SM_Controller */
	  switch(state)
	  {
		  case INIT:   /*Initialize the State Machine*/
			           Prepare_CANFilter();
			           CAN_Message1 = GetCANMessage(0x322);                                     //Creates a CAN Message object.
			           CAN_Start();

			           distance_thresholds.danger = 10;
			           distance_thresholds.warning = 20;

			           xTaskNotify((TaskHandle_t)led_greenHandle, ON, eSetValueWithOverwrite);   //Sets GREEN led to OFF
			  	  	   xTaskNotify((TaskHandle_t)led_yellowHandle, OFF, eSetValueWithOverwrite); //Sets YELLOW led to OFF
			  	  	   xTaskNotify((TaskHandle_t)led_redHandle, OFF, eSetValueWithOverwrite);    //Sets RED led to OFF
			  	  	   debounce.downcounter = RESTART;                                           //Sets the debounce counter to initial value.
			  	  	   debounce.enable = OFF;                                                    //Disables debounce logic.

			  	  	   state = MAIN;
					   break;

		  case MAIN:   /*MAIN*/
					   status = xTaskNotifyWait(0, 0, &timer_ticks, pdMS_TO_TICKS(20));

					   if(status == pdPASS)
					   {
						   if(debounce.enable == ON)
						   {
							   timer_ticks = debounce.ticks;                                     //Uses the debounce ticks to prevent bouncing.
							   debounce.downcounter--;
						   }

						  distance = (SPEED_OF_SOUND * TIMER_PERIOD * timer_ticks)/2;            //Divided by 2 because sound goes from the sensor to object
						  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  	 //and back to the sensor.

						  sprintf(string_number, "%0.3f", distance);
						  print_to_console(string_number);

						  /*This case will trigger the DANGER ZONE indicator (RED LED)*/
						  if(distance >= 0 && distance <= distance_thresholds.danger)
						  {
							   xTaskNotify((TaskHandle_t)led_redHandle, TOGGLE, eSetValueWithOverwrite);
							   xTaskNotify((TaskHandle_t)led_greenHandle, OFF, eSetValueWithOverwrite);
							   xTaskNotify((TaskHandle_t)led_yellowHandle, OFF, eSetValueWithOverwrite);

							   if(debounce.downcounter == 0)
							   {
								   debounce.enable = OFF;              //Once debounce memory buffer finish offload. Disable debounce mem buffer.
								   debounce.downcounter = RESTART;
							   }
							   else
							   {
								   debounce.enable = ON;               //Enables debounce memory buffer if previous cycle was OK(GREEN LED).
								   debounce.ticks = timer_ticks;       //Holder for ticks. Will be used for the upcoming cycles.
							   }

							   CAN_Message1->Tx_Payload[7] = 0x02;     //Danger signal is BIT #2.
						  }
						  /*This case will trigger the WARNING ZONE indicator (YELLOW LED)*/
						  else if(distance > distance_thresholds.danger && distance <= distance_thresholds.warning)
						  {
								xTaskNotify((TaskHandle_t)led_yellowHandle, ON, eSetValueWithOverwrite);
								xTaskNotify((TaskHandle_t)led_greenHandle, OFF, eSetValueWithOverwrite);
							    xTaskNotify((TaskHandle_t)led_redHandle, OFF, eSetValueWithOverwrite);

								if(debounce.downcounter == 0)          //Same above logic applies to the WARNING case.
								{
									debounce.enable = OFF;
									debounce.downcounter = RESTART;
								}
								else
								{
									debounce.enable = ON;
									debounce.ticks = timer_ticks;
								}

								CAN_Message1->Tx_Payload[7] = 0x01;    //Warning signal is BIT #1.
						  }
						  /*This case will trigger the OK ZONE indicator (GREEN LED)*/
						  else
						  {
							  xTaskNotify((TaskHandle_t)led_yellowHandle, OFF, eSetValueWithOverwrite);
							  xTaskNotify((TaskHandle_t)led_redHandle, OFF, eSetValueWithOverwrite);
							  xTaskNotify((TaskHandle_t)led_greenHandle, ON, eSetValueWithOverwrite);

							  CAN_Message1->Tx_Payload[7] = 0x00;       //No Warning or Danger signals.
						  }

						  reversed_array_size = number_to_byte_arr(reversed_array, (uint8_t)distance);
						  reversed_array_elem = reversed_array_size;

						  /*Takes the values from the reversed array and populates the array that will be sent via CAN*/
						  for(uint8_t counter = 0; counter <= reversed_array_size; counter++)
						  {
							  CAN_Message1->Tx_Payload[counter] = reversed_array[reversed_array_elem];
							  reversed_array_elem--;
						  }

						  reversed_array_size = 0;

						  xTaskNotify((TaskHandle_t)CAN_Tx_CtrlrHandle, 0, eNoAction);
						  //HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_Message1->TxHeader, CAN_Message1->Tx_Payload); //Sends the distance to the CAN network.
					   }
					   else
						   print_to_console("Waiting for sensor data...");

					   break;

		  case ERR_h:  /*Case to handle errors prior calling error_handler()*/
					   HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, ON);
					   HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, ON);
					   HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, ON);

					   Error_Handler();
					   break;

		  default:     break;
	  }
	  /* USER CODE END SM_Controller */

   }
  /* USER CODE END Controller_handler */
}

/* USER CODE BEGIN Header_led_green_handler */
/**
* @brief Function implementing the led_green thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_green_handler */
void led_green_handler(void *argument)
{
  /* USER CODE BEGIN led_green_handler */
  uint32_t flag;
  BaseType_t status;

  /* Infinite loop */
  for(;;)
  {
	  //HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	  status = xTaskNotifyWait(0, 0, &flag, pdMS_TO_TICKS(10));

	  if(status == pdPASS)
	  {
		  switch(flag)
		  {
			  case ON:     HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, ON);   break;
			  case OFF:    HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, OFF);  break;
			  case TOGGLE: HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);      break;
			  default:                                                                  break;
		  }
	  }
  }
  /* USER CODE END led_green_handler */
}

/* USER CODE BEGIN Header_Start_ultra_sensor_tr */
/**
* @brief Function implementing the ultra_sensor_tr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ultra_sensor_tr */
void Start_ultra_sensor_tr(void *argument)
{
  /* USER CODE BEGIN Start_ultra_sensor_tr */
  TickType_t xLastWakeTime;
  HAL_TIM_Base_Start(&htim8);

  xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(ULTR_TRG_GPIO_Port, ULTR_TRG_Pin, GPIO_PIN_SET);
	  delay_us(100);  //The HC-SR10 needs a delay of 10 us as a minimum for the TRIG to happen.
	  HAL_GPIO_WritePin(ULTR_TRG_GPIO_Port, ULTR_TRG_Pin, GPIO_PIN_RESET);

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
  }
  /* USER CODE END Start_ultra_sensor_tr */
}

/* USER CODE BEGIN Header_led_yellow_handler */
/**
* @brief Function implementing the led_yellow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_yellow_handler */
void led_yellow_handler(void *argument)
{
  /* USER CODE BEGIN led_yellow_handler */
  uint32_t flag;
  BaseType_t status;

  /* Infinite loop */
  for(;;)
  {
	  status = xTaskNotifyWait(0, 0, &flag, pdMS_TO_TICKS(10));

	  if(status == pdPASS)
	  {
		  switch(flag)
		  {
			  case ON:     HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, ON);  break;
			  case OFF:    HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, OFF); break;
			  case TOGGLE: HAL_GPIO_TogglePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin);     break;
			  default:                                                                   break;
		  }
	  }
  }
  /* USER CODE END led_yellow_handler */
}

/* USER CODE BEGIN Header_led_red_handler */
/**
* @brief Function implementing the led_red thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_red_handler */
void led_red_handler(void *argument)
{
  /* USER CODE BEGIN led_red_handler */
  uint32_t flag;
  BaseType_t status;

  /* Infinite loop */
  for(;;)
  {
	  status = xTaskNotifyWait(0, 0, &flag, pdMS_TO_TICKS(10));

	  if(status == pdTRUE)
	  {
		  switch(flag)
		  {
			  case ON:     HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, ON);  break;
			  case OFF:    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, OFF); break;
			  case TOGGLE: HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
			               vTaskDelay(pdMS_TO_TICKS(100));
						   break;
			  default:     break;
		  }
	  }
  }
  /* USER CODE END led_red_handler */
}

/* USER CODE BEGIN Header_CAN_Rx_Ctrlr_handler */
/**
* @brief Function implementing the CAN_Rx_Ctrlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Rx_Ctrlr_handler */
void CAN_Rx_Ctrlr_handler(void *argument)
{
  /* USER CODE BEGIN CAN_Rx_Ctrlr_handler */
  BaseType_t status;
  CANobject *CAN_MessageRx;

  CAN_MessageRx = GetCANMessage(0x72E);                                                          //Creates a CAN Message object.

  /* Infinite loop */
  for(;;)
  {
	  status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(20));
	  if(status == pdPASS)
	  {

		  if(CAN_MSG_Received.Rx_Payload[1] == 0x22 && CAN_MSG_Received.Rx_Payload[2] == 0xFE)
		  {
			  switch(CAN_MSG_Received.Rx_Payload[3])
			  {
			       case 0x01: /*CONFIGURATION CASE: Update Thresholds*/
			    	          distance_thresholds.danger  = CAN_MSG_Received.Rx_Payload[4];      //Sets the new danger threshold value.
			       	   	   	  distance_thresholds.warning = CAN_MSG_Received.Rx_Payload[5];      //Sets the new warning threshold value.

			       	   	      CAN_MessageRx->Tx_Payload[0] = 0x07;                               //Sets the 0x762 header for response part 1.
			       	   	      CAN_MessageRx->Tx_Payload[1] = 0x62;								 //Sets the 0x762 header for response part 2.
			       	   	      CAN_MessageRx->Tx_Payload[2] = 0xFE;                               //Sets the DID number.
			       	   	      CAN_MessageRx->Tx_Payload[3] = 0x01;                               //Sets the DID configuration case.
			       	   	      CAN_MessageRx->Tx_Payload[4] = CAN_MSG_Received.Rx_Payload[4];     //Sets the acknowledge danger threshold.
			       	   	      CAN_MessageRx->Tx_Payload[5] = CAN_MSG_Received.Rx_Payload[5];     //Sets the acknowledge warning threshold.

			       	   	   	  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_MessageRx->TxHeader, CAN_MessageRx->Tx_Payload); //Sends positive acknowledgment.

			    	          break;

			       case 0x02: /*CONFIGURATION CASE: Distance notification mode */
			    	          if(CAN_MSG_Received.Rx_Payload[4] == 0x01)                         //Continuous mode
			    	          {

			    	          }
			    	          break;

			       default:   CAN_MessageRx->Tx_Payload[0] = 0x07;                               //Sets the 0x762 header for response part 1.
					          CAN_MessageRx->Tx_Payload[1] = 0x62;								 //Sets the 0x762 header for response part 2.
					          CAN_MessageRx->Tx_Payload[2] = 0x7F;								 //Negative Response.

					          HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_MessageRx->TxHeader, CAN_MessageRx->Tx_Payload); //Sends negative response.

					          break;
			  }
		  }
		  else
		  {
			  CAN_MessageRx->Tx_Payload[0] = 0x07;                               //Sets the 0x762 header for response part 1.
			  CAN_MessageRx->Tx_Payload[1] = 0x62;								 //Sets the 0x762 header for response part 2.
			  CAN_MessageRx->Tx_Payload[2] = 0x7F;								 //Negative Response.

			  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_MessageRx->TxHeader, CAN_MessageRx->Tx_Payload); //Sends negative response.
		  }

	  }

  }
  /* USER CODE END CAN_Rx_Ctrlr_handler */
}

/* USER CODE BEGIN Header_CAN_Tx_Ctrlr_handler */
/**
* @brief Function implementing the CAN_Tx_Ctrlr thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Tx_Ctrlr_handler */
void CAN_Tx_Ctrlr_handler(void *argument)
{
  /* USER CODE BEGIN CAN_Tx_Ctrlr_handler */
  BaseType_t status;

  /* Infinite loop */
  for(;;)
  {
	  status = xTaskNotifyWait(0, 0, NULL, pdMS_TO_TICKS(20));
	  if(status == pdPASS)
	  {
		  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_Message1->TxHeader, CAN_Message1->Tx_Payload); //Sends the distance to the CAN network.
	  }

  }
  /* USER CODE END CAN_Tx_Ctrlr_handler */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
* @brief Function that converts a number to a byte array.
* @param argument: Array address, number to convert
* @retval Number or array elements
*/
uint8_t number_to_byte_arr(uint8_t *byte_array, uint32_t number_to_convert)
{
	uint8_t arr_index = 0;

	while(number_to_convert > 0)
	{
		byte_array[arr_index] = number_to_convert & 0xFF;
	    number_to_convert >>= 8;
	    arr_index++;
	}

	return arr_index - 1;  //Decrease 1 unit due to zero indexing.
}
/* USER CODE END Application */

