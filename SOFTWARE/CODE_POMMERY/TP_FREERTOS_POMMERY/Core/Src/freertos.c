/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include "shell.h"
#include "MCP23S17.h"
#include "sgtl5000.h"
#include "RCfilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern h_shell_t shellstruct;
QueueHandle_t uartRxQueue;



extern int16_t i2s_rx_buf[AUDIO_BUF_BYTES];
extern int16_t i2s_tx_buf[AUDIO_BUF_BYTES];

extern volatile uint8_t rx_half_flag;
extern volatile uint8_t rx_full_flag;

extern SAI_HandleTypeDef hsai_BlockA2;  // TX
extern SAI_HandleTypeDef hsai_BlockB2;  // RX

extern h_RC_filter_t filter_struct;

uint32_t tps_filter=0;


int tremolo = 0;
int trem_step = 20;
int trem_min = 4000;
int trem_max = 32767;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId SecondTaskHandle;
osThreadId ShellTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int fonction(h_shell_t * h_shell, int argc, char ** argv)
{
	int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Je suis une fonction bidon\r\n");
	h_shell->drv.transmit(h_shell->print_buffer, size);

	return 0;
}


int fonction_led(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc==2){
		int lednum=atoi(argv[0]);
		int ledstate=atoi(argv[1]);
		MCP23S17_SetPin(lednum,ledstate );
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "LED n°%d\r\n",lednum);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}
	else if(argc==3){
		int lednum=atoi(argv[0])*10+atoi(argv[1]);
		int ledstate=atoi(argv[2]);
		MCP23S17_SetPin(lednum,ledstate );
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "LED n°%d\r\n",lednum);
			h_shell->drv.transmit(h_shell->print_buffer, size);

	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}



	return 1;
}

int fonction_filter(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc>0){
		int freq=atoi(argv[1]);
		RC_filter_init(&filter_struct,(uint16_t ) freq,(uint16_t ) SAMPLING_FREQ);
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "filter cutoff freq at %d\r\n",freq);
		h_shell->drv.transmit(h_shell->print_buffer, size);
	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}



	return 1;
}

int fonction_tremolo(h_shell_t * h_shell, int argc, char ** argv)
{
	if(argc>0){
		if(atoi(argv[1])==1){
			tremolo=0;
			trem_step=20;
			int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "tremolo enabled\r\n");
			h_shell->drv.transmit(h_shell->print_buffer, size);

		}
		else{
			tremolo=32767;
			trem_step=0;
			int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "tremolo disabled\r\n");
			h_shell->drv.transmit(h_shell->print_buffer, size);

		}

	}
	else{
		int size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "mauvais arg\r\n");
		h_shell->drv.transmit(h_shell->print_buffer, size);
		return 0;
	}



	return 1;
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void Startshelltask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
	uartRxQueue = xQueueCreate(128, sizeof(uint8_t));



  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of SecondTask */
  osThreadDef(SecondTask, StartTask02, osPriorityNormal, 0, 256);
  SecondTaskHandle = osThreadCreate(osThread(SecondTask), NULL);

  /* definition and creation of ShellTask */
  osThreadDef(ShellTask, Startshelltask, osPriorityNormal, 0, 256);
  ShellTaskHandle = osThreadCreate(osThread(ShellTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	  VU_UpdateFromBuffer((int16_t *)i2s_rx_buf,AUDIO_BUF_BYTES);
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the SecondTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
    HAL_SAI_Receive_DMA(&hsai_BlockB2,(int16_t *)i2s_rx_buf,AUDIO_BUF_BYTES);

    HAL_SAI_Transmit_DMA(&hsai_BlockA2,(int16_t *)i2s_tx_buf,AUDIO_BUF_BYTES);
    uint32_t t0=0;
  /* Infinite loop */
  for(;;)
  {
	  if (rx_half_flag)
	  {
	      rx_half_flag = 0;

	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          int16_t x = (int16_t)i2s_rx_buf[i];

	          tremolo += trem_step;
	          if (tremolo >= trem_max) { tremolo = trem_max; trem_step = -trem_step; }
	          if (tremolo <= trem_min) { tremolo = trem_min; trem_step = -trem_step; }

	          int32_t y = ((int32_t)x * (int32_t)tremolo) / 32767;

	          if (y > 32767) y = 32767;
	          if (y < -32768) y = -32768;

	          i2s_tx_buf[i] = RC_filter_update(&filter_struct, (uint16_t)(int16_t)y);
	      }

	  }

	  if (rx_full_flag)
	  {
	      rx_full_flag = 0;

	      t0 = DWT->CYCCNT;

	      for (int i = 0; i < AUDIO_HALF_BYTES; i++)
	      {
	          int idx = i + AUDIO_HALF_BYTES;
	          int16_t x = (int16_t)i2s_rx_buf[idx];

	          tremolo += trem_step;
	          if (tremolo >= trem_max) { tremolo = trem_max; trem_step = -trem_step; }
	          if (tremolo <= trem_min) { tremolo = trem_min; trem_step = -trem_step; }

	          int32_t y = ((int32_t)x * (int32_t)tremolo) / 32767;

	          if (y > 32767) y = 32767;
	          if (y < -32768) y = -32768;

	          i2s_tx_buf[idx] = RC_filter_update(&filter_struct, (uint16_t)(int16_t)y);
	      }

	      tps_filter = DWT->CYCCNT - t0;
	  }

  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_Startshelltask */
/**
* @brief Function implementing the ShellTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startshelltask */
void Startshelltask(void const * argument)
{
  /* USER CODE BEGIN Startshelltask */


	  shell_init(&shellstruct);
	  shell_add(&shellstruct, 'l', fonction_led, "control des led");
	  shell_add(&shellstruct, 'f', fonction_filter, "Control du filtre RC");
	  shell_add(&shellstruct, 't', fonction_tremolo, "Active ou desactive le tremolo");
  /* Infinite loop */
  for(;;)
  {

	shell_run(&shellstruct);
    osDelay(100);
  }
  /* USER CODE END Startshelltask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
