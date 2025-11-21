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
#include <stdio.h>
#include "usart.h"
#include "shell.h"
#include "drv_uart2.h"
#include "queue.h"
#include <string.h>
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
QueueHandle_t uart_rx_queue;
uint8_t rx_char;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId shellTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartShellTask(void const * argument);

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
	//	printf("freertos init\n\r");

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
	uart_rx_queue = xQueueCreate(128, sizeof(uint8_t));
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of shellTask */
	osThreadDef(shellTask, StartShellTask, osPriorityIdle, 0, 128);
	shellTaskHandle = osThreadCreate(osThread(shellTask), NULL);

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
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//		printf("Led toggled\n\r");
		osDelay(1000);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartShellTask */
/**
 * @brief Function implementing the shellTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartShellTask */
void StartShellTask(void const * argument)
{
	/* USER CODE BEGIN StartShellTask */

	HAL_UART_Receive_IT(&huart2, &rx_char, 1);


	int fonction(int argc, char **argv)
	{
		printf("Je suis une fonction bidon\r\n");
		return 0;
	}

	shell_init();
	shell_add('f', fonction, "Une fonction inutile");

	//	printf("\r\n=== Shell en mode interruption (FreeRTOS + IT) ===\r\n> ");

	static char cmd_buffer[BUFFER_SIZE];
	int pos = 0;
	const char backspace[] = "\b \b";
	const char prompt[] = "> ";

	// Premier prompt
	HAL_UART_Transmit(&huart2, (uint8_t*)prompt, strlen(prompt), 100);

	/* Infinite loop */
	for(;;)
	{
		char c;
		if (xQueueReceive(uart_rx_queue, &c, portMAX_DELAY) == pdPASS)
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)&c, 1, HAL_MAX_DELAY);
			switch(c)
			{
			case '\r':
			case '\n':
				printf("\r\n");                                     // \r\n
				cmd_buffer[pos] = '\0';

				printf(":%s\r\n", cmd_buffer);                       // Affiche la commande reçue (comme l'original)

				if (pos > 0)
				{
					shell_exec(cmd_buffer);                         // Exécution !
				}

				printf("%s", prompt);
				fflush(stdout);
				pos = 0;
				break;

			case '\b':
			case 0x7F:  // Delete
				if (pos > 0)
				{
					pos--;
					printf("%s", backspace);                        // Efface sur le terminal
				}
				break;

			default:
				if (pos < BUFFER_SIZE - 1 && c >= 32 && c <= 126)
				{
					putchar(c);                                     // Echo propre
					cmd_buffer[pos++] = c;
				}
				break;
			}
		}
	}
	/* USER CODE END StartShellTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
