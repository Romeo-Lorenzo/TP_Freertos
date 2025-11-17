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

#include "shell.h"
#include "drv_uart2.h"

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
h_shell_t h_shell;
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
	printf("freertos init\n\r");

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
	//	int fonction(h_shell_t * h_shell, int argc, char ** argv)
	//	{
	//		int size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "Je suis une fonction bidon\r\n");
	//		h_shell->drv.transmit(h_shell->print_buffer, size);
	//
	//		return 0;
	//	}
	//
	//	h_shell.drv.receive = drv_uart2_receive;
	//	h_shell.drv.transmit = drv_uart2_transmit;
	//
	//	shell_init(&h_shell);
	//	shell_add(&h_shell, 'f', fonction, "Une fonction inutile");

	h_shell_t *h_shell = (h_shell_t *)pvParameters;  // Récupère l'instance shell
	shell_driver_t *drv = &h_shell->drv;  // Si tu as déjà drv, sinon adapte

	const char backspace[] = "\b \b";
	const char prompt[] = "> ";

	/* Infinite loop */
	for(;;)
	{
		// Affiche le prompt
		h_shell->drv.transmit((uint8_t *)prompt, 2);

		int reading = 1;
		int pos = 0;
		h_shell->cmd_buffer[0] = '\0';  // Reset buffer

		while (reading) {
			char c;

			// NOUVEAU : Attente non-bloquante via queue FreeRTOS
			// portMAX_DELAY = bloque indéfiniment, mais proprement (0 CPU)
			if (xQueueReceive(h_shell->rx_queue, &c, portMAX_DELAY) == pdPASS) {

				int size;
				switch (c) {
				// Retour chariot
				case '\r':
				case '\n':
					size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "\r\n");
					h_shell->drv.transmit((uint8_t *)h_shell->print_buffer, size);

					h_shell->cmd_buffer[pos] = '\0';  // Termine la string

					// Debug : affiche la commande
					size = snprintf(h_shell->print_buffer, BUFFER_SIZE, "Exec: %s\r\n", h_shell->cmd_buffer);
					h_shell->drv.transmit((uint8_t *)h_shell->print_buffer, size);

					// Exécute la commande
					shell_exec(h_shell, h_shell->cmd_buffer);

					reading = 0;  // Sort de la boucle de lecture
					break;

					// Backspace
				case '\b':
				case 0x7F:  // DEL aussi
					if (pos > 0) {
						pos--;
						h_shell->drv.transmit((uint8_t *)backspace, 3);
					}
					break;

					// Caractères normaux
				default:
					if (pos < BUFFER_SIZE - 1) {
						h_shell->drv.transmit(&c, 1);
						h_shell->cmd_buffer[pos++] = c;
					}
					break;
				}
			}
		}
		pos = 0;  // Reset pour prochaine ligne
	}
	/* USER CODE END StartShellTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
