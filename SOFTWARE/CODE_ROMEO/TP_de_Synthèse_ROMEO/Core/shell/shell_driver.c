/*
 * shell_driver.c
 *
 *  Created on: Nov 22, 2025
 *      Author: lorenzo
 */


#include "shell_driver.h"
#include "shell.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

void shell_create(shell_t *sh, UART_HandleTypeDef *huart) {
    sh->huart = huart;
    sh->pos   = 0;

    // Démarre la réception IT
    HAL_UART_Receive_IT(huart, &sh->rx_char, 1);
}

void shell_task(void *argument) {
    shell_t *sh = (shell_t*)argument;
    const char prompt[] = "\r\n> ";
    const char backspace[] = "\b \b";

    // Prompt initial
    HAL_UART_Transmit(sh->huart, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);

    for(;;)
    {
        uint8_t c;
        if (xQueueReceive(sh->rx_queue, &c, portMAX_DELAY) == pdPASS)
        {
            switch(c)
            {
                case '\r':
                case '\n':
                    HAL_UART_Transmit(sh->huart, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
                    sh->cmd_buffer[sh->pos] = '\0';
                    printf(":%s\r\n", sh->cmd_buffer);
                    if (sh->pos > 0) shell_exec(sh->cmd_buffer);
                    HAL_UART_Transmit(sh->huart, (uint8_t*)prompt, strlen(prompt), HAL_MAX_DELAY);
                    sh->pos = 0;
                    break;

                case '\b':
                case 0x7F:
                    if (sh->pos > 0) {
                        sh->pos--;
                        HAL_UART_Transmit(sh->huart, (uint8_t*)backspace, 3, HAL_MAX_DELAY);
                    }
                    break;

                default:
                    if (sh->pos < SHELL_BUFFER_SIZE-1 && c >= 32 && c <= 126) {
                        HAL_UART_Transmit(sh->huart, &c, 1, HAL_MAX_DELAY);
                        sh->cmd_buffer[sh->pos++] = c;
                    }
                    break;
            }
        }
    }
}
