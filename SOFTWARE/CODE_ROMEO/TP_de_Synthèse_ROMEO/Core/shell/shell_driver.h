/*
 * shell_driver.h
 *
 *  Created on: Nov 22, 2025
 *      Author: lorenzo
 */

#ifndef SHELL_SHELL_DRIVER_H_
#define SHELL_SHELL_DRIVER_H_

#include "usart.h"
#include "shell.h"
#include <stdint.h>

#define SHELL_BUFFER_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             rx_char;
    char                cmd_buffer[SHELL_BUFFER_SIZE];
    int                 pos;
} shell_t;

void shell_create(shell_t *sh, UART_HandleTypeDef *huart);
void shell_task(void *argument);   // la tâche FreeRTOS générique

#endif /* SHELL_SHELL_DRIVER_H_ */
