/*
 * shell.h
 *
 *  Created on: 7 juin 2019
 *      Author: laurent
 */

#ifndef INC_LIB_SHELL_SHELL_H_
#define INC_LIB_SHELL_SHELL_H_

#include <stdint.h>
#include "usart.h"

#define UART_DEVICE huart1

#define ARGC_MAX 8
#define BUFFER_SIZE 40
#define SHELL_FUNC_LIST_MAX_SIZE 64

struct h_shell_struct;

typedef int (*shell_func_pointer_t)(struct h_shell_struct * h_shell, int argc, char ** argv);

typedef struct shell_func_struct
{
	char c;
	shell_func_pointer_t func;
	char * description;
} shell_func_t;

typedef struct drv_shell_struct
{
	UART_HandleTypeDef uart;
	void (*transmit)(char * buf, uint16_t size);
	void (*receive)(uint8_t * carac);
} drv_shell_t;

typedef struct h_shell_struct
{
	int func_list_size;
	shell_func_t func_list[SHELL_FUNC_LIST_MAX_SIZE];
	char print_buffer[BUFFER_SIZE];
	char cmd_buffer[BUFFER_SIZE];
	drv_shell_t drv;
	uint8_t receive_car;
} h_shell_t;

void shell_init(h_shell_t * h_shell);
int shell_add(h_shell_t * h_shell, char c, shell_func_pointer_t pfunc, char * description);
int shell_run(h_shell_t * h_shell);



#endif /* INC_LIB_SHELL_SHELL_H_ */
