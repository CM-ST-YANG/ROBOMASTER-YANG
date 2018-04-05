#pragma once
#ifndef __UART4_H__
#define __UART4_H__

#ifdef __cplusplus
extern "C" {
#endif
	 
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>

#define Stepmoto_Data_Len 8

#define ERROR		0
#define OK			1
#define READY		2
#define BUSY		3
#define TIME_OUT	4

#define SPEED_CTRL		0x00
#define POSITION_CTRL	0x01
#define CMD_CTRL		0x03

	void uart4_init(void);
	void uart4_send(const char *b);
	uint8_t Give_cmd_to_stepmoto(uint8_t time_out);
	void UART4_Handler(void);

	uint8_t Get_Ready_Status(uint8_t type);
	void Clear_Ready_Status(uint8_t type);

	uint8_t Send_Speed_to_stepmoto(uint8_t grab, uint8_t up_down, uint8_t  forward_backward);
	uint8_t Send_Position_to_stepmoto(uint8_t grab, uint8_t up_down, uint8_t  forward_backward);
	uint8_t Send_Cmd_to_stepmoto(uint8_t cmdtype);

	 
#ifdef __cplusplus
}
#endif
	 
#endif
