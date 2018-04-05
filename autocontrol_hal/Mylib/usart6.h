#pragma once
#ifndef __USART6_H__
#define __USART6_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include <string.h>
#include <stdarg.h>

#define Grab_Infrared_I 0
#define Ultrasonic_H	1
#define Ultrasonic_L	2
#define Car_Infrared_L	3
#define Car_Infrared_M	4
#define Car_Infrared_R	5
#define Grab_Infrared_L 6
#define Grab_Infrared_R 7


	void usart6_init(void);
	void usart6_send(const char *b);
	void UART6_Handler(void);
	void Send_To_Sensor();

#ifdef __cplusplus
}
#endif

#endif
