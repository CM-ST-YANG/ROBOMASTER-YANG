#ifndef __USART2_H__
#define __USART2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "system_stm32f4xx.h"  
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
	/*#include <stdarg.h>*/
#define USE_PRINTF

	typedef struct
	{
		struct
		{
			uint16_t  ch0;
			uint16_t ch1;
			uint16_t ch2;
			uint16_t ch3;
			uint16_t s1;
			uint16_t s2;
		}rc;
		struct
		{
			unsigned int x;
			unsigned int y;
			unsigned int z;
			unsigned int press_l;
			unsigned int press_r;
		}mouse;
		struct
		{
			unsigned int v;
		}key;
	}RC_Ctl_t;
	void usart2_init(void);
	void usart2_send(char *b);
	void usart2_ptint(char *str, int data, int mode);
	void usart2_ptintf(char *str, float data, int len);
	//void _printf(char *fmt, ...);
	void UART2_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
