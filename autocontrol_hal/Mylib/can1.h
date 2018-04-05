#ifndef __CAN1_H__
#define __CAN1_H__

#ifdef __cplusplus
extern "C" {
#endif
	 
#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_hal.h"              // Keil::Device:STM32Cube HAL:Common
#include "system_stm32f4xx.h"  

#define CAN1_FREQ 1000000
#define CAN1_Prescaler (SystemCoreClock / 4 / CAN1_FREQ)
#define ALL_KP 25
#define ALL_KI 0.2
#define ALL_KD 5
	typedef struct 
	{
		short id;
		short Iset;
		short Ireal;
		char hall;
		short angle;
		short anglelast;
		short speed;
		short speedtemp;
		short speedlast;
		short speedreal;
		short speedcount;
		short count;
		int position;
	}PTZmoto_TypeDef;

HAL_StatusTypeDef can1_init(void);
void can1_out();
void CAN1_RX0_Handler();

#ifdef __cplusplus
}
#endif

#endif