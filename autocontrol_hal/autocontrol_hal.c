/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "os_task.h"
#include "uart4.h"
#include "usart2.h"
#include "usart6.h"
#include "usart3.h"
#include "tim3.h"
#include "tim4.h"
#include "button.h"
#include "can2.h"
#include "can1.h"
#include "rng.h"
#include "oled.h"
#include <math.h>
#include "arm_math.h"
#include "mymath.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/

void setSystemClock(void);

short speedset[4] = { 0 };
int Position_real[3] = { 0 };
extern PLACE_TYPE Place_real[3];
short accdata[4] = { 50, 50, 50, 50 };

extern button_type button_state;
//short candata[4] = { 0 ,0,0,0 };
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


int RNG_DATA;
//int T_TEST;

int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
	setSystemClock();
	SystemCoreClockUpdate();
	HAL_Init();  
	
	
	usart3_init();
	uart4_init();
	usart6_init();
	tim3_init();
	tim4_init();
	button_init();
	can2_init();
	can1_init();
	rng_init();
	OLED_Configuration();
	usart2_init();

	//float a;
	//float b = 3.364786219364798126;
	//float c = 733.7312847980125892;

	//T_TEST = HAL_GetTick();

	//for (int i=0;i<1000000;i++)
	//{
	//	a = 1/Q_rsqrt(b);
	//}
	//T_TEST = HAL_GetTick() - T_TEST-84;
	
	

	__GPIOA_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);

	os_task_init();
	os_task_start();

	for (;;)
	{
		
	}
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	if (huart->Instance == USART2)
	{
		UART2_Handler();
	}
	else if (huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 1);
		UART3_Handler();
	}
	else if (huart->Instance == UART4)
	{
		HAL_UART_Receive_IT(huart, huart->pRxBuffPtr, 1);
		UART4_Handler();
	}
	else if (huart->Instance == USART6)
	{
		UART6_Handler();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //5ms
{
	if (htim->Instance == TIM3)
	{
		TIM3_Handler();
	}
	else if (htim->Instance == TIM4)
	{
		TIM4_Handler();
	}
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan->Instance == CAN1)
	{
		CAN1_RX0_Handler();
	}
	if (hcan->Instance == CAN2)
	{
		CAN2_RX0_Handler();
	}

}

void Error_Handler(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	while (1)
	{
		
	}
}



void setSystemClock(void)
{
/******************************************************************************/
/*            PLL (clocked by HSE) used as System clock source                */
/*            System clock source = HSE / PLL_M *  PLL_N / PLL_P              */
/******************************************************************************/
	
#ifndef HSE_STARTUP_TIMEOUT
	uint32_t HSE_STARTUP_TIMEOUT = 1000;
#endif // !HSE_STARTUP_TIMEOUT

	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;
	uint16_t	PLL_M = 25,
				PLL_N = 336,
				PLL_P = 2,
				PLL_Q = 7,
				PLLI2S_N = 302,
				PLLI2S_R = 2;
	
	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
 
    /* Wait till HSE is ready and if Time out is reached exit */
	do
	{
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while (HSEStatus == 0);// && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		HSEStatus = (uint32_t)0x01;
	}
	else
	{
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01)
	{
	  /* Select regulator voltage output Scale 1 mode, System frequency up to 168 MHz */
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		PWR->CR |= PWR_CR_VOS;

		    /* HCLK = SYSCLK / 1*/
		RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      
		/* PCLK2 = HCLK / 2*/
		RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    
		/* PCLK1 = HCLK / 4*/
		RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

		    /* Configure the main PLL */
		RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) - 1) << 16) |
		               (RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
		
		RCC->PLLI2SCFGR = (PLLI2S_R << 28) + (PLLI2S_N << 6);

    /* Enable the main PLL and PLLI2S*/
		RCC->CR |= RCC_CR_PLLON | RCC_CR_PLLI2SON;

		    /* Wait till the main PLL is ready */
		while ((RCC->CR & RCC_CR_PLLRDY) == 0)
		{
		}
   
	    /* Configure Flash prefetch, Instruction cache, Data cache and wait state */
		FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;

		    /* Select the main PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;

		    /* Wait till the main PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
			;
		{
		}
	}
	else
	{ /* If HSE fails to start-up, the application will have wrong clock
	       configuration. User can add here some code to deal with this error */
	}

}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
