#include "usart2.h"

UART_HandleTypeDef husart2;
uint8_t receive2[24], transmit2[10];
uint32_t usart2_time = 0;
RC_Ctl_t RC_Ctl;

/**
* @brief 串口2初始化
* @param 无
* @note 无
* @retval 无
*/
void usart2_init(void)
{
	GPIO_InitTypeDef gpio_init;

	HAL_NVIC_SetPriority(USART2_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();



	gpio_init.Alternate = GPIO_AF7_USART2;
	gpio_init.Pin = GPIO_PIN_3;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	husart2.Init.BaudRate = 100000;
	husart2.Init.WordLength = UART_WORDLENGTH_8B;
	husart2.Init.StopBits = UART_STOPBITS_1;
	husart2.Init.Parity = USART_PARITY_NONE;
	husart2.Init.Mode = UART_MODE_RX;
	husart2.Instance = USART2;

	RC_Ctl.rc.ch0 = 1024;
	RC_Ctl.rc.ch1 = 1024;
	RC_Ctl.rc.ch2 = 1024;
	RC_Ctl.rc.ch3 = 1024;

	HAL_UART_Init(&husart2);
	usart2_time = HAL_GetTick();
	HAL_UART_Receive_IT(&husart2, receive2, 18);
}

/**
* @brief 串口2发送字符串
* @param *b：字符串数组指针
* @note 无
* @retval 无
*/
void usart2_send(char *b)
{
	char len = strlen(b);
	if (*(b + len - 1) == 0)
		len--;
	//HAL_USART_Transmit(&husart2, (uint8_t *)b, len, 100);
}

static char buffer[100];
static char stradd[10];

void usart2_ptint(char *str, int data, int mode)
{
#ifdef USE_PRINTF
	usart2_send(str);
	itoa((int)data, buffer, mode);
	usart2_send(buffer);
	usart2_send("\n");
#endif // USE_PRINTF
}

int tenpow(int len)
{
	int data = 1;
	while (len--)
		data = 10 * data;
	return data;
}

void usart2_ptintf(char *str, float data, int len)
{
#ifdef USE_PRINTF

	usart2_send(str);
	itoa((int)data, buffer, 10);
	if (len)
	{
		strncat(buffer, ".", 1);
		itoa((int)((data - (int)data) * tenpow(len)), stradd, 10);
		strncat(buffer, stradd, 10);
	}
	usart2_send(buffer);
	usart2_send("\n");

#endif // USE_PRINTF
}


void USART2_IRQHandler(void)
{
	if (HAL_GetTick() - usart2_time > 5)
	{
		husart2.pRxBuffPtr = receive2;
		husart2.RxXferSize = 18;
		husart2.RxXferCount = 18;
	}
	usart2_time = HAL_GetTick();
	HAL_UART_IRQHandler(&husart2);
}

void UART2_Handler(void)
{
	HAL_UART_Receive_IT(&husart2, receive2, 18);

	RC_Ctl.rc.ch0 = (receive2[0] | (receive2[1] << 8)) & 0x07ff; //!< Channel 0	 
	RC_Ctl.rc.ch1 = ((receive2[1] >> 3) | (receive2[2] << 5)) & 0x07ff; //!< Channel 1
	RC_Ctl.rc.ch2 = ((receive2[2] >> 6) | (receive2[3] << 2) | (receive2[4] << 10)) & 0x07ff; //!< Channel 2  
	RC_Ctl.rc.ch3 = ((receive2[4] >> 1) | (receive2[5] << 7)) & 0x07ff; //!< Channel 3
	RC_Ctl.rc.s1 = ((receive2[5] >> 4) & 0x000C) >> 2; //!< Switch left
	RC_Ctl.rc.s2 = ((receive2[5] >> 4) & 0x0003); 	//!< Switch right							
}

