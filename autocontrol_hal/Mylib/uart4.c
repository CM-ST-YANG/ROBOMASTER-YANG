#include "uart4.h"

UART_HandleTypeDef huart4;
uint8_t receive4[10], transmit4[10];
uint8_t Stepmoto_Data[Stepmoto_Data_Len] = {0};
uint8_t Stepmoto_Status;
uint8_t Stepmoto_Data_Lock = 0;
uint8_t Last_Cmd = 0;
uint8_t Ready_status[5] = {0};

/**
  * @brief 初始化串口4
  * @param none
  * @note none
  * @retval none
  */
void uart4_init(void)
{
	GPIO_InitTypeDef gpio_init;

	HAL_NVIC_SetPriority(UART4_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	
	__HAL_RCC_UART4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	gpio_init.Alternate = GPIO_AF8_UART4;
	gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio_init.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &gpio_init);
	
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = USART_WORDLENGTH_8B;
	huart4.Init.StopBits = USART_STOPBITS_1;
	huart4.Init.Parity = USART_PARITY_NONE;
	huart4.Init.Mode = USART_MODE_TX_RX;
	huart4.Instance = UART4;
	//receive4[0] = READY;
	HAL_UART_Init(&huart4);
	HAL_UART_Receive_IT(&huart4, receive4, 1);
}

/**
  * @brief 串口4发送字符串
  * @param *b：字符串数组指针
  * @note none
  * @retval none
  */
void uart4_send(const char *b)
{
	char len = strlen(b);
	HAL_UART_Transmit(&huart4, (uint8_t *)b, len, len*2);
}

void UART4_IRQHandler(void)
{
	huart4.pRxBuffPtr = receive4;
	HAL_UART_IRQHandler(&huart4);
}

void UART4_Handler(void)
{
	if (receive4[0]==READY)
	{
		Ready_status[Last_Cmd] = 1;
	}
	Stepmoto_Status = receive4[0];
	HAL_UART_Receive_IT(&huart4, huart4.pRxBuffPtr, 1);
}

/////////////////////////////////////////////


uint8_t Get_Ready_Status(uint8_t type)
{
	return Ready_status[type];
}

void Clear_Ready_Status(uint8_t type)
{
	Ready_status[type] = 0;
}


void give_data()
{
	Last_Cmd = Stepmoto_Data[0];
	HAL_UART_Transmit(&huart4, Stepmoto_Data, Stepmoto_Data_Len, Stepmoto_Data_Len*2);
}

void ask_Stepmoto_Status()
{
	uint8_t ask_data = 0xAA;
	HAL_UART_Transmit(&huart4, &ask_data, 1, 2);
}


void wait_a(uint8_t time_out)
{
	int timer;
	//uint8_t time_out = 2;
	timer = HAL_GetTick();

	while (HAL_GetTick() - timer < time_out && Stepmoto_Status != OK)
	{
		Stepmoto_Status = receive4[0];
	}
	//if (HAL_GetTick() - timer >= time_out)
	//{
	//	Stepmoto_Status = TIME_OUT;
	//}
}

void Lock_Data()
{
	Stepmoto_Data_Lock = 1;
}

void UnLock_Data()
{
	Stepmoto_Data_Lock = 0;
}

uint8_t Is_Lock()
{
	return Stepmoto_Data_Lock;
}

void Clear_Data()
{
	for (uint8_t i = 0; i < Stepmoto_Data_Len; i++)
		Stepmoto_Data[i] = 0;
}

uint8_t Give_cmd_to_stepmoto(uint8_t time_out)
{

	Stepmoto_Status = receive4[0];
	if (Stepmoto_Status == READY)
	{
		//receive4[0] = TIME_OUT;
		ask_Stepmoto_Status();
		wait_a(time_out);
		if (Stepmoto_Status != OK && Stepmoto_Status != READY)
		{
			return Stepmoto_Status;
		}
		//receive4[0] = BUSY;
		Stepmoto_Status = BUSY;
		give_data();
		return 1;
		//wait_a();
		//if (Stepmoto_Status != OK)
		//	return;
	}
	return 0;

}

uint8_t Send_Speed_to_stepmoto(uint8_t grab,uint8_t up_down,uint8_t  forward_backward)
{
	if (Is_Lock())
	{
		return 0;
	}
	else
	{
		Lock_Data();
		Clear_Data();
		Stepmoto_Data[0] = SPEED_CTRL;
		Stepmoto_Data[1] = grab;
		Stepmoto_Data[2] = up_down;
		Stepmoto_Data[3] = forward_backward;
		Give_cmd_to_stepmoto(3);
		UnLock_Data();
		return 1;
	}
}

uint8_t Send_Position_to_stepmoto(uint8_t grab, uint8_t up_down, uint8_t  forward_backward)
{
	if (Is_Lock())
	{
		return 0;
	}
	else
	{
		Lock_Data();
		Clear_Data();
		Stepmoto_Data[0] = POSITION_CTRL;
		Stepmoto_Data[1] = grab;
		Stepmoto_Data[2] = up_down;
		Stepmoto_Data[3] = forward_backward;
		Give_cmd_to_stepmoto(3);
		UnLock_Data();
		return 1;
	}
}

uint8_t Send_Cmd_to_stepmoto(uint8_t cmdtype)
{
	uint8_t status = 0;
	if (Is_Lock())
	{
		return 0;
	}
	else
	{
		Lock_Data();
		Clear_Data();
		Stepmoto_Data[0] = CMD_CTRL;
		Stepmoto_Data[1] = cmdtype;
		Last_Cmd = CMD_CTRL;
		status=Give_cmd_to_stepmoto(3);
		UnLock_Data();
		return status;
	}
}





