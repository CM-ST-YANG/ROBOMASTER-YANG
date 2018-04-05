#include "can1.h"
#include "arm_math.h"

CAN_HandleTypeDef hcan1;
CanTxMsgTypeDef   can1_TxMsg;
CanRxMsgTypeDef   can1_pRxMsg;
static PTZmoto_TypeDef	  PTZmoto[3];
static  arm_pid_instance_f32 pidval[2];
static  uint32_t CAN1_TIME;
int16_t PTZmoto_speedset[4] = { 0 };

HAL_StatusTypeDef can1_init(void)
{
	HAL_StatusTypeDef state;
	GPIO_InitTypeDef gpio_init;
	CAN_FilterConfTypeDef can1_filter;

	__HAL_RCC_CAN1_CLK_ENABLE();
	//__HAL_RCC_CAN2_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();

	gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Alternate = GPIO_AF9_CAN1;
	HAL_GPIO_Init(GPIOA, &gpio_init);

	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

	hcan1.Instance = CAN1;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = ENABLE;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_9TQ;
	hcan1.Init.BS2 = CAN_BS2_4TQ;
	hcan1.Init.Prescaler = CAN1_Prescaler / (1 + 9 + 4);
	hcan1.pTxMsg = &can1_TxMsg;
	hcan1.pRxMsg = &can1_pRxMsg;

	state = HAL_CAN_Init(&hcan1);

	can1_filter.FilterNumber = 0;
	can1_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can1_filter.FilterIdHigh = 0x0000;
	can1_filter.FilterIdLow = 0x0000;
	can1_filter.FilterMaskIdHigh = 0x0000;
	can1_filter.FilterMaskIdLow = 0x0000;
	can1_filter.FilterFIFOAssignment = 0;
	can1_filter.FilterActivation = ENABLE;

	pidval[0].Kp = ALL_KP;
	pidval[0].Ki = ALL_KI;
	pidval[0].Kd = ALL_KD;

	pidval[1].Kp = ALL_KP;
	pidval[1].Ki = ALL_KI;
	pidval[1].Kd = ALL_KD;

	arm_pid_init_f32(&pidval[0], 1);
	arm_pid_init_f32(&pidval[1], 1);

	HAL_CAN_ConfigFilter(&hcan1, &can1_filter);

	CAN1_TIME = HAL_GetTick();

	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	
	return state;
}

HAL_StatusTypeDef can1_send(uint8_t *msg, uint8_t len, uint8_t id)
{
	uint8_t i = 0;
	HAL_StatusTypeDef state;


	can1_TxMsg.StdId = (id);
	can1_TxMsg.ExtId = 0;
	can1_TxMsg.IDE = CAN_ID_STD;
	can1_TxMsg.RTR = CAN_RTR_DATA;
	can1_TxMsg.DLC = len;

	for (i = 0; i < len; i++)
	{
		can1_TxMsg.Data[i] = msg[i];
	}

	state = HAL_CAN_Transmit(&hcan1, 10);
	return state;
}

void Send_to_PTZmoto(short Ip, short Ir, short Iy)
{
	can1_TxMsg.StdId = 0x200;
	can1_TxMsg.ExtId = 0;
	can1_TxMsg.IDE = CAN_ID_STD;
	can1_TxMsg.RTR = CAN_RTR_DATA;
	can1_TxMsg.DLC = 8;

	can1_TxMsg.Data[0] = Ip >> 8 & 0x00ff;
	can1_TxMsg.Data[1] = Ip & 0x00ff;

	can1_TxMsg.Data[2] = Ir >> 8 & 0x00ff;
	can1_TxMsg.Data[3] = Ir & 0x00ff;

	can1_TxMsg.Data[4] = Iy >> 8 & 0x00ff;
	can1_TxMsg.Data[5] = Iy & 0x00ff;

	can1_TxMsg.Data[6] = 0;
	can1_TxMsg.Data[7] = 0;

	HAL_CAN_Transmit(&hcan1, 10);
}

void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan1);
}

static char times1 = 1;
static char times2 = 1;

static short speedreal[2] = { 0 };
static short out[2] = { 0 };

float Pset = 10;
float Iset = 2;
float Dset = 1;
static float aset = 0.4;

float Pset2 = 0.25;
float Iset2 = 0.01;
float Dset2 = 1;

short errorsump = 0;
short errornowp = 0;
short errorlastp = 0;
short errordiffp = 0;

int slimt = 1000;



void get_PTZmoto_data(PTZmoto_TypeDef *PTZmoto)
{
	PTZmoto->id = can1_pRxMsg.StdId;
	PTZmoto->hall = can1_pRxMsg.Data[6];
	PTZmoto->Iset = (can1_pRxMsg.Data[4] << 8) + can1_pRxMsg.Data[5];
	PTZmoto->Ireal = (can1_pRxMsg.Data[2] << 8) + can1_pRxMsg.Data[3];
	PTZmoto->anglelast = PTZmoto->angle;
	PTZmoto->angle = (can1_pRxMsg.Data[0] << 8) + can1_pRxMsg.Data[1];

	PTZmoto->speedtemp = PTZmoto[0].angle - PTZmoto->anglelast;
	if (PTZmoto->speedtemp > 4096)
	{
		PTZmoto->count--;
		PTZmoto->speed = -8192 + PTZmoto->speedtemp;
	}
	else if (PTZmoto->speedtemp < -4096)
	{
		PTZmoto->count++;
		PTZmoto->speed = 8192 + PTZmoto->speedtemp;
	}
	else
	{
		PTZmoto->speed = PTZmoto->speedtemp;
	}
	PTZmoto->position += PTZmoto->speed;

	PTZmoto->speedcount += PTZmoto->speed;
}

void CAN1_RX0_Handler()
{

	if (can1_pRxMsg.StdId == 0x201)
	{
		times1++;
		if (times1 == 5)
		{
			times1 = 0;
		}
		get_PTZmoto_data(&PTZmoto[0]);

		if (times1==0)
		{	
			speedreal[0] = aset*PTZmoto[0].speedcount + (1- aset)*speedreal[0];

			if (HAL_GetTick()-CAN1_TIME < 100)
			{
				speedreal[0] = 0;
				PTZmoto_speedset[0] = 0;
			}

			out[0] = arm_pid_f32(&pidval[0], (float)(PTZmoto_speedset[0] - speedreal[0]));

			PTZmoto[0].speedcount = 0;

			out[0] = out[0] > 5000 ? 5000 : out[0];
			out[0] = out[0] < -5000 ? -5000 : out[0];

		//	Send_to_PTZmoto(out, 0, 0);
		}
	}
	if (can1_pRxMsg.StdId == 0x203)
	{
		times2++;
		if (times2 == 5)
		{
			times2 = 0;
		}
		get_PTZmoto_data(&PTZmoto[1]);
		if (times2 == 0)
		{
			speedreal[1] = aset*PTZmoto[1].speedcount + (1 - aset)*speedreal[1];

			if (HAL_GetTick() - CAN1_TIME < 100)
			{
				speedreal[1] = 0;
				PTZmoto_speedset[1] = 0;
			}

			out[1] = arm_pid_f32(&pidval[1], (float)(PTZmoto_speedset[1] - speedreal[1]));

			PTZmoto[1].speedcount = 0;

			out[1] = out[1] > 5000 ? 5000 : out[1];
			out[1] = out[1] < -5000 ? -5000 : out[1];

			//	
		}
	}

	
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
}

void can1_out()
{
	Send_to_PTZmoto(out[0], 0, out[1]);
}