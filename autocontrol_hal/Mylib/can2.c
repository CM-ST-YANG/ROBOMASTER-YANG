#include "can2.h"
#include "can1.h"
#include "arm_math.h"
#include <math.h>

CAN_HandleTypeDef hcan2;
CanTxMsgTypeDef   can2_TxMsg;
CanRxMsgTypeDef   can2_pRxMsg;
short RC_ch3 = 0;
short RC_ch2 = 0;
short RC_ch1 = 0;
uint8_t RC_s1 = 0;
uint8_t RC_s2 = 0;

static PTZmoto_TypeDef	  PTZmoto_2[3];
static  arm_pid_instance_f32 pidval_2[2];
static  uint32_t CAN2_TIME;
extern  int16_t PTZmoto_speedset[4];


extern short speedset[4];
extern int Position_real[3]; //相对位置

PLACE_TYPE Place_real[3] = { 0 };//全局位置
extern short accdata[4];

HAL_StatusTypeDef can2_init(void)
{
	HAL_StatusTypeDef state;
	GPIO_InitTypeDef gpio_init;
	CAN_FilterConfTypeDef can2_filter;
	
	__HAL_RCC_CAN1_CLK_ENABLE();
	__HAL_RCC_CAN2_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	
	gpio_init.Pin = GPIO_PIN_12 | GPIO_PIN_13;
	gpio_init.Mode = GPIO_MODE_AF_PP;
	gpio_init.Alternate = GPIO_AF9_CAN2;
	HAL_GPIO_Init(GPIOB, &gpio_init);
	
	hcan2.Instance = CAN2;
	hcan2.Init.TTCM = DISABLE;
	hcan2.Init.ABOM = DISABLE;
	hcan2.Init.AWUM = DISABLE;
	hcan2.Init.NART = DISABLE;
	hcan2.Init.RFLM = DISABLE;
	hcan2.Init.TXFP = ENABLE;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SJW = CAN_SJW_1TQ;
	hcan2.Init.BS1 = CAN_BS1_9TQ;
	hcan2.Init.BS2 = CAN_BS2_4TQ;
	hcan2.Init.Prescaler = CAN2_Prescaler / (1 + 9 + 4);
	hcan2.pTxMsg = &can2_TxMsg;
	hcan2.pRxMsg = &can2_pRxMsg;
	
	state = HAL_CAN_Init(&hcan2);
	
	can2_filter.FilterNumber = 14;
	can2_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can2_filter.FilterScale = CAN_FILTERSCALE_32BIT;
	can2_filter.FilterIdHigh = 0x0000;
	can2_filter.FilterIdLow = 0x0000;
	can2_filter.FilterMaskIdHigh = 0x0000;
	can2_filter.FilterMaskIdLow = 0x0000;
	can2_filter.FilterFIFOAssignment = 0;
	can2_filter.FilterActivation = ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan2, &can2_filter);

	pidval_2[0].Kp = ALL_KP;
	pidval_2[0].Ki = ALL_KI;
	pidval_2[0].Kd = ALL_KD;

	pidval_2[1].Kp = ALL_KP;
	pidval_2[1].Ki = ALL_KI;
	pidval_2[1].Kd = ALL_KD;

	arm_pid_init_f32(&pidval_2[0], 1);
	arm_pid_init_f32(&pidval_2[1], 1);

	CAN2_TIME = HAL_GetTick();


	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);

	HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);


	return state;
}

#pragma region CalAccel

void CalAccel(short *ch, short *out, short *acc, short len)
{
	int i = 0;
	int a = 0;
	int d = 0;
	for (i = 0; i < len; i++)
	{
		d = ch[i] - out[i];
		a = acc[i];// +abs(d) / 20;
		if (d > a){
			out[i] += a;
		}
		else if (d < -a){
			out[i] -= a;
		}
		else{
			out[i] = ch[i];
		}
	}
}

float Place_temp[4];
#define X_Scale 50.0f
#define Y_Scale 50.0f
#define A_Scale 50.0f

//49.57

void CalPosition(short speedset[], short *accdata)
{
	static short ToCar[4];
	static int Position_data[4];
	

	CalAccel(speedset, ToCar, accdata, 4);
#if (USE_LOCAL_PLACE ==1 )
	Position_data[CH_X] += ToCar[CH_X];
	Position_data[CH_Y] += ToCar[CH_Y];
	Position_data[CH_A] += ToCar[CH_A];
	
#if (USE_DOUBLE_PLACE ==1 )
	Place_real[CH_X] += ToCar[CH_Y] / Y_Scale * sin(Position_real[CH_A] / 180.0*PI) + ToCar[CH_X] / X_Scale * cos(Position_real[CH_A] / 180.0*PI);
	Place_real[CH_Y] += ToCar[CH_Y] / Y_Scale * cos(Position_real[CH_A] / 180.0*PI) - ToCar[CH_X] / X_Scale * sin(Position_real[CH_A] / 180.0*PI);
	Place_real[CH_A] += ToCar[CH_A] / A_Scale;
#else
	Place_real[CH_X] += ToCar[CH_Y] / Y_Scale * arm_sin_f32(Position_real[CH_A] / 180.0*PI) + ToCar[CH_X] / X_Scale * arm_cos_f32(Position_real[CH_A] / 180.0*PI);
	Place_real[CH_Y] += ToCar[CH_Y] / Y_Scale * arm_cos_f32(Position_real[CH_A] / 180.0*PI) - ToCar[CH_X] / X_Scale * arm_sin_f32(Position_real[CH_A] / 180.0*PI);
	Place_real[CH_A] += ToCar[CH_A] / A_Scale;
#endif // USE_DOUBLE_PLACE==1

	

	Position_real[CH_X] = Position_data[CH_X] / X_Scale;
	Position_real[CH_Y] = Position_data[CH_Y] / Y_Scale;
	Position_real[CH_A] = Position_data[CH_A] / A_Scale;

	//Place_real[CH_X] = Place_temp[CH_X] / 230;
	//Place_real[CH_Y] = Place_temp[CH_Y] / 40.8;
	//Place_real[CH_A] = Position_real[CH_A];
#endif
	CAN2_Send(ToCar);
}
#pragma endregion

HAL_StatusTypeDef can2_send(uint8_t *msg, uint8_t len, uint8_t id)
{	
	uint8_t i = 0;
	HAL_StatusTypeDef state;
	
	
	can2_TxMsg.StdId = (id << 3);
	can2_TxMsg.ExtId = 0;
	can2_TxMsg.IDE = CAN_ID_STD;
	can2_TxMsg.RTR = CAN_RTR_DATA;
	can2_TxMsg.DLC = len;
	
	for (i = 0; i < len; i++)
	{
		can2_TxMsg.Data[i] = msg[i];
	}
	
	state = HAL_CAN_Transmit(&hcan2, 10);
	return state;
}

void CAN2_Send(short *ch)
{
	uint8_t CAN_CH[8];
	short temp[4];
	temp[0] = ch[0] / 2.60 ;
	temp[1] = ch[1] / 2.90 ;
	temp[2] = ch[2] * 3.10 ;

	CAN_CH[0] = (temp[0]) >> 8;
	CAN_CH[1] = (temp[0]) & 0x00ff;

	CAN_CH[2] = (temp[1]) >> 8;
	CAN_CH[3] = (temp[1]) & 0x00ff;

	CAN_CH[4] = (temp[2]) >> 8;
	CAN_CH[5] = (temp[2]) & 0x00ff;

	CAN_CH[6] = (temp[3]) >> 8;
	CAN_CH[7] = (temp[3]) & 0x00ff;

	can2_send(CAN_CH, 8, 0);
}

void Car_Move_Place_To(uint8_t ch, int val, short speed_set, uint8_t mode)
{
	int dis; //目的位置
	short speed = speed_set; //
	short dir = 1; //移动方向
	if (mode == ABS) {
		dis = val; //绝对式移动
	}
	else {
		dis = Place_real[ch] + val; //增量式移动
	}
	if (dis < Place_real[ch]) //判断方向
	{
		dir = -1; //负方向移动
	}
	switch (ch)
	{
	case CH_X:if (speed_set < 100)speed = 100; break;
	case CH_Y:if (speed_set < 100)speed = 100; break;
	case CH_A:if (speed_set < 20)speed = 20; break;
	default:
		break;
	}

#if(USE_DEFAULT_SPEED==1) 
	switch (ch)
	{
	case CH_X:speed = 200; break;
	case CH_Y:speed = 200; break;
	case CH_A:speed = 30;  break;
	default:
		break;
	}
#endif
	speedset[ch] = (short)(speed*dir);

	while ((dis - Place_real[ch])*dir>speed*0.1)
	{
		osDelay(10);
	}

	while ((dis - Place_real[ch])*dir > speed*0.01)
	{
		osDelay(10);
		speedset[ch] = (short)((dis - Place_real[ch])*dir*5 + speed*0.5*dir);
	}
	speedset[ch] = 0;
}

void Car_Move_Position_To(uint8_t ch, int val, short speed_set, uint8_t mode)
{
	//int dis; //目的位置
	//short speed=speed_set; //
	//short dir = 1; //移动方向
	//float raio = 1;
	//if (mode == ABS) {
	//	dis = val; //绝对式移动
	//}
	//else {
	//	dis = Position_real[ch] + val; //增量式移动
	//}
	//if (dis < Position_real[ch]) //判断方向
	//{
	//	dir = -1; //负方向移动
	//}

	//switch (ch)
	//{
	//case CH_X:if(speed_set<100)speed = 100; break;
	//case CH_Y:if (speed_set<100)speed = 100; break;
	//case CH_A:if (speed_set<20)speed = 20; break;
	//default:
	//	break;
	//}

	//#if(USE_DEFAULT_SPEED==1)
	//	switch (ch)
	//	{
	//	case CH_X:speed = 200; break;
	//	case CH_Y:speed = 200; break;
	//	case CH_A:speed = 30;  break;
	//	default:
	//		break;
	//	}
	//#endif
	//speedset[ch] = (short)(speed*dir);

	////if (ch==CH_A)
	////{
	////	raio = 0.2;
	////}

	//while ((dis - Position_real[ch])*dir>speed*0.1)
	//{
	//	osDelay(10);
	//}

	//while ((dis - Position_real[ch])*dir > speed*0.01)
	//{
	//	osDelay(10);
	//	speedset[ch] = (short)((dis - Position_real[ch])*dir*5 + speed*0.5*dir);
	//}
	int32_t P_Start = Position_real[ch];
	int32_t P_End;
	int8_t dir = 1;
	float speed;
	float a = 50;
	int16_t Time = 10;

	if (mode == ABS) {
		P_End = val;
	}
	else {
		P_End = P_Start + val;
	}

	if (P_End < P_Start)
	{
		dir = -1;
	}

	while ((Position_real[ch] - P_Start)*dir < (speed_set*1.0*speed_set / 2.0 / a) && 2* Position_real[ch]*dir <= (P_Start + P_End)*dir)
	{
		speed = speed + a*Time / 1000.0;
		speedset[ch] = speed*dir;
		osDelay(Time);
	}

	while ((P_End - Position_real[ch])*dir > (speed_set*1.0*speed_set / 2.0 / a))
	{
		//speed = speed + a*Time / 1000.0;
		//speedset[ch] = speed;
		osDelay(Time);
	}

	while (speed > 0 && (P_End - Position_real[ch])*dir > 0)
	{
		speed = speed - a*Time / 1000.0;
		speedset[ch] = speed*dir;
		osDelay(Time);
	}

	speedset[ch] = 0;
}

void move_to(int x, int y, int f, uint8_t mode)
{
	double origin[2],car_a;
	Car_Move_Position_To(CH_A, 0, 10, ABS);
	if (mode==ABS)
	{
		x = x - Place_real[CH_X];
		y = y - Place_real[CH_Y];
	}
	if (x==0)
	{
		Car_Move_Position_To(CH_Y, y, f, INC);
		return;
	}
	if (y==0)
	{
		Car_Move_Position_To(CH_X, x, f, INC);
		return;
	}
	origin[CH_X] = Place_real[CH_X];
	origin[CH_Y] = Place_real[CH_Y];

	car_a = atanf((float)y / (float)x);

	if (x < 0)
	{
		car_a = car_a + PI;
	}
	else if (y < 0)
	{
		car_a = PI * 2 + car_a;
	}

	speedset[CH_X] = cos(car_a)*f;
	speedset[CH_Y] = sin(car_a)*f;

	if ((int32_t)x>0)
	{
		while ((Place_real[CH_X] - origin[CH_X] -(float)x) < -10)
		{
			osDelay(5);
		}
	}
	else
	{
		while ((Place_real[CH_X] - origin[CH_X] - (float)x) > 10)
		{
			osDelay(5);
		}
	}
	
	speedset[CH_X] = 0;
	speedset[CH_Y] = 0;

}


HAL_CAN_StateTypeDef CAN2_GetState(void)
{
	return hcan2.State;
}

void CAN2_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan2);
}

void Send_to_PTZmoto2(short Ip, short Ir, short Iy)
{
	can2_TxMsg.StdId = 0x200;
	can2_TxMsg.ExtId = 0;
	can2_TxMsg.IDE = CAN_ID_STD;
	can2_TxMsg.RTR = CAN_RTR_DATA;
	can2_TxMsg.DLC = 8;

	can2_TxMsg.Data[0] = Ip >> 8 & 0x00ff;
	can2_TxMsg.Data[1] = Ip & 0x00ff;

	can2_TxMsg.Data[2] = Ir >> 8 & 0x00ff;
	can2_TxMsg.Data[3] = Ir & 0x00ff;

	can2_TxMsg.Data[4] = Iy >> 8 & 0x00ff;
	can2_TxMsg.Data[5] = Iy & 0x00ff;

	can2_TxMsg.Data[6] = 0;
	can2_TxMsg.Data[7] = 0;

	HAL_CAN_Transmit(&hcan2, 10);
}

void get_PTZmoto_data_2(PTZmoto_TypeDef *PTZmoto)
{
	PTZmoto->id = can2_pRxMsg.StdId;
	PTZmoto->hall = can2_pRxMsg.Data[6];
	PTZmoto->Iset = (can2_pRxMsg.Data[4] << 8) + can2_pRxMsg.Data[5];
	PTZmoto->Ireal = (can2_pRxMsg.Data[2] << 8) + can2_pRxMsg.Data[3];
	PTZmoto->anglelast = PTZmoto->angle;
	PTZmoto->angle = (can2_pRxMsg.Data[0] << 8) + can2_pRxMsg.Data[1];

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

static char times1_2 = 1;
static char times2_2 = 1;

static short speedreal_2[2] = { 0 };
static short out_2[2] = { 0 };

static float aset_2 = 0.4;

void CAN2_RX0_Handler()
{
	if (can2_pRxMsg.StdId == 0x201)
	{
		times1_2++;
		if (times1_2 == 5)
		{
			times1_2 = 0;
		}
		get_PTZmoto_data_2(&PTZmoto_2[0]);

		if (times1_2 == 0)
		{
			speedreal_2[0] = aset_2*PTZmoto_2[0].speedcount + (1 - aset_2)*speedreal_2[0];

			if (HAL_GetTick() - CAN2_TIME < 100)
			{
				speedreal_2[0] = 0;
				PTZmoto_speedset[0] = 0;
			}

			out_2[0] = arm_pid_f32(&pidval_2[0], (float)(PTZmoto_speedset[2] - speedreal_2[0]));

			PTZmoto_2[0].speedcount = 0;

			out_2[0] = out_2[0] > 5000 ? 5000 : out_2[0];
			out_2[0] = out_2[0] < -5000 ? -5000 : out_2[0];

			//	Send_to_PTZmoto(out, 0, 0);
		}
	}
	if (can2_pRxMsg.StdId == 0x203)
	{
		times2_2++;
		if (times2_2 == 5)
		{
			times2_2 = 0;
		}
		get_PTZmoto_data_2(&PTZmoto_2[1]);
		if (times2_2 == 0)
		{
			speedreal_2[1] = aset_2*PTZmoto_2[1].speedcount + (1 - aset_2)*speedreal_2[1];

			if (HAL_GetTick() - CAN2_TIME < 100)
			{
				speedreal_2[1] = 0;
				PTZmoto_speedset[1] = 0;
			}

			out_2[1] = arm_pid_f32(&pidval_2[1], (float)(PTZmoto_speedset[3] - speedreal_2[1]));

			PTZmoto_2[1].speedcount = 0;

			out_2[1] = out_2[1] > 5000 ? 5000 : out_2[1];
			out_2[1] = out_2[1] < -5000 ? -5000 : out_2[1];

			//	
		}
	}

	//Send_to_PTZmoto2(out_2[0], 0, out_2[1]);

	HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
}

void can2_out()
{
	Send_to_PTZmoto2(out_2[0], 0, out_2[1]);
}