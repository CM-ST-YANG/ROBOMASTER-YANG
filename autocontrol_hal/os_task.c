#include "os_task.h"
#include "uart4.h"
#include "usart6.h"
#include "usart3.h"
#include "tim3.h"
#include "tim4.h"
#include "button.h"
#include "can2.h"
#include "rng.h"
#include "mymath.h"
#include <math.h>
#include "arm_math.h"
#include "oled.h"
#include "usart2.h"


#define GREED_LED_ON()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET)
#define GREED_LED_OFF()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET)
#define GREED_LED_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6)

#define RED_LED_ON()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET)
#define RED_LED_OFF()		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET)
#define RED_LED_TOGGLE()	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7)

#define OVERRRANGE	210
#define YES			1
#define NO			0

osThreadId OLEDDisplayThreadHandle, SetSpeedThreadHandle;
osThreadId KeyThreadHandle;
osThreadId StepmotoThreadHandle;
osThreadId SearchBoxThreadHandle;
osThreadId MoveToDestinationThreadHandle;
osThreadId TESTThreadHandle;

//short test_para			= 0;
//float test_kal_para		= 0;

HAL_CAN_StateTypeDef can2_status;

float	cam_distance		= 0;
uint8_t OLED_PAGE = 0;

static void OLEDDisplay_Thread(void const *argument);
static void Set_Car_Speed_Thread(void const *argument);
static void KeyThread(void const *argument);
static void Send_To_Stepmoto_Thread(void const *argument);
static void Search_Box(void const *argument);
static void MoveTo_Destination(void const *argument);
static void TEST(void const *argument);

extern short		speedset[4];
extern int			Position_real[3];
extern short		accdata[4];
extern PLACE_TYPE	Place_real[3];

extern uint8_t		Stepmoto_Data[Stepmoto_Data_Len];
extern short		vision_receive_data[5];
extern uint8_t		Sensor_Data[8];

extern short		RC_ch3;
extern uint8_t		RC_s1;
extern uint8_t		RC_s2;

extern RC_Ctl_t RC_Ctl;

extern button_type	button_state;
extern int			RNG_DATA;
extern uint8_t		Stepmoto_Status;
extern  int16_t PTZmoto_speedset[4];
extern RC_Ctl_t RC_Ctl;

void os_task_init(void)
{

	osThreadDef(OLEDDisplayHandle,			OLEDDisplay_Thread,			osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(SetSpeedHandle,				Set_Car_Speed_Thread,		osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(KeyHandle,					KeyThread,					osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(StepmotoHandle,				Send_To_Stepmoto_Thread,	osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(SearchBoxHandle,			Search_Box,					osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(MoveToDestinationHandle,	MoveTo_Destination,			osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	osThreadDef(TESTHandle,					TEST,						osPriorityNormal, 0, configMINIMAL_STACK_SIZE);


	OLEDDisplayThreadHandle			= osThreadCreate(osThread(OLEDDisplayHandle),			NULL);
	SetSpeedThreadHandle			= osThreadCreate(osThread(SetSpeedHandle),				NULL);
	KeyThreadHandle					= osThreadCreate(osThread(KeyHandle),					NULL);
	StepmotoThreadHandle			= osThreadCreate(osThread(StepmotoHandle),				NULL);
	SearchBoxThreadHandle			= osThreadCreate(osThread(SearchBoxHandle),				NULL);
	MoveToDestinationThreadHandle	= osThreadCreate(osThread(MoveToDestinationHandle),		NULL);
	TESTThreadHandle= osThreadCreate(osThread(TESTHandle), NULL);
}
void os_task_start(void)
{
	/* Start scheduler */
	//osThreadSuspend(OLEDDisplayThreadHandle);
	osThreadSuspend(SearchBoxThreadHandle);
	osThreadSuspend(MoveToDestinationThreadHandle);
	//osThreadSuspend(TESTThreadHandle);

	osKernelStart();


}

void Wait_Send_Cmd(uint8_t cmd)
{
	Send_Cmd_to_stepmoto(cmd); //ץȡ���
	while (Stepmoto_Status != 0x02){
		osDelay(20);
	}
}

uint8_t Search_Edge(short speed ,int inc_dis)
{
	int		dis		= Position_real[CH_Y] + inc_dis;//��һ�ξ��벢����Ƿ�ӽ���Ե
	uint8_t	set_val = OVERRRANGE;

	speedset[CH_Y] = speed;  //�������ʹ�õ��ٶ�

	while ( (Position_real[CH_Y]			< dis)		&& //��������ľ���
		    (Sensor_Data[Car_Infrared_L]	> set_val)	&& //��ߴ������Ѽ�⵽
			(Sensor_Data[Car_Infrared_R]	> set_val)	)  //�ұߴ������Ѽ�⵽
		osDelay(2);

	speedset[CH_Y] = 0;

	if ((Sensor_Data[Car_Infrared_L]) >set_val && (Sensor_Data[Car_Infrared_R]) > set_val){
		return 0; //��������ľ���δ��⵽
	}
	return 1; //��⵽��Ե
}

void Line_Up_Edge(void)
{
	while (!Search_Edge(100,100)) //�ȴ���⵽��Ե
	{
		osDelay(10);
	}

	short differ = Sensor_Data[Car_Infrared_L] - Sensor_Data[Car_Infrared_R]; //���Ҵ������Ĳ�ֵ
	while (fabs(differ) > 10) //�ȴ����Ҵ������Ĳ�ֵС��5
	{
		while (fabs(differ) > 5) //��ת�������������Ĳ�ֵС��5
		{
			if (differ > 0){
				speedset[CH_A] = 3;
			}
			else{
				speedset[CH_A] = -3;
			}
			osDelay(10);
			differ = Sensor_Data[Car_Infrared_L] - Sensor_Data[Car_Infrared_R];//�������Ҵ������Ĳ�ֵ
		}
		speedset[CH_A] = 0;

		Car_Move_Position_To(CH_Y, -50, 100, INC); //�˺�һ�ξ���

		while (!Search_Edge(100,50)) //�ٴζ�����Ե
		{
			osDelay(10);
		}
		osDelay(200);
		differ = Sensor_Data[Car_Infrared_L] - Sensor_Data[Car_Infrared_R];//�ٴθ������Ҵ������Ĳ�ֵ
	}
}


/**
* @brief  Toggle LED1
* @param  thread not used
* @retval None
*/



static void Search_Box(void const *argument)
{
	(void)argument;

	for (;;)
	{
		speedset[CH_A] = 30; //˳ʱ����ת����ʼ������
		for (uint16_t i=0;i<600;i++) //��һ�������ʱ������
		{
			osDelay(20);
			if (vision_receive_data[ID] != 0)	//�����⵽�����
				speedset[CH_A] = 5;				//����
			if (vision_receive_data[ID]==1 && fabs(vision_receive_data[XVAL])<20) //��������ĽϽ�
			{
				short receive_x = 0;
				float a = 0.8; //һ�׹����˲�ϵ��
				while (fabs(vision_receive_data[XVAL] )>5 || cam_distance > 800) //��������׼��� ���Ҿ���Ϊ800mm
				{
					if (vision_receive_data[ID] == 1)
					{
						receive_x = a*receive_x + (1 - a)*(vision_receive_data[XVAL]); //�����˲�ϵ��
						speedset[CH_A] = receive_x * 0.1; //��׼���
						if (cam_distance > 800)
							speedset[CH_Y] = 200; //�������
						else
						{
							speedset[CH_Y] = 0;
						}
					}
					osDelay(20);
				}

				speedset[CH_Y] = 0;
				speedset[CH_A] = 0; //ֹͣ�˶�

				osDelay(400); //��ʱ���ڹ۲�

				int dis = cam_distance+Position_real[CH_Y]-120;
				speedset[CH_Y] = 200;
				while (Position_real[CH_Y] < dis && (Sensor_Data[Grab_Infrared_I] >> 1) == 0) //�ߵ���������⵽���
					osDelay(20);


				//dis = Position_real[CH_Y] + 300;
				//while (Position_real[CH_Y] < dis) //����300mm���������
				//	osDelay(20);
				Car_Move_Position_To(CH_Y, 300, 200, INC);//����300mm���������

				//dis = Position_real[CH_Y] - 100;
				//speedset[CH_Y] = -100;
				//while (Position_real[CH_Y] > dis) //�˻�100mm
				//	osDelay(20);
				Car_Move_Position_To(CH_Y, -100, 100, INC);//�˻�100mm


				dis = Position_real[CH_Y] - 300;
				speedset[CH_Y] = -100;
				while (Position_real[CH_Y] > dis && (Sensor_Data[Grab_Infrared_I] >> 1) == 1) //����뿪���
					osDelay(5);


				speedset[CH_Y] = 30;
				dis = Position_real[CH_Y] + 50;
				while (Position_real[CH_Y] < dis && (Sensor_Data[Grab_Infrared_I] >> 1) == 0) //�ٴνӽ����
					osDelay(5);

				speedset[CH_Y] = 0; //ֹͣ

				i = 600; //�˳�forѭ��
			}
		}
		speedset[CH_A] = 0;

		Wait_Send_Cmd(0x00); //ץȡ���

		Car_Move_Position_To(CH_Y, -400, 200, INC);//����ƶ�400mm

		speedset[CH_Y] = 0;
		osThreadResume(MoveToDestinationThreadHandle);
		osThreadSuspend(SearchBoxThreadHandle);
		
	}
}

static void MoveTo_Box(void const *argument)
{
	(void)argument;

	for (;;)
	{
		;
	}
}

static void Grab_Box(void const *argument)
{
	(void)argument;

	for (;;)
	{
		;
	}
}

static void Search_Destination(void const *argument)
{
	(void)argument;

	for (;;)
	{

	}
}



static void MoveTo_Destination(void const *argument)
{
	(void)argument;
	
	int t = 0;
	for (;;)
	{
		t++;

		Car_Move_Place_To(CH_A, 0, 30, ABS);
		osDelay(100);
		Car_Move_Place_To(CH_X, 0, 200, ABS);
		osDelay(100);
		Car_Move_Place_To(CH_Y, 950, 200, ABS);
		Line_Up_Edge(); //����Ե
		
		Place_real[CH_Y] = 1000;
		Place_real[CH_A] = 0;

		if ((t % 2) == 1){
			Wait_Send_Cmd(0x01); //�ͷ����
		}
		if ((t % 2) == 0){
			Wait_Send_Cmd(0x03); //�ڸߴ��ͷ����
		}

		Car_Move_Place_To(CH_Y, 0, 200, ABS);


		if ((t % 2) == 1)
			osThreadResume(SearchBoxThreadHandle);

		osThreadSuspend(MoveToDestinationThreadHandle);

		osDelay(20);

	}
}

/**
* @brief  Toggle LED1
* @param  thread not used
* @retval None
*/

static void OLEDDisplay_Thread(void const *argument)
{
	(void)argument;
	char tick = 0;
	for (;;)
	{
		osDelay(50);

		if (OLED_PAGE == 0)
		{
			switch (tick)
			{
			case 0:OLED_PrintN(0, 2, "X:", Place_real[CH_X]); break;
			case 1:OLED_PrintN(0, 4, "Y:", Place_real[CH_Y]); break;
			case 2:OLED_PrintN(0, 6, "A:", Place_real[CH_A]); break;
			default: LCD_P8x16Str(0, 0, "Real Place"); break;
			}
		}
		if (OLED_PAGE == 1)
		{
			switch (tick)
			{
			case 0:OLED_PrintN(0, 2, "I:", Sensor_Data[Grab_Infrared_I] >> 1); break;
			case 1:OLED_PrintN(0, 4, "L:", Sensor_Data[Car_Infrared_L]); break;
			case 2:OLED_PrintN(0, 6, "R:", Sensor_Data[Car_Infrared_R]); break;
			default: LCD_P8x16Str(0, 0, "Sensor Data"); break;
			}
		}


		tick++;
		if (tick==3){
			tick = 0;
		}
	//	DisplayMap(tick);
	}
}

/**
* @brief  Toggle LED2 thread
* @param  argument not used
* @retval None
*/


static void Set_Car_Speed_Thread(void const *argument)
{
	char count=1;
	(void)argument;
	for (;;)
	{	
		osDelay(20);
		
		can2_status = CAN2_GetState();
		CalPosition(speedset, accdata);
	
		count--;
		if (count<=0)
		{
			if		(can2_status == HAL_CAN_STATE_TIMEOUT)
				count = LED_10HZ;
			else if (can2_status == HAL_CAN_STATE_BUSY_RX)
				count = LED_2HZ;
			else if (can2_status == HAL_CAN_STATE_READY	 )
				count = LED_1HZ;
			else
				count = LED_10HZ;

			RED_LED_TOGGLE();
		}
		
	}
}


/**
* @brief  key thread
* @param  argument not used
* @retval None
*/

void move_example()
{
	Car_Move_Place_To(CH_A, 0, 30, ABS);
	osDelay(200);

	Car_Move_Place_To(CH_X, 1000, 300, ABS);
	osDelay(200);
	Car_Move_Place_To(CH_Y, 1000, 300, ABS);
	osDelay(200);
	Car_Move_Place_To(CH_X, 0, 300, ABS);
	osDelay(200);
	Car_Move_Place_To(CH_Y, 0, 300, ABS);
	osDelay(200);

}

static void KeyThread(void const *argument)
{

	(void)argument;

	Kalman filter = { 0,0,1,10,1 };

	for (;;)
	{
		GREED_LED_ON();
		if (button_state == LONG) //��⵽����
		{	
			GREED_LED_OFF(); //���̵�ָʾ�ѽ��볤��
			/////////////////������������
			while (RC_s1 != 1 && button_state != PLUSE)
				osDelay(5);
			if (RC_s1 == 1)
			{
				osThreadResume(SearchBoxThreadHandle);
				//osThreadResume(MoveToDestinationThreadHandle);
			}
			if (button_state == PLUSE)
			{
				button_state = 0;
				OLED_PAGE += 1;
				if (OLED_PAGE==2)
				{
					OLED_PAGE = 0;
				}
			}
			/////////////////���������������
		}
		if (button_state == PLUSE)
		{
			/////////////////������������
			
			//osThreadResume(MoveToDestinationThreadHandle);
			//osThreadResume(SearchBoxThreadHandle);
			//move_example();
			//move_to( 100, -50, 200, ABS);
			//Car_Move_Position_To(CH_Y, 500, 100, INC);
			PTZmoto_speedset[0] = 50;
			PTZmoto_speedset[1] = -50;
			PTZmoto_speedset[2] = 50;
			PTZmoto_speedset[3] = -50;
			osDelay(1500);
			PTZmoto_speedset[0] = 0;
			PTZmoto_speedset[1] = 0;
			PTZmoto_speedset[2] = 0;
			PTZmoto_speedset[3] = 0;
			/////////////////���������������
		}
		if (button_state == DOUBLE)
		{
			/////////////////˫����������
			//osThreadResume(TESTThreadHandle);
			/////////////////˫�������������	
		}

		if (vision_receive_data[ID] != 0)
		{
			//test_para = vision_receive_data[SVAL];
			//test_kal_para = Kalman_filter(&filter, (float)test_para);
			//cam_distance = Q_rsqrt(test_kal_para / 100) * 1788;
			cam_distance = Q_rsqrt(Kalman_filter(&filter, (float)vision_receive_data[SVAL])/100)*1788;
		}

		button_state = 0;
		osDelay(10);

	}
}


static void Send_To_Stepmoto_Thread(void const *argument)
{

	(void)argument;
	Kalman kalman;
	kalman.kg = 1;
	kalman.Q = 1;
	kalman.R = 1;
	for (;;)
	{

		Send_To_Sensor();
		osDelay(20);
	}
}

void TEST(void const *argument)
{
	(void)argument;
	for (;;)
	{
		osDelay(5);
		PTZmoto_speedset[0] = (+(RC_Ctl.rc.ch1 - 1024) + (RC_Ctl.rc.ch0 - 1024) + (RC_Ctl.rc.ch2 - 1024)) / 9;
		PTZmoto_speedset[1] = (-(RC_Ctl.rc.ch1 - 1024) + (RC_Ctl.rc.ch0 - 1024) + (RC_Ctl.rc.ch2 - 1024)) / 9;
		PTZmoto_speedset[2] = (+(RC_Ctl.rc.ch1 - 1024) - (RC_Ctl.rc.ch0 - 1024) + (RC_Ctl.rc.ch2 - 1024)) / 9;
		PTZmoto_speedset[3] = (-(RC_Ctl.rc.ch1 - 1024) - (RC_Ctl.rc.ch0 - 1024) + (RC_Ctl.rc.ch2 - 1024)) / 9;
		can1_out();
		can2_out();
		//if (RC_Ctl.rc.s2 == 2)
		//{
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		//}
		//else {
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		//}
	}
}