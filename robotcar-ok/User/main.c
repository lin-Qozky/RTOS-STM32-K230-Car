#include "stm32f10x.h"                  // Device header
#include "FreeRTOS.h"
#include "Task.h"
#include "queue.h"

#include "config.h"
#include <string.h>

/*
电机：PB6\PB7\PA4\PA5\PB4\PB5
OLED:PB12\PB13
舵机：PA0\PA1\PA2\PA3\PB0\PB1
串口：PA9\PA10
超声波：PA6(TRIG)\PA7(ECHO)
五路循迹：PB8\PB9\PB10\PB11\PB14
*/

/*任务周期(ms)*/
#define ULTRASONIC_PERIOD_MS	100		//超声波采样
#define XUNJI_PERIOD_MS			10		//循迹控制
#define SERIAL_PERIOD_MS		50		//串口数据打印
#define CATCH_PERIOD_MS			12		//动作步进周期(ms)，每次转1度
#define CATCH_SETTLE_MS			500		//阶段之间停留(ms)
#define CATCH_SETTLE_TICKS		((CATCH_SETTLE_MS + CATCH_PERIOD_MS - 1) / CATCH_PERIOD_MS)	//换算成20ms步数
#define AVOID_PERIOD_MS			100		//避障判断周期(ms)，复用超声波采样结果
#define AVOID_STOP_DIST_CM		20.0f	//距离小于等于该值视为有障碍物，停车
#define ARM_START_DELAY_MS		1000	//收到颜色指令后先停车等待该时长再开始抓取
#define ARM_START_DELAY_TICKS	((ARM_START_DELAY_MS + CATCH_PERIOD_MS - 1) / CATCH_PERIOD_MS)	//换算成20ms步数

/* ------------ 坐标跟踪：串口 x 坐标 -> 电机笔直前进/后退（参数可调） ------------ */
#define TRACK_TARGET_X			413		/* 目标窗口中心 X */
#define TRACK_TOL_X				40		/* X 允许误差：窗口为 413±40 */
#define TRACK_SPEED				1500	/* 前进/后退车速，1000~2000 可调 */
#define TRACK_FRESH_TICKS		50		/* 坐标保鲜期：超过 50x10ms 未收到坐标则退出跟踪并恢复循迹 */

/* 抓取结束后串口停收时长：期间新指令一律丢弃，让车继续循迹避障，
 * 等下一次循迹到目标点、k230 重新识别后再开始新一轮抓取 */
#define SERIAL_STOP_AFTER_CATCH_MS		500
/* 停收时长 = 最后一段到位后的停留 + 恢复循迹后的500ms，
 * 保证收尾动作和刚恢复的瞬间都不会被k230的旧目标重新拉起 */
#define SERIAL_STOP_AFTER_CATCH_TICKS	((SERIAL_STOP_AFTER_CATCH_MS + CATCH_SETTLE_MS + SERIAL_PERIOD_MS - 1) / SERIAL_PERIOD_MS)

TaskHandle_t UltrasonicTaskHandler;
TaskHandle_t XunjiTaskHandler;
TaskHandle_t SerialTaskHandler;
TaskHandle_t CatchLitterTaskHandler;
TaskHandle_t AvoidTaskHandler;

void catch_litter(void);				//抓取动作步进函数，实现见文件末尾

/* ---------------- 任务间共享状态 ---------------- */

/* 超声波最新距离(cm)，-1 表示无回波：超声波任务写，避障任务读 */
static volatile float g_DistanceCm = -1.0f;

/* 各子任务运行开关：1=运行，0=停止 */
static volatile uint8_t g_XunjiEnable  = 1;	/* 循迹 */
static volatile uint8_t g_AvoidEnable  = 1;	/* 避障 */
static volatile uint8_t g_CatchRunning = 0;	/* 抓取动作：收到串口颜色指令后置1 */
static volatile uint8_t g_StopFlag     = 0;	/* @stop暂停：1=小车停车禁动，只能复位恢复 */

#define OBJ_COLOR_LEN		16
static char g_ObjColor[OBJ_COLOR_LEN];
static int  g_ObjX     = 0;
static int  g_ObjY     = 0;
static int  g_ObjCount = 0;


static volatile uint8_t g_Servo1DownTarget = 90;

static volatile uint8_t g_ClawGripTarget = 20;

static volatile uint8_t  g_TrackEnable   = 0;

static volatile uint16_t g_ObjFreshTicks = 0;

static volatile uint16_t g_SerialStopTicks = 0;

/* 抓取动作状态机（原本定义在文件末尾抓取区，上移供串口任务使用） */
static uint8_t  s_CatchStage  = 0;	/* 当前阶段，0开始 */
static uint16_t s_SettleTicks = 0;	/* 阶段到位后的停留倒计时，单位：20ms步进 */
static uint16_t s_FinishTicks = 0;	/* 全部阶段走完后的收尾停留倒计时，单位：20ms步进 */
static uint16_t g_ArmDelayTicks = 0;/* 抓取启动倒计时(20ms步)：>0表示已排队等待，倒计时结束才开始动作 */


static void Ultrasonic_Task(void *arg)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	float distance;
	uint32_t cm;
	(void)arg;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ULTRASONIC_PERIOD_MS));

		distance = Ultrasonic_GetDistance();
		g_DistanceCm = distance;				//供避障任务使用

		if (distance < 0.0f)					//无回波/超量程
		{
			OLED_ShowString(1, 6, "---");
		}
		else
		{
			cm = (uint32_t)(distance + 0.5f);	//四舍五入到整数cm
			OLED_ShowNum(1, 6, cm, 3);
		}
	}
}


static void Avoid_Task(void *arg)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	float distance;
	(void)arg;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(AVOID_PERIOD_MS));

		if (!g_AvoidEnable || g_StopFlag)	
		{
			continue;
		}

		distance = g_DistanceCm;
		if (distance < 0.0f)		
		{
			continue;
		}

		if (distance <= AVOID_STOP_DIST_CM)
		{

			g_XunjiEnable = 0;
			Motor_SetSpeed(0);
			Motor_SetSpeed2(0);
		}
		else if (!g_XunjiEnable)
		{

			g_XunjiEnable = 1;
			Xunji_Reset();
		}
	}
}


static void CameraTrack_Control(void)
{
	int16_t dx = (int16_t)(g_ObjX - TRACK_TARGET_X);

	if (g_ObjFreshTicks > 0) g_ObjFreshTicks--;

	if (g_ObjFreshTicks > 0 && dx >= -TRACK_TOL_X && dx <= TRACK_TOL_X)
	{
		Motor_SetSpeed(0);
		Motor_SetSpeed2(0);
		g_TrackEnable   = 0;
		s_CatchStage    = 0;
		s_SettleTicks   = 0;
		g_ArmDelayTicks = ARM_START_DELAY_TICKS;	/* 停稳后再开始抓取 */
		return;
	}


	if (g_ObjFreshTicks == 0)
	{
		Motor_SetSpeed(0);
		Motor_SetSpeed2(0);
		g_TrackEnable = 0;
		g_XunjiEnable = 1;
		g_AvoidEnable = 1;
		Xunji_Reset();
		return;
	}

	if (dx <= -TRACK_TOL_X)
	{

		Motor_SetSpeed2(TRACK_SPEED);	/* 左轮 */
		Motor_SetSpeed(TRACK_SPEED);	/* 右轮 */
	}
	else
	{

		Motor_SetSpeed2(-TRACK_SPEED);
		Motor_SetSpeed(-TRACK_SPEED);
	}
}


static void Xunji_Task(void *arg)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	(void)arg;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(XUNJI_PERIOD_MS));

		if (g_StopFlag)
		{

			Motor_SetSpeed(0);
			Motor_SetSpeed2(0);
			continue;
		}

		if (g_TrackEnable)
		{
			CameraTrack_Control();	
		}
		else if (g_XunjiEnable)
		{
			Xunji_Control();
		}
		else
		{
			Motor_SetSpeed(0);
			Motor_SetSpeed2(0);
		}
	}
}


static void Serial_Task(void *arg)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	char buffer[128];
	char color[OBJ_COLOR_LEN];
	int x, y, count;
	uint16_t length;
	(void)arg;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERIAL_PERIOD_MS));

		length = Serial_GetReceivedData(buffer, sizeof(buffer) - 1);
		if (length > 0)
		{
			Serial_SendArray((uint8_t *)buffer, length);
		}

		if (Serial_RxFlag && strcmp(Serial_RxPacket, "stop") == 0)
		{
			g_StopFlag    = 1;
			g_XunjiEnable = 0;
			g_AvoidEnable = 0;
			g_TrackEnable = 0;
			Motor_SetSpeed(0);
			Motor_SetSpeed2(0);
			Xunji_Reset();
			Serial_RxFlag = 0;
			continue;
		}

		if (g_SerialStopTicks > 0)
		{
			g_SerialStopTicks--;
			Serial_RxFlag = 0;
			continue;
		}


		if (Serial_RxFlag)
		{
			if (sscanf(Serial_RxPacket, "%15[^,],%d,%d,%d", color, &x, &y, &count) == 4)
			{
				uint8_t stage4Angle = 255;
				uint8_t gripAngle   = 255;

				if      (strcmp(color, "red")  == 0) { stage4Angle = 110; gripAngle = 20; }
				else if (strcmp(color, "pink") == 0) { stage4Angle = 90;  gripAngle = 20; }
				else if (strcmp(color, "blue") == 0) { stage4Angle = 75;  gripAngle = 20; }

				if (stage4Angle != 255)
				{

					if (g_CatchRunning || g_ArmDelayTicks > 0)
					{

					}

					else if (y < 0 || y > 300)
					{

					}
					else
					{

						strncpy(g_ObjColor, color, sizeof(g_ObjColor) - 1);
						g_ObjColor[sizeof(g_ObjColor) - 1] = '\0';
						g_ObjX     = x;
						g_ObjY     = y;
						g_ObjCount = count;
						g_Servo1DownTarget = stage4Angle;	//阶段6舵机1：red=110/pink=90/blue=75
						g_ClawGripTarget   = gripAngle;		//阶段7舵机4夹紧：三色均为20
						g_ObjFreshTicks = TRACK_FRESH_TICKS;

						if (!g_TrackEnable)
						{

							g_TrackEnable = 1;
							g_XunjiEnable = 0;
							g_AvoidEnable = 0;
							Motor_SetSpeed(0);
							Motor_SetSpeed2(0);
							Xunji_Reset();
						}
					}
				}
			}
			Serial_RxFlag = 0;
		}
	}
}


static void CatchLitter_Task(void *arg)
{
	TickType_t lastWakeTime = xTaskGetTickCount();
	(void)arg;

	while (1)
	{
		vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CATCH_PERIOD_MS));
		catch_litter();				
	}
}

int main(void)
{

	MOTOR_Init();
	PWM_Servo_Init();
	Infrared_Init();
	Ultrasonic_Init();
	Serial_Init();

	Angle_SetCompare(Servo1, 1, 180);
	Delay_ms(500);
	Angle_SetCompare(Servo1, 2, 100);	
	Delay_ms(500);
	Angle_SetCompare(Servo1, 3, 180);
	Delay_ms(500);
	Angle_SetCompare(Servo1, 4, 0);
	Delay_ms(500);
//	Angle_SetCompare(Servo2, 3, 0);
//	Delay_ms(500);
	Angle_SetCompare(Servo2, 4, 115);
	Delay_ms(500);
	OLED_Init();

	OLED_ShowString(1, 1, "Dist:");
	OLED_ShowString(1, 10, "cm");

	Xunji_Reset();

	xTaskCreate(Xunji_Task,       "Xunji",      256, NULL, 3, &XunjiTaskHandler);
	xTaskCreate(CatchLitter_Task, "Catch",      256, NULL, 3, &CatchLitterTaskHandler);
	xTaskCreate(Avoid_Task,       "Avoid",      256, NULL, 2, &AvoidTaskHandler);
	xTaskCreate(Ultrasonic_Task,  "Ultrasonic", 256, NULL, 2, &UltrasonicTaskHandler);
	xTaskCreate(Serial_Task,      "Serial",     256, NULL, 1, &SerialTaskHandler);

	vTaskStartScheduler();

	while (1)
	{
	}
}



/* ---------------- 抓取动作状态机 ---------------- */

/*舵机映射：Servo1(CH1~CH4) = 舵机1~4，Servo2(CH4) = 舵机6，舵机5(TIM3 CH3)暂不使用*/
typedef struct
{
	TIM_TypeDef *TIMx;		//所属定时器
	uint8_t channel;		//通道号
	uint8_t target;			//目标角度
} CatchMove_t;

typedef struct
{
	const CatchMove_t *Move;	//本阶段要动的舵机
	uint8_t Count;				//舵机个数
} CatchStage_t;

/* 阶段定义：同一阶段内多个舵机每20ms同步各转1度；
 * 阶段之间严格串行，上一阶段全部到位并停留约500ms后才进下一阶段 */

/* 阶段1：舵机4 20 -> 90（固定，不再映射y） */
static const CatchMove_t s_Stage1[] =
{
	{ Servo1, 4, 90 },
};
/* 阶段2：舵机2 100 -> 47（固定） */
static const CatchMove_t s_Stage2[] =
{
	{ Servo1, 2, 47 },
};
/* 阶段3：舵机6 115 -> 153 */
static const CatchMove_t s_Stage3[] =
{
	{ Servo2, 4, 153 },
};
/* 阶段4：舵机2 50 -> 120 */
static const CatchMove_t s_Stage4[] =
{
	{ Servo1, 2, 120 },
};
/* 阶段5：舵机4 -> 90（单独转动） */
static const CatchMove_t s_Stage5[] =
{
	{ Servo1, 4, 90 },
};
/* 阶段6：舵机1 180 -> 颜色下压角（red=110/pink=90/blue=75） */
static const CatchMove_t s_Stage6[] =
{
	{ Servo1, 1, 90 },
};
/* 阶段7：舵机4 -> 颜色夹紧角（三色均为20） */
static const CatchMove_t s_Stage7[] =
{
	{ Servo1, 4, 90 },
};
/* 阶段8：舵机6 153 -> 115 */
static const CatchMove_t s_Stage8[] =
{
	{ Servo2, 4, 115 },
};
/* 阶段9：舵机4 -> 90（松开） */
static const CatchMove_t s_Stage9[] =
{
	{ Servo1, 4, 90 },
};
/* 阶段10：舵机1 -> 180 */
static const CatchMove_t s_Stage10[] =
{
	{ Servo1, 1, 180 },
};
/* 阶段11：舵机4 90 -> 0 */
static const CatchMove_t s_Stage11[] =
{
	{ Servo1, 4, 0 },
};

/* 阶段12：舵机2 120 -> 100（复位） */
static const CatchMove_t s_Stage12[] =
{
	{ Servo1, 2, 100 },
};
#define CATCH_MOVE_COUNT(arr)	((uint8_t)(sizeof(arr) / sizeof(arr[0])))

static const CatchStage_t s_Stages[] =
{
	{ s_Stage1,  CATCH_MOVE_COUNT(s_Stage1)  },
	{ s_Stage2,  CATCH_MOVE_COUNT(s_Stage2)  },
	{ s_Stage3,  CATCH_MOVE_COUNT(s_Stage3)  },
	{ s_Stage4,  CATCH_MOVE_COUNT(s_Stage4)  },
	{ s_Stage5,  CATCH_MOVE_COUNT(s_Stage5)  },
	{ s_Stage6,  CATCH_MOVE_COUNT(s_Stage6)  },
	{ s_Stage7,  CATCH_MOVE_COUNT(s_Stage7)  },
	{ s_Stage8,  CATCH_MOVE_COUNT(s_Stage8)  },
	{ s_Stage9,  CATCH_MOVE_COUNT(s_Stage9)  },
	{ s_Stage10, CATCH_MOVE_COUNT(s_Stage10) },
	{ s_Stage11, CATCH_MOVE_COUNT(s_Stage11) },
	{ s_Stage12, CATCH_MOVE_COUNT(s_Stage12)},
};

#define CATCH_STAGE_COUNT		((uint8_t)(sizeof(s_Stages) / sizeof(s_Stages[0])))


#define CATCH_STAGE_SERVO1_IDX		5	/* 阶段6：舵机1，目标=g_Servo1DownTarget(颜色下压) */
#define CATCH_STAGE_SERVO4_GRIP_IDX	6	/* 阶段7：舵机4，目标=g_ClawGripTarget(颜色夹紧) */

/* 当前角度，下标就是通道号；初值必须与 main 里上电姿态完全一致，
 * 否则首次动作会先跳到软件记录的旧角度（抽搐）再走余下行程：
 * 舵机1=180、舵机2=100、舵机3=180、舵机4=20、舵机6=115 */
static uint8_t s_Servo1Angle[5] = { 0, 180, 100, 180, 0 };
static uint8_t s_Servo2Angle[5] = { 0,   0,   0,   0, 115 };



static uint8_t *Servo_AnglePtr(TIM_TypeDef *TIMx, uint8_t channel)
{
	if (TIMx == Servo1)
	{
		return &s_Servo1Angle[channel];
	}
	return &s_Servo2Angle[channel];
}


void catch_litter(void)
{
	const CatchStage_t *stage;
	uint8_t allArrived = 1;
	uint8_t i;
	uint8_t *angle;
	uint8_t target;

	if (s_FinishTicks > 0)
	{

		s_FinishTicks--;
		if (s_FinishTicks == 0 && !g_StopFlag)
		{
			g_XunjiEnable = 1;
			g_AvoidEnable = 1;
			Xunji_Reset();
		}
		return;
	}

	if (!g_CatchRunning)
	{

		if (g_ArmDelayTicks > 0)
		{
			g_ArmDelayTicks--;
			if (g_ArmDelayTicks == 0)
			{
				g_CatchRunning = 1;
				s_CatchStage   = 0;
				s_SettleTicks  = 0;
			}
		}
		return;
	}


	if (s_SettleTicks > 0)
	{
		s_SettleTicks--;
		return;
	}

	stage = &s_Stages[s_CatchStage];

	for (i = 0; i < stage->Count; i++)
	{
		angle = Servo_AnglePtr(stage->Move[i].TIMx, stage->Move[i].channel);
		target = stage->Move[i].target;

		/* 运行期覆盖目标角度 */
		if (s_CatchStage == CATCH_STAGE_SERVO1_IDX)
		{
			target = g_Servo1DownTarget;	/* 阶段6舵机1：由颜色决定(red=110/pink=90/blue=75) */
		}
		else if (s_CatchStage == CATCH_STAGE_SERVO4_GRIP_IDX)
		{
			target = g_ClawGripTarget;	/* 阶段7舵机4夹紧：由颜色决定(三色均为20) */
		}

		if (*angle < target)			//每步只加1度
		{
			(*angle)++;
			Angle_SetCompare(stage->Move[i].TIMx, stage->Move[i].channel, (float)(*angle));
		}
		else if (*angle > target)		//每步只减1度
		{
			(*angle)--;
			Angle_SetCompare(stage->Move[i].TIMx, stage->Move[i].channel, (float)(*angle));
		}

		if (*angle != target)
		{
			allArrived = 0;
		}
	}

	if (allArrived)						//本阶段所有舵机到位
	{
		if (s_CatchStage + 1 >= CATCH_STAGE_COUNT)
		{

			s_CatchStage   = 0;
			g_CatchRunning = 0;
			s_FinishTicks     = CATCH_SETTLE_TICKS;
			g_SerialStopTicks = SERIAL_STOP_AFTER_CATCH_TICKS;
		}
		else
		{
			s_CatchStage++;
			s_SettleTicks = CATCH_SETTLE_TICKS;	//下一段动作开始前停留约500ms
		}
	}
}
