#include "xunji.h"

/* ============================ 用户配置区 ============================ */

/* 传感器检测到黑线时 GPIO 读到的电平：0 = 低电平有效，1 = 高电平有效 */
#define XUNJI_SENSOR_ACTIVE       0

/* 左右电机映射：前进时正值为正转。实测发现转向相反时，互换这两个宏即可 */
#define XUNJI_LEFT_MOTOR_SET(s)   Motor_SetSpeed2(s)
#define XUNJI_RIGHT_MOTOR_SET(s)  Motor_SetSpeed(s)

/* 直行基础速度，PWM 占空比范围 0 ~ 7199（TIM4 的 ARR = 7200-1） */
#define XUNJI_BASE_SPEED          2500
/* 基础速度下限：偏差越大降速越多，但整车速度不低于 1500 */
#define XUNJI_BASE_MIN            1500
/* 每 1 个误差单位（满舵为 2）降低的基础速度，实现过弯自动减速 */
#define XUNJI_TURN_SLOW           500

/* 单侧车轮速度上限（直线、钝角、圆弧：两轮一律正转） */
#define XUNJI_WHEEL_MAX           7000
/* 单侧车轮正向速度下限：直线/钝角/圆弧时两轮一律正转且不低于 1500，
   占空比太低时电机根本不转，会走一路一卡一卡 */
#define XUNJI_WHEEL_MIN           1500

/* 转向修正量限幅：实际差速最终由下面 1500 ~ 7000 的车轮限幅决定 */
#define XUNJI_CORR_MAX            4500

/* 直角判定：最外侧探头（L1/R2）单独压线时 |err| = 2。
   必须连续 XUNJI_HARD_ENTER_CYCLE 个周期都贴最外侧才算直角，
   避免圆弧/钝角过冲时短暂碰到最外侧就被误判。 */
#define XUNJI_HARD_TURN_ERR       2.0f
/* 连续周期数：圆弧被误判成直角就加大，直角漏判就减到 1 */
#define XUNJI_HARD_ENTER_CYCLE    2
/* 直角结束条件：偏差回到 |err| <= 1（线回到中间区域）才切回 PID，
   整个直角过程内两轮都保持一正一反、|占空比| >= 5000 */
#define XUNJI_HARD_EXIT_ERR       1.0f

/* ------------ 直角内外弯轮速度：初值与限值，蓝牙 8/9 可实时调整 ------------ */
#define XUNJI_TURN_INNER_INIT     (-5500)   /* 内侧轮反转，|占空比| >= 5000 */
#define XUNJI_TURN_OUTER_INIT     6000      /* 外侧轮正转，占空比 >= 5000 */
#define XUNJI_TURN_INNER_MIN      (-6000)
#define XUNJI_TURN_INNER_MAX      (-5000)   /* 上限 -5000：直角时内侧反转的绝对值不低于 5000 */
#define XUNJI_TURN_OUTER_MIN      5000      /* 下限 5000：直角时外侧正转不低于 5000 */
#define XUNJI_TURN_OUTER_MAX      7000
#define XUNJI_TURN_STEP           500       /* 8/9 每按一次的变化量 */

/* ------------ PID 参数：初值与步进，蓝牙 A~F 可实时调整 ------------ */
#define XUNJI_KP_INIT             1500.0f
#define XUNJI_KI_INIT             0.0f
#define XUNJI_KD_INIT             300.0f
#define XUNJI_KP_STEP             100.0f    /* A/B 每按一次的变化量 */
#define XUNJI_KI_STEP             10.0f     /* C/D 每按一次的变化量 */
#define XUNJI_KD_STEP             50.0f     /* E/F 每按一次的变化量 */
#define XUNJI_KP_MAX              8000.0f
#define XUNJI_KI_MAX              500.0f
#define XUNJI_KD_MAX              2000.0f

/* 积分限幅，防止积分饱和 */
#define XUNJI_I_MAX               2000.0f

/* 丢线搜索（直角丢线使用）：单方向时长，10ms 周期数（250 = 2.5s），超时后反向，双向都失败就停车 */
#define XUNJI_SEARCH_CYCLE        250
/* 原地转向搜索时的轮速：反转侧需要更大占空比才带得动，给低了会表现为"一个轮子不转" */
#define XUNJI_SEARCH_SPEED        3500
/* 丢线确认：线正好卡在两个探头之间时会有一两拍全白，
   连续 XUNJI_LOST_CONFIRM 拍都全白才真正进入丢线处理，确认期间保持上一拍车速 */
#define XUNJI_LOST_CONFIRM        3

/* ------------ 平时（直线/钝角/圆弧）丢线：两轮都正转，只朝线最后出现的一侧轻扫 ---------- */
/* 前向扫描时长，10ms 周期数（50 = 0.5s），超时仍没找到线才退回原地反转搜索 */
#define XUNJI_LOST_FORWARD_CYCLE  20
/* 前向扫描的内侧轮 / 外侧轮速度：都正转，且必须落在 1500 ~ 7000 内 */
#define XUNJI_LOST_INNER          2500
#define XUNJI_LOST_OUTER          5000

/* 贴边记忆：最近 XUNJI_EDGE_HOLD 拍内到过 |err| >= XUNJI_EDGE_ERR，
   说明线已经跑到最边上（直角/急弯的特征）。这之后如果丢线，
   直接按"直角丢线"处理，用一正一反原地把线找回来，绝不再直着往前冲；
   线回到中间（|err| 小）时这个记忆自然衰减掉 */
#define XUNJI_EDGE_ERR            1.5f
#define XUNJI_EDGE_HOLD           8

/* ============================ 内部状态 ============================ */

/* 可实时调整的参数（初值取自上面的宏） */
 int16_t Xunji_TurnInner = XUNJI_TURN_INNER_INIT;
 int16_t Xunji_TurnOuter = XUNJI_TURN_OUTER_INIT;
 float   Xunji_Kp = XUNJI_KP_INIT;
 float   Xunji_Ki = XUNJI_KI_INIT;
 float   Xunji_Kd = XUNJI_KD_INIT;

static float   s_last_error  = 0.0f;   /* 上一次偏差，用于整定微分项 */
static float   s_integral    = 0.0f;   /* 积分累计量 */
static int8_t  s_line_dir    = 0;      /* 线最后一次出现的方向：-1 左，+1 右，0 未知 */
static int8_t  s_search_dir  = 1;      /* 丢线搜索方向：+1 左转，-1 右转 */
static int16_t s_lost_cycle  = 0;      /* 连续丢线周期数 */
static uint8_t s_hard_cycle  = 0;      /* 连续贴最外侧探头的周期数，用于确认直角 */
static uint8_t s_hard_turn   = 0;      /* 1 = 直角模式保持中（两轮一正一反） */
static uint8_t s_blank_cycle = 0;      /* 连续全白周期数，用于丢线确认 */
static uint16_t s_fwd_cycle  = 0;      /* 平时丢线时的前向轻扫周期数 */
static uint8_t s_edge_hold   = 0;      /* 贴边记忆倒计时，>0 表示最近到过最边上 */
static int16_t s_last_left   = 0;      /* 上一拍左轮占空比，短暂全白时保持用 */
static int16_t s_last_right  = 0;      /* 上一拍右轮占空比，短暂全白时保持用 */

static float Xunji_ClampFloat(float value, float min, float max)
{
	if (value > max) return max;
	if (value < min) return min;
	return value;
}

/* 读取单个传感器：返回 1 表示压到黑线 */
static uint8_t Xunji_ReadSensor(uint16_t pin)
{
	return (GPIO_ReadInputDataBit(GPIOB, pin) == XUNJI_SENSOR_ACTIVE) ? 1 : 0;
}

/* 读取五路传感器：输出压线个数，返回加权平均偏差（负值表示线偏左，无压线时返回 0） */
static float Xunji_ReadError(int16_t *count)
{
	int32_t sum = 0;
	int16_t cnt = 0;

	if (Xunji_ReadSensor(GPIO_Pin_8))  { sum -= 2; cnt++; }   /* L1 最左 */
	if (Xunji_ReadSensor(GPIO_Pin_9))  { sum -= 1; cnt++; }   /* L2 */
	if (Xunji_ReadSensor(GPIO_Pin_10)) { sum += 0; cnt++; }   /* M1 中间 */
	if (Xunji_ReadSensor(GPIO_Pin_11)) { sum += 1; cnt++; }   /* R1 */
	if (Xunji_ReadSensor(GPIO_Pin_14)) { sum += 2; cnt++; }   /* R2 最右 */

	*count = cnt;
	if (cnt == 0)
	{
		return 0.0f;
	}
	return (float)sum / (float)cnt;	/* 五路全黑时权重和恰为 0，同样得到居中直行 */
}

/* 原地转向搜索：一侧反转、一侧正转，车体绕自身中心旋转扫线 */
static void Xunji_Pivot(int8_t dir)
{
	if (dir > 0)	/* 左转：左轮反转，右轮正转 */
	{
		XUNJI_LEFT_MOTOR_SET(-XUNJI_SEARCH_SPEED);
		XUNJI_RIGHT_MOTOR_SET(XUNJI_SEARCH_SPEED);
	}
	else			/* 右转：左轮正转，右轮反转 */
	{
		XUNJI_LEFT_MOTOR_SET(XUNJI_SEARCH_SPEED);
		XUNJI_RIGHT_MOTOR_SET(-XUNJI_SEARCH_SPEED);
	}
}

/* ============================ 对外接口 ============================ */

/* 红外传感器引脚初始化（PB8/PB9/PB10/PB11/PB14，浮空输入） */
void Infrared_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/* 清空 PID 与搜索状态，上电或切换进入循迹模式时调用（不影响已调好的参数） */
void Xunji_Reset(void)
{
	s_last_error = 0.0f;
	s_integral   = 0.0f;
	s_line_dir   = 0;
	s_search_dir = 1;
	s_lost_cycle = 0;
	s_hard_cycle = 0;
	s_hard_turn  = 0;
	s_blank_cycle = 0;
	s_fwd_cycle   = 0;
	s_edge_hold   = 0;
	s_last_left   = 0;
	s_last_right  = 0;
}

/* 蓝牙调参：item 取 XUNJI_TUNE_xxx，dir 为 +1 增加、-1 减少 */
void Xunji_Tune(uint8_t item, int8_t dir)
{
	switch (item)
	{
		case XUNJI_TUNE_TURN:
			/* 8：内弯轮反转更深、外弯轮更快；9：反向 */
			Xunji_TurnInner = (int16_t)(Xunji_TurnInner - dir * XUNJI_TURN_STEP);
			Xunji_TurnOuter = (int16_t)(Xunji_TurnOuter + dir * XUNJI_TURN_STEP);
			if (Xunji_TurnInner < XUNJI_TURN_INNER_MIN) Xunji_TurnInner = XUNJI_TURN_INNER_MIN;
			if (Xunji_TurnInner > XUNJI_TURN_INNER_MAX) Xunji_TurnInner = XUNJI_TURN_INNER_MAX;
			if (Xunji_TurnOuter < XUNJI_TURN_OUTER_MIN) Xunji_TurnOuter = XUNJI_TURN_OUTER_MIN;
			if (Xunji_TurnOuter > XUNJI_TURN_OUTER_MAX) Xunji_TurnOuter = XUNJI_TURN_OUTER_MAX;
			break;

		case XUNJI_TUNE_KP:
			Xunji_Kp = Xunji_ClampFloat(Xunji_Kp + dir * XUNJI_KP_STEP, 0.0f, XUNJI_KP_MAX);
			break;

		case XUNJI_TUNE_KI:
			Xunji_Ki = Xunji_ClampFloat(Xunji_Ki + dir * XUNJI_KI_STEP, 0.0f, XUNJI_KI_MAX);
			break;

		case XUNJI_TUNE_KD:
			Xunji_Kd = Xunji_ClampFloat(Xunji_Kd + dir * XUNJI_KD_STEP, 0.0f, XUNJI_KD_MAX);
			break;

		default:
			break;
	}
}

/* 循迹控制：建议 10ms 周期调用一次（与 Xunji_Kd 的微分周期对应） */
void Xunji_Control(void)
{
	int16_t count;
	float raw, err, diff, output, abs_err;
	int16_t base, left_speed, right_speed;

	raw = Xunji_ReadError(&count);

	/* ------------------ 五路全黑：停止线/横线，停车 ------------------ */
	if (count == 5)
	{
		s_lost_cycle = 0;
		s_hard_cycle = 0;
		s_hard_turn  = 0;
		s_blank_cycle = 0;
		s_fwd_cycle  = 0;
		s_edge_hold  = 0;
		XUNJI_LEFT_MOTOR_SET(0);
		XUNJI_RIGHT_MOTOR_SET(0);
		return;
	}

	/* ------------------ 五路全白：先确认，再按"平时/直角"分别处理 ------------------ */
	if (count == 0)
	{
		s_hard_cycle = 0;	/* 中间丢线不算连续贴最外侧，防止误判直角 */
		if (s_blank_cycle < 255) s_blank_cycle++;

		/* 刚全白的头几拍：很可能只是线卡在两个探头之间，
		   保持上一拍车速继续走，不要立刻处理丢线 */
		if (s_blank_cycle <= XUNJI_LOST_CONFIRM)
		{
			XUNJI_LEFT_MOTOR_SET(s_last_left);
			XUNJI_RIGHT_MOTOR_SET(s_last_right);
			return;
		}

		/* ---------- 平时（直线/钝角/圆弧）丢线：两轮都正转，朝线最后出现的一侧轻扫 ----------
		   前提是丢线前没贴过边（s_edge_hold == 0）；贴过边说明是直角/急弯跑出探头阵列，
		   直接走下面的原地反转搜索，避免直着冲出轨 */
		if (!s_hard_turn && s_edge_hold == 0 && s_fwd_cycle < XUNJI_LOST_FORWARD_CYCLE)
		{
			s_fwd_cycle++;
			if (s_line_dir < 0)			/* 线最后在左：左内右外，向左扫 */
			{
				XUNJI_LEFT_MOTOR_SET(XUNJI_LOST_INNER);
				XUNJI_RIGHT_MOTOR_SET(XUNJI_LOST_OUTER);
			}
			else if (s_line_dir > 0)	/* 线最后在右：左外右内，向右扫 */
			{
				XUNJI_LEFT_MOTOR_SET(XUNJI_LOST_OUTER);
				XUNJI_RIGHT_MOTOR_SET(XUNJI_LOST_INNER);
			}
			else						/* 没有记忆：两轮同速直行找线 */
			{
				XUNJI_LEFT_MOTOR_SET(XUNJI_LOST_OUTER);
				XUNJI_RIGHT_MOTOR_SET(XUNJI_LOST_OUTER);
			}
			return;
		}

		/* ---------- 直角模式中丢线 / 前向扫描超时：原地一正一反扫线 ---------- */
		if (s_lost_cycle < 30000) s_lost_cycle++;

		/* 朝线最后一次出现的方向搜索，没有记忆时默认向左 */
		if (s_line_dir < 0)      s_search_dir =  1;
		else if (s_line_dir > 0) s_search_dir = -1;
		else                     s_search_dir =  1;

		if (s_lost_cycle <= XUNJI_SEARCH_CYCLE)
		{
			Xunji_Pivot(s_search_dir);
		}
		else if (s_lost_cycle <= 2 * XUNJI_SEARCH_CYCLE)
		{
			Xunji_Pivot(-s_search_dir);
		}
		else
		{
			/* 两个方向都没扫到线，停车防止跑飞 */
			XUNJI_LEFT_MOTOR_SET(0);
			XUNJI_RIGHT_MOTOR_SET(0);
		}
		return;
	}

	s_lost_cycle = 0;
	s_blank_cycle = 0;
	s_fwd_cycle = 0;
	if (raw != 0.0f)
	{
		s_line_dir = (raw > 0.0f) ? 1 : -1;
	}

	err = raw;
	if (err >  2.0f) err =  2.0f;
	if (err < -2.0f) err = -2.0f;

	abs_err = (err < 0.0f) ? -err : err;

	/* 贴边记忆：线到过最边上就续期，回到中间就自然衰减。
	   丢线时用它区分"直角/急弯跑出探头"和"直线/圆弧上偶尔漏读" */
	if (abs_err >= XUNJI_EDGE_ERR)      s_edge_hold = XUNJI_EDGE_HOLD;
	else if (s_edge_hold > 0)           s_edge_hold--;

	/* ------------------ 直角判定：最外侧探头单独压线并连续保持若干周期 ------------------ */
	if (abs_err >= XUNJI_HARD_TURN_ERR)
	{
		if (s_hard_cycle < 255) s_hard_cycle++;
		if (s_hard_cycle >= XUNJI_HARD_ENTER_CYCLE) s_hard_turn = 1;
	}
	else
	{
		s_hard_cycle = 0;
		/* 偏差回到中间区域才认为直角走完，切回普通 PID */
		if (abs_err <= XUNJI_HARD_EXIT_ERR) s_hard_turn = 0;
	}

	if (s_hard_turn)
	{
		/* ------------------ 直角：内侧轮反转、外侧轮正转，两侧 |占空比| 都不低于 5000 ------------------ */
		if (err < 0.0f)		/* 线在最左，左转：左轮内弯反转，右轮外弯正转 */
		{
			left_speed  = Xunji_TurnInner;
			right_speed = Xunji_TurnOuter;
		}
		else				/* 线在最右，右转：右轮内弯反转，左轮外弯正转 */
		{
			left_speed  = Xunji_TurnOuter;
			right_speed = Xunji_TurnInner;
		}
		s_integral = 0.0f;	/* 直角期间不累加积分，切回直线时不带积分冲击 */
	}
	else
	{
		/* ------------------ 正常压线：位置式 PID ------------------ */
		s_integral += err;
		if (s_integral >  XUNJI_I_MAX) s_integral =  XUNJI_I_MAX;
		if (s_integral < -XUNJI_I_MAX) s_integral = -XUNJI_I_MAX;

		diff   = err - s_last_error;
		output = Xunji_Kp * err + Xunji_Ki * s_integral + Xunji_Kd * diff;

		if (output >  XUNJI_CORR_MAX) output =  XUNJI_CORR_MAX;
		if (output < -XUNJI_CORR_MAX) output = -XUNJI_CORR_MAX;

		/* 偏差越大基础速度越低，降低转弯半径，减少甩线 */
		base = (int16_t)(XUNJI_BASE_SPEED - XUNJI_TURN_SLOW * abs_err);
		if (base < XUNJI_BASE_MIN) base = XUNJI_BASE_MIN;

		/* 车偏左（err<0，output<0）时左轮减速、右轮加速，实现左转 */
		left_speed  = (int16_t)(base + output);
		right_speed = (int16_t)(base - output);

		/* 直线/钝角/圆弧：两轮一律正转，范围 1500 ~ 7000，
		   低于 1500 电机不转，会一路一卡一卡 */
		if (left_speed > XUNJI_WHEEL_MAX) left_speed  = XUNJI_WHEEL_MAX;
		if (left_speed < XUNJI_WHEEL_MIN) left_speed  = XUNJI_WHEEL_MIN;
		if (right_speed > XUNJI_WHEEL_MAX) right_speed = XUNJI_WHEEL_MAX;
		if (right_speed < XUNJI_WHEEL_MIN) right_speed = XUNJI_WHEEL_MIN;
	}

	s_last_left  = left_speed;
	s_last_right = right_speed;
	XUNJI_LEFT_MOTOR_SET(left_speed);
	XUNJI_RIGHT_MOTOR_SET(right_speed);

	s_last_error = err;
}
