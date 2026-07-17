#include "IR_Module.h"
#include <math.h>
uint32_t ir_dh1_state, ir_dh2_state, ir_dh3_state, ir_dh4_state;//四个红外传感器电平
/*=============================================================================
 * 可调参数区域
 *=============================================================================*/
// 转向角度参数（左右轮速差值）
float Turn90Angle  = 70;   // 直角弯转向参数
float TurnMaxAngle = 45;   // 大弯道转向参数
float TurnMidAngle = 20;   // 中等转向参数（丢线时使用）
float TurnMinAngle = 15;   // 微调转向参数
extern int temp;
// 速度参数
float BaseSpeed = 220;      // 基础巡线速度，单位mm/s
float ForwardLimit = 90;    // 弯道最低中心速度，单位mm/s
/*=============================================================================
 * 传感器状态定义
 * 手册规定从左到右为 DH1、DH2、DH3、DH4；白底输出 1，黑线输出 0。
 *=============================================================================*/
typedef enum {
    STATE_CROSS         = 0,    // 0000 - 十字路口（全黑）
    STATE_LEFT_90_A     = 1,    // 0001 - 左直角弯
	STATE_LEFT_90_B		= 3,	// 0011
    STATE_RIGHT_90_A    = 8,  	// 1000 - 右直角弯
	STATE_RIGHT_90_B    = 12,	// 1100
    STATE_LEFT_BIG      = 7,    // 0111 - 左大弯
    STATE_RIGHT_BIG     = 14,   // 1110 - 右大弯
    STATE_LEFT_SMALL    = 11,   // 1011 - 左微调
    STATE_RIGHT_SMALL   = 13,   // 1101 - 右微调
    STATE_STRAIGHT      = 9,    // 1001 - 直行
    STATE_LOST          = 15    // 1111 - 丢线（全白）
} SensorState_t;
float base_speed_mm = 0;        // 基础速度（mm/s）
float turn_diff = 0;            // 左右轮差速
static int last_state = STATE_STRAIGHT; // 丢线时沿用上一次有效转向状态

void IR_LineReset(void)
{
    /*
     * 修改原因：不同圆弧之间隔着无标线直线，进入下一段圆弧时不能继续使用
     * 上一段圆弧遗留的转向方向，否则刚到黑线端点时可能先向错误方向转弯。
     */
    last_state = STATE_STRAIGHT;
    turn_diff = 0.0f;
}

/*=============================================================================
 * 巡线功能函数（输出两电机目标速度）
 *=============================================================================*/
void IRDM_line_inspection(void)
{
	float left_motor_speed = 0;     // 左电机临时速度（m/s）
    float right_motor_speed = 0;    // 右电机临时速度（m/s）
    // 读取传感器状态：4个传感器组合值
	    // 读取四个引脚的状态并强制转换为0或1
    ir_dh4_state = DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1 : 0;
    ir_dh3_state = DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1 : 0;
    ir_dh2_state = DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1 : 0;
    ir_dh1_state = DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1 : 0;
	
    int sensor_state = (ir_dh1_state << 3) | (ir_dh2_state << 2) | (ir_dh3_state << 1) | ir_dh4_state; // 将传感器状态组合成一个整数
	
    /*
     * 修改原因：本赛题只有半径 40 cm 的平滑半圆，没有直角弯。
     * 原代码把函数调用次数误当成毫秒，并会在半圆上误触发长时间强制转向，
     * 因此取消“两段式直角弯计数”，各传感器状态直接参与本次差速控制。
     */
  /*=========================================================================*
     * 状态判断：设置转向差速												   *
     *=========================================================================*/
    switch (sensor_state)//对检测到的状态进行处理
    {
        case STATE_CROSS:// 交叉路口处理
			turn_diff = 0;
            break;
        case STATE_LEFT_90_A: // 左直角弯
		case STATE_LEFT_90_B: // 左直角弯
            turn_diff = Turn90Angle;
            break;
        case STATE_RIGHT_90_A: // 右直角弯
		case STATE_RIGHT_90_B: // 右直角弯
            turn_diff = -Turn90Angle;
            break;
        case STATE_LEFT_BIG://左大弯
            turn_diff = TurnMaxAngle;
            break;
        case STATE_RIGHT_BIG://右大弯
            turn_diff = -TurnMaxAngle;
            break;
        case STATE_LEFT_SMALL://左微调
            turn_diff = TurnMinAngle;
            break;
        case STATE_RIGHT_SMALL://右微调
            turn_diff = -TurnMinAngle;
            break;
        case STATE_STRAIGHT://直行
            turn_diff = 0;
            break;
        case STATE_LOST://丢线处理
            if (last_state == STATE_LEFT_SMALL) turn_diff = TurnMidAngle;//继续左转
			else if (last_state == STATE_RIGHT_SMALL) turn_diff = -TurnMidAngle;//继续右转
			else if(last_state == STATE_LEFT_BIG ) turn_diff = TurnMaxAngle;//继续左转
			else if(last_state == STATE_RIGHT_BIG ) turn_diff = -TurnMaxAngle;//继续右转
            break;
        default: // 未定义状态，直行
            turn_diff = 0;
            break;
    }
	//保存传感器状态
		if(sensor_state!=STATE_LOST)//只有不丢线时才更新
	{
		last_state=sensor_state;
	}
   
	base_speed_mm = BaseSpeed - 0.6f * fabsf(turn_diff); // 转向速度越大，基础速度越低
	if (base_speed_mm < ForwardLimit) base_speed_mm = ForwardLimit;
    /*========================================================================*
     * 设置电机目标速度（左-转向差速，右+转向差速，单位：mm/s）                   *
     *=========================================================================*/
	left_motor_speed = 0.001f * (base_speed_mm - turn_diff); 
    right_motor_speed = 0.001f * (base_speed_mm + turn_diff);
    // 赋值给电机目标速度
    MotorA.Target_Encoder = left_motor_speed;//左电机
	MotorB.Target_Encoder = right_motor_speed;//右电机
}

void IR_Module_Read(void)
{
	/* 读取思路红外状态 */
	IRDM_line_inspection();
}

uint8_t IR_LineDetected(void)
{
	/* 返回1：至少有一路检测到线  返回0：四路都丢线 */
	uint32_t sensor_state;
	sensor_state = ((DL_GPIO_readPins(IR_DH1_PORT, IR_DH1_PIN_27_PIN) ? 1U : 0U) << 3) |
		((DL_GPIO_readPins(IR_DH2_PORT, IR_DH2_PIN_12_PIN) ? 1U : 0U) << 2) |
		((DL_GPIO_readPins(IR_DH3_PORT, IR_DH3_PIN_16_PIN) ? 1U : 0U) << 1) |
		 (DL_GPIO_readPins(IR_DH4_PORT, IR_DH4_PIN_17_PIN) ? 1U : 0U);
	return sensor_state != STATE_LOST;
}








