/***********************************************
公司：轮趣科技（东莞）有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29


Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: 5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "control.h"

u8 CCD_count,ELE_count;
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;

Encoder OriginalEncoder; 					//编码器原始数据   
Motor_parameter MotorA,MotorB;				//左右电机相关变量
float Velocity_KP=400,Velocity_KI=400;	
/* 上电默认选择最简单的 A->B 模式，先便于检查循迹、编码器和电机方向。 */
int Run_Mode=0;//默认选择第(1)项：A到B
volatile int Flag_Stop=1;//小车启动标志位
volatile float EncoderA_Distance;
volatile float EncoderB_Distance;

void TIMER_0_INST_IRQHandler(void)
{
	/*
	 * 修改原因：SysConfig 给 TIMER_0 分配的是 TIMG0，应使用 DL_TimerG 接口；
	 * 每 10 ms 原子地取走编码器增量，然后完成按键、速度换算和双轮 PI 控制。
	 */
	if (DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO) {
			int32_t count_a = Get_Encoder_countA;
			int32_t count_b = Get_Encoder_countB;
			Get_Encoder_countA = 0;
			Get_Encoder_countB = 0;

			Key();
			/* PID 调试期间关闭状态灯闪烁，只保留按键、编码器、速度环和 PWM。 */
//			LED_Flash(100);
			Get_Velocity_From_Encoder(count_a, count_b);

//			//计算左右电机对应的PWM
			MotorA.Motor_Pwm = Incremental_PI_Left(MotorA.Current_Encoder,MotorA.Target_Encoder);	
			MotorB.Motor_Pwm = Incremental_PI_Right(MotorB.Current_Encoder,MotorB.Target_Encoder);
			if(!Flag_Stop)
			{
				Set_PWM(-MotorA.Motor_Pwm,MotorB.Motor_Pwm);
			}else Set_PWM(0,0);
	}
}

/**************************************************************************
Function: Get_Velocity_From_Encoder
Input   : none
Output  : none
函数功能：读取编码器和转换成速度
入口参数: 无 
返回  值：无
**************************************************************************/	 	
void Get_Velocity_From_Encoder(int Encoder1,int Encoder2)
{
	/* 修改原因：将“每控制周期脉冲数”统一换算为 m/s，并同时积分成左右轮里程。 */
	const float meters_per_count = Perimeter /
		(EncoderMultiples * ENCODER_RESOLUTION * MOTOR_GEAR_RATIO);

	OriginalEncoder.A = Encoder1;
	OriginalEncoder.B = Encoder2;
	MotorA.Current_Encoder = ENCODER_A_SIGN * Encoder1 *
		CONTROL_FREQUENCY * meters_per_count;
	MotorB.Current_Encoder = ENCODER_B_SIGN * Encoder2 *
		CONTROL_FREQUENCY * meters_per_count;
	EncoderA_Distance += ENCODER_A_SIGN * Encoder1 * meters_per_count;
	EncoderB_Distance += ENCODER_B_SIGN * Encoder2 * meters_per_count;
}

void Control_ResetOdometry(void)
{
	/* 每次开始新路线时从零计算路段距离，避免沿用上一次运行的累计值。 */
	EncoderA_Distance = 0.0f;
	EncoderB_Distance = 0.0f;
}

float Control_GetAverageDistance(void)
{
	float left = EncoderA_Distance;
	float right = EncoderB_Distance;
	if (left < 0.0f) left = -left;
	if (right < 0.0f) right = -right;
	return 0.5f * (left + right);
}
//运动学逆解，由x和y的速度得到编码器的速度,Vx是m/s,Vz单位是度/s(角度制)
void Get_Target_Encoder(float Vx,float Vz)
{
	/* 修改原因：运动学公式需要 rad/s，调用接口保留较直观的 deg/s，因此在此转换。 */
	float angular_velocity = Vz * (PI / 180.0f);
	if(Vx<0) angular_velocity=-angular_velocity;
	//Inverse kinematics //运动学逆解
	 MotorA.Target_Encoder = Vx - angular_velocity * Wheelspacing / 2.0f;
	 MotorB.Target_Encoder = Vx + angular_velocity * Wheelspacing / 2.0f;
}


/**************************************************************************
Function: Absolute value function
Input   : a：Number to be converted
Output  : unsigned int
函数功能：绝对值函数
入口参数：a：需要计算绝对值的数
返回  值：无符号整型
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}

int Turn_Off(void)
{
	u8 temp = 0;
//	if(Voltage>700&&EN==0)//电压高于7V且使能开关打开
//	{
//		temp = 1;
//	}
	return temp;			
}
/**************************************************************************
Function: PWM_Limit
Input   : IN;max;min
Output  : OUT
函数功能：限制PWM赋值
入口参数: IN：输入参数  max：限幅最大值  min：限幅最小值 
返回  值：限幅后的值
**************************************************************************/	 	
float PWM_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_Left (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}


int Incremental_PI_Right (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                					//计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   	//增量式PI控制器
	if(Flag_Stop) Pwm=0;
	 if(Pwm>7800)Pwm=7800;
	 if(Pwm<-7800)Pwm=-7800;
	 Last_bias=Bias;	                   					//保存上一次偏差 
	 return Pwm;                         					//增量输出
}

/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
	u8 tmp;
	tmp=key_scan(CONTROL_FREQUENCY);
	if(tmp==1)
	{
		Flag_Stop=!Flag_Stop;
	}		//单击控制小车的启停
	else if(tmp==2)
	{
		/* 修改原因：仅在停车时允许双击切换路线，防止行驶中途突然更改状态机。 */
		if (Flag_Stop) {
			Run_Mode = (Run_Mode + 1) % 4;
		}
	}
}
