/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "bsp_printf.h"
#include "bsp_siic.h"
#include "MPU6050.h"
#include "board.h"
#include "oled.h"
#include "IR_Module.h"

int32_t encoderA_cnt,PWMA,encoderB_cnt,PWMB;

int main(void)
{
    SYSCFG_DL_init();
	OLED_Init();
	/* mpu6050 */
//		pIICInterface_t siic = &User_sIICDev;
//	siic->init();
//	NVIC_EnableIRQ(MPU6050_INT_IRQN);
//	MPU6050_initialize();           //MPU6050初始化	
//	DMP_Init();  	//初始化DMP 
	DL_Timer_startCounter(PWM_0_INST);
	NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
	NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    while (1) {
			/*  MPU6050  */
//			printf("Pitch:%.2lf\tRoll:%.2lf\tYaw:%.2lf\r\n",mpu6050.pitch,mpu6050.roll,mpu6050.yaw);
			
//		printf("%d  %d\n\r",encoderA_cnt,encoderB_cnt);
//		delay_ms(100);
			
			/*  OLED  */
//			memset(OLED_GRAM,0, 128*8*sizeof(u8));
//			
//			OLED_ShowNumber(8,8,6,2,12);
//			OLED_ShowString(16,16,"Hello,world");
//			OLED_Refresh_Gram();
			
			/*  红外四路巡线  */
//			IR_Module_Read();
    }
}

////外部中断  dmp相关内容
//void GROUP1_IRQHandler(void)//Group1的中断服务函数
//{
//	uint32_t portE2_intp = DL_GPIO_getEnabledInterruptStatus(MPU6050_PORT,MPU6050_INT_PIN_PIN);

//	//MPU6050下降沿中断,频率可以通过DMP配置,默认200Hz
//	if( (portE2_intp&MPU6050_INT_PIN_PIN)==MPU6050_INT_PIN_PIN )
//	{
//		Read_DMP();
//		mpu6050.pitch = Roll;
//		mpu6050.roll = Pitch;
//		mpu6050.yaw = Yaw;
//		mpu6050.gyro.x=gyro[0];
//		mpu6050.gyro.y=gyro[1];
//		mpu6050.gyro.z=gyro[2];
//		mpu6050.accel.x=accel[0];
//		mpu6050.accel.y=accel[0];
//		mpu6050.accel.z=accel[0];
//		//清空标志位
//		DL_GPIO_clearInterruptStatus(MPU6050_PORT,MPU6050_INT_PIN_PIN);
//	}
//}