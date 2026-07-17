#include "ti_msp_dl_config.h"
#include "bsp_printf.h"
#include "bsp_siic.h"
#include "MPU6050.h"
#include "board.h"
#include "control.h"
#include "route.h"
#include "oled.h"

int main(void)
{
    uint8_t last_stop = 1U;

    SYSCFG_DL_init();
    SysTick_Init();
//    OLED_Init();

//    User_sIICDev.init();
//    MPU6050_initialize();
//    DMP_Init();

    DL_Timer_startCounter(PWM_0_INST);
    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    while (1) {
//        uint8_t imu_updated = 0U;

        /*
         * GPIO 中断只置位数据就绪标志，耗时的 I2C/FIFO 读取放在主循环。
         * Route_Update 只在取得一帧新 yaw 后执行，避免把同一帧数据重复当成
         * 多次“角度已稳定”判断。
         */
//        if (mpu6050_data_ready != 0U) {
//            mpu6050_data_ready = 0U;
//            Read_DMP();
//            imu_updated = 1U;
//        }

        /*
         * Route_Start 只能在“停止->启动”的边沿调用一次。
         * 如果把它放在 while 中无条件调用，里程会被不断清零，路线无法前进。
         */
//        if ((last_stop != 0U) && (Flag_Stop == 0)) {
//            Route_Select((RouteProgram)Run_Mode);
//            Route_Start(mpu6050.yaw);
//        }

//        if ((Flag_Stop == 0) && (imu_updated != 0U)) {
//            Route_Update(mpu6050.yaw);
//        }

//        last_stop = (Flag_Stop != 0) ? 1U : 0U;

					Set_PWM(MotorA.Motor_Pwm,0);
					printf("%f,%f,%f\r\n",MotorA.Target_Encoder,MotorA.Current_Encoder,MotorA.Motor_Pwm);
    }
}
