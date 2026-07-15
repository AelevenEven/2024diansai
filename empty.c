#include "ti_msp_dl_config.h"
#include "bsp_printf.h"
#include "bsp_siic.h"
#include "MPU6050.h"
#include "board.h"
#include "control.h"
#include "route.h"
#include "oled.h"

/*
 * 临时 PID 调试参数：一次只调一台电机，串口每行固定输出三列：
 * 目标速度(mm/s),实际速度(mm/s),PWM输出
 */
#define PID_DEBUG_MOTOR_A          0
#define PID_DEBUG_MOTOR_B          1
#define PID_DEBUG_MOTOR            PID_DEBUG_MOTOR_A
#define PID_DEBUG_TARGET_MPS       0.200f
#define PID_DEBUG_PRINT_PERIOD_MS  20U

static uint32_t last_report_ms;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    /* PID 调试期间关闭 OLED、MPU6050/DMP 和路线初始化，避免占用处理时间。 */
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
        /*
         * PID 调试时不调用 Read_DMP()、Route_Start()、Route_Update()、IR_Module_Read()。
         * 单击按键后 Flag_Stop 变为 0，速度环开始让选中的电机跟随固定目标速度。
         */
        /* 以下是完整循迹/航向功能的原调用位置，现仅为调速度环而注释保留： */
//      Read_DMP();
//      Route_Select(ROUTE_AB);
//      Route_Start(mpu6050.yaw);
//      Route_Update(mpu6050.yaw);
//      IR_Module_Read();

        if (Flag_Stop == 0) {
#if PID_DEBUG_MOTOR == PID_DEBUG_MOTOR_A
            MotorA.Target_Encoder = PID_DEBUG_TARGET_MPS;
            MotorB.Target_Encoder = 0.0f;
#else
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = PID_DEBUG_TARGET_MPS;
#endif
        } else {
            MotorA.Target_Encoder = 0.0f;
            MotorB.Target_Encoder = 0.0f;
        }

        /*
         * CSV 数据，无表头，直接导入串口助手/Serial Studio：
         * target_mm_s,actual_mm_s,output
         * output 是 PID 解算、限幅后实际传给 Set_PWM() 的控制量。
         */
        if ((uint32_t)(Board_GetMillis() - last_report_ms) >= PID_DEBUG_PRINT_PERIOD_MS) {
            last_report_ms = Board_GetMillis();
#if PID_DEBUG_MOTOR == PID_DEBUG_MOTOR_A
            printf("%d,%d,%d\r\n", (int)(MotorA.Target_Encoder * 1000.0f),
                   (int)(MotorA.Current_Encoder * 1000.0f), (int)(-MotorA.Motor_Pwm));
#else
            printf("%d,%d,%d\r\n", (int)(MotorB.Target_Encoder * 1000.0f),
                   (int)(MotorB.Current_Encoder * 1000.0f), (int)MotorB.Motor_Pwm);
#endif
        }
    }
}
