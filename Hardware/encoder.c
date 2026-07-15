#include "encoder.h"
#include "MPU6050.h"

volatile int32_t Get_Encoder_countA;
volatile int32_t Get_Encoder_countB;
/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
	uint32_t status;

	/*
	 * 修改原因：MSPM0 的 GPIOA、GPIOB 都汇入 GROUP1。原工程分别定义中断函数会发生
	 * GROUP1_IRQHandler 重复定义，因此在一个入口内按 pending source 分发处理。
	 */
	switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
	case GPIO_MULTIPLE_GPIOA_INT_IIDX:
		status = DL_GPIO_getEnabledInterruptStatus(
			GPIOA, MPU6050_INT_PIN_PIN | ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);

		/*
		 * 修改原因：I2C/FIFO 读取耗时较长，在 GPIO ISR 内调用 Read_DMP() 容易阻塞编码器
		 * 和速度环中断。此处只置位，真正的 DMP 读取放到主循环执行。
		 */
		if ((status & MPU6050_INT_PIN_PIN) != 0U) {
			mpu6050_data_ready = 1U;
		}
		if ((status & ENCODERA_E1A_PIN) != 0U) {
			Get_Encoder_countA += DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN) ? 1 : -1;
		}
		if ((status & ENCODERA_E1B_PIN) != 0U) {
			Get_Encoder_countA += DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN) ? -1 : 1;
		}
		DL_GPIO_clearInterruptStatus(GPIOA, status);
		break;

	case ENCODERB_INT_IIDX:
		status = DL_GPIO_getEnabledInterruptStatus(
			ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
		if ((status & ENCODERB_E2A_PIN) != 0U) {
			Get_Encoder_countB += DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN) ? 1 : -1;
		}
		if ((status & ENCODERB_E2B_PIN) != 0U) {
			Get_Encoder_countB += DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN) ? -1 : 1;
		}
		DL_GPIO_clearInterruptStatus(ENCODERB_PORT, status);
		break;

	default:
		break;
	}
}
