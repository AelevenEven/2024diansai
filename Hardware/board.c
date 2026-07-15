#include "ti_msp_dl_config.h"
#include "board.h"

volatile uint32_t tick_ms;

/*
 * 修改原因：原工程没有可靠的毫秒时基，MPU6050/DMP 的 get_ms() 得不到真实时间，
 * delay_ms() 也只能靠空循环估算。这里将 SysTick 配置成 1 ms 中断，统一提供系统时间。
 */
void SysTick_Init(void)//开启每1ms的系统计时
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 3);
}

void SysTick_Handler(void)
{
    tick_ms++;
}

uint32_t Board_GetMillis(void)//获取上电到现在的毫秒数
{
    return tick_ms;
}

//返回SysTick计数值
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


//ms阻塞延迟
void delay_ms(uint32_t ms)
{
	/* 修改原因：使用毫秒计数等待，避免 CPU 主频变化后原来的空循环延时严重不准。 */
	uint32_t start = Board_GetMillis();
	while ((uint32_t)(Board_GetMillis() - start) < ms) {
		__WFI();
	}
}


void delay_us(uint32_t us)
{
	/* 修改原因：按 CPU 实际频率换算周期，使微秒延时不再依赖固定魔数。 */
	if (us != 0U) {
		delay_cycles(us * (CPUCLK_FREQ / 1000000U));
	}
}

void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }

#if !defined(__MICROLIB)
//不使用微库的话就需要添加下面的函数
#if (__ARMCLIB_VERSION <= 6000000)
//如果编译器是AC5  就定义下面这个结构体
struct __FILE
{
	int handle;
};
#endif

FILE __stdout;

//定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
#endif

//printf函数重定义
//int fputc(int ch, FILE *stream)
//{
//	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
//	while( DL_UART_isBusy(UART_0_INST) == true );
//	
//	DL_UART_Main_transmitData(UART_0_INST, ch);
//	
//	return ch;
//}
