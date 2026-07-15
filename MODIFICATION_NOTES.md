# 本次工程修改说明

## 编码器方向系数

`Hardware/control.h` 中的 `ENCODER_A_SIGN` 和 `ENCODER_B_SIGN` 用来统一左右编码器的计数方向。

以小车实际向前推动为准：

- 向前时该轮累计里程增加：使用 `+1.0f`；
- 向前时该轮累计里程减少：使用 `-1.0f`。

当前工程暂设为：A 路 `+1.0f`、B 路 `-1.0f`。如果串口打印中小车向前而某一路距离为负，只修改对应的符号即可。

## 修改原则

- `board.c/h`：补充 1 ms SysTick，修正毫秒、微秒延时和 DMP 时间戳。
- `encoder.c/h`：合并 MSPM0 GROUP1 中断入口，避免重复定义；中断只做快速计数和置位。
- `MPU6050.c/h`：DMP 数据移到主循环读取，并修正加速度结构体赋值。
- `control.c/h`：按 TIMER0 的 100 Hz 周期换算速度，增加左右轮里程积分和路线起点清零。
- `route.c/h`：增加循迹段、偏航角无标线段和四圈路线状态机。
- `IR_Module.c/h`：补齐统一循迹入口，避免转弯时基础速度降为零。
- `empty.c`：整理初始化、DMP 读取、路线启动和调试输出主流程。
- `empty.syscfg`：关闭 UART 内部回环，保留真实串口调试输出。
- `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx`：将头文件搜索路径改为工程根目录和 `Hardware`，并把新增的 `route.c` 加入 Keil 编译组；否则源码虽存在，Keil 仍不会参与编译。

## 需要实车标定的参数

`Perimeter`、`MOTOR_GEAR_RATIO`、`ENCODER_RESOLUTION`、`Wheelspacing`、左右编码器符号、路线距离以及 `ROUTE_CLOCKWISE_YAW_SIGN` 都与实际底盘和 MPU6050 安装方向有关，代码中的数值是可运行的初始值，不是免标定的最终值。
