#include "route.h"
#include "control.h"
#include "IR_Module.h"
#include "MPU6050.h"
#include "led.h"
#include <math.h>

#define ROUTE_AB_LENGTH_M       1.000f
#define ROUTE_ARC_LENGTH_M      1.256637f
#define ROUTE_CD_LENGTH_M       1.000f
#define ROUTE_DIAGONAL_M        1.280625f
#define ROUTE_DIAGONAL_DEG      38.6598f
#define ROUTE_LINE_SPEED_MPS    0.220f
#define ROUTE_OPEN_SPEED_MPS    0.220f

/*
 * MPU6050 的偏航角正方向与底盘安装方向可能相反，集中用一个符号修正。
 * 实车顺时针转动时若 yaw 增大，就把 -1.0f 改为 +1.0f；反之保持当前值。
 */
#define ROUTE_CLOCKWISE_YAW_SIGN (-1.0f)

typedef enum {
    ROUTE_STATE_IDLE,
    ROUTE_STATE_OPEN_AB,
    ROUTE_STATE_LINE_BC,
    ROUTE_STATE_OPEN_CD,
    ROUTE_STATE_LINE_DA,
    ROUTE_STATE_OPEN_AC,
    ROUTE_STATE_TURN_CB,
    ROUTE_STATE_LINE_CB,
    ROUTE_STATE_OPEN_BD,
    ROUTE_STATE_TURN_DA,
    ROUTE_STATE_DONE
} RouteState;

static RouteProgram program = ROUTE_AB;
static RouteState state = ROUTE_STATE_IDLE;
static float line_start_distance;
static uint8_t remaining_laps;
static float nav_target_yaw;
static float nav_start_distance;
static float nav_distance;
static float nav_speed;
static uint8_t nav_turning;
static uint8_t nav_settle_count;

static float wrap_angle(float angle)
{
    /* 处理 yaw 从 +180 跳到 -180，防止跨界时误判成接近 360 度的大误差。 */
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static float clampf(float value, float low, float high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void point_signal(void)
{
    /*
     到达 A/B/C/D 后统一在这里发出提示。翻转 LED
     */
    LED_Toggle();
}

static void line_begin(void)//记录进入当前循迹路段时的里程
{
    /* 每次进入新的黑线圆弧前清除上一段圆弧遗留的转向状态。 */
    IR_LineReset();
    line_start_distance = Control_GetAverageDistance();
}

static uint8_t line_finished(float length)
{
    return (Control_GetAverageDistance() - line_start_distance) >= length;
}

static void nav_begin(float current_yaw, float relative_yaw, float distance, float speed)//无标线导航
{
    /* 以每段开始时的 yaw 为零基准 */
    nav_target_yaw = wrap_angle(current_yaw + relative_yaw);//设置目标偏航角
    nav_start_distance = Control_GetAverageDistance();
    nav_distance = distance;
    nav_speed = speed;
    nav_turning = 1U;
    nav_settle_count = 0U;
    MotorA.Target_Encoder = 0.0f;
    MotorB.Target_Encoder = 0.0f;
}

static uint8_t nav_update(float current_yaw)
{
    float error = wrap_angle(nav_target_yaw - current_yaw);
    float correction;

    if (nav_turning != 0U) {
        /*
         * 先转到目标偏航，连续稳定 3 次后再直行。
         * 修改原因：赛题规定小车只能前进，因此不再让一侧车轮反转原地转向；
         * 需要增大 yaw 时右轮前进、左轮停止，需要减小 yaw 时反之。
         */
        if (fabsf(error) > 2.5f) {//误差大于 2.5° 时使用前进式差速转向
            float turn_speed = 0.120f;
            if (error > 0.0f) {
                MotorA.Target_Encoder = 0.0f;
                MotorB.Target_Encoder = turn_speed;
            } else {
                MotorA.Target_Encoder = turn_speed;
                MotorB.Target_Encoder = 0.0f;
            }
            nav_settle_count = 0U;
            return 0U;
        }
        if (++nav_settle_count < 3U) return 0U;
        nav_turning = 0U;
        nav_start_distance = Control_GetAverageDistance();
    }

    if (nav_distance <= 0.0f) {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        return 1U;
    }
    if ((Control_GetAverageDistance() - nav_start_distance) >= nav_distance) {
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        return 1U;
    }

    /* 无标线时用偏航误差差速修正，并限幅，避免单侧轮速度反转。 */
    correction = clampf(0.0035f * error, -0.080f, 0.080f);
    MotorA.Target_Encoder = nav_speed - correction;
    MotorB.Target_Encoder = nav_speed + correction;
    return 0U;
}

void Route_Select(RouteProgram selected)//路线选择
{
    program = selected;
}

void Route_Start(float current_yaw_deg)
{
    /* 每次起跑清零里程并锁存当前 yaw，保证重复测试互不影响。 */
    Control_ResetOdometry();
    remaining_laps = (program == ROUTE_DIAGONAL_4) ? 4U : 1U;

    if (program == ROUTE_AB || program == ROUTE_LOOP) {
        /*
         * 赛题场地图中 A->B 没有黑线，使用起跑时的 yaw 保持直行 1 m，
         * 不能调用红外循迹。
         */
        state = ROUTE_STATE_OPEN_AB;
        nav_begin(current_yaw_deg, 0.0f,
            ROUTE_AB_LENGTH_M, ROUTE_OPEN_SPEED_MPS);
    } else {
        state = ROUTE_STATE_OPEN_AC;
        nav_begin(current_yaw_deg,
            ROUTE_CLOCKWISE_YAW_SIGN * ROUTE_DIAGONAL_DEG,
            ROUTE_DIAGONAL_M, ROUTE_OPEN_SPEED_MPS);
    }
    /* 起始点 A 同样需要 LED 提示。 */
    point_signal();
}

void Route_Update(float current_yaw_deg)
{
    /* 非阻塞状态机每次只推进一步，速度环中断可持续以 100 Hz 正常运行。 */
    if (Flag_Stop) return;

    switch (state) {
    case ROUTE_STATE_OPEN_AB:
        if (nav_update(current_yaw_deg)) {
            point_signal();
            if (program == ROUTE_AB) state = ROUTE_STATE_DONE;
            else { state = ROUTE_STATE_LINE_BC; line_begin(); }
        }
        break;
    case ROUTE_STATE_LINE_BC:
        IR_Module_Read();
        if (line_finished(ROUTE_ARC_LENGTH_M)) {
            point_signal();
            state = ROUTE_STATE_OPEN_CD;
            nav_begin(current_yaw_deg, 0.0f, ROUTE_CD_LENGTH_M, ROUTE_OPEN_SPEED_MPS);
        }
        break;
    case ROUTE_STATE_OPEN_CD:
        if (nav_update(current_yaw_deg)) {
            point_signal();
            state = ROUTE_STATE_LINE_DA;
            line_begin();
        }
        break;
    case ROUTE_STATE_LINE_DA:
        IR_Module_Read();
        if (line_finished(ROUTE_ARC_LENGTH_M)) {
            point_signal();
            if (program == ROUTE_DIAGONAL_4 && remaining_laps > 1U) {
                remaining_laps--;
                state = ROUTE_STATE_OPEN_AC;
                nav_begin(current_yaw_deg,
                    ROUTE_CLOCKWISE_YAW_SIGN * ROUTE_DIAGONAL_DEG,
                    ROUTE_DIAGONAL_M, ROUTE_OPEN_SPEED_MPS);
            } else {
                state = ROUTE_STATE_DONE;
            }
        }
        break;
    case ROUTE_STATE_OPEN_AC:
        if (nav_update(current_yaw_deg)) {
            point_signal();
            state = ROUTE_STATE_TURN_CB;
            nav_begin(current_yaw_deg,
                -ROUTE_CLOCKWISE_YAW_SIGN * ROUTE_DIAGONAL_DEG,
                0.0f, ROUTE_OPEN_SPEED_MPS);
        }
        break;
    case ROUTE_STATE_TURN_CB:
        if (nav_update(current_yaw_deg)) {
            state = ROUTE_STATE_LINE_CB;
            line_begin();
        }
        break;
    case ROUTE_STATE_LINE_CB:
        IR_Module_Read();
        if (line_finished(ROUTE_ARC_LENGTH_M)) {
            point_signal();
            state = ROUTE_STATE_OPEN_BD;
            nav_begin(current_yaw_deg,
                -ROUTE_CLOCKWISE_YAW_SIGN * ROUTE_DIAGONAL_DEG,
                ROUTE_DIAGONAL_M, ROUTE_OPEN_SPEED_MPS);
        }
        break;
    case ROUTE_STATE_OPEN_BD:
        if (nav_update(current_yaw_deg)) {
            point_signal();
            state = ROUTE_STATE_TURN_DA;
            nav_begin(current_yaw_deg,
                ROUTE_CLOCKWISE_YAW_SIGN * ROUTE_DIAGONAL_DEG,
                0.0f, ROUTE_OPEN_SPEED_MPS);
        }
        break;
    case ROUTE_STATE_TURN_DA:
        if (nav_update(current_yaw_deg)) {
            state = ROUTE_STATE_LINE_DA;
            line_begin();
        }
        break;
    case ROUTE_STATE_DONE:
        Flag_Stop = 1;
        MotorA.Target_Encoder = 0.0f;
        MotorB.Target_Encoder = 0.0f;
        break;
    default:
        break;
    }
}

uint8_t Route_IsComplete(void)
{
    if (state == ROUTE_STATE_DONE) return 1U;
    return 0U;
}
