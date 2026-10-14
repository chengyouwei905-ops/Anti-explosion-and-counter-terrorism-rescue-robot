#include "app_chassis_closeloop.h"
#include <math.h>

int16_t basic_speed = 0;
/* 增益与限幅统一在 app_chassis_closeloop_init() 里配置，这里不再重复初始化 */
static PID_t angle_pid_t;

Chassis_CloseLoop_State_t chassis_closeloop_state ={0.0f,0.0f,0.0f};

void app_chassis_closeloop_init(void)
{
    /* 初始化 PID 控制器 */
    PID_Init(&angle_pid_t, 0.1f, 0.0f, 0.0f, -5.0f, 5.0f);
}

void app_chassis_closeloop_set_target(float_t vx, float_t vy,float_t yawangle)
{
    chassis_closeloop_state.speed_x = vx;
    chassis_closeloop_state.speed_y = vy;
    chassis_closeloop_state.yaw = yawangle;
}

/* 定时中断中定周期调用  需注意，底盘控制板已经完成了速度闭环，此处只需完成yaw角度闭环即可*/
void app_chassis_closeloop_update(void)
{
    //角度闭环流程
    float current_yaw[3];
    if(!app_imu_get_euler(current_yaw))
    {
        return;
    }
    angle_pid_t.Actual = current_yaw[2]*57.29578f;
    angle_pid_t.Target = chassis_closeloop_state.yaw;
    PID_Update(&angle_pid_t);

    /* 获取 PID 输出 */
    /* x和y直接输入目标值即可，下层已做速度闭环，角速度需要输入pid后的值 */
    float output_x, output_y, omega;
    output_x = chassis_closeloop_state.speed_x;
    output_y = chassis_closeloop_state.speed_y;
    omega = angle_pid_t.Out;

    /* 将 PID 输出转换为底盘速度指令 */
    app_chassis_move(output_x, output_y, omega);
}
