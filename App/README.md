# App — 应用层（业务逻辑）

## 放什么

- 具体业务：视觉、底盘、机械臂、任务状态机
- 一个模块一对文件：`app_vision.h` / `app_vision.c`
- 命名前缀：`app_`

## 不放什么

- ❌ 直接调 HAL（`HAL_GPIO_WritePin` / `HAL_UART_Transmit`）→ 放 `Bsp/`
- ❌ 帧头、CRC 这类协议细节 → 放 `Protocol/`
- ❌ PID、运动学等纯计算 → 放 `Algorithm/`

## 允许 include

`bsp_*.h`、`proto_*.h`、`common_*.h`、自家头文件

## 当前内容

| 文件 | 职责 |
| --- | --- |
| `app_vision.c` | 视觉链路收发的统一入口（搬运 + 组包），**不做业务解释** |
| `app_roboarm.c` | 机械臂（总线舵机控制板）链路：组帧 + 下发，目前只做"用户主动发送" |

业务处理由调用方通过 `app_vision_set_rx_cb()` 注册回调实现，
payload 里装的是什么由业务层自己决定。

机械臂链路是单向的：发一条指令就调一次 `app_roboarm_xxx()`，没有 poll()，
也不需要注册回调（控制板上报的数据暂未解析）。

## 拿不准就放这里

放错了以后挪一下很便宜，纠结半小时的代价更大。

## 连续底盘路径

`app_chassis_motion_path_start(segments, count, limits)` 一次提交一组前进直线和带曲率过渡的转弯。参数会复制，单组最多16段；忙时返回 `MOTION_START_BUSY`，不覆盖在途路径。原有直线、圆弧和原地转动接口继续可用。

```c
const AlgPathSegment_t path[] = {
    {ALG_PATH_LINE, 400.0f, 0.0f, 0.0f, 250.0f, 300.0f, 300.0f},
    {ALG_PATH_ARC, 0.0f, 200.0f, 1.57079632679f, 140.0f, 200.0f, 250.0f},
    {ALG_PATH_LINE, 350.0f, 0.0f, 0.0f, 220.0f, 250.0f, 300.0f}
};
const AlgPathLimits_t limits = {30.0f, 0.9f, 2.0f, 400.0f};
Chassis_Motion_Start_t result = app_chassis_motion_path_start(path, 3U, &limits);
```

直线的 `distance` 是前进距离；转弯的 `radius` 和带符号 `angle` 定义名义圆弧终点，正角左转、负角右转。转弯使用两端五次多项式曲率过渡，中间保持定曲率，并归一化弧长以保持名义终点和转角；实际半径随位置变化。当前R200、90度、30mm过渡相对名义圆弧的最大几何偏移约6.25mm，场地净空仍需实车检查。

规划器根据后续限速做前后向速度约束，在段间保留非零速度；曲率变化项 `k' * v^2` 与切向加速度项 `k * a` 分别预留角加速度预算。`max_omega` 是参考角速度上限，应低于闭环1.2rad/s限幅，保留误差修正余量。参考角速度连续且角加速度有界；切向加速度仍采用分段常值，不承诺全程加加速度连续。

主循环继续每10ms先调用 `app_chassis_motion_update(0.01f)`，再调用 `app_chassis_closeloop_update()`。段间在同次更新中消费剩余时间，无串口等待、动态内存分配或每段停车。规划/更新均为有界计算；固件上仍应测量执行时间和控制周期抖动。

通过 `app_chassis_motion_path_get_sample()` 可读取当前段号、参考位移/航向、速度、角速度、预计总时长；这些是规划参考量，不是实际定位。`app_chassis_motion_get_result()` 区分正常完成、取消和 IMU 错误，任务不能只凭空闲状态宣布成功。

当前 `task.c` 将路线分为 MOVE1、MOVE2~11、MOVE12、MOVE13~15、MOVE16 五组。只在扫码、排爆、反恐、救援和终点停车1秒；四个作业状态仍各以1秒占位，后续接入视觉确认和实际作业完成条件。
