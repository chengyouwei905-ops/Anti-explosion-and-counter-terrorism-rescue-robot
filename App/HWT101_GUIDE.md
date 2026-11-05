# HWT101 底盘航向接入

HWT101 已替换底盘闭环、运动规划、视觉作业中的航向来源。三个模块统一调用
`app_chassis_closeloop_get_yaw()`，避免目标角和反馈角来自不同传感器。
BMI088 代码及原有接口保留，`main.c` 中 `APP_IMU_BMI088_ENABLE` 默认 0：
跳过初始化、8秒零偏标定和轮询。即使以后启用 BMI088，其初始化失败也不会阻止 HWT101 启动。
BMI088 的 CSV 调试分支同时受该开关约束，停用 BMI088 时不会占用机械臂串口或暂停路线。
`yawangle` 是 HWT101 连续航向；`testangle/temperature` 只在启用 BMI088 后更新。

## 接线与模块设置

按用户提供的[HWT101 通信手册](https://wit-motion.yuque.com/wumwnr/ltst03/seuiba)
截图中的角度输出格式实现，只读取 Z 轴角度。

| HWT101 | 达秒 STM32H723VGT6 |
| --- | --- |
| TX | PE0 / UART8_RX |
| RX | PE1 / UART8_TX |
| GND | GND，共地 |

UART8 由用户通过 CubeMX 生成，目前已核对 `main.c`、MSP、IRQ 和 `.ioc`：
**PE0=RX、PE1=TX、115200、8N1、无流控**，UART8 global interrupt 已启用。
模块自身的波特率必须一致。上位机应启用角度输出，建议设为 **100Hz** 并保存；
出厂默认 10Hz 也能解析，但原有 10ms 控制周期、PID 尤其微分项需要按实际反馈频率复测。
驱动只接收数据，不自动更改传感器波特率、输出频率或写入保存指令。

当前实车倒装，已将 `APP_IMU_HWT101_YAW_SIGN` 默认设为 `-1.0`，在驱动发布角度时统一取反，
对外仍为逆时针正。正装时改为 `1.0`。安装后先低速向左转，确认 `yawangle` 增大。

## 分层与调用

| 文件 | 职责 |
| --- | --- |
| `Bsp/Src/bsp_uart.c` | 复用原有单字节接收中断、512 字节环形缓冲、UART 错误后重装 |
| `Protocol/Src/proto_hwt101.c` | 11 字节流式组帧、加和校验、错帧重同步、0x53 航向提取 |
| `App/Src/app_imu_hwt101.c` | poll 搬运、连续角累计、首帧归零、超时、快照读取 |
| `App/Src/app_chassis_closeloop.c` | 对外统一航向接口及闭环反馈 |

接收路径为 `UART8_IRQHandler -> HAL_UART_RxCpltCallback -> rb_putc`；
解析路径为 `app_imu_hwt101_poll -> bsp_uart_read/rb_read -> proto_hwt101_feed`。
ISR 只搬运字节，解包和角度计算全部在主循环。每次 poll 只处理调用开始时已缓存的
字节，最多 511 字节，无动态内存、串口等待或延时。

`main.c` 的 USER CODE 区已在启动延时结束后调用 `app_imu_hwt101_init(&huart8)`，并在每轮主循环
开始调用 `app_imu_hwt101_poll()`，先更新航向再运行任务和底盘控制。
新增文件由现有 CMake 模块扫描自动加入构建。驱动只提供三个接口：
`app_imu_hwt101_init()`、`app_imu_hwt101_poll()`、`app_imu_hwt101_getangle()`。

```c
double yaw_rad;
if (app_imu_hwt101_getangle(&yaw_rad)) {
    /* 所有底盘目标与反馈共用这个连续弧度坐标。 */
}
```

角度帧格式为 `55 53 00 00 00 00 YawL YawH VL VH SUM`。
校验为前 10 字节加和的低 8 位；Yaw 是字节 6/7 的有符号小端值，
`yaw_rad = raw * PI / 32768`。只用 0x53 角度帧更新反馈，角速度帧不刷新航向时间。
支持跨 poll 的半帧、连续多帧、载荷内含 0x55，以及丢字节/错校验后重新找帧。

## 角度与失效语义

- 首个有效角度为本次运行的零点，不发送硬件归零命令。
- 相邻帧使用原始整数相减：大于 32768（180°）减 65536（360°），
  小于 -32768 加 65536；否则直接累计。累计值用 int64_t 保存，读取时转换为弧度。
  边界两侧来回跳变的增量相抵，不会凭空累加整圈。目标仍使用连续弧度，允许多圈转动。
- 默认超过 `APP_IMU_HWT101_TIMEOUT_MS=500` 未更新角度，读取返回 false；
  即使停止调用 poll，读取接口也会检查时间。未收到首帧时同样返回 false。
- RX 溢出或两次 poll 间隔超过 500ms，丢弃当时积压和半帧并使快照失效，
  等待后续新帧，避免旧报文被当作当前反馈。不在 ISR 并发期间重置环形缓冲索引。
- 失效时闭环输出零速度并清 PID，运动规划返回 `MOTION_RESULT_IMU_ERROR`。
  正在执行的任务按原有失败流程处理，不能将恢复来包视作运动完成。
- 短暂中断后来包沿用原角度基准，不重新归零。两个有效角度间实际转动需小于 180°；
  长时间断流期间若转过半圈以上，单圈报文无法确定遗漏圈数，应停车后重新初始化基准。
- double 快照读写使用短临界区，并恢复原 PRIMASK；poll 只允许主循环单一写者。

可在调试器观察 `s_imu_hwt101_ret`、`yawangle`。
若自由运行仍一直报 `RET_TIMEOUT=-3`，运行1～2秒后暂停，查看
`hwt101_rx_bytes`（poll从UART8缓冲区读出的累计字节数）和
`hwt101_angle_frames`（校验通过的角度帧数）。字节数为0时先查接线、供电与接收中断；
字节数增长但角度帧数为0时核对模块实际波特率及 `55 53` 报文，另查是否有轮询停顿/溢出导致丢弃积压。
`RET_IDLE` 表示没有新角度，属于正常结果；
`RET_TIMEOUT` 表示当前没有可用航向，`RET_ERROR` 表示本轮发现 RX 溢出。

## 验证

```powershell
./tests/run_host_tests.ps1 -TestNames imu_hwt101,chassis_motion,chassis_path,uart_it
```

`test_imu_hwt101` 使用真实 BSP 环形缓冲、HAL 接收回调、协议及 App 驱动，并联接实际
运动规划和闭环：覆盖碎帧、噪声、丢字节、错误校验、圈数累计、正反边界各2000次跳变、缓冲回绕、超时、
HAL tick 回绕、RX 溢出、UART 错误恢复和失效停车。实际串口电气连接与底盘 PID 仍需上板验证。

2026-11-05 验证结果：上述四项测试、HWT101 反向安装宏测试、
BMI088/CSV 回归及 `task_camera_test` 通过；用户重新生成 CubeMX 后的完整固件也编译链接通过，
产物为 `build/HWT101/STM32H723VGT6test.elf`。
`task_path`、`task_move12`、`task_distance` 未通过；换回修改前的运动/闭环/作业应用代码，
保留当前工作区路线参数后，同样在各自原断言失败，因此未更改这些路线或测试期望值。
