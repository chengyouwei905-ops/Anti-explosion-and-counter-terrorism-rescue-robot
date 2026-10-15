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
