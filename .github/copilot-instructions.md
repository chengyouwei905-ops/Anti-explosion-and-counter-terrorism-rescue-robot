# 项目约定（AI 助手与人都必须遵守）

STM32H723VGT6 机器人主控：底盘控制 + 机械臂控制 + 任务状态机。
构建：CMake + Ninja，`cmake --build build/Debug`（产物 `build/Debug/STM32H723VGT6test.elf`）。

## 一、目录分层（依赖只能自上而下，禁止反向）

```
App → Protocol → Bsp → Drivers(ST HAL)
App → Algorithm
所有层 → Common
```

| 目录 | 放什么 | 禁止 |
| --- | --- | --- |
| `Core/` `Drivers/` `cmake/` | CubeMX / ST 生成 | 手改（`Core/` 只允许写在 `USER CODE BEGIN/END` 之间） |
| `App/` | 业务逻辑：视觉、底盘、机械臂、状态机 | 直接调 HAL |
| `Bsp/` | HAL 外设封装（uart/can/spi/gpio/tim） | 业务逻辑、协议解析 |
| `Protocol/` | 帧打包解包（帧头/长度/CRC） | 业务语义、调 HAL |
| `Common/` | 通用工具（与硬件无关） | include 项目内其它模块 |
| `Algorithm/` | PID、运动学（用到再建，不要提前建空目录） | 调 HAL |

## 二、硬性规则

1. **不要往 `Core/`、`Drivers/`、`cmake/` 里加用户代码**。用户代码一律放
   `App/` `Bsp/` `Protocol/` `Common/`。
2. **新增 `.c/.h` 不用改 CMakeLists.txt**。根 `CMakeLists.txt` 已用
   `GLOB_RECURSE CONFIGURE_DEPENDS` 自动收集这五个目录的 `Src/*.c` 并添加 `Inc` 路径。
3. **不要实现空的占位目录**。目录只在真的放文件时创建。
4. `bsp_uart.c` **独占**实现 `HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback`，
   其它文件不要再实现。上层感知数据用 `bsp_uart_set_rx_cb()` 或 `bsp_uart_read()`。
5. **禁止把结构体强转为 `uint8_t*` 当协议 payload 发送**（结构体有对齐 padding
   和内部字段）。协议 payload 必须逐字段用 `le_i16_put()` 等显式序列化。
6. 命名前缀：`app_` / `bsp_` / `proto_`；回调类型统一 `xxx_cb_t`；
   头文件用 `#ifndef __XXX_H` 卫哨，并包 `extern "C"`。
7. 注释与提交信息用中文。提交信息格式：`type(scope): 中文描述`
   （type: feat/fix/refactor/docs/build/chore）。
8. 编译必须零告警（`-Wall` 已开启）。

## 三、硬件事实（改动前先确认）

| 项 | 值 |
| --- | --- |
| MCU | STM32H723VGT6，HSI 64MHz（无外部晶振），VOS Scale3 |
| UART7 | K230 视觉链路：**PE8 = TX、PE7 = RX**，115200-8-N-1，中断接收 |
| `Power_OUT1_EN` / `Power_OUT2_EN` | PC14 / PC13，高电平使能 |

## 四、⚠️ CubeMX 相关（踩过坑）

- 在 VS Code 里对 CubeMX 生成的文件（`main.c` / `main.h`）有**未保存改动**时，
  保存旧缓冲区会覆盖 CubeMX 新生成的代码（丢失 `huart7`、`MX_xxx_Init`）。
  **Generate Code 之后先选 "Revert File"**。
- 改完 `.ioc` 重新生成前，建议先 `git commit` 存一次档，方便 `git restore` 回退。

## 五、视觉链路现状

- `App/Src/app_vision.c` **只做串口收发**（搬运 + 组包），不做任何业务解释。
  业务处理由调用方通过 `app_vision_set_rx_cb()` 注册回调实现。
- 调用方只需三件事：
  `app_vision_init(&huart7)` → 主循环 `app_vision_poll()` → 发数据 `app_vision_send(cmd, payload, len)`。
- payload 的字段含义由业务层自行定义，`Protocol` 层不关心。
- **不要预先编写业务代码**（底盘状态、目标位姿、PID、状态机等），
  等用户真正开始写那个模块时再加。当前阶段目标是保持精简。

## 六、详细文档

`Doc/项目结构说明.md`（分层规则、数据流、帧协议、Git 规范）
`Doc/Git速查.md`（日常操作速查）
