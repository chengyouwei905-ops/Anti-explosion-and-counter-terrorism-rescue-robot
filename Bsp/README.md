# Bsp — 板级支持层（外设封装）

## 放什么

- HAL 外设的封装：`uart` / `can` / `spi` / `gpio` / `tim` / `adc`
- 命名前缀：`bsp_`
- 对外只暴露**逻辑编号 + 收发接口**，不要把 `UART_HandleTypeDef` 泄漏给上层

## 不放什么

- ❌ 业务逻辑 —— 文件里出现「底盘」「视觉坐标」就说明放错了
- ❌ 协议解析（帧头、CRC）

## 必须知道的约束

- `bsp_uart.c` **独占**实现 `HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback`，
  其它文件不要再实现这两个回调，否则链接时重复定义。
- 需要感知数据请用 `bsp_uart_set_rx_cb()` 或轮询 `bsp_uart_read()`

## 允许 include

`stm32h7xx_hal.h`、`common_*.h`（**不允许** include `App/` 或 `Protocol/`）
