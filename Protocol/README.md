# Protocol — 通信协议层

## 放什么

- 帧的**打包 / 解包**：帧头、长度、序号、CRC 校验
- 一个链路一个文件：`proto_vision.c`（K230），以后 `proto_can.c` 等
- 命名前缀：`proto_`

## 不放什么

- ❌ 对 payload 做业务解释（"第 0 字节是 x_mm"）→ 那是 `App/` 的事
- ❌ 调用 HAL 或任何硬件相关代码
- ❌ 延时、阻塞、动态内存

## 改协议时只改这一层

视觉端换了帧格式？只改 `proto_vision.h` 的常量和 `proto_vision.c` 的状态机。
`App/` 层一行都不用动 —— 这就是分层的意义。

## 允许 include

`common_*.h`（**不允许** include `App/` 或 `Bsp/`）
