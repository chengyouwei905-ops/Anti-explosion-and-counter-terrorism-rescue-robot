# IMU 每秒串口记录

当前在 `App/Inc/app_imu_uart_debug.h` 启用 `APP_IMU_UART_DEBUG_ENABLE=1`。
该模式借用 USART10，仅输出 CSV；跳过机械臂初始化动作、初始移动及主循环的任务/底盘运动控制。
机械臂 TX 暂时接到 USB 转串口接收端，完成采样后恢复接线，将宏改成 `0` 并重新编译下载即可恢复原业务。

## 接线和接收设置

- MCU `PE3 / USART10_TX` 接调试器的 **UART/VCP RX**，并连接两端 GND；SWDIO/SWCLK 不接收串口日志。
- 使用兼容 MCU 的 **3.3V TTL 电平**，避免使用 RS-232 电平接口。
- 串口工具选择对应 COM 端口，**9600 波特率、8 数据位、无校验、1 停止位、无流控**。
- 选择文本/ASCII 显示，并保存接收内容为 `.csv`；先打开接收，再复位 MCU，可以保留表头。

### VOFA+ 设置

- 波特率使用 **9600**，与 USART10 固件设置一致；不能使用 115200。
- 只查看接收文本时选择 **RawData**；它不解析曲线数据。
- 绘制曲线时选择 **FireWater**；当前逗号分隔、CRLF 结尾的数据行直接符合该协议。
- 三列分别对应通道 0=时间毫秒、通道 1=yaw 弧度、通道 2=温度。绘图时隐藏通道 0，避免大时间数值撑大纵轴。
- 数据每秒一条，波形控件的 `Δt` 设为 **1000ms**；切换后清空旧数据再采集。
- 同一时刻只用一个程序打开 COM 口；关闭脚本/串口工具后再连接 VOFA+。

协议说明：[RawData](https://www.vofa.plus/docs/learning/dataengines/rawdata/)、
[FireWater](https://www.vofa.plus/docs/learning/dataengines/firewater/)。

## 输出格式

```csv
time_ms,yaw_rad,temp_c
1000,0.000123456,30.125
2000,0.000234567,30.250
```

- `time_ms`：日志初始化后的毫秒数，第一条数据正常约为 1000；这是日志时间，不是物理上电时间。
- `yaw_rad`：当前连续 yaw，弧度，9 位小数；正负号保留。
- `temp_c`：当前芯片温度，摄氏度，3 位小数。
- 当前发送的是每秒一个最新快照，不是这一秒内的平均值，也不包含原始陀螺角速度。

`snprintf()` 将 float 转成可读文本，返回文本的字节长度；发送长度不包括末尾的 `\0`。
工程使用 newlib-nano，CMake 已加入 `-u _printf_float`，使 `%f` 正常输出。
直接发送 `float values[2]` 会得到二进制字节；长度 `2` 只会发送两个字节，并不是两个浮点数。

发送使用 `bsp_uart_send_it()`：BSP 先复制局部文本，然后在 UART 中断里完成传输，主循环继续轮询 IMU。
9600 波特率下典型一行约占几十毫秒的线路时间，每秒一行足够；主循环不等待线路发送完成。
端口忙或发送失败时跳过本行，下一秒再试；`s_imu_uart_debug_ret` 可观察最近一次尝试的返回码。

## 采样方法

固定散热条件，车体保持水平静止，电机停止，持续接收至温度稳定。采样中让程序连续运行，避免断点暂停。
每轮同时记录一次初始化诊断值：`s_calibrated`、`s_gyro_cali_start_temp_c`、`s_gyro_cali_end_temp_c`、
`s_gyro_ref_temp_c`、`s_gyro_bias[2]` 和当前 `APP_IMU_GYRO_TEMP_COEFF_Z`。
这些诊断字段保留在调试器中，不混入三列 CSV。

## 验证范围

`tests/test_imu_uart_debug.c` 验证实际文本模块与 BSP 的组合：正负数/精度、字节长度、局部缓冲区寿命、
一秒限频、忙时不覆盖和不连续重试、无效 IMU 数据、发送失败恢复，以及 HAL tick 回绕。
编译和主机测试不替代实际接线、COM 端口及收发验证。
