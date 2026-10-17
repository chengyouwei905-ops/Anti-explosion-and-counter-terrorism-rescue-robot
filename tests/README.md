# 底盘运动模块本机测试

使用本机 GCC 编译实际的规划器、运动模块、航向闭环和 PID，仅替换 IMU 与电机发送接口：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/stubs -IApp/Inc -IAlgorithm/Inc tests/test_chassis_motion.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_pid.c -o build/test_chassis_motion.exe -lm
./build/test_chassis_motion.exe
```

测试覆盖参考位移、全向直线、左右圆弧、相对起始航向、非零切线朝向、原地多圈转动、连续指令、停止和弧度制闭环。运动仿真使用理想速度跟踪，不包含轮胎打滑、轮速限幅和通信延迟；不能替代实车标定和位置反馈验证。

串口 IT 发送测试编译实际 BSP、底盘应用、协议和运动学，仅替换 HAL：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/uart_stubs -IBsp/Inc -IApp/Inc -ICommon/Inc -IProtocol/Inc -IAlgorithm/Inc tests/test_uart_it.c Bsp/Src/bsp_uart.c Common/Src/ring_buffer.c App/Src/app_chassis.c Protocol/Src/proto_chassis.c Algorithm/Src/alg_chassis_kinematics.c -o build/test_uart_it.exe -lm
./build/test_uart_it.exe
```

覆盖初始化配置顺序、实际速度帧接入 IT、调用方缓冲复用、忙时不覆盖在途帧、发送完成后再次提交、不同串口缓冲隔离及错误返回。测试不模拟硬件 UART 中断时序；实车仍需验证。
