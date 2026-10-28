# 底盘运动模块本机测试

使用本机 GCC 编译实际的规划器、运动模块、航向闭环和 PID，仅替换 IMU 与电机发送接口：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/stubs -IApp/Inc -IAlgorithm/Inc tests/test_chassis_motion.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_chassis_path.c Algorithm/Src/alg_pid.c -o build/test_chassis_motion.exe -lm
./build/test_chassis_motion.exe
```

测试覆盖参考位移、全向直线、布尔量选择左右圆弧、当前航向平滑起步、非零起始航向、原地多圈转动、连续指令、停止和弧度制闭环。运动仿真使用理想速度跟踪，不包含轮胎打滑、轮速限幅和通信延迟；不能替代实车标定和位置反馈验证。

串口 IT 发送测试编译实际 BSP、底盘应用、协议和运动学，仅替换 HAL：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/uart_stubs -IBsp/Inc -IApp/Inc -ICommon/Inc -IProtocol/Inc -IAlgorithm/Inc tests/test_uart_it.c Bsp/Src/bsp_uart.c Common/Src/ring_buffer.c App/Src/app_chassis.c Protocol/Src/proto_chassis.c Algorithm/Src/alg_chassis_kinematics.c -o build/test_uart_it.exe -lm
./build/test_uart_it.exe
```

覆盖初始化配置顺序、实际速度帧接入 IT、调用方缓冲复用、忙时不覆盖在途帧、发送完成后再次提交、不同串口缓冲隔离及错误返回。测试不模拟硬件 UART 中断时序；实车仍需验证。

# 视觉指令与任务状态机本机测试

编译实际任务、协议和运动模块，替换 HAL 类型、时钟、IMU 和电机输出：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/vision_stubs -ICore/Inc -Itests/stubs -IApp/Inc -IBsp/Inc -ICommon/Inc -IProtocol/Inc -IAlgorithm/Inc tests/test_task_vision.c Protocol/Src/proto_vision.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_chassis_path.c Algorithm/Src/alg_pid.c -o build/test_task_vision.exe -lm
./build/test_task_vision.exe
```

覆盖精确指令匹配、非 NUL 结尾与满载 payload、半帧与 CRC 错误拒绝、事件锁存、等待启动及重复启动忽略。

`OFFSET,X=12.5,Y=-3.25` 的测试覆盖正负数、科学计数法、最新值覆盖、非法参数、溢出/下溢、NaN/Inf、内嵌 NUL 和尾部垃圾拒绝。偏移目前只保存，不执行纠偏或作业；测试不验证硬件串口。

# 连续路径与曲率过渡

```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Itests/stubs -IApp/Inc -IAlgorithm/Inc tests/test_chassis_path.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_chassis_path.c Algorithm/Src/alg_pid.c -o build/test_chassis_path.exe -lm
./build/test_chassis_path.exe
```

覆盖非零起止速度、不可达边界速度拒绝、前后向速度规划、段间不停车、转弯终点和航向、角速度/角加速度/向心加速度约束、16段/48阶段容量、大步长跨阶段、输入复制、忙时拒绝、取消及 IMU 失败；另有40组确定性随机路径检查位置与速度一致性。

```powershell
gcc -std=c11 -O2 -Wall -Wextra -Werror -pedantic -Itests/vision_stubs -ICore/Inc -Itests/stubs -IApp/Inc -IBsp/Inc -ICommon/Inc -IProtocol/Inc -IAlgorithm/Inc tests/test_task_path.c Protocol/Src/proto_vision.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_chassis_path.c Algorithm/Src/alg_pid.c -o build/test_task_path.exe -lm
./build/test_task_path.exe
```

整圈测试使用实际任务和规划代码，检查全部16个移动状态、11个不停顿衔接、扫码/排爆/反恐/救援/终点五处停车、1秒停稳、时钟回绕、IMU 失效、控制停滞超时和取消后不继续作业。当前1秒作业占位下，理想模型约69秒；这不包含真实作业时间，也不包含轮胎打滑、传感器误差和通信延迟。
