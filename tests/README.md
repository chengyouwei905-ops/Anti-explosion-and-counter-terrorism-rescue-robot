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

运行全部主机测试（GCC 不在 PATH 时指定实际编译器路径）：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:\compiler\msys2\ucrt64\bin\gcc.exe'
```

`test_imu.c` 使用真实 IMU 应用和 AHRS 算法，模拟传感器与 DWT/HAL；以非零三轴温度系数验证标定首尾均温、非匀速升温时零偏与基准温度对应、升温/降温、真实转动保留、读取失败保持快照、重新初始化，以及标定晃动/读取失败后禁用补偿并清除标定温度。实际温度系数仍需在本机静止测量后填写。

`test_imu_uart_debug.c` 编译实际 IMU 文本日志模块、UART BSP 和环形缓冲，验证 CSV 精度/长度、中断发送缓冲寿命、一秒限频、忙/错误恢复、无效数据和 tick 回绕。接线和使用方法见 `App/IMU_UART_DEBUG.md`。

`test_vision_result.c` 验证交接文档的五个 HEX 样例、分片/粘包、CRC、超长/短包、版本/保留位、可选字段长度、NaN/Inf、扫描成功与失败、候选观测拒绝、机械臂 X 轴换向、正解/齿轮比例、坐标变换、平移符号、QR/法向 yaw 的可观测性。

`test_task_vision.c` 编译真实 task、作业状态机、app_vision、BSP/环形缓冲、协议、坐标换算和底盘控制；只替换 HAL、IMU/电机、机械臂 hook 和物理场景。覆盖真实发送 SEQ、回绕/忙/失败、旧结果与重复终态、观测包不控制、微调后复测、全赛程、时钟回绕、激光与微调中总停、延迟结果、23秒超时、机构参数缺失、机械臂等待超时、微调不收敛及救援 reason=9。测试中的机械臂动作完成和相机测量均为模拟，不是实际硬件验证。旧 OFFSET 文本不再用于控制。

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

`test_task_path.c` 专门验证原路线，使用作业已完成的桩，检查全部16个移动状态及顺序、11个不停顿衔接、五处停车、1秒停稳、时钟回绕、IMU失效、控制停滞超时和取消。纯路线理想模型约65秒；正式作业的真实模块由上面的 test_task_vision 单独覆盖。仿真时间不代表实车完赛时间。
