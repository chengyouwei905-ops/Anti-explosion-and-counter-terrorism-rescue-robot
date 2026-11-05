# 底盘运动模块本机测试

2026-11-06逐位停稳命中即作业：`./tests/run_host_tests.ps1 -TestNames task_stop_scan,task_config,task_camera_test,bomb_arm_split,chassis_motion`。
`task_stop_scan`覆盖排爆/打靶/救援、目标在1/2/3位、时钟回绕及有/无纠偏的36组场景，
验证命中即结束扫描、每位只在停车时请求、放桶/照射/夹紧救援、回扫描起点及航向，再进入下一段MOVE。
另检查旧SEQ/错位回包、三位无目标、模型故障、无回包保持停车和超时、航向变化及总停。
K230交付代码回归：`python -B tests/test_k230_stop_scan.py`，模拟检测结果与时钟，验证真实factory/manager/protocol，
涵盖QR目标缓存、三种作业/停车位、非目标、空位、稳定帧、歧义、异常、取消、暂停及旧空载荷兼容。
共享视觉场景桩已更新为当前3帧定位结果和4字节逐位扫描请求；硬件图像识别和实车运动仍需联调。

排爆/打靶/救援机械臂拆分回归：`./tests/run_host_tests.ps1 -TestNames bomb_arm_split`。
编译实际port、机械臂应用、正逆解及协议，验证夹取后/放置前/放置后收回三处先ID2/3/4再ID1，
通过发送帧及正解检查中间/最终坐标、夹爪时机、每段等待、第二段BUSY重试/错误/取消/超时和tick回绕，
并确认观察动作仍同步四关节。独立执行该回归，避免历史 `work_port` 参考坐标断言掩盖新动作验证。
打靶覆盖三个目标位、正常计时及tick回绕：由观察姿态先保持yaw抬至QR高度/伸展姿态，
再单独转yaw回QR，检查中间/最终正解、夹爪保持及每段等待、BUSY重试、错误、取消和超时。
爆炸物正解预期使用当前夹取(200,0,-23,-1.25)及放置(-174,0,20,-1.2)配置。
救援覆盖三个目标位、正常计时及tick回绕：夹取闭爪后及搬运回位均先ID2/3/4、再ID1，
检查中间/最终正解、夹爪保持闭合、每段等待、BUSY重试、错误、取消和超时，并确认禁止放置松爪。
救援沿用夹取(195,0,78,-0.5)与QR回位(0,260,60,0)参数；分段收臂先保持yaw，末端Z并不升高。

作业配置回归：`./tests/run_host_tests.ps1 -TestNames task_config`。
使用实际port配置及作业状态机，模拟机械臂完成，验证四类作业通过启动校验并发送QR/扫描命令；
纯平移时忽略未启用的yaw限幅/容差，包括零、超界、NaN/Inf，仍可平移纠偏和视觉复测，覆盖tick回绕。
启用QR轴或靶法向yaw纠偏时，非法参数仍在动作前拒绝，并检查第三个靶位及合法yaw纠偏。

当前正式二次纠偏已按用户要求换回 `app_chassis_motion_linear_move()`，保持最大速度80mm/s、独立加减速度200mm/s²。`test_task_distance.c` 保留历史文件名，现验证无编码器反馈仍可时间纠偏、完成后视觉复测、四次不收敛上限、恢复规划时长超时、tick回绕及总停；独立里程接口仍由 `test_chassis_motion` 覆盖。下方里程接入记录为历史过程。

2026-11-03 实测速度接入：正式二次纠偏设为最大80mm/s、独立加减速度200mm/s²，扫描/换位/返回仍用原accel=100。保留用户当前末段下限50mm/s、视觉容差20mm。`chassis_motion/chassis_path/task_distance/task_camera_test` 四组通过，Debug固件构建通过。`work_port` 在第138行参考坐标断言失败：测试仍期望QR(0,1250,55)等旧值，当前正式参考坐标已更新；将速度恢复为修改前40mm/s的对照也在同一处失败，本次未改参考坐标或该断言。

使用本机 GCC 编译实际的规划器、运动模块、航向闭环和 PID，仅替换 IMU 与电机发送接口：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/stubs -IApp/Inc -IAlgorithm/Inc tests/test_chassis_motion.c App/Src/app_chassis_motion.c App/Src/app_chassis_closeloop.c Algorithm/Src/alg_motion_profile.c Algorithm/Src/alg_chassis_path.c Algorithm/Src/alg_pid.c -o build/test_chassis_motion.exe -lm
./build/test_chassis_motion.exe
```

测试覆盖参考位移、全向直线、布尔量选择左右圆弧、当前航向平滑起步、非零起始航向、原地多圈转动、连续指令、停止和弧度制闭环。运动仿真使用理想速度跟踪，不包含轮胎打滑、轮速限幅和通信延迟；不能替代实车标定和位置反馈验证。

2026-11-03 新增仅视觉纠偏使用的里程接口：`test_chassis_motion` 覆盖20%速度执行、1s启动延迟、各方向/航向、到量停止、原时间接口行为保留、里程缺失/过期/清零/回退、停滞、总超时及取消。`test_uart_it` 使用实际串口收帧与里程解算验证反馈时效、静止帧刷新、无效帧拒绝、tick回绕、全局清零代次及原0.2mm死区。

`test_task_distance.c` 复用真实任务/作业/协议/运动模块，模拟实际速度仅10%，验证纠偏超过旧时间界限仍等待里程、到量后复测成功及tick回绕；同时检查反馈故障、总停，以及无编码器反馈时扫描/放桶/返回仍使用原时间移动。该文件也执行高度诊断、启停/超时/不收敛及搬运的已有回归，不执行存在历史失败的全路线测试。

只运行本次相关测试：

```powershell
./tests/run_host_tests.ps1 -TestNames chassis_motion,chassis_path,uart_it,task_distance,task_camera_test
```

本次五组相关回归与Debug固件编译链接通过；未烧录。完整 `test_task_vision` 仍在后续作业进入 `WORK_ERROR/WORK_ERR_ALIGNMENT`，改动前的时间纠偏对照也在该处失败，因此未将完整赛程记为通过。

圆弧平移回归覆盖96种起始航向/切线方向/正负扫角组合，独立检查圆周轨迹、终点和车头保持；另覆盖航向扰动下的固定方向平移、后继直线、零扫角、忙时不覆盖、非法数值/溢出/不可规划输入、取消、IMU失效及大时间步。连续路径回归同时检查保持车头的左右弯路径、非零段间速度、采样切线与车头的区别、输入复制和生命周期。

2026-11-03 圆弧平移接口验证：`test_chassis_motion`、`test_chassis_path` 及 Debug 固件编译链接通过。额外运行的 `test_task_path` 在第100行 `error<15` 断言失败；使用本次修改前的 motion 源码也在同一处失败，当前路线含实测距离修正而该测试仍使用旧停车坐标。此项记录为既有路线回归失败，本次未切换或调整 `task.c`，未烧录验证。

串口 IT 发送测试编译实际 BSP、底盘应用、协议和运动学，仅替换 HAL：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -pedantic -Itests/uart_stubs -IBsp/Inc -IApp/Inc -ICommon/Inc -IProtocol/Inc -IAlgorithm/Inc tests/test_uart_it.c Bsp/Src/bsp_uart.c Common/Src/ring_buffer.c App/Src/app_chassis.c Protocol/Src/proto_chassis.c Algorithm/Src/alg_chassis_kinematics.c -o build/test_uart_it.exe -lm
./build/test_uart_it.exe
```

覆盖初始化配置顺序、实际速度帧接入 IT、调用方缓冲复用、忙时不覆盖在途帧、发送完成后再次提交、不同串口缓冲隔离及错误返回。测试不模拟硬件 UART 中断时序；实车仍需验证。

2026-11-03 实车纯横移测试发现：`linear_move(10,-PI/2,20,100,100)` 原应向右，却实际向左。现于麦轮逆解统一反转横向轮速组合，并同步修改正解的X分量；前后和角速度组合保留。UART测试验证左右横移的实际电机ID发送帧、正逆解一致性及前后/转向组合；修正后物理方向仍需烧录复测。

# 视觉指令与任务状态机本机测试

运行全部主机测试（GCC 不在 PATH 时指定实际编译器路径）：

```powershell
./tests/run_host_tests.ps1 -Compiler 'D:\compiler\msys2\ucrt64\bin\gcc.exe'
```

`test_imu.c` 使用真实 IMU 应用和双精度 AHRS 算法，模拟传感器与 DWT/HAL；以非零三轴温度系数验证全部标定样本均温、非匀速升温、升温/降温、真实转动保留、读取失败保持快照、重新初始化，以及标定晃动/读取失败后禁用补偿并清除标定温度。另验证微小零偏的累加精度、大连续角下的小增量及跨 ±π 连续性、快照访问恢复原中断屏蔽状态。实际温度系数仍需在本机静止测量后填写。

`test_imu_uart_debug.c` 编译实际 IMU 文本日志模块、UART BSP 和环形缓冲，验证 CSV 精度/长度、中断发送缓冲寿命、一秒限频、忙/错误恢复、无效数据和 tick 回绕。接线和使用方法见 `App/IMU_UART_DEBUG.md`。

`test_work_port.c` 编译真实任务接口和机械臂应用，验证赛道表格及后续复测位姿的实际发送帧、观察坐标变换、QR与夹爪默认值、非阻塞重试与取消、第一关节正逆解 ±78°，以及修正后的人质观察参数成功下发、旧越界参数仍被拒绝。现场TCP观测回放验证人质参考点的符号和零平面误差，同时验证Z超过±50mm观测范围仍被拒绝。

`test_vision_result.c` 验证交接文档的五个 HEX 样例、分片/粘包、CRC、超长/短包、版本/保留位、可选字段长度、NaN/Inf、扫描成功与失败、候选观测拒绝、机械臂 X 轴换向、正解/齿轮比例、坐标变换、平移符号、QR/法向 yaw 的可观测性。

`test_task_vision.c` 编译真实 task、作业状态机、app_vision、BSP/环形缓冲、协议、坐标换算和底盘控制；只替换 HAL、IMU/电机、机械臂 hook 和物理场景。覆盖真实发送 SEQ、回绕/忙/失败、旧结果与重复终态、观测包不控制、微调后复测、全赛程、时钟回绕、激光与微调中总停、延迟结果、23秒超时、机构参数缺失、机械臂等待超时、微调不收敛及救援 reason=9。测试中的机械臂动作完成和相机测量均为模拟，不是实际硬件验证。旧 OFFSET 文本不再用于控制。

高度诊断回归覆盖目标高度偏高/偏低20mm时继续执行X/Y修正及救援作业，以及重新观测恢复正常高度后清除诊断标志。

2026-11-03 验证：作业接口、现场TCP回放、高度诊断、请求/异常处理、搬运流程及无相机模式通过。完整 `test_task_vision` 的 `test_full_course(0)` 仍在仿真时间48455ms报告 `WORK_ERR_ALIGNMENT`；使用本次高度修改前的作业状态机也在相同时间复现，因此未将完整赛程记为通过。

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
