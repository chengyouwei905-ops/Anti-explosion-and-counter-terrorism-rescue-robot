# MCU 视觉作业接入说明

对应 K230 `MCU_VISION_HANDOFF.md`，2026-10-29，二进制结果版本 1。

## task.c 现在做什么

原来的 MOVE1～MOVE16、路线参数、连续转弯和 `switch (task_state)` 保留。
只把四个作业 case 的“直接跳过”换成轮询作业子状态机：

| task 状态 | 作业过程 | 成功后 |
|---|---|---|
| TASK_SCAN | 观察姿态 → 0x10 → QR 位姿 → 微调/复测 → 收臂 | MOVE2～11 |
| TASK_EXPLOSIVE | 0x20 扫描三个位置 → 按序号选位 → 0x21 → 微调/复测 → 夹取/QR抬升 → 搬到第二位 → 桶内放置/收臂 | MOVE12 |
| TASK_CS | 0x30 扫描 → 按序号选位 → 0x31 → 微调/复测 → 激光照射/关闭 → 收臂 | MOVE13～15 |
| TASK_RESCUE | 0x40/0x41 → 微调/复测 → 夹取/QR抬升 → 保持夹紧带到终点 | MOVE16 |

作业通过 `app_task_work_poll()` 返回 WAITING、DONE 或 FAILED。只有 DONE 才开始下一组路线。
扫描序号 1/2/3 是本次稳定首次出现顺序，不是颜色。三个观察位置按该顺序填写；一次只让一个目标进入 K230 计数门。
第一个观察位停稳后发扫描命令并保持观察，再移动到第二、第三个位置。扫描结果允许在这期间到达；底盘先完成当前移动，再前往选中的观察位。

选位/作业完成后，会回到该作业区原来的路线停车点，再执行下一组原路线。
因此三个 slot 的期望位置必须对应同一个地图基准和同一个底盘航向。
返回时只撤回扫描观察位的名义偏移，不撤销用于纠正惯导误差的微调。

## 机械臂参数留在哪

修改 [app_task_work_port.c](Src/app_task_work_port.c) 的两个函数：

1. `task_work_load_config()`：已配置三个累计观察位(0,0)/(0,150)/(0,300)，从实测舵机姿态计算观察XYZ/pitch。每类目标期望坐标集中在文件顶部 `work_reference`。
2. `task_work_arm_poll()`：已实现非阻塞观察、张爪、夹取、闭爪、QR抬升、桶内放置和QR收臂；人质不调用放置步骤。

2026-11-01 目标底盘系XYZ已由用户确认并启用：QR(0,1250,55)、球心(-225,0,0)、靶心(-620,0,20)、人质中点(-225,0,55)，单位mm，高度相对机械臂基座原点。四类ready均为true，不再因缺少配置停在 `WORK_WAIT_CONFIG`。
等待期间其他poll仍工作。不要把机械臂扫码XYZ/pitch当成目标参考点坐标，两者定义不同。
动作每段默认3000ms、夹爪1000ms，额外留100ms发送裕量与500ms停稳时间；以非阻塞tick计时判断预计完成，无实际舵机反馈。串口BUSY每20ms重试，超过1s或发送错误则失败。
爆炸物闭爪并完成QR抬升后，新增 `WORK_MOVE_TO_BUCKET` 移动到slot[1]，停稳后放入桶。放置后从该位置撤回扫描名义偏移；视觉纠偏量保留。人质跳过 `WORK_PLACE`，后续QR收臂只发送ID1..4，保持ID5夹紧。

每个 slot 需要：

| 参数 | 含义 |
|---|---|
| offset_mm[2] | 从原路线停车点到观察位的偏移，沿停车时车体系 X右/Y前，mm |
| observe_pose | 与机械臂位置接口相同的 x/y/z/pitch；X左/Y前/Z上，pitch 为 rad |
| desired_reference_body[3] | 底盘正确到位时，QR中心/球心/靶心/救援中点在底盘系的坐标 |
| min/max_reference_body[3] | 允许的观测范围，超出时拒绝运动 |
| yaw_mode | 默认只平移；QR轴或靶法向有已知参考方向时才启用 yaw 修正 |
| qr_axis | 选用 QR 自身的哪根单位轴；仅 QR yaw 模式使用 |
| desired_direction_body | 正确航向下上述轴或法向在底盘中的单位方向，需要水平分量 |
| map_reference_valid | 目标地图位置已知且没有被搬动时设 true，否则保留 false |
| reference_map | 已知参考点在地图中的 XYZ |
| desired_map_yaw | 正确到位时的地图航向；与期望方向对应 |
| imu_to_map_yaw | 不使用视觉 yaw 时，将 IMU yaw 转到地图 yaw 的偏置 |

QR 仅使用 slot[0]。非 QR 使用 slot[0..2]。
`body_from_tcp` 自动由 `observe_pose` 生成，不需要手填矩阵。
有直接关节角时，可用 `alg_vision_transform_from_arm_joints()`：它先调用现有 `alg_roboarm_forward()`，再生成同一变换。
注意这仍是指令/正解姿态，不是舵机反馈；观测期间机械臂必须已经到位并保持不动。

机械臂 hook 的 `entering` 仅第一次为 true。指令成功发送后继续返回 WAITING，直到机构停稳才返回 DONE。
如果发送返回 RET_BUSY，应在后续 poll 重试，不要只在 entering 时尝试一次。
本模块使用 `app_roboarm_pose_it()` / `app_roboarm_positions_it()`，没有调用旧阻塞发送接口或 HAL_Delay。取消会清除步骤且不松爪；动作组停止指令能否中止控制板直接舵机插值，仍需按控制板实际能力验证。

## 坐标换算

已核对 K230 `calibration/handeye/calibrate_handeye.py::robot_pose()`：

- 机械臂指令坐标 A：X 向左，Y 向前，Z 向上。
- 底盘坐标 B：X 向右，Y 向前，Z 向上。
- 按本次约定，机械臂原点与底盘中心重合，不另加安装平移。
- TCP 坐标 E 与当前手眼标定相同：零姿态下 X右/Y前/Z上。

给定机械臂指令 `(x,y,z,pitch)`：

```text
tool_yaw = atan2(x,y)
R_B_E = Rz(tool_yaw) * Rx(pitch)
t_B_E = (-x, y, z)
p_B = R_B_E * p_E + t_B_E
```

机械臂正解已统一使用 `ROBOARM_YAW_OUTPUT_PER_SERVO=35.0/44.0`（舵机35齿、转台44齿），视觉层不重复处理齿轮比。
当 x=y=0 时，单凭 XYZ/pitch 无法确定 tool_yaw，因此拒绝这个观察姿态，不猜测朝向。

`reference_body` 是参考物相对底盘的位置；`chassis_from_reference_body=-reference_body` 是底盘相对参考物的位移，以底盘轴表达。

不修正 yaw 时：

```text
delta_B = measured_reference_B - desired_reference_B
distance = hypot(delta_B.x, delta_B.y)
direction = atan2(-delta_B.x, delta_B.y)
app_chassis_motion_linear_move(distance, direction, ...)
```

例如期望参考点为 `(0,300,100)`，观测换算后为 `(20,280,100)`，应向右20mm、向后20mm：距离约28.28mm，方向约-2.356rad。
这只是符号检查示例，不是实车默认标定值。

启用 yaw 时，比较当前参考方向与期望方向的水平投影，计算需要实际转动的 delta_yaw。
点本身不能解出 yaw；垂直方向、无效四元数、法向正反不明也不能随意提供 yaw。
先使用原地转动接口修正 yaw，停稳后重新发定位命令，再用新坐标修正平移。
每次微调之后都重新测量，最后一次必须在位置、航向和高度容差内才进入实际作业。
高度误差不能靠平面底盘消除，超限时停下等待修改机械臂姿态。

若地图参考点已知：

```text
p_map_chassis = p_map_reference - Rz(map_yaw) * p_body_reference
```

只有点时 map_yaw 仍依赖 IMU；有已知方向时从视觉方向计算。
此结果放在诊断结构里，不篡改 IMU 原始数据，也不把累计路程当作二维位置。
微调使用上述参考点误差，等价于移动到该参考物定义的正确底盘位置。

## 请求、回包与异常

`app_vision_send_with_seq()` 通过已有 BSP 中断发送返回真实 SEQ；失败时不建立等待请求。
RET_BUSY 每20ms重试，最多1秒，不在主循环忙等。RET_OK 表示交给串口发送，不是对端 ACK。
旧 `app_vision_send()` 也改为相同的非阻塞发送方式。

始终只等待一个请求。结果需同时匹配 payload 中的 task_cmd 和 input_seq，外层 K230 SEQ 不作为请求序号。
版本、长度、flags、状态、保留位和浮点数先校验，再检查5帧样本、scatter≤8mm、目标序号与机构工作空间。
可选四元数/法向/抓取点依 flags 顺序解析，scatter 不固定在27字节偏移。
当前定位使用文档定义的球心/靶心/中点 XYZ；GRASP 只解析保存，不擅自替换参考点。
status=3 观测、候选标定、旧请求、重复结果、错误 CMD 的文本启停都不能启动运动。

本地定位超时23秒、扫描超时63秒。匹配的失败/取消立即报错；异常终止后不自动重发作业或跳过。
当前救援模型未配置，收到 reason=9 时保持故障状态，不报告救援成功。
取消命令0x7F不等待应答，因为对端空闲时不会回包。

TOTALSTOP 先关闭激光并清除等待/成功事件，随后本轮 task_poll 取消底盘轨迹、复位闭环旧误差，回到 IDLE。
取消视觉、停止机械臂动作组使用非阻塞发送。新一轮启动要等取消发送成功交给串口；失败则进入故障。
动作组停止不等于能够中止控制板所有单舵机插值；以后新增直接舵机动作时，需要在 ARM_CANCEL hook 中补上控制板支持的保持/停止动作。
故障后可先 TOTALSTOP 清理，再 TOTALSTART 重新从扫码开始。

## PA0 激光

使用 CubeMX 的 `LASER_GPIO_Port` / `LASER_Pin`（GPIOA/PA0），高电平开启、低电平关闭。
只有打靶视觉复测已经对准时才开启；照射 `laser_ms` 后关闭，总停/任何作业错误也会关闭。
当前默认1000ms，是可调工程参数，不代表已验证的赛场判定时间。
按约定忽略激光与夹爪的位置偏差；仍须把观察姿态和期望靶心配置到实际激光射线上，底盘不能替代俯仰调整。

## 调试看什么

`task_get_state()` 查看仍保留的主任务状态。
`app_task_work_get_status()` 查看作业子阶段、error、K230 vision_reason、selected_slot、corrections、请求 CMD/SEQ、取消状态及最近坐标换算结果。
`alignment_valid` 只在观测仍可用于当前动作时有效；开始移动、搬动物体或结束作业后失效，数值保留供回看。

主循环调用顺序不变：IMU/视觉/底盘 poll → task_poll → 按10ms节拍 motion_update → closeloop_update。
全流程没有新增阻塞延时。现有 main 初始化阶段的机械臂调试动作不属于这里的作业 hook，本次没有修改其目标位置。

## 验证范围

`tests/run_host_tests.ps1` 编译实际 MCU 源码，覆盖交接文档 HEX 样例、分片/粘包/CRC、严格载荷解析、坐标与 yaw 符号、请求序号与错误发送、微调后复测、总停、超时、时钟回绕和救援失败。
整圈测试使用合成观测、理想底盘和已完成的模拟机械臂动作；不是实车定位精度或真实完赛时间证明。
没有烧录 MCU、运行 K230 或触发实际机械臂/激光。
