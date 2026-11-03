# MCU 视觉作业接入说明

对应 K230 `MCU_VISION_HANDOFF.md`，2026-10-29，二进制结果版本 1。

## 2026-11-03 赛道实测作业位姿

`app_task_work_port.c` 的 `task_arm_poses` 接入 `广工赛合适参数.xlsx` 的 `Sheet1!A2:E7`。
XYZ 使用机械臂指令坐标（X左/Y前/Z上，mm），pitch 为 rad。

| 作业位姿 | x | y | z | pitch |
|---|---:|---:|---:|---:|
| 观察爆炸物 | 154 | 0 | 6 | -1.25 |
| 夹取爆炸物 | 190 | 0 | -60 | -1.25 |
| 放置爆炸物 | -180 | 0 | 50 | -1.2 |
| 观察恐怖分子 | 180 | 0 | 25 | 0 |
| 观察人质（用户后续复测更新） | 133.537333615 | 1.525593299 | 82.971289222 | -0.338 |
| 夹取人质 | 185 | 0 | 78 | -0.5 |

观察姿态的视觉坐标变换与动作下发共用此表；动作经 `app_roboarm_servo_move_to_it()` 逆解并非阻塞发送。
二维码观察、抬升与收臂仍使用原 QR 舵机姿态。夹爪、扫描位移、底盘目标参考点、速度与动作时序保持原值。

用户要求第一关节正逆解统一限制为 ±78°。人质观察使用后续提供的复测参数覆盖 Excel 原值：第一关节约 -74.562°，ID1..4 下发为 `{32,455,996,601}`，通过当前限位和位置检查。六组均已通过主机实际逆解和发送帧验证，尚未验证整条任务流的实车动作。

## 视觉二次纠偏：时间规划（当前）

`start_alignment()` 的平移纠偏已换回 `app_chassis_motion_linear_move()`。
距离和方向由视觉计算，`app_task_work_port.c` 的 `correction_speed=80mm/s`、
`correction_accel=200mm/s²` 控制最大速度与加减速度，IMU继续保持起始航向。
执行时间由距离/速度/加减速度规划，外层恢复规划时长加2秒的超时检查。
完成后继续停稳和视觉复测；视觉容差20mm、最多四次纠偏保留。其他移动不变。

### 备用里程接口（未接入正式纠偏）

`app_chassis_motion_linear_move_distance()` 仍可单独调用，使用累计路程差，
不是IMU加速度积分或二维位置闭环。

里程到量容差20mm、末段最低目标速度50mm/s（不超过本次max_speed），均在
`app_chassis_motion.h` 调整。反馈年龄超过500ms、在途清零/回退、连续2s没有0.1mm
进展或运动总时长达到30s均停止并报错；外层等待留1s余量。
单独调用该接口时根据启动返回值及motion结果区分NO_ODOM、ODOM_ERROR、STALLED、TIMEOUT。
到量后继续视觉复测；当前视觉容差20mm及最多四次纠偏保留。
里程容差表示允许提前到量：目标100mm时累计走到80mm即可结束；目标不超过20mm时不启动平移。

用户实测原时间接口 `linear_move(100,-PI/2,80,200,200)` 右移距离稳定到100mm附近。
据此将纠偏最大速度设为80mm/s，新增独立 `correction_accel=200mm/s²`，用于纠偏
的加速和减速；扫描/换位/返回继续使用原 `accel=100mm/s²`。50mm/s末段下限为
用户当前设置，尚不能由100mm整段测试认定为最小稳定速度，小距离里程纠偏仍需复测。

## 无摄像头联调（2026-11-01历史说明；当前以头文件开关为准）

`App/Inc/app_task_work.h` 的 `TASK_WORK_CAMERA_TEST` 当前为1。相机恢复后改成0并重新编译烧录，即恢复原视觉结果接收流程，无需改K230协议。

- 仍由K230发送真实TOTALSTART启动，TOTALSTOP仍有效，任务请求仍正常发送。
- QR在 `WORK_WAIT_LOCATE` 模拟成功；其他三类仍走完三个扫描观察位，在 `WORK_WAIT_SCAN` 模拟扫描完成，统一选第一个作业位置。
- 各定位结果由配置中的期望底盘XYZ反变换为夹爪E系XYZ，原换算链路得到零平移误差。适用于当前四类 `VISION_YAW_NONE` 配置。
- 测试模式忽略真实0x81结果，避免故障相机回传失败或晚到消息打断联调；正常模式的解码与SEQ匹配保持原样。
- 只模拟视觉结果，底盘、机械臂、夹爪、激光仍会实际动作。爆炸物仍搬到第二位放桶，人质仍夹紧带到终点。
- 九组主机测试通过，覆盖正常模式与无回包测试模式；固件编译通过，未烧录。

## 爆炸物三处机械臂动作拆分

原来一次四关节set到目标姿态，现在分为两次：先仅ID2/3/4到该目标的升降/伸缩/pitch，
保持原yaw；预计动作时间及稳定等待结束后，再仅ID1转yaw。中间姿态由原目标坐标反解，
不另外假定抬升高度；放置前第一段高度仍为原放置点Z=50，并非额外抬高。

- 夹球后：张爪、到夹取位、闭爪、ID2/3/4抬到QR搬运几何姿态、ID1转到QR方向。
- 放置前：ID2/3/4到放置几何姿态、ID1转到桶方向、张爪。
- 放置后收回：ID2/3/4回QR搬运几何姿态、ID1转回QR方向。

三处都保留原目标XYZ/pitch，夹取高度沿用当前配置-35mm。各段3000ms，另留100ms发送裕量
及500ms稳定等待，保持非阻塞和BUSY重试；只按时间预计到位。人质及其他作业动作不拆分。
原姿态坐标在 `task_arm_poses`，动作时间在 `ARM_POSE_TIME_MS`；每步超时仍为30s。

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

1. `task_work_load_config()`：已配置三个累计观察位(0,0)/(0,150)/(0,300)，非QR观察XYZ/pitch读取 `task_arm_poses`，QR继续从原舵机姿态正解。每类目标期望坐标集中在文件顶部 `work_reference`。
2. `task_work_arm_poll()`：已实现非阻塞观察、张爪、夹取、闭爪、QR抬升、桶内放置和QR收臂；人质不调用放置步骤。

目标底盘系XYZ已启用：QR(0,1250,55)、球心(-225,0,0)、靶心(-620,0,20)、人质中点(-247.784439,19.8726902,33.2348137)，单位mm，高度相对机械臂基座原点。人质参考点于2026-11-03按现场到位读数更新，其余三类保留原值。四类ready均为true，不再因缺少配置停在 `WORK_WAIT_CONFIG`。
现场调试的 `arm_origin_relative_qr` 是基座相对目标的位置，`work_reference` 是目标相对底盘的位置，必须取反号；原始 `vision_result.xyz_mm` 属于TCP系，也不能直接填入 `work_reference`。
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
本模块使用 `app_roboarm_servo_move_to_it()` / `app_roboarm_pose_it()` / `app_roboarm_positions_it()`，没有调用旧阻塞发送接口或 HAL_Delay。取消会清除步骤且不松爪；动作组停止指令能否中止控制板直接舵机插值，仍需按控制板实际能力验证。

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
每次微调之后都重新测量，最后一次平面位置和启用的航向误差必须在容差内才进入实际作业。
高度误差不能靠平面底盘消除；`height_tolerance_mm`（默认10mm）仅用于诊断，不阻止X/Y微调或进入后续作业。完整XYZ变换、有限数检查及观测工作空间检查仍生效；当前Z观测范围仍为期望高度±50mm，超过范围的观测会被拒绝。

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
`height_error_mm` 是最近有效观测的目标高度减期望高度，`height_out_of_tolerance` 表示是否超过高度诊断阈值；二者与最近的 `alignment` 一同保留，下一次有效定位会更新，不作为底盘到位条件。

主循环调用顺序不变：IMU/视觉/底盘 poll → task_poll → 按10ms节拍 motion_update → closeloop_update。
全流程没有新增阻塞延时。现有 main 初始化阶段的机械臂调试动作不属于这里的作业 hook，本次没有修改其目标位置。

## 验证范围

`tests/run_host_tests.ps1` 编译实际 MCU 源码，覆盖交接文档 HEX 样例、分片/粘包/CRC、严格载荷解析、坐标与 yaw 符号、请求序号与错误发送、微调后复测、总停、超时、时钟回绕和救援失败。
整圈测试使用合成观测、理想底盘和已完成的模拟机械臂动作；不是实车定位精度或真实完赛时间证明。
没有烧录 MCU、运行 K230 或触发实际机械臂/激光。
