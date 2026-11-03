# app_task_work_port 配置填表与示例

日期：2026-11-01。对应当前 `TaskWorkConfig_t`、`task_work_load_config()` 和 `task_work_arm_poll()`。

2026-11-03 纠偏速度更新：用户实测时间接口右移100mm、最大速度80mm/s、加减速度
200mm/s²时位移稳定到100mm附近。当前正式纠偏采用 `correction_speed=80`、
`correction_accel=200`；扫描/换位/返回仍使用 `accel=100`。填写自定义配置时
需同时提供正数 `correction_accel`（上限500mm/s²），不再与扫描共用加减速度。
下方历史示例仅用于填写参考，正式数值以 `app_task_work_port.c` 为准。

本文保留填写方法及示例。2026-11-01 已接入动作hook及三类扫描位(0,0)/(0,150)/(0,300)，QR姿态抬升/收臂；爆炸物搬到第二位放桶，人质保持夹紧带到终点。四类目标底盘系XYZ也已收到并启用：QR(0,1250,55)、球心(-225,0,0)、靶心(-620,0,20)、人质中点(-247.784439,19.8726902,33.2348137)，单位mm；人质参考点于2026-11-03按现场到位读数取反号更新。下文原始空表、假设示例及流程询问仅作为以后修改配置的参考，不代表当前仍缺数据。

## 1. 已有数据，不必重复提供

七组原始舵机姿态已记录在 `app_roboarm.c`，顺序固定为 ID1、2、3、4：

| 用途 | 原始位置 |
|---|---|
| QR观察 | 480, 755, 900, 435 |
| 爆炸物观察 | 0, 735, 800, 510 |
| 靶标观察 | 0, 1000, 800, 339 |
| 人质观察 | 0, 770, 865, 565 |
| 爆炸物夹取 | 0, 985, 800, 535 |
| 排爆桶放置 | 960, 920, 780, 310 |
| 人质夹取 | 0, 800, 800, 425 |

零偏、35:44减速比、机械臂基座与底盘中心重合、PA0激光高开低关也已明确。

`observe_pose` 可以由上述原始位置反算关节角，再调用现有正解得到；不需要用户重新手填 XYZ/pitch。发送仍可使用原始位置接口，换算必须使用同一组姿态。

## 2. 最少还缺什么

### A. 三个观察位相对路线停车点的位移

坐标单位 mm；以该作业区路线停车时的车头方向为准，X右、Y前。给每个观察位的累计偏移，不是相邻两位之间的增量。

| 作业区 | slot1 (X,Y) | slot2 (X,Y) | slot3 (X,Y) |
|---|---|---|---|
| QR | 默认(0,0)，如需挪车请填 | 不使用 | 不使用 |
| 爆炸物 | 待填 | 待填 | 待填 |
| 靶标 | 待填 | 待填 | 待填 |
| 人质 | 待填 | 待填 | 待填 |

如果目标沿车头方向等距排布，第一位就是停车点、间距200，则可写 `(0,0)、(0,200)、(0,400)`。这只是坐标填写例子，不是对真实赛场间距的判断。

同时确认：三个位点能否共用该类的同一观察姿态；每个位点是否只让一个目标进入 K230 计数门。扫描顺序是首次稳定出现的1/2/3，不是颜色或形状编号。

### B. 对准后目标应在底盘哪里

每类目标给 `desired_reference_body=(X,Y,Z)`，原点为底盘中心，也就是约定的机械臂基座原点；Z不是离地高度，除非两者原点高度相同。

| 类型 | 坐标对应的点 |
|---|---|
| QR | 二维码中心 |
| 爆炸物 | 球心，不是球面接触点 |
| 靶标 | 激光应该打中的靶心 |
| 人质 | K230输出的物体轴线中点，不一定等于夹取接触点 |

最省事的实测方法：

1. 手动将车放在你认为正确的位置：能按现有夹取姿态抓到目标，或激光正中靶心。
2. 保持底盘不动，把机械臂摆到该任务的观察姿态，取得有效视觉定位结果。
3. 记录 MCU 的 `app_task_work_get_status()->alignment.reference_body[0..2]`，或给我 K230 原始 E 系 XYZ，我用观察姿态换算。
4. 把这组正确停车位置下的坐标作为期望值；多次采样检查重复性，避免把明显异常测量写成标定。

当前配置未启用时尚不能靠正常任务流直接完成上述记录，可先做单项视觉测量；必须使用相应观察姿态，注明数据是 E 系还是底盘 B 系。

若三个位点几何关系一致，每类可先共用一组期望值；高度或放置距离不同则分别给三组。仅有夹取舵机位置不能证明球心/轴线中点恰好在TCP原点，不能自动用TCP坐标代替。

### C. 缺失的机械臂动作

- 行驶收臂姿态：ID1～4，是否各作业区通用。
- 爆炸物夹住后的抬升/避障中间姿态，及转向桶之前需要经过的姿态。
- 人质夹住后的抬升/搬运姿态。
- 人质放置/松爪姿态。
- 是否确认夹爪张开437、闭合747能用于两种物体；如不同，请分别给值。
- 现有“夹取人质”姿态是否同时代表放置姿态：默认不能这样推断。

不能根据两个端点自动认定直接四舵机插值不会碰到台面。可以提供“先升高，再转yaw，再下降”的中间姿态顺序，我来写非阻塞步骤。

### D. 放置时底盘是否要移动（影响流程）

当前流程是：微调对准目标 → ARM_PICK → ARM_PLACE → 收臂 → 撤回观察位名义偏移 → 下一段路线。

所以需要确认：

- 在任意一个目标位置抓取后，底盘不动，是否都能用已有桶放置姿态把球放入桶？
- 人质是否也能在当前底盘位置直接放入指定安置区？

若不能，请给“从抓取停车点到桶/安置区停车点”的位移、是否需要转向、放置后从哪里继续路线。这样需要在作业子状态机中补搬运步骤，仅填机械臂姿态无法实现；主 task 的 switch 框架仍可保持。

## 3. 可先采用的控制参数示例

下面是初次联调示例，并非已验证的比赛参数；与几何数据分开调整。

```c
*out = (TaskWorkConfig_t){
    .settle_ms = 1000U,              // 停稳后等待1s
    .scan_dwell_ms = 3000U,          // 每观察位先给3s，按板上连续识别速度调整
    .arm_timeout_ms = 30000U,        // 每个机械臂步骤的最长等待
    .laser_ms = 1000U,              // 初测照射1s，之后按实际判定调整
    .scan_speed = 80.0f,            // mm/s
    .correction_speed = 80.0f,      // mm/s，当前实测选取的速度上限
    .correction_accel = 200.0f,     // mm/s²，纠偏独立加减速度
    .accel = 100.0f,                // mm/s²
    .max_correction_mm = 100.0f,    // 超过则报错，不自动截短后继续
    .max_correction_yaw = 0.174532925f, // 10°，单位rad
    .position_tolerance_mm = 5.0f,
    .yaw_tolerance_rad = 0.017453293f,  // 1°
    .height_tolerance_mm = 10.0f,
    .max_corrections = 4U
};
```

机械臂可先用每段3000ms、到时额外停稳500ms、夹爪动作1000ms作为初测时序；还需留出串口传输时间。整个动作阶段不能超过30s。`RET_OK`仅表示发送开始，没有位置反馈时完成判据只是预计时间，不是实测到位。

扫描停留3s仍需板上测试；K230扫描总体超时60s、MCU等待63s，不宜无限增大每位停留时间。

## 4. slot全部字段填写示例

以下假设某个目标对准后在底盘右侧200mm、高于基座100mm，观察位在停车点前方200mm。**纯粹演示字段，不能据此判断真实抓取方向。** QR、爆炸物、靶标、人质都按此结构填写，QR仅填slot0，其他填3个slot。

```c
unsigned i = 1U; // 示例：第二个观察位
out->slot[i].offset_mm[0] = 0.0f;
out->slot[i].offset_mm[1] = 200.0f;

// observe_pose 应从该任务实测原始舵机位置得到，见下一节。
out->slot[i].observe_pose = calculated_observe_pose;

VisionAlignmentConfig_t *a = &out->slot[i].alignment;
a->desired_reference_body[0] = 200.0f;
a->desired_reference_body[1] = 0.0f;
a->desired_reference_body[2] = 100.0f;

// 示例工作空间：围绕期望值XY各±150mm，Z±50mm。
// 这是数据接受窗口，不是底盘微调允许距离，两项都会检查。
for (unsigned axis = 0U; axis < 3U; ++axis) {
    float margin = axis == 2U ? 50.0f : 150.0f;
    a->min_reference_body[axis] = a->desired_reference_body[axis] - margin;
    a->max_reference_body[axis] = a->desired_reference_body[axis] + margin;
}
a->yaw_mode = VISION_YAW_NONE;  // 初次先闭环平移
a->map_reference_valid = false;
// 因为上面整体初始化为0，此模式暂不使用的字段均为0：
// qr_axis、desired_direction_body、reference_map、desired_map_yaw、imu_to_map_yaw。
// body_from_tcp 不必手写矩阵：工作模块validate_config由observe_pose生成。
```

`height_tolerance_mm` 对所有四类任务仅作为高度诊断阈值，结果保存在 `height_error_mm` / `height_out_of_tolerance`。底盘只能平移/转向，到位判断使用X/Y及启用的yaw；完整XYZ换算与 `min/max_reference_body` 的观测范围检查仍然生效，Z坐标原点应保持正确。

若希望从一开始就修正 yaw，还需提供 QR 选用哪条局部轴/靶法向及其正确停车时在底盘中的方向。爆炸物球心和人质中点不单独提供 yaw。初版设 NONE 不影响使用XYZ做平移微调，但不能消除航向误差。

若需要地图绝对定位，另给固定参考物的地图XYZ、期望地图yaw以及IMU到地图yaw偏置；仅做局部对准时可先关闭。绝对定位和本地平移纠偏不是同一项要求。

## 5. 从已知舵机位置生成观察姿态的代码示例

```c
const uint16_t *p = app_roboarm_pose_positions(ROBOARM_POSE_QR_OBSERVE);
const double rad_per_position = 0.24 * 3.14159265358979323846 / 180.0;
Roboarm_State_t joints = {
    .yaw_angle = (YAW_SERVO_OFFSET - (double)p[0]) * rad_per_position,
    .joint1_angle = ((double)p[1] - JOINT1_OFFSET) * rad_per_position,
    .joint2_angle = (JOINT2_OFFSET - (double)p[2]) * rad_per_position,
    .joint3_angle = ((double)p[3] - JOINT3_OFFSET) * rad_per_position
};
End_Effector_Pos_t calculated_observe_pose;
if (alg_roboarm_forward(&joints, &calculated_observe_pose) != ROBOARM_KINEMATICS_OK) {
    return false;
}
// yaw减速比已在正解内部处理，这里不能再次乘35/44。
```

## 6. 可直接复制回复的最小数据表

```text
单位：mm；底盘X右、Y前、Z上，原点是底盘中心/机械臂基座。

扫描位偏移（相对各自作业区路线停车点，累计偏移）：
爆炸物：( , ) / ( , ) / ( , )
靶标：  ( , ) / ( , ) / ( , )
人质：  ( , ) / ( , ) / ( , )
QR是否就在路线停车点观察：是/否；否则偏移( , )
同一类三个位置是否共用现有观察姿态：

正确对准后的参考点XYZ（或者给对应观察姿态下的K230原始E系XYZ，并注明）：
QR中心：
球心：
靶心：
人质轴线中点：
各slot是否可以共用本类期望XYZ：

行驶收臂 ID1/2/3/4：
爆炸物抬升/避障中间姿态及顺序：
人质抬升/搬运中间姿态及顺序：
人质放置姿态：
夹爪437张开/747闭合能否共用：

三处夹球后，底盘不动是否都能放进桶：
夹人质后，底盘不动是否能放到安置区：
若需搬运：底盘位移/转角/放置后继续路线的位置：

第一版是否只做平移纠偏：是/否
若否，补QR轴/靶法向的已知方向：
是否需要地图绝对位置：是/否；若是，补目标地图坐标及航向关系。
```

数据齐全后，可集中完成 `app_task_work_port.c` 的配置加载与非阻塞动作步骤；只有确需底盘搬运时才补相应作业子状态，保留原task主框架。
