# App — 应用层（业务逻辑）

## 放什么

- 具体业务：视觉、底盘、机械臂、任务状态机
- 一个模块一对文件：`app_vision.h` / `app_vision.c`
- 命名前缀：`app_`

## 不放什么

- ❌ 直接调 HAL（`HAL_GPIO_WritePin` / `HAL_UART_Transmit`）→ 放 `Bsp/`
- ❌ 帧头、CRC 这类协议细节 → 放 `Protocol/`
- ❌ PID、运动学等纯计算 → 放 `Algorithm/`

## 允许 include

`bsp_*.h`、`proto_*.h`、`common_*.h`、自家头文件

## 当前内容

| 文件 | 职责 |
| --- | --- |
| `app_vision.c` | 视觉链路收发的统一入口（搬运 + 组包），统一禁止向K230发送CANCEL(0x7F) |
| `app_roboarm.c` | 机械臂（总线舵机控制板）链路：组帧 + 下发，目前只做"用户主动发送" |

业务处理由调用方通过 `app_vision_set_rx_cb()` 注册回调实现，
payload 里装的是什么由业务层自己决定。

机械臂链路是单向的：发一条指令就调一次 `app_roboarm_xxx()`，没有 poll()，
也不需要注册回调（控制板上报的数据暂未解析）。

## 拿不准就放这里

放错了以后挪一下很便宜，纠结半小时的代价更大。

## 爆炸物机械臂动作：先关节动作，再单独yaw

`app_task_work_port.c::task_work_arm_poll()` 将三处原四关节同步set拆成两次：
第一段只发ID2/3/4，保持进入时yaw；等待该段结束后，第二段只发ID1转yaw。
目标仍从 `task_arm_poses` 原XYZ/pitch逆解，零偏/35:44换算共用机械臂应用层。

| 位置 | 拆分后的动作 |
|---|---|
| 夹球后到搬运位 | 闭爪 → 保持夹取yaw，ID2/3/4到QR搬运几何姿态 → ID1转到QR方向 |
| 放置前 | 保持搬运yaw，ID2/3/4到原放置几何姿态 → ID1转到桶方向 → 张爪 |
| 放置后收回 | 保持放置yaw，ID2/3/4到QR搬运几何姿态 → ID1转回QR方向 |

没有另加固定高度；第一段是原目标的伸缩/高度/pitch关节动作，不是笛卡尔竖直插补。
按当前坐标，夹球后从Z=-35到QR的Z=60；放置前第一段沿用原放置Z=50；放置后回Z=60。
如果需要放置前先升到更高的避线高度，需要另设该高度的目标姿态。
每段使用 `ARM_POSE_TIME_MS=3000`，另等100ms发送裕量和500ms稳定时间，均为非阻塞tick计时。
没有实际关节到位反馈；每步仍有BUSY重试/错误/取消处理，张闭爪时机保留。
人质、观察、二维码和打靶动作继续使用原四关节同步方式。

## 连续底盘路径

### 视觉二次纠偏：时间规划（当前）

`app_task_work.c::start_alignment()` 的平移纠偏已换回 `app_chassis_motion_linear_move()`。
距离为本次视觉误差长度，方向为 `atan2(-dx,dy)`；在 `app_task_work_port.c` 调整
`correction_speed=80mm/s` 和 `correction_accel=200mm/s²`（同时用于加速和减速）。
对应实测 `linear_move(100,-PI/2,80,200,200)` 的速度参数，正式纠偏的距离和方向随视觉变化。
原 `motion_started()` 按规划时长加2秒做超时检查，不再使用里程模式的31秒等待。
IMU继续保持起始航向；运动结束后停稳、视觉复测，视觉容差20mm、最多四次纠偏。
主路线、扫描换位、到桶、返回和原地yaw修正保留当前调用及参数。

### 备用里程移动接口（未接入正式纠偏）

`app_chassis_motion_linear_move_distance(distance, direction, max_speed, accel, decel)` 保留供单独试验。

启动时记录 `app_chassis_get_distance()` 对应的累计里程基准，不清零全局里程。
运动中根据剩余路程减速，路程差达到目标减20mm时置零速度并返回DONE；
目标距离不超过20mm时直接完成。例如目标100mm，累计走到80mm即可结束本次移动，再由视觉复测。
即使实际执行速度低于指令速度，也不会因原速度曲线的时间结束而提前成功。
这里用的是编码器累计路程，不是IMU加速度积分或完整X/Y位置闭环，侧滑及往返
也会计入，因此仍由K230复测决定是否继续纠偏和进入下一步。

参数集中在 `App/Inc/app_chassis_motion.h`：里程容差20mm，末段最低目标速度50mm/s
（不超过本次max_speed，起步仍受加速度限制），反馈最大年龄500ms，无0.1mm进展
持续2s报STALLED，总时限30s报TIMEOUT。末段最低速度需按实车低速能力标定。
没有收到有效编码器帧或反馈已过期时返回NO_ODOM；运动中断流、里程清零/回退时
报ODOM_ERROR。静止时的有效编码器帧仍刷新反馈时间，不会误认为通信断开。

直接调用该备用接口时检查启动返回值及 `app_chassis_motion_get_result()`。
正式时间纠偏的执行结果仍保留在作业状态 `motion_result` 中。

2026-11-03 用户实测：原时间接口 `linear_move(100,-PI/2,80,200,200)` 的右移距离
稳定到100mm附近。据此将作业 `correction_speed` 设为80mm/s，独立的
`correction_accel` 设为200mm/s²（加速与减速共用），只作用于二次平移纠偏。
扫描速度80mm/s、扫描/换位/返回加减速度 `accel=100mm/s²` 继续使用原值。
保留用户调整的末段速度50mm/s和视觉容差20mm；100mm测试尚不能证明50mm/s
是最小稳定速度，或小距离纠偏也有同等精度。里程接口末端速度曲线与时间接口有差异，需实车复测。

### 圆弧平移：保持车头朝向

`app_chassis_motion_arc_move(radius, angle, direction, max_speed, accel, decel)`
用于单段严格定半径圆弧平移，车头保持开始时的 IMU 航向。`angle` 是带符号扫角，
正值向左弯、负值向右弯；`direction` 是起始平移切线相对开始时车头的方向，
0 向前、+PI/2 向左。半径单位 mm，角度 rad，切向速度 mm/s，加减速度 mm/s²。

```c
/* 从静止起步，向左走R200的90度圆弧；终点相对起点左200、前200mm，车头不转。 */
app_chassis_motion_arc_move(
    200.0f, 1.57079632679f, 0.0f, 100.0f, 150.0f, 150.0f);
```

与 `linear_move()` 一样返回 `void`，只在状态入口启动一次；忙、参数错误、IMU 无效
时忽略本次调用。通过 `app_chassis_motion_get_state()` 和 `app_chassis_motion_get_result()`
读取执行状态。需要判断本次是否启动成功时，使用保留的状态返回接口
`app_chassis_motion_arc_translate()`，成功/忙/参数错误/IMU 无效分别返回
`MOTION_START_OK`、`MOTION_START_BUSY`、`MOTION_START_INVALID`、`MOTION_START_NO_IMU`。
两者共用同一规划和更新逻辑：以 `radius * fabsf(angle)` 为总路程，按与直线相同的
三角形或梯形速度曲线加减速，再用 `direction + sign(angle) * 弧长 / radius`
计算相对起始车头的平移切线方向。`radius` 必须大于零，零扫角立即完成，速度规划零速起止。
仍由主循环周期调用 `app_chassis_motion_update()` 和 `app_chassis_closeloop_update()`。
航向闭环继续修正车头偏差，圆弧切线不会作为车头目标或角速度前馈。
本接口未另设向心加速度上限，选速时需考虑 `max_speed² / radius` 和实际轮速限幅。

需要直线与圆弧之间不停顿时，使用
`app_chassis_motion_path_translate_start(segments, count, limits)`。它复用下面的连续
路径规划，但整组车头保持起始朝向。第一段沿初始车头方向，后续直线沿上一弯的
出口切线继续平移；`path_get_sample()` 的 `yaw/omega/angular_accel` 表示路径切线，
不表示车头目标。圆弧仍使用曲率过渡，`limits` 继续约束切线变化和向心加速度。

当前 `task.c` 仍使用原 `path_start()`，车头随弯转动。以后切换平移走法时，需一起
核对每组路线的起始方向、停车时车头朝向、机械臂观察方向和视觉参考点；后一组
不会自动继承上一组的出口切线。原 `circular_move()` 的车头随弯转动及原地转动语义保持不变。

### 车头随路径转动

`app_chassis_motion_path_start(segments, count, limits)` 一次提交一组前进直线和带曲率过渡的转弯。参数会复制，单组最多16段；忙时返回 `MOTION_START_BUSY`，不覆盖在途路径。原有直线、圆弧和原地转动接口继续可用。

```c
const AlgPathSegment_t path[] = {
    {ALG_PATH_LINE, 400.0f, 0.0f, 0.0f, 250.0f, 300.0f, 300.0f, 0.0f},
    {ALG_PATH_ARC, 0.0f, 200.0f, 1.57079632679f, 140.0f, 200.0f, 250.0f, 0.0f},
    {ALG_PATH_LINE, 350.0f, 0.0f, 0.0f, 220.0f, 250.0f, 300.0f, 0.0f}
};
const AlgPathLimits_t limits = {30.0f, 0.9f, 2.0f, 400.0f};
Chassis_Motion_Start_t result = app_chassis_motion_path_start(path, 3U, &limits);
```

直线的 `distance` 是前进距离；转弯的 `radius` 和带符号 `angle` 定义名义圆弧终点，正角左转、负角右转。转弯使用两端五次多项式曲率过渡，中间保持定曲率，并归一化弧长以保持名义终点和转角；实际半径随位置变化。当前R200、90度、30mm过渡相对名义圆弧的最大几何偏移约6.25mm，场地净空仍需实车检查。

末段直线可设置 `right_offset_mm`，相对该段名义车头方向向右偏移指定距离，
前进距离和车头目标保持原值；速度规划使用合成路程 `hypotf(distance, right_offset_mm)`。
MOVE15、MOVE16 分别配置为右移5mm，MOVE13～15仍连续执行。采样的 `direction`
给出实际平移切线，`yaw` 保留名义车头方向；其余段的横向补偿必须为零。

规划器根据后续限速做前后向速度约束，在段间保留非零速度；曲率变化项 `k' * v^2` 与切向加速度项 `k * a` 分别预留角加速度预算。`max_omega` 是参考角速度上限，应低于闭环1.2rad/s限幅，保留误差修正余量。参考角速度连续且角加速度有界；切向加速度仍采用分段常值，不承诺全程加加速度连续。

主循环继续每10ms先调用 `app_chassis_motion_update(0.01f)`，再调用 `app_chassis_closeloop_update()`。段间在同次更新中消费剩余时间，无串口等待、动态内存分配或每段停车。规划/更新均为有界计算；固件上仍应测量执行时间和控制周期抖动。

通过 `app_chassis_motion_path_get_sample()` 可读取当前段号、参考位移/航向、速度、角速度、预计总时长；这些是规划参考量，不是实际定位。`app_chassis_motion_get_result()` 区分正常完成、取消和 IMU 错误，任务不能只凭空闲状态宣布成功。

当前 `task.c` 在 `switch (task_state)` 中逐个展开 MOVE1~16 和四个作业状态。路线仍分为 MOVE1、MOVE2~11、MOVE12、MOVE13~15、MOVE16 五组，由底层保持组内连续运动。每组结束停稳1秒后，四个作业状态轮询 `app_task_work_poll()`，等待匹配的 K230 结果、坐标换算、微调复测和作业完成，再启动下一组。机械臂位置及机构参数在 `app_task_work_port.c` 留空，未填写时等待，不假报成功。具体流程、坐标轴、配置与调试见 [视觉作业接入说明](VISION_TASK_GUIDE.md)。
