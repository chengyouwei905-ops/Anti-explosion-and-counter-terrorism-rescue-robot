# MCU 视觉作业接入说明

对应 K230 `MCU_VISION_HANDOFF.md`，2026-10-29，二进制结果版本 1。

## 2026-11-06 打靶/救援返回只撤回固定扫描位移

WORK_TARGET和WORK_RESCUE作业完成后的WORK_RETURN改为move_offset(0,0)，只撤销current_offset记录的名义扫描位移。
不再把累计视觉平移correction_offset取负后叠加到返回目标，二次定位产生的实际平移修正保留到后续MOVE。
按当前三个停车位(0,0)/(0,150)/(0,300)，对应不平移/沿扫描方向退150mm/退300mm，与本轮纠偏量无关。
后续恢复进入扫描时航向的步骤保留，排爆返回行为沿用原实现；本次只更新MCU，K230无需修改。

## 2026-11-06 人质观察姿态改为四关节同步

WORK_RESCUE的ARM_OBSERVE一次发送ID1/2/3/4，yaw与其余关节同时运动。
沿用当前人质观察XYZ/pitch，动作3000ms，传输余量100ms及稳定等待500ms，总计3600ms后完成此步骤。
抓取后及ARM_TRAVEL仍先ID2/3/4抬升、再ID1回到搬运姿态(0,250,100,-0.05)。
排爆/打靶观察仍先yaw后其余关节，QR观察仍同步；本次只需更新MCU固件。

## 2026-11-06 救援扫描改为YOLO目标类别单帧命中

旧救援停车扫描已经使用YOLO，但仍要求连续3帧、同一身份与框中心移动<=60像素，且ROI中任何多个目标都会形成歧义。
新独立模块tasks/task_rescue_stop_scan.py只用于4字节停车RESCUE_SCAN(0x40)：
底盘停稳后对原图进行原YOLO推理，在原ROI内按框中心筛选指定shape_id；唯一匹配目标出现一帧即返回成功slot。
其他类别同时存在不会阻止命中；同类别多个目标仍报告歧义，不任意选择。
未匹配目标时继续当前窗口，窗口结束且有可用推理帧才返回20；模型/推理异常不伪装为非目标。

原YOLO置信度/NMS/模型/ROI/尺寸参数未改。当前模型后处理置信度门槛0.55，原适配层下限0.45，实际两层均保留。
YOLO仍保持原640x480输入与640x640 letterbox推理，ROI在原图检测框中心处筛选，不是裁图推理。
扫描不进行框宽测距、轮廓或三维拟合，也不进行5帧位置平均。
成功后MCU按原逻辑发0x41，人质精定位/5帧平均/底盘纠偏复测通过后才抓取。

交付包：output/k230_rescue_yolo_single_hit_20261106；相对扫码黑屏修复包仅修改task_runner.py和competition_main.py，新增救援扫描模块。
所有配置、模型、标定、原定位代码保持字节一致；扫码只读任务码、黑屏修复、MAX_POINTS_PER_RING=24和打靶单帧扫描均保留。
44项本机回归通过，未上板验证。本次无需修改或重刷MCU。

## 2026-11-06 K230扫码日志空对象导致黑屏/总停修复

扫码只读任务码后manager.average为None，但competition_main.run_loop每秒日志仍访问average.samples，
等待扫码超过1秒会抛AttributeError。main的finally随后执行RunControl.close发送TOTALSTOP并关闭CameraService显示。
已在实际main主循环测试中复现；修复仅将QR日志改为打印decode_only模式和请求SEQ，不访问位置平均对象。

修复包 `output/k230_qr_black_screen_fix_20261106/guanggongsai` 相对上一版仅competition_main.py变化，
参数/ROI/LAB/模型/标定/MAX_POINTS_PER_RING=24均保持字节一致。用户已确认MCU烧录配套qr_decode固件，本次无需重刷。
K230只覆盖competition_main.py并重启即可，也可使用完整ZIP。30项回归通过，尚未上板验证。

## 2026-11-06 QR只读任务码，成功后直接继续路线

基于11点版及仅修复打靶扫描的完整包，新增 `tasks/task_qr_read.py`。
二维码使用原生find_qrcodes读取ROI内唯一有效三位任务码，各位仍严格限定1..3。
不调用QRPoseEstimator、不计算坐标或手眼变换、不累计5帧；第一帧有效结果即缓存任务码并成功回传。
原二维码ROI、1920x1080灰度采集、对比度重试和超时配置保留；任务码仍只存K230，供后续作业选择目标。

新的QR成功结果为版本1的15字节头，task_cmd=0x10，原input_seq回显，status=SUCCESS，
flags/slot_id/sample_count/reason均为0，不带XYZ和scatter。MCU严格验证该格式后直接WORK_COMPLETE，
下一轮task_poll进入MOVE2；QR不进入WORK_ALIGN、不纠偏、不重复下发同一QR观察姿态等待3.6秒。
旧QR坐标包不会驱动底盘纠偏；收到时转为重试。原失败/超时重试、TOTALSTOP及禁止发CANCEL保留。

排爆、打靶和救援的定位/纠偏/5帧平均保持原行为；打靶单帧ROI扫描继续保留。
本次唯一参数改动：aim_config.py 的MAX_POINTS_PER_RING从16改为24；其余参数字节不变。

此版本必须同时更新MCU和K230。旧MCU不能解析15字节的QR成功ACK，会出现K230发OK但底盘不走。
K230完整包位于 `output/k230_qr_decode_only_20261106/guanggongsai`。
本机二维码状态机/协议/下一段MOVE回归通过，未实车或上板运行。

## 2026-11-06 进入观察位先yaw，打靶扫描使用ROI单帧LAB识别

MCU排爆、打靶、救援的ARM_OBSERVE改为两段：先仅ID1到目标yaw，等待3600ms；
再仅ID2/3/4到各自观察位的伸展/高度/pitch，等待3600ms。夹爪不参与。
QR观察仍为四关节同步。作业后回搬运位仍先ID2/3/4、后ID1，目标(0,250,100,-0.05)。
各段仍使用非阻塞发送、BUSY重试和错误处理，观察全部完成后才开始settle_ms与K230扫描请求。

此前K230打靶扫描先全图find_blobs，再用Python逐像素HSV提取所有候选的内外轮廓，最后才筛ROI；
LAB预览正确也可能因后续HSV筛选失败而被当作空帧。扫描要求连续3帧，屏幕却显示定位采样的0/5。
新包 `output/k230_fast_aim_scan_20261106/guanggongsai` 沿用9点备份及12秒协议补丁：
停车打靶扫描使用原生find_blobs(roi=SCAN_GATE_RAW_PX)，只检测QR指定颜色的一组LAB阈值，
一帧命中即返回目标位，不提取内外轮廓、不计算靶三维位置、不经过5帧位置平均。
未命中等待窗口结束返回20，多目标/识别异常不会伪装成正常未命中。

后续靶心精定位继续5帧平均，颜色轮廓改用同一LAB原生binary掩膜；只对ROI内候选提取轮廓，
再使用原几何模型/标定/精度校验。单组阈值检测返回的code=1会映射回实际目标颜色ID，而非一律当红色。
扫描显示“识别:0/1”和目标颜色，不再显示定位的0/5。

更新时保留板上现场aim_config.py的LAB/ROI：新代码对新增参数有默认值，无需覆盖该文件。
新包中的aim_config仅添加STOPPED_SCAN_MIN_HITS=1与颜色像素/框面积配置项，原LAB/ROI沿用9点备份。
阈值顺序仍是红/绿/蓝；若阈值重叠或顺序填错，必须以现场颜色检查为准，代码不自动修改现场阈值。
本机机械臂帧/FK/时序、MCU任务回归与26项K230回归通过；未实车或实机图像验证。

## 2026-11-06 扫码前配置错误：12秒扫描窗口

现场将 `scan_dwell_ms` 改为12000ms后，旧的通用配置校验仍限制最多10000ms，
连不使用此字段的QR阶段也会进入WORK_ERR_CONFIG，因此机械臂观察动作和0x10扫码指令都不会执行。
修复后QR完全不校验未使用的scan_dwell_ms；三类停车扫描的有效窗口为250..60000ms，保留当前12000ms配置。
MCU常量位于 `proto_vision_result.h` 的VISION_SCAN_WINDOW_MIN_MS/MAX_MS。

K230旧protocol.py也只接受最多10000ms。基于11月6日9点备份生成
`output/k230_scan_window_20261106/guanggongsai`，仅修改protocol.py的窗口校验到相同范围。
使用12秒扫描时须将这个protocol.py同步到板上；其余5帧采样、20mm散布、ROI、模型及标定文件均沿用9点备份。
窗口是识别的最长等待时间，提前确认目标/非目标仍立即返回，不强制等待12秒。

配置回归复现修复前WORK_ERR_CONFIG，修复后验证真实生产配置启动MOVE1、进入QR并发送0x10；
另覆盖QR忽略未使用字段、扫描窗口边界及12秒载荷。K230回归验证12秒窗口不会在10秒提前过期。
这些为本机状态机/协议验证，未烧录或实车运行。

## 2026-11-06 MCU不发送CANCEL，视觉失败自动重试

对照K230当前源码 `C:\Users\26617\OneDrive\Desktop\广工赛\11月6日9点备份k230\guanggongsai`：
定位使用5帧、散布上限20mm，定位超时60000ms；MCU接受3..32帧，作业层/算法层共用
`VISION_MEASUREMENT_MAX_SCATTER_MM=20.0f`，坐标范围、控制许可及非候选标定检查保留。

MCU不再发送K230取消命令0x7F；`app_vision_send()` 与 `app_vision_send_with_seq()` 的统一入口
也拒绝此命令并返回RET_PARAM。正常视觉异常不调用机械臂停止或结束主任务。
视觉请求发送失败/忙超时、等待超时、失败/取消回包、无效定位结果、观测航向变化、
纠偏幅度过大均进入WORK_RETRY_VISION：失效旧请求/坐标，等待settle_ms，再以新SEQ重发当前扫描或定位。
结构非法的回包仍忽略，由本地等待超时触发重试；旧SEQ和重复结果仍不能驱动动作。

重试不设终止次数。`max_corrections` 改为每批纠偏上限，达到后停稳重新采样并开始下一批；
之前已执行的纠偏偏移继续累计，用于作业结束回扫描起点。三位均未命中时返回第一位重新扫描。
目标未确认时不会假报作业成功，也不会进入下一段MOVE；识别恢复且对准后按原流程完成作业并继续路线。

主动TOTALSTOP，以及配置、IMU、机械臂或实际底盘运动故障，仍执行MCU本地停止，均不向K230发送CANCEL。
调试查看 `vision_retries`、`last_retry_error`、`vision_reason`；`error` 仅表示终止性故障。
自动重试本身仅修改MCU；使用超过10秒的扫描窗口还需同步上方K230协议补丁。尚未实车验证。

## 2026-11-06 定位采样数兼容

MCU作业层和坐标算法层共用的 `proto_vision_result_control_ready()` 现在接受至少3帧、最多32帧，
允许K230的 `MEASUREMENT_SAMPLES` 调整为5或7等值；低于3或超过32帧仍拒绝。
有效状态、控制许可、非候选标定、有限坐标的校验保留；散布上限现为20mm，与9点K230备份对齐。
此修改需要重新烧录MCU；旧固件仍严格要求3帧，收到5帧会以WORK_ERR_DATA停止。
K230当前 `MEASUREMENT_SAMPLES` 为各定位任务共用，增加它会同时增加各类作业的定位采样数。

## 2026-11-06 当前机械臂搬运/收臂姿态

排爆夹取后、排爆放置后收臂、打靶结束收臂、救援夹取及搬运回位，共用
`app_task_work_port.c` 的 `task_arm_travel_pose={0.0,250.0,100.0,-0.05}`，单位mm/rad。
先仅ID2/3/4到达该目标的伸展/高度/pitch，保持当前yaw；等待动作时间及停稳余量后，
再仅ID1转向目标yaw。夹爪保持进入收臂前的状态，仍通过原逆解和非阻塞发送路径下发。
二维码观察及QR任务自身收臂仍使用其独立的(0,260,60,0)姿态。

## 2026-11-06 当前流程：逐位停稳，命中即作业

排爆、打靶、救援在每个配置停车位完成观察姿态和停稳等待后，分别发送一次0x20/0x30/0x40。
请求载荷为4字节 `[0, slot, window_ms低字节, window_ms高字节]`：0表示目标沿用已读QR，
slot是本次停车位1/2/3，window使用 `scan_dwell_ms`（当前配置12000ms，范围250..60000ms）。
逐位扫描需要配套K230版本；二维码、定位请求保持空载荷，MCU不发送取消请求，结果仍是版本1的0x81。

K230只观察本次停车位，连续3帧稳定确认扫码目标后立即成功返回该slot，随后直接定位、纠偏和作业。
稳定识别到其他物品时，返回失败reason=20（SCAN_NOT_TARGET）；有效空帧累计且窗口到期也返回20。
只有这个明确的非目标结果允许MCU移至下一位。未收回包时保持停车；模型、相机、歧义和超时会触发当前位重试。
三位均返回20则返回第一位重新扫描。已命中时不扫描后面的物品，也不再折返目标位。

爆炸物仍搬运到原第二位对应的排爆桶，人质持续夹紧带到返回区，机械臂先ID2/3/4后yaw的动作顺序保留。
各非QR作业完成后，撤销本区扫描位移及已成功执行的视觉平移，恢复进入扫描时的航向，再进入下一段MOVE。
返回量来自运动指令累计和IMU，不是新增二维位置反馈；实车应验证回位误差。QR本身的纠偏行为保留。

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

`App/Inc/app_task_work.h` 的 `TASK_WORK_CAMERA_TEST` 当前为0；设1并重新编译可启用下面的模拟视觉联调。

- 仍由K230发送真实TOTALSTART启动，TOTALSTOP仍有效，任务请求仍正常发送。
- QR在 `WORK_WAIT_LOCATE` 模拟成功；其他三类在第一个停车位的 `WORK_WAIT_SCAN` 模拟命中，后续位置不扫描。
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
| TASK_EXPLOSIVE | 逐位停稳0x20 → 命中即0x21 → 微调/复测 → 夹取 → 搬到第二位放桶/收臂 → 回扫描起点 | MOVE12 |
| TASK_CS | 逐位停稳0x30 → 命中即0x31 → 微调/复测 → 激光照射/关闭 → 收臂 → 回扫描起点 | MOVE13～15 |
| TASK_RESCUE | 逐位停稳0x40 → 命中即0x41 → 微调/复测 → 夹取/收臂 → 夹紧回扫描起点 | MOVE16 |

作业通过 `app_task_work_poll()` 返回 WAITING、DONE 或 FAILED。只有 DONE 才开始下一组路线。
扫描序号1/2/3是MCU三个停车位的配置顺序，不是颜色或形状。一次只让一个物品进入K230识别门。
每位停稳后独立请求、独立SEQ；非目标才换下一位。移动期间K230空闲，不累计扫描结果。

选位/作业完成后，会回到该作业区原来的路线停车点，再执行下一组原路线。
三个slot的配置以同一扫描起点为基准。返回时撤回本区扫描位移和已执行的视觉纠偏，并恢复扫描起始航向。

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
`VISION_YAW_NONE` 只做平移，不校验或使用 `max_correction_yaw` 和 `yaw_tolerance_rad`。
启用QR轴或靶法向yaw纠偏的slot才使用这两个弧度参数：最大修正角须大于0且不超过30度，
容差须大于0且小于最大修正角。20度应写成 `0.017453293f * 20.0f`；`0.174532925f * 20` 实为200度。
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
RET_BUSY 每20ms重试，持续1秒后转入停稳/重发周期，不在主循环忙等。RET_OK 表示交给串口发送，不是对端 ACK。
旧 `app_vision_send()` 也改为相同的非阻塞发送方式。

始终只等待一个请求。结果需同时匹配 payload 中的 task_cmd 和 input_seq，外层 K230 SEQ 不作为请求序号。
版本、长度、flags、状态、保留位和浮点数先校验，再检查3..32帧样本、scatter≤20mm、目标序号与机构工作空间。
可选四元数/法向/抓取点依 flags 顺序解析，scatter 不固定在27字节偏移。
当前定位使用文档定义的球心/靶心/中点 XYZ；GRASP 只解析保存，不擅自替换参考点。
status=3 观测、候选标定、旧请求、重复结果、错误 CMD 的文本启停都不能启动运动。

本地定位超时65秒、扫描超时63秒。匹配的失败/取消回包和等待超时会停稳后重发当前请求。
收到reason=9（模型未就绪）仍在当前位重试，不报告作业成功；MCU不发送取消命令0x7F。

TOTALSTOP 先关闭激光并清除等待/成功事件，随后本轮 task_poll 取消底盘轨迹、复位闭环旧误差，回到 IDLE。
停止机械臂动作组使用非阻塞发送。新一轮启动等待本地机械臂停止完成；失败则进入故障，K230不接收取消命令。
动作组停止不等于能够中止控制板所有单舵机插值；以后新增直接舵机动作时，需要在 ARM_CANCEL hook 中补上控制板支持的保持/停止动作。
故障后可先 TOTALSTOP 清理，再 TOTALSTART 重新从扫码开始。

## OUT5V_EN 激光

直接使用 CubeMX 的 `OUT5V_EN_GPIO_Port` / `OUT5V_EN_Pin`（GPIOC/PC15）控制激光供电，
高电平开启、低电平关闭；GPIO 初始化默认低电平，启动时关闭激光。
只有打靶视觉复测已经对准时才开启；照射 `laser_ms` 后关闭，总停/任何作业错误也会关闭。
当前默认1000ms，是可调工程参数，不代表已验证的赛场判定时间。
按约定忽略激光与夹爪的位置偏差；仍须把观察姿态和期望靶心配置到实际激光射线上，底盘不能替代俯仰调整。

## 调试看什么

`task_get_state()` 查看仍保留的主任务状态。
`app_task_work_get_status()` 查看作业子阶段、error、K230 vision_reason、selected_slot、corrections、请求 CMD/SEQ、本地停止状态及最近坐标换算结果。
视觉重试时error保持NONE，查看vision_retries及last_retry_error；WORK_RETRY_VISION表示任务仍在自动恢复。
`alignment_valid` 只在观测仍可用于当前动作时有效；开始移动、搬动物体或结束作业后失效，数值保留供回看。
`height_error_mm` 是最近有效观测的目标高度减期望高度，`height_out_of_tolerance` 表示是否超过高度诊断阈值；二者与最近的 `alignment` 一同保留，下一次有效定位会更新，不作为底盘到位条件。

主循环调用顺序不变：IMU/视觉/底盘 poll → task_poll → 按10ms节拍 motion_update → closeloop_update。
全流程没有新增阻塞延时。现有 main 初始化阶段的机械臂调试动作不属于这里的作业 hook，本次没有修改其目标位置。

## 验证范围

`tests/run_host_tests.ps1` 编译实际 MCU 源码，覆盖交接文档 HEX 样例、分片/粘包/CRC、严格载荷解析、坐标与 yaw 符号、请求序号与错误发送、微调后复测、总停、超时、时钟回绕和救援失败。
整圈测试使用合成观测、理想底盘和已完成的模拟机械臂动作；不是实车定位精度或真实完赛时间证明。
没有烧录 MCU、运行 K230 或触发实际机械臂/激光。
