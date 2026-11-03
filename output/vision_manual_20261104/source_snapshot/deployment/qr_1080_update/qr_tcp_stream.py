"""K230 独立调试：持续回传二维码中心在夹爪 TCP/E 坐标系中的位置。

部署：将本文件放到 /data/guanggongsai/qr_tcp_stream.py，单独运行它。
也可在板上 REPL 执行：from qr_tcp_stream import main; main()
不要与 competition_main.py 同时运行；本文件依赖原目录中的模块和标定文件。
二维码仍使用原 QRTask：边长由 config.QR_SIZE_MM 指定（当前80mm），内容
必须是三个1/2/3数字，例如123；画面有多个二维码时不发送有效坐标。

坐标：p_E = T_E_from_C * p_C，单位mm；位置是“二维码中心在夹爪夹取中心
坐标系下的XYZ”，不是夹爪在二维码系的位置，也不是底盘/机械臂基座坐标。
不读取、改变机械臂位姿，不重新标定，不改正式任务入口，不发TOTALSTART!。
高分辨率开关在runtime_config.QR_HIGH_RES_ENABLED：启用时仅二维码读取
1920x1080灰度通道，VGA彩色预览与YOLO保持原样。QRTask按同一完整视场
推导高分辨率内参，需上板复核距离/多姿态一致性；不对1080图直接套VGA内参。

默认启动即连续测量。用户键GPIO53仅在本脚本内切换暂停/继续，不发送总启停。
换姿态时：暂停 -> 改机械臂姿态 -> 到位停稳 -> 继续 -> 记录多组数据。
K230不知道机械臂是否真正停稳，5帧散布检查也不能代替物理停稳。
每次启动/继续、相机重启、识别无效及每次完成一批测量，均清空采样窗口。
仅发送新采集的有效批次；失去二维码时不重复发送上次坐标，不发送假零坐标。

串口复用runtime_config：UART2，TX=11，RX=12，115200，8N1。
复用VisionLink.pack/send和protocol.encode_result，完整帧为：
  AA 55 LEN SEQ 81 [原15字节结果头][XYZ][四元数wxyz][scatter] CRC8
正常接受标定时：LEN=47，完整53字节，task_cmd=0x10，status=0，flags=0x07，
slot_id=0，sample_count=5；浮点小端float32。scatter位于payload[43..46]。
手眼接受状态沿用runtime_config.HANDEYE_ACCEPTED_BY_USER；候选标定只会发
原协议OBSERVATION_ONLY状态，不擅自提升为可控制结果。

这是“连续观察”调试模式，数据布局与正式协议一致，连续发送语义不同：
不必先收到MCU请求；初始input_seq=0，收到空载荷0x10后回显其SEQ并继续发送。
收到空载荷0x7F暂停发送；再次0x10或按用户键继续。外层SEQ由VisionLink自增。
默认两次发送至少间隔1秒；SEND_INTERVAL_MS仅限制最短间隔，真实频率还受
5帧定位耗时限制。若定位较慢则延后发送，不重发旧坐标来凑固定频率。

MCU配合要点（本文件不会修改MCU）：
1. 当前task.c空闲时忽略0x81，作业层也只接收一次匹配请求。请在独立调试
   接收分支调用proto_vision_decode_result，不用正式任务状态机驱动底盘。
2. 对每个已停稳机械臂姿态，用对应的当前关节角/末端位姿生成T_B_from_E：
   alg_vision_transform_from_arm_joints 或 alg_vision_transform_from_arm_pose。
   再算 p_B[i] = t[i] + sum(R[i][j] * result.xyz_mm[j])。
   不要一直用task_work_load_config内固定二维码观察姿态的body_from_tcp。
3. 现有B系是X右/Y前/Z上，而机械臂位置指令+X向左；现有transform函数已
   处理此差异。比较p_B时统一坐标系，不要再手工翻转一次X。
4. 底盘与二维码固定，机械臂每次到位后保持不动，比较各姿态下p_B均值和
   波动。舵机指令值不等于位置反馈，应先确认实际到位。不要边动臂边比较。
"""

import gc
import os
import sys

PROJECT_ROOT = "/data/guanggongsai"
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

import runtime_config as cfg
import protocol as protocol
from calibration_params import load_calibration
from camera_service import CameraService, ticks_ms, ticks_diff
from measurement import MeasurementAverage
from tasks.task_qr import QRTask
from vision_link import VisionLink, _sleep_ms


# 独立调试开关；不修改runtime_config或正式比赛程序。
AUTO_START = True
ENABLE_USER_BUTTON = True
SEND_INTERVAL_MS = 1000          # 每秒至多一包新的有效结果。
SAMPLE_COUNT = 5                 # 与当前MCU两处5帧检查一致。
MAX_SCATTER_MM = 8.0             # 与当前MCU两处8mm检查一致。
MAX_ROTATION_SPREAD_DEG = 5.0
SETTLE_MS = 250                  # 手动停稳后恢复采样的额外等待。
INITIAL_INPUT_SEQ = 0
PREVIEW_INTERVAL_MS = 200
LOG_INTERVAL_MS = 1000


class QRStream:
    """无硬件初始化的流会话；测量、暂停和回包可离线验证。"""

    def __init__(self, task, link, clock=ticks_ms, diff=ticks_diff):
        self.task, self.link = task, link
        self.clock, self.diff = clock, diff
        self.average = MeasurementAverage(SAMPLE_COUNT, MAX_SCATTER_MM,
                                          MAX_ROTATION_SPREAD_DEG)
        self.active = False
        self.generation = 0
        self.run_id = 0
        self.input_seq = INITIAL_INPUT_SEQ
        self.started_ms = clock()
        self.last_send_ms = None
        self.epoch = None
        self.last_reason = "PAUSED"
        self.last_observation = None
        self.sent_count = 0

    def resume(self, input_seq=None):
        if input_seq is not None:
            self.input_seq = input_seq
        self.active = True
        self.generation += 1
        self.run_id = (self.run_id + 1) & 0xffff
        self.average.reset()
        self.started_ms = self.clock()
        self.last_reason = "SETTLING"

    def pause(self):
        self.active = False
        self.generation += 1
        self.average.reset()
        self.last_reason = "PAUSED"

    def process_frame(self, frame):
        if frame["epoch"] != self.epoch:
            self.epoch = frame["epoch"]
            self.average.reset()
            self.started_ms = self.clock()
        if not self.active:
            return None
        now = self.clock()
        if self.diff(now, self.started_ms) < SETTLE_MS:
            self.last_reason = "SETTLING"
            return None
        if self.last_send_ms is not None and self.diff(now, self.last_send_ms) < SEND_INTERVAL_MS:
            self.last_reason = "WAIT_NEXT_BATCH"
            return None
        gray = frame.get("qr_image")
        geometry = frame["geometry_confirmed"]
        if gray is None:
            gray = frame["image"].to_grayscale()
        else:
            geometry = frame.get("qr_geometry_confirmed", False)
        try:
            # 操作者须保持机械臂不动；此声明不是机械运动反馈。
            observed = self.task.measure(gray, frame["frame_id"],
                                         geometry_confirmed=geometry, stable=True)
        finally:
            gray = None
        observed["reference_point"] = "qr_center"
        observed["slot_id"] = 0
        observation, self.last_reason = self.average.add(observed)
        if observation is not None:
            # 批次互不重叠，不把不同机械臂姿态的旧样本带到下一批。
            self.average.reset()
        return observation

    def send_observation(self, observation):
        if not self.active:
            return False
        ready = observation.get("control_ready") is True
        status = protocol.SUCCESS if ready else protocol.OBSERVATION_ONLY
        reason = "OK" if ready else observation.get("reason", "HANDEYE_CANDIDATE_ONLY")
        request = {"cmd": protocol.QR_START, "input_seq": self.input_seq, "slot_id": 0}
        payload = protocol.encode_result(request, self.run_id, status, reason, observation)
        self.link.send(protocol.RESULT, payload)
        self.last_send_ms = self.clock()
        self.last_observation = observation
        self.sent_count += 1
        self.last_reason = "SENT" if ready else "OBSERVATION_ONLY"
        point = observation["position_tool_mm"]
        print("QR_TCP_TX", self.sent_count, "frame", observation["frame_id"],
              "input_seq", self.input_seq, "status", status,
              "p_E_mm=(%.3f, %.3f, %.3f)" % (point[0], point[1], point[2]),
              "scatter_mm=%.3f" % observation["scatter_mm"])
        return True


def pump_controls(link, stream, button=None):
    if not link.healthy():
        raise OSError("QR_STREAM_LINK_ERROR:" + str(link.diagnostics()))
    if button is not None:
        if not button.healthy():
            raise OSError("QR_STREAM_BUTTON_ERROR:" + str(button.error))
        while button.pressed():
            if stream.active:
                stream.pause()
            else:
                stream.resume()
            print("QR_STREAM", "RUNNING" if stream.active else "PAUSED")
    stop = link.pop_priority()
    if stop is not None:
        if not stop.payload:
            stream.pause()
            print("QR_STREAM_PAUSED_BY_MCU")
        else:
            print("QR_STREAM_BAD_STOP_PAYLOAD")
        return
    if link.take_overflow() is not None:
        raise OSError("QR_STREAM_COMMAND_QUEUE_OVERFLOW")
    for _ in range(cfg.COMMANDS_PER_POLL):
        frame = link.pop_command()
        if frame is None:
            break
        if frame.cmd == protocol.QR_START and not frame.payload:
            stream.resume(frame.seq)
            print("QR_STREAM_REQUEST", frame.seq)
        else:
            print("QR_STREAM_IGNORED_CMD", frame.cmd)


def render_preview(camera, stream, image=None):
    lines = ["QR坐标持续回传", "运行中" if stream.active else "已暂停",
             "坐标系:夹爪E", "单位:mm", "已发送:%d" % stream.sent_count,
             "采样:%d/%d" % (len(stream.average.samples), SAMPLE_COUNT), stream.last_reason]
    lines.insert(2,"QR:%dx%d" % cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else "QR:640x480")
    observation = stream.last_observation
    if observation is not None:
        age = stream.diff(stream.clock(), stream.last_send_ms)
        lines.append("上次发送:%dms前" % age)
        lines.extend(["%s:%+.1f" % (axis, value)
                      for axis, value in zip("XYZ", observation["position_tool_mm"])])
        lines.append("散布:%.2f" % observation["scatter_mm"])
    camera.render_debug(image, {"lines": lines})


def run_loop(link, stream, camera, button=None, max_loops=None,
             exitpoint=None, sleep_ms=_sleep_ms):
    loops = 0
    last_preview = last_log = stream.clock()
    frame = None
    while max_loops is None or loops < max_loops:
        loops += 1
        if exitpoint is not None:
            exitpoint()
        pump_controls(link, stream, button)
        select_qr = getattr(camera, "set_qr_capture", None)
        if select_qr is not None:
            select_qr(stream.active)
        generation = stream.generation
        frame = camera.get_frame()
        if camera.state == "ERROR":
            raise OSError(camera.error or "CAMERA_ERROR")
        pump_controls(link, stream, button)
        if frame is None:
            # 相机预热/恢复期间也不能保留此前的半批坐标。
            stream.average.reset()
            stream.last_reason = "CAMERA_" + camera.state if stream.active else "PAUSED"
        elif generation == stream.generation:
            observation = stream.process_frame(frame)
            # 检测可能耗时；其间收到暂停/新请求时，丢弃这一旧批次。
            pump_controls(link, stream, button)
            if observation is not None and generation == stream.generation and stream.active:
                stream.send_observation(observation)
        now = stream.clock()
        if stream.diff(now, last_preview) >= PREVIEW_INTERVAL_MS:
            render_preview(camera, stream, frame["image"] if frame is not None else None)
            last_preview = now
        if stream.diff(now, last_log) >= LOG_INTERVAL_MS:
            print("QR_STREAM_STATE", stream.last_reason,
                  "samples", len(stream.average.samples), "sent", stream.sent_count)
            last_log = now
        frame = None
        gc.collect()
        sleep_ms(cfg.LOOP_SLEEP_MS)


def main(max_loops=None):
    link = camera = button = stream = None
    try:
        calibration = load_calibration(cfg.PROJECT_ROOT,
                                       handeye_accepted_override=cfg.HANDEYE_ACCEPTED_BY_USER)
        if calibration.T_tool_from_camera is None:
            raise ValueError("HANDEYE_NOT_CONFIGURED")
        link = VisionLink(uart_id=cfg.UART_ID, baudrate=cfg.UART_BAUDRATE,
                          tx_pin=cfg.UART_TX_PIN, rx_pin=cfg.UART_RX_PIN,
                          queue_size=cfg.COMMAND_QUEUE_SIZE, priority_cmds=(protocol.STOP,),
                          read_chunk=cfg.RX_READ_BYTES)
        camera = CameraService(qr_enabled=cfg.QR_HIGH_RES_ENABLED)
        if ENABLE_USER_BUTTON:
            from user_button import UserButton
            button = UserButton()
        task = QRTask(calibration, high_resolution_size=cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else None)
        stream = QRStream(task, link)
        if AUTO_START:
            stream.resume()
        print("QR_TCP_STREAM", "UART", cfg.UART_ID, cfg.UART_BAUDRATE,
              "TX", cfg.UART_TX_PIN, "RX", cfg.UART_RX_PIN,
              "handeye_accepted", calibration.handeye_accepted)
        print("QR_TCP_STREAM: fixed base/QR; pause, move arm, settle, resume.")
        print("QR_TCP_STREAM: MCU must use CURRENT arm pose; no TOTALSTART is sent.")
        print("QR_CAPTURE_SIZE", cfg.QR_CAPTURE_SIZE if cfg.QR_HIGH_RES_ENABLED else (640,480),
              "RGB_PREVIEW_YOLO_SIZE", (640,480))
        render_preview(camera, stream)
        run_loop(link, stream, camera, button, max_loops, getattr(os, "exitpoint", None))
    except KeyboardInterrupt:
        print("QR_TCP_STREAM_STOPPED")
    finally:
        if stream is not None:
            stream.pause()
        try:
            if button is not None and not button.close():
                print("BUTTON_SHUTDOWN_TIMEOUT")
        finally:
            try:
                if camera is not None:
                    camera.close()
            finally:
                if link is not None and not link.deinit():
                    print("UART_SHUTDOWN_TIMEOUT")


if __name__ == "__main__":
    main()
