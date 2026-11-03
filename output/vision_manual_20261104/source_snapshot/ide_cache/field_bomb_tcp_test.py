"""赛道固定底盘测试：爆炸物球心。
单独运行本文件；外层CMD=0x81、内层task_cmd=0x10、slot_id=0，兼容现有MCU回调。
这是坐标调试通道；真实测量点为爆炸物球心，不改变MCU或正式程序。
"""
import sys
if "/data/guanggongsai" not in sys.path:sys.path.insert(0,"/data/guanggongsai")
from field_tcp_stream import main as run_test

# TARGET_ID: 1红 / 2绿 / 3蓝；与位置序号无关
TARGET_ID = 1
SAMPLE_COUNT = 3
SEND_INTERVAL_MS = 1000
SAMPLE_WINDOW_MS = 3000  # 仅二维码允许窗口内短暂漏检；其他目标仍需连续有效帧。
AUTO_START = True
ENABLE_USER_BUTTON = True
SHOW_TO_IDE = True  # LCD与CanMV IDE预览同时显示目标框。

# 下面只用于计算/打印理论对照，绝不替代视觉实测值，也不会驱动机械臂。
# 如需对比理论TCP，把此处填成你本次MCU的vision_arm_pose，单位mm、rad。
# 保持None仍可正常测量/发包；此时不输出假定姿态的B系实测换算值。
ARM_POSE_FOR_REFERENCE = None
# None沿用field_tcp_reference.py中当前作业配置的B系停车参考点。
# 现场量出基座到目标参考点的真实坐标后，可在这里覆盖为(x,y,z)。
REFERENCE_BODY_MM = None


def main():
    return run_test("bomb",target_id=TARGET_ID,arm_pose=ARM_POSE_FOR_REFERENCE,
                    body_reference=REFERENCE_BODY_MM,sample_count=SAMPLE_COUNT,
                    send_interval_ms=SEND_INTERVAL_MS,sample_window_ms=SAMPLE_WINDOW_MS,
                    auto_start=AUTO_START,enable_user_button=ENABLE_USER_BUTTON,show_to_ide=SHOW_TO_IDE)


if __name__=="__main__":main()
