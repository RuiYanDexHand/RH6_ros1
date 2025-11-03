#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS1 测试节点（极简版，改为正弦演示）：
- mode=0：发送原始电机位置（正弦）。
- mode=1：发送关节角（正弦，6 个驱动关节：j0,j1,j3,j5,j7,j9）。
- mode=2：发送末端位置（正弦，5 指尖 z 方向起伏）。
"""
import rospy
from rh6_cmd.msg import Rh6Cmd
from rh6_msg.msg import Rh6Msg
import math, time

# 固定参数
LR = 0                  # 0=左手，1=右手
SPEED = 800.0
CURRENT = 80.0

# 正弦设置（通用）
FREQ_HZ = 0.2

# mode=0（电机原始命令）的正弦设置
AMP_CMD = 700.0
CENTER_CMD = 2048.0

# mode=1（关节角）幅值上限（度）。按电机允许角度的 60% 作为幅值，留边界
JOINT_RANGE_DEG = [135.0, 40.0, 0.0, 87.0, 0.0, 90.0, 0.0, 90.0, 0.0, 88.5, 0.0]
JOINT_USE_IDX = [0, 1, 3, 5, 7, 9]  # 6 个驱动关节
JOINT_PHASE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 同步运动，避免耦合导致速度不均匀
AMP_RATIO = 0.4  # 使用范围的百分比

# mode=2（末端位置）正弦：以 z 方向起伏（米）
Z_AMP = 0.01  # 1 cm 起伏
XYZ_BASE = [0.0, 0.0, 0.0]  # 基座位姿
RPY_BASE = [0.0, 0.0, 0.0]
FINGER_PHASE = [0.0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3]

# 选择运行模式（可通过参数 ~test_mode 覆盖：0/1/2）
DEFAULT_MODE = 1


class Rh6TestNode(object):
    def __init__(self):
        rospy.init_node('rh6_test')
        
        # 先获取模式参数，然后再初始化其他属性
        self.mode = rospy.get_param('~test_mode', DEFAULT_MODE)
        self.start_t = time.time()  # 正弦开始时间
        
        self.pub = rospy.Publisher('ryhand6_cmd', Rh6Cmd, queue_size=10)
        self.sub = rospy.Subscriber('ryhand6_status', Rh6Msg, self.cb_status, queue_size=10)
        
        rospy.loginfo('hello rh6_test (sine demo), mode=%s' % self.mode)
        
        # 初始化：先发送零位置命令
        self.send_zero_cmd()
        
        # 创建定时器（放在最后）
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_cb)
        
        rospy.spin()

    def send_zero_cmd(self):
        # 发送零位置命令，让电机归零
        msg = Rh6Cmd()
        msg.mode = int(self.mode)
        msg.lr = LR
        msg.m_spd = [SPEED]*6
        msg.m_curlimit = [CURRENT]*6
        
        # 初始化末端姿态的默认值
        roll = [0.0]*5
        pitch = [0.0]*5
        yaw = [0.0]*5
        
        if msg.mode == 0:
            msg.m_pos = [CENTER_CMD]*6  # 零位置
        elif msg.mode == 1:
            msg.j_ang = [0.0]*11  # 零关节角
        else:
            msg.x_base, msg.y_base, msg.z_base = XYZ_BASE
            msg.roll_base, msg.pitch_base, msg.yaw_base = RPY_BASE
            msg.x = [0.0]*5; msg.y = [0.0]*5; msg.z = [0.0]*5
            msg.roll = roll; msg.pitch = pitch; msg.yaw = yaw
        
        self.pub.publish(msg)
        rospy.loginfo("Sent zero command for mode %d" % msg.mode)
        rospy.sleep(1.0)  # 使用 rospy.sleep 而不是 time.sleep

    def timer_cb(self, _evt):
        t = time.time() - self.start_t
        phase0 = 2.0 * math.pi * FREQ_HZ * t

        # 1) mode=0：电机命令正弦
        phases_m = [0.0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3]
        pos = [CENTER_CMD + AMP_CMD * math.sin(phase0 + p) for p in phases_m]
        pos = [float(max(0.0, min(4095.0, v))) for v in pos]

        # 2) mode=1：关节角正弦（弧度），仅填充 j0,j1,j3,j5,j7,j9
        j = [0.0]*11
        for k, j_idx in enumerate(JOINT_USE_IDX):
            rng = JOINT_RANGE_DEG[j_idx]
            amp_deg = AMP_RATIO * rng
            ang_deg = amp_deg * math.sin(phase0 + JOINT_PHASE[k])
            j[j_idx] = math.radians(ang_deg)

        # 3) mode=2：末端 5 指尖 z 方向正弦
        x = [0.0]*5; y = [0.0]*5; z = [0.0]*5
        roll = [0.0]*5; pitch = [0.0]*5; yaw = [0.0]*5
        for i in range(5):
            x[i] = 0.0
            y[i] = 0.0
            z[i] = Z_AMP * math.sin(phase0 + FINGER_PHASE[i])

        # 填充消息
        msg = Rh6Cmd()
        msg.mode = int(self.mode)
        msg.lr = LR
        msg.m_spd = [SPEED]*6
        msg.m_curlimit = [CURRENT]*6

        if msg.mode == 0:
            msg.m_pos = pos
        elif msg.mode == 1:
            msg.j_ang = j
        else:  # mode==2
            msg.x_base, msg.y_base, msg.z_base = XYZ_BASE
            msg.roll_base, msg.pitch_base, msg.yaw_base = RPY_BASE
            msg.x = x; msg.y = y; msg.z = z
            msg.roll = roll; msg.pitch = pitch; msg.yaw = yaw

        self.pub.publish(msg)

    def cb_status(self, m):
        # 移除不必要的状态输出
        pass


def main():
    rospy.init_node('rh6_test')
    Rh6TestNode()
    rospy.spin()

if __name__ == '__main__':
    main()