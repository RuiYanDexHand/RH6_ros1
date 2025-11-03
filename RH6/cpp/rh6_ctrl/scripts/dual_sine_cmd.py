#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dual-hand sine command publisher for RH6 (ROS1 Noetic)
- Publishes mode=0 raw motor commands to both left and right command topics
- Topics (match BothHands.py + node private NH("~")):
  * Left:  /rh6_left/left/ryhand6_cmd
  * Right: /rh6_right/right/ryhand6_cmd

Params (private):
- ~rate: publish rate Hz (default: 50.0)
- ~freq: sine frequency Hz (default: 0.25)
- ~base: center position (default: 2000.0)
- ~amp : amplitude (default: 800.0)
"""
import math
import rospy
from rh6_cmd.msg import Rh6Cmd

MOTOR_MIN = 0.0
MOTOR_MAX = 4095.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def build_msg(lr, t, base=2000.0, amp=800.0, freq=0.25, phase=0.0):
    msg = Rh6Cmd()
    msg.mode = 0
    msg.lr = lr

    positions = []
    omega = 2.0 * math.pi * freq
    for i in range(6):
        pos = base + amp * math.sin(omega * t + phase + i * 0.3)
        positions.append(clamp(pos, MOTOR_MIN, MOTOR_MAX))

    msg.m_pos = positions
    msg.m_spd = [1000.0] * 6
    msg.m_curlimit = [80.0] * 6

    # Fill remaining fields to satisfy message
    msg.j_ang = [0.0] * 11
    msg.x_base = 0.0
    msg.y_base = 0.0
    msg.z_base = 0.0
    msg.roll_base = 0.0
    msg.pitch_base = 0.0
    msg.yaw_base = 0.0
    msg.x = [0.0] * 5
    msg.y = [0.0] * 5
    msg.z = [0.0] * 5
    msg.roll = [0.0] * 5
    msg.pitch = [0.0] * 5
    msg.yaw = [0.0] * 5
    return msg


def main():
    rospy.init_node('rh6_dual_sine_cmd', anonymous=True)
    rate_hz = rospy.get_param('~rate', 50.0)
    freq = rospy.get_param('~freq', 0.25)
    base = rospy.get_param('~base', 2000.0)
    amp = rospy.get_param('~amp', 800.0)

    left_topic = '/rh6_left/left/ryhand6_cmd'
    right_topic = '/rh6_right/right/ryhand6_cmd'
    pub_l = rospy.Publisher(left_topic, Rh6Cmd, queue_size=1)
    pub_r = rospy.Publisher(right_topic, Rh6Cmd, queue_size=1)

    rospy.loginfo('Dual sine publisher -> %s and %s', left_topic, right_topic)
    rospy.sleep(1.0)

    r = rospy.Rate(rate_hz)
    t0 = rospy.get_time()
    phase_right = math.pi  # make motions out of phase for clarity

    while not rospy.is_shutdown():
        t = rospy.get_time() - t0
        pub_l.publish(build_msg(lr=0, t=t, base=base, amp=amp, freq=freq, phase=0.0))
        pub_r.publish(build_msg(lr=1, t=t, base=base, amp=amp, freq=freq, phase=phase_right))
        r.sleep()


if __name__ == '__main__':
    main()