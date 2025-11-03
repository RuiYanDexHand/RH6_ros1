#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在 ROS1 中用 Python 启动左右手两个 rh6_ctrl 节点的脚本（不使用 XML .launch）
用法：
  rosrun rh6_ctrl BothHands.py                 # 默认 can0 / can1
  rosrun rh6_ctrl BothHands.py can2 can3       # 指定左右接口
或者：
  python BothHands.py                          # 在已 source 的环境下

注意：请确保可执行名称与 CMakeLists.txt 中生成的一致（默认 rh6_ctrl_node）。
"""
from __future__ import print_function
import sys
import signal
import argparse

import rospy
import roslaunch


def parse_args(argv):
    parser = argparse.ArgumentParser(description='ROS1 dual-hand launcher (Python)')
    parser.add_argument('left_can', nargs='?', default='can0', help='left hand CAN interface (default: can0)')
    parser.add_argument('right_can', nargs='?', default='can1', help='right hand CAN interface (default: can1)')
    return parser.parse_args(argv)


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    args = parse_args(argv)

    rospy.init_node('rh6_dual_hands_launcher', anonymous=True)

    # 设置每个节点的私有参数（~can_interface）
    # 注意：私有参数路径为 /<namespace>/<name>/can_interface
    rospy.set_param('/rh6_left/left/can_interface', args.left_can)
    rospy.set_param('/rh6_right/right/can_interface', args.right_can)

    # 启动两个节点
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    # 如你的可执行名不同（默认 rh6_ctrl_node），请修改下面的 node_type
    node_left = roslaunch.core.Node(
        package='rh6_ctrl', node_type='rh6_ctrl_node', name='left', namespace='rh6_left', output='screen'
    )
    node_right = roslaunch.core.Node(
        package='rh6_ctrl', node_type='rh6_ctrl_node', name='right', namespace='rh6_right', output='screen'
    )

    # 启动并保持句柄，便于退出时停止
    process_left = launch.launch(node_left)
    process_right = launch.launch(node_right)

    def shutdown_handler(signum, frame):
        try:
            if process_left and process_left.is_alive():
                process_left.stop()
        except Exception:
            pass
        try:
            if process_right and process_right.is_alive():
                process_right.stop()
        except Exception:
            pass
        rospy.signal_shutdown('Received signal {}'.format(signum))

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    rospy.loginfo('Dual-hand nodes started: left(%s), right(%s)', args.left_can, args.right_can)

    # 等待直到节点退出或收到 Ctrl-C
    try:
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
    finally:
        shutdown_handler('finally', None)


if __name__ == '__main__':
    main()