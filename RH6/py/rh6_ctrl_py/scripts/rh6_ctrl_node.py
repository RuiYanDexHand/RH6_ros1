#!/usr/bin/env python3
# -*- coding: utf-8 -*-


from __future__ import print_function
import rospy
from threading import Lock
import math
import numpy as np
import os
from typing import List

from rh6_cmd.msg import Rh6Cmd
from rh6_msg.msg import Rh6Msg
from rh6_cmd.srv import Rh6fk, Rh6fkResponse, Rh6ik, Rh6ikResponse

# 导入 CAN 封装（与 ROS2 相同接口）
# from rh6_can_py_ros1.ryhandlib_wrapper import (
from rh6_can_py.ryhandlib_wrapper import (
    init_can_servo_lib,
    servo_move_mix,
    get_servo_data,
    cleanup_can_servo_lib,
)

# 运动学
# from rh6_kin_py_ros1.kinematics import (
from rh6_kin_py.kinematics import (
    Transform3D,
    KinematicsModel,
)


class RH6CtrlRos1(object):
    MOTOR_NUM = 6

    def __init__(self, name="rh6"):
        self._log = rospy.loginfo
        self._warn = rospy.logwarn
        self._err = rospy.logerr
        self._log("hello %s (ROS1)" % name)

        # 状态/命令缓存
        self.rh6msg = Rh6Msg()
        self.rh6cmd = Rh6Cmd()
        self._lock = Lock()

        # 多项式系数
        self.poly_coeff = np.zeros((5, 4))
        self.poly_coeff[0] = [0.000329, -0.035054, 2.558963, 0.272863]
        self.poly_coeff[1] = [0.000010, -0.004996, 1.426094, -0.044273]
        self.poly_coeff[2] = [0.000002, -0.002910, 1.283182, -0.088568]
        self.poly_coeff[3] = [0.000010, -0.004996, 1.426094, -0.044273]
        self.poly_coeff[4] = [0.000016, -0.006612, 1.529302, -0.011082]

        # URDF 路径
        pkg_share = rospy.get_param('~share_dir', None)
        if pkg_share is None:
            # 兼容：直接相对路径
            pkg_share = os.path.join(os.path.dirname(__file__), '..', 'urdf')
        self.urdf_filename_l = os.path.abspath(os.path.join(pkg_share, 'ruihand6z.urdf'))
        self.urdf_filename_r = os.path.abspath(os.path.join(pkg_share, 'ruihand6y.urdf'))

        # 运动学加载
        try:
            self.kin_model_l = KinematicsModel(self.urdf_filename_l)
            self.kin_model_r = KinematicsModel(self.urdf_filename_r)
            self._log("Kinematics (Pinocchio) loaded for L/R")
        except Exception as e:
            self._err("Kinematics load failed: %s" % e)
            self.kin_model_l = None
            self.kin_model_r = None

        # 指尖
        self.fingertip_l = ["fz14", "fz23", "fz33", "fz43", "fz53"]
        self.fingertip_r = ["fy14", "fy23", "fy33", "fy43", "fy53"]

        # 关节向量
        self.q_ = np.zeros(16)
        self.q_fk_ = np.zeros(16)
        self.q_ik_ = np.zeros(16)
        self.q_iik_ = np.zeros(16)

        # 参数
        self.pub_name = rospy.get_param('~ryhand_pub_topic_name', 'ryhand6_status')
        self.sub_name = rospy.get_param('~ryhand_sub_topic_name', 'ryhand6_cmd')
        self.control_type = rospy.get_param('~control_type', 'normal')
        # 默认 6 个电机 ID 映射
        self.servo_id_map = rospy.get_param('~servo_id_map', [1, 2, 3, 4, 5, 6])
        # 关节零位（度）与方向校准（+1/-1），按电机序号 0..5
        self.joint_zero_deg = rospy.get_param('~joint_zero_deg', [0, 0, 0, 0, 0, 0])
        self.joint_sign = rospy.get_param('~joint_sign', [1, 1, 1, 1, 1, 1])
        self._log("control_type = %s" % self.control_type)
        self._log("servo_id_map = %s" % self.servo_id_map)
        self._log("joint_zero_deg = %s" % self.joint_zero_deg)
        self._log("joint_sign = %s" % self.joint_sign)

        # Service
        self.srv_fk = rospy.Service('rh6_fk', Rh6fk, self.rh6fk_callback)
        self.srv_ik = rospy.Service('rh6_ik', Rh6ik, self.rh6ik_callback)

        # 初始化 CAN
        can_if = rospy.get_param('~can_interface', 'can0')
        self._log("CAN backend: real(libRyhand+SocketCAN), iface=%s" % can_if)
        init_can_servo_lib(can_if)

        # 电机结构（在订阅前初始化，避免回调早到导致属性未就绪）
        self.sut_servo_data_w = [self._create_servo_data() for _ in range(self.MOTOR_NUM)]
        self.sut_servo_data_r = [self._create_servo_data() for _ in range(self.MOTOR_NUM)]
        for i in range(self.MOTOR_NUM):
            self.sut_servo_data_w[i]['cmd']['usTp'] = 4095
            self.sut_servo_data_w[i]['cmd']['usTv'] = 1000
            self.sut_servo_data_w[i]['cmd']['usTc'] = 80

        # Pub/Sub（放到最后，确保内部状态已准备好）
        if self.control_type == 'normal':
            self.pub = rospy.Publisher(self.pub_name, Rh6Msg, queue_size=10)
            self.sub = rospy.Subscriber(self.sub_name, Rh6Cmd, self.cmd_callback, queue_size=10)
            self.timer = rospy.Timer(rospy.Duration(0.01), self.pub_state)  # 10ms

        self._log("RH6 控制节点初始化完成 (ROS1)")

    def shutdown(self):
        # 停止发布定时器，避免并发访问库
        try:
            if hasattr(self, 'timer') and self.timer is not None:
                self.timer.shutdown()
        except Exception:
            pass
        # 注销订阅与发布，尽量减少回调并发
        try:
            if hasattr(self, 'sub') and self.sub is not None:
                self.sub.unregister()
        except Exception:
            pass
        try:
            if hasattr(self, 'pub') and self.pub is not None:
                self.pub.unregister()
        except Exception:
            pass
        # 清理底层 CAN 库
        try:
            cleanup_can_servo_lib()
        except Exception:
            pass

    # ---------------------- helpers ----------------------
    def _create_servo_data(self):
        return {
            'cmd': {'usTp': 0, 'usTv': 0, 'usTc': 0},
            'info': {'ucStatus': 0, 'ub_P': 0, 'ub_V': 0, 'ub_I': 0, 'ub_F': 0},
        }

    def evaluate_polynomial(self, coeffs: np.ndarray, degree: int, x: float) -> float:
        result = 0.0
        for i in range(degree + 1):
            result += coeffs[i] * (x ** i)
        return result

    def deg_to_rad(self, deg: float) -> float:
        return deg * math.pi / 180.0

    def rad_to_deg(self, rad: float) -> float:
        return rad * 180.0 / math.pi

    def cmd_to_radx(self, cmd: int, range_rad: float) -> float:
        return (cmd - 2048) * range_rad / 2048.0

    def radx_to_cmd(self, rad: float, range_rad: float) -> int:
        return int(rad * 2048.0 / range_rad + 2048)

    def req_angle_to_model_q(self, req: List[float], q: np.ndarray, size: int = 16):
        for i in range(size):
            if i == 0:
                q[0] = req[0]
            elif i == 1:
                q[1] = req[1]
            elif i == 2:
                q[2] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[0], 3, self.rad_to_deg(q[i - 1])))
            elif i == 3:
                q[3] = 0
            elif i == 4:
                q[4] = req[3]
            elif i == 5:
                q[5] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[1], 3, self.rad_to_deg(q[i - 1])))
            elif i == 6:
                q[6] = 0
            elif i == 7:
                q[7] = req[5]
            elif i == 8:
                q[8] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[2], 3, self.rad_to_deg(q[i - 1])))
            elif i == 9:
                q[9] = 0
            elif i == 10:
                q[10] = req[7]
            elif i == 11:
                q[11] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[3], 3, self.rad_to_deg(q[i - 1])))
            elif i == 12:
                q[12] = 0
            elif i == 13:
                q[13] = req[9]
            elif i == 14:
                q[14] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[4], 3, self.rad_to_deg(q[i - 1])))
            elif i == 15:
                q[15] = 0

    def update_motor(self):
        for i in range(self.MOTOR_NUM):
            pos = int(self.sut_servo_data_w[i]['cmd']['usTp'])
            vel = int(self.sut_servo_data_w[i]['cmd']['usTv'])
            cur = int(self.sut_servo_data_w[i]['cmd']['usTc'])
            sid = i + 1
            if i < len(self.servo_id_map):
                sid = int(self.servo_id_map[i])
            servo_move_mix(sid, pos, vel, cur)

    def _select_kin(self, lr):
        if lr:
            return self.kin_model_r, self.fingertip_r
        else:
            return self.kin_model_l, self.fingertip_l

    def _build_q_from_j(self, j_ang: List[float]) -> np.ndarray:
        q = np.zeros(16)
        self.req_angle_to_model_q(list(j_ang), q, 16)
        return q

    # ---------------------- callbacks ----------------------
    def cmd_callback(self, cmd: Rh6Cmd):
        with self._lock:
            self.rh6msg.lr = cmd.lr
            for i in range(self.MOTOR_NUM):
                if i < len(cmd.m_pos):
                    self.sut_servo_data_w[i]['cmd']['usTp'] = cmd.m_pos[i]
                if i < len(cmd.m_spd):
                    self.sut_servo_data_w[i]['cmd']['usTv'] = cmd.m_spd[i]
                if i < len(cmd.m_curlimit):
                    self.sut_servo_data_w[i]['cmd']['usTc'] = cmd.m_curlimit[i]

            if cmd.mode == 2:
                # IK: 根据末端目标求关节角，并下发到电机
                ik_req = type('obj', (), {})()
                ik_req.lr = cmd.lr
                ik_req.x_base = cmd.x_base; ik_req.y_base = cmd.y_base; ik_req.z_base = cmd.z_base
                ik_req.roll_base = cmd.roll_base; ik_req.pitch_base = cmd.pitch_base; ik_req.yaw_base = cmd.yaw_base
                ik_req.x = cmd.x; ik_req.y = cmd.y; ik_req.z = cmd.z
                ik_req.roll = cmd.roll; ik_req.pitch = cmd.pitch; ik_req.yaw = cmd.yaw
                ik_res = self._solve_ik_service(ik_req)
                if ik_res and ik_res.ok:
                    # 将求得的 j_ang 映射到电机命令
                    for i in range(11):
                        if i >= len(ik_res.j_ang):
                            continue
                        if i == 0:
                            self.sut_servo_data_w[0]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(135))
                        elif i == 1:
                            self.sut_servo_data_w[1]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(40))
                        elif i == 3:
                            self.sut_servo_data_w[2]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(87))
                        elif i == 5:
                            self.sut_servo_data_w[3]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(90))
                        elif i == 7:
                            self.sut_servo_data_w[4]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(90))
                        elif i == 9:
                            self.sut_servo_data_w[5]['cmd']['usTp'] = self.radx_to_cmd(ik_res.j_ang[i], self.deg_to_rad(88.5))
                self.update_motor()
            elif cmd.mode == 1:
                # 使用零位与方向校准将关节角映射到电机命令
                for i in range(11):
                    if i >= len(cmd.j_ang):
                        continue
                    # 将关节角(弧度) -> 电机角(弧度)，再 -> 命令
                    def map_joint_to_cmd(j_idx: int, motor_idx: int, range_deg: float):
                        zero = self.deg_to_rad(float(self.joint_zero_deg[motor_idx]))
                        sign = float(self.joint_sign[motor_idx])
                        ang = zero + sign * float(cmd.j_ang[j_idx])
                        return self.radx_to_cmd(ang, self.deg_to_rad(range_deg))
                    if i == 0:
                        self.sut_servo_data_w[0]['cmd']['usTp'] = map_joint_to_cmd(0, 0, 135)
                    elif i == 1:
                        self.sut_servo_data_w[1]['cmd']['usTp'] = map_joint_to_cmd(1, 1, 40)
                    elif i == 3:
                        self.sut_servo_data_w[2]['cmd']['usTp'] = map_joint_to_cmd(3, 2, 87)
                    elif i == 5:
                        self.sut_servo_data_w[3]['cmd']['usTp'] = map_joint_to_cmd(5, 3, 90)
                    elif i == 7:
                        self.sut_servo_data_w[4]['cmd']['usTp'] = map_joint_to_cmd(7, 4, 90)
                    elif i == 9:
                        self.sut_servo_data_w[5]['cmd']['usTp'] = map_joint_to_cmd(9, 5, 88.5)
                self.update_motor()
            else:
                self.update_motor()
            self.rh6cmd = cmd

    def pub_state(self, _evt):
        with self._lock:
            for i in range(self.MOTOR_NUM):
                sd = get_servo_data(i + 1)
                if sd is None:
                    continue
                self.sut_servo_data_r[i]['info']['ub_P'] = sd.info['ub_P']
                self.sut_servo_data_r[i]['info']['ub_V'] = sd.info['ub_V']
                self.sut_servo_data_r[i]['info']['ub_I'] = sd.info['ub_I']
                self.sut_servo_data_r[i]['info']['ub_F'] = sd.info['ub_F']

                p = self.sut_servo_data_r[i]['info']['ub_P']
                v = self.sut_servo_data_r[i]['info']['ub_V']
                t = self.sut_servo_data_r[i]['info']['ub_I']

                if v > 2047:
                    v -= 4096
                if t > 2047:
                    t -= 4096

                if i < len(self.rh6msg.m_pos):
                    self.rh6msg.m_pos[i] = float(p)
                    self.rh6msg.m_spd[i] = float(v)
                    self.rh6msg.m_cur[i] = float(t)
                    self.rh6msg.m_force[i] = float(self.sut_servo_data_r[i]['info']['ub_F'])

                # 反向映射到关节角（加入零位/方向校准的反算）
                def cmd_to_joint(j_idx: int, motor_idx: int, range_deg: float):
                    ang = self.cmd_to_radx(p, self.deg_to_rad(range_deg))
                    zero = self.deg_to_rad(float(self.joint_zero_deg[motor_idx]))
                    sign = float(self.joint_sign[motor_idx])
                    # j = (ang - zero)/sign，若 sign=-1 则与方向相反
                    return (ang - zero) / (sign if sign != 0 else 1.0)

                if i == 0:
                    self.rh6msg.j_ang[0] = cmd_to_joint(0, 0, 135)
                elif i == 1:
                    self.rh6msg.j_ang[1] = cmd_to_joint(1, 1, 40)
                    self.rh6msg.j_ang[2] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[0], 3, self.rad_to_deg(self.rh6msg.j_ang[1])))
                elif i == 2:
                    self.rh6msg.j_ang[3] = cmd_to_joint(3, 2, 87)
                    self.rh6msg.j_ang[4] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[1], 3, self.rad_to_deg(self.rh6msg.j_ang[3])))
                elif i == 3:
                    self.rh6msg.j_ang[5] = cmd_to_joint(5, 3, 90)
                    self.rh6msg.j_ang[6] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[2], 3, self.rad_to_deg(self.rh6msg.j_ang[5])))
                elif i == 4:
                    self.rh6msg.j_ang[7] = cmd_to_joint(7, 4, 90)
                    self.rh6msg.j_ang[8] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[3], 3, self.rad_to_deg(self.rh6msg.j_ang[7])))
                elif i == 5:
                    self.rh6msg.j_ang[9] = cmd_to_joint(9, 5, 88.5)
                    self.rh6msg.j_ang[10] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[4], 3, self.rad_to_deg(self.rh6msg.j_ang[9])))

            # 发布
            self.pub.publish(self.rh6msg)

    # ---- FK/IK 实现 ----
    def rh6fk_callback(self, req):
        res = Rh6fkResponse()
        try:
            kin, fingertip = self._select_kin(req.lr)
            if kin is None:
                res.ok = False
                return res
            # j->q
            q = self._build_q_from_j(list(req.j_ang))
            # base transform
            Rb = KinematicsModel.euler_to_rotation_matrix(req.roll_base, req.pitch_base, req.yaw_base)
            tb = np.array([req.x_base, req.y_base, req.z_base])
            # FK for 5 tips
            xs, ys, zs = [], [], []
            rolls, pitchs, yaws = [], [], []
            ws, is_, js, ks = [], [], [], []
            for name in fingertip:
                tf = kin.forward_kinematics(q, name)
                if tf is None:
                    res.ok = False
                    return res
                # to world/base
                p = Rb.dot(tf.translation) + tb
                R = Rb.dot(tf.rotation)
                xs.append(float(p[0])); ys.append(float(p[1])); zs.append(float(p[2]))
                rpy = KinematicsModel.rotation_matrix_to_euler(R)
                rolls.append(float(rpy[0])); pitchs.append(float(rpy[1])); yaws.append(float(rpy[2]))
                qw, qx, qy, qz = KinematicsModel.rotation_matrix_to_quaternion(R)
                ws.append(float(qw)); is_.append(float(qx)); js.append(float(qy)); ks.append(float(qz))
            res.ok = True
            res.x = xs; res.y = ys; res.z = zs
            res.roll = rolls; res.pitch = pitchs; res.yaw = yaws
            res.w = ws; res.i = is_; res.j = js; res.k = ks
            return res
        except Exception as e:
            self._err("rh6fk failed: %s" % e)
            res.ok = False
            return res

    def _solve_ik_service(self, req):
        # 内部求解器，返回与服务一致的对象
        class _Res:
            pass
        out = _Res()
        try:
            kin, fingertip = self._select_kin(req.lr)
            if kin is None:
                out.ok = False
                return out
            # 目标在基座世界坐标，需转换到模型坐标
            Rb = KinematicsModel.euler_to_rotation_matrix(req.roll_base, req.pitch_base, req.yaw_base)
            tb = np.array([req.x_base, req.y_base, req.z_base])
            Rb_inv = Rb.T
            # 收集目标位置（仅用位置）
            targets = []
            for i in range(5):
                pw = np.array([req.x[i], req.y[i], req.z[i]])
                pt = Rb_inv.dot(pw - tb)
                targets.append(pt)
            # 初始 u = [j0, j1, j3, j5, j7, j9]
            j_init = getattr(self.rh6cmd, 'j_ang', [0.0]*11)
            if not j_init or len(j_init) < 11:
                j_init = [0.0]*11
            u = np.array([j_init[0], j_init[1], j_init[3], j_init[5], j_init[7], j_init[9]], dtype=float)

            def j_from_u(uvec):
                j = [0.0]*11
                j[0]=uvec[0]; j[1]=uvec[1]; j[3]=uvec[2]; j[5]=uvec[3]; j[7]=uvec[4]; j[9]=uvec[5]
                # coupled via polynomial like C++
                j[2] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[0], 3, self.rad_to_deg(j[1])))
                j[4] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[1], 3, self.rad_to_deg(j[3])))
                j[6] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[2], 3, self.rad_to_deg(j[5])))
                j[8] = self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[3], 3, self.rad_to_deg(j[7])))
                j[10]= self.deg_to_rad(self.evaluate_polynomial(self.poly_coeff[4], 3, self.rad_to_deg(j[9])))
                return j
            def q_from_u(uvec):
                return self._build_q_from_j(j_from_u(uvec))
            def f_pos(uvec):
                q = q_from_u(uvec)
                pos = []
                for name in fingertip:
                    tf = kin.forward_kinematics(q, name)
                    pos.extend(tf.translation.tolist())
                return np.array(pos)

            max_iters = 60
            tol = 1e-4
            alpha = 0.6
            du_eps = 1e-4
            # build stacked target vector
            y = np.concatenate([targets[i] for i in range(5)])
            for _ in range(max_iters):
                f = f_pos(u)
                e = f - y
                if np.linalg.norm(e) < tol:
                    break
                # numerical Jacobian (15x6)
                J = np.zeros((15, 6))
                for k in range(6):
                    u2 = u.copy(); u2[k] += du_eps
                    f2 = f_pos(u2)
                    J[:, k] = (f2 - f) / du_eps
                try:
                    du = -np.linalg.pinv(J) @ e
                except Exception:
                    break
                u = u + alpha * du
                u = np.clip(u, -2.0, 2.0)
            j_sol = j_from_u(u)
            out.ok = True
            out.j_ang = j_sol
            return out
        except Exception as e:
            self._err("rh6ik failed: %s" % e)
            out.ok = False
            return out

    def rh6ik_callback(self, req):
        res = Rh6ikResponse()
        out = self._solve_ik_service(req)
        if out and getattr(out, 'ok', False):
            res.ok = True
            res.j_ang = out.j_ang
        else:
            res.ok = False
        return res


def main():
    rospy.init_node('rh6')
    node = RH6CtrlRos1()
    _shutting_down = {'v': False}
    def _on_shutdown():
        # 防重复清理，降低 Ctrl+C 时段错误概率
        if _shutting_down['v']:
            return
        _shutting_down['v'] = True
        node.shutdown()
    rospy.on_shutdown(_on_shutdown)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        _on_shutdown()


if __name__ == '__main__':
    main()