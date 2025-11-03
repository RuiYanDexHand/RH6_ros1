#!/usr/bin/env python3

import numpy as np
import math
from typing import Optional, Tuple

# 使用 Pinocchio 进行真实运动学计算
try:
    import pinocchio as pin
except Exception as e:
    pin = None
    _PIN_ERROR = e
else:
    _PIN_ERROR = None

class Transform3D:
    def __init__(self, translation: np.ndarray = None, rotation: np.ndarray = None):
        self.translation = translation if translation is not None else np.zeros(3)
        self.rotation = rotation if rotation is not None else np.eye(3)

    def to_matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self.rotation
        T[:3, 3] = self.translation
        return T

class KinematicsModel:
    """基于 Pinocchio 的运动学模型。要求系统已安装 pinocchio。
    URDF 由构造函数传入，q 的维度取决于 URDF 模型。默认我们仍按 16 维准备/裁剪。
    """
    def __init__(self, urdf_path: str):
        if pin is None:
            raise RuntimeError(f"Pinocchio 未安装或导入失败: {_PIN_ERROR}")
        self.urdf_path = urdf_path
        # 构建模型与数据
        try:
            self.model = pin.buildModelFromUrdf(self.urdf_path)
        except Exception as e:
            raise RuntimeError(f"加载 URDF 失败: {urdf_path}, 错误: {e}")
        self.data = self.model.createData()
        # 默认 16，自行按节点代码裁剪/填充
        self.dof = 16
        # 根据模型实际关节数准备界限（若模型维度不同则扩展/裁剪到 16）
        nq = self.model.nq
        self.joint_limits_lower = np.full(self.dof, -2.0)
        self.joint_limits_upper = np.full(self.dof,  2.0)
        # 常见指尖链接名（左手）；右手可传 fy14 等
        self.link_names = ["fz14", "fz23", "fz33", "fz43", "fz53"]

    def _fit_q_to_model(self, q16: np.ndarray) -> np.ndarray:
        """将 16 维 q 裁剪/填充到模型维度（self.model.nq）。多余部分裁剪，不足部分补 0。"""
        nq = self.model.nq
        q = np.zeros(nq)
        n = min(nq, q16.shape[0])
        q[:n] = q16[:n]
        return q

    def forward_kinematics(self, q: np.ndarray, link_name: str) -> Optional[Transform3D]:
        if link_name is None:
            return None
        try:
            q_m = self._fit_q_to_model(q)
            pin.forwardKinematics(self.model, self.data, q_m)
            pin.updateFramePlacements(self.model, self.data)
            fid = self.model.getFrameId(link_name)
            oMf = self.data.oMf[fid]
            t = np.array(oMf.translation).reshape(3)
            R = np.array(oMf.rotation)
            return Transform3D(t, R)
        except Exception:
            return None

    def compute_jacobian(self, q: np.ndarray, link_name: str) -> np.ndarray:
        q_m = self._fit_q_to_model(q)
        pin.computeJointJacobians(self.model, self.data, q_m)
        pin.updateFramePlacements(self.model, self.data)
        fid = self.model.getFrameId(link_name)
        J6 = pin.computeFrameJacobian(self.model, self.data, q_m, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        return np.array(J6)[:3, :self.dof]

    def inverse_kinematics(self, target: Transform3D, q0: np.ndarray, link_name: str,
                            iters: int = 100, tol: float = 1e-6, step: float = 0.1) -> Optional[np.ndarray]:
        q = q0.copy()
        tgt = target.translation
        for _ in range(iters):
            cur = self.forward_kinematics(q, link_name)
            if cur is None:
                return None
            e = tgt - cur.translation
            if np.linalg.norm(e) < tol:
                return q
            J = self.compute_jacobian(q, link_name)
            try:
                dq = np.linalg.pinv(J) @ e
            except Exception:
                return None
            q[:self.dof] += step * dq[:self.dof]
            q = np.clip(q, self.joint_limits_lower, self.joint_limits_upper)
        return q

    @staticmethod
    def euler_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])

    @staticmethod
    def rotation_matrix_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
        sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(R[2,1], R[2,2])
            pitch = math.atan2(-R[2,0], sy)
            yaw = math.atan2(R[1,0], R[0,0])
        else:
            roll = math.atan2(-R[1,2], R[1,1])
            pitch = math.atan2(-R[2,0], sy)
            yaw = 0.0
        return roll, pitch, yaw

    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
        tr = np.trace(R)
        if tr > 0:
            s = math.sqrt(tr + 1.0) * 2
            w = 0.25 * s
            x = (R[2,1] - R[1,2]) / s
            y = (R[0,2] - R[2,0]) / s
            z = (R[1,0] - R[0,1]) / s
        elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
            w = (R[2,1] - R[1,2]) / s
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
        elif R[1,1] > R[2,2]:
            s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
            w = (R[0,2] - R[2,0]) / s
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
        else:
            s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
            w = (R[1,0] - R[0,1]) / s
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
        return np.array([w, x, y, z])
