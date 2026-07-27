"""KinematicsBackend adapter using pytorch_kinematics."""

import torch

from d1_ik_controller import KinematicsBackend, KinSnapshot
from control_d1 import D1ArmKinematics


class PyTorchKinematicsBackend(KinematicsBackend):
    def __init__(self, urdf_path: str, end_effector_link: str = "Link6", device: str = "cpu"):
        self.kin = D1ArmKinematics(urdf_path, end_effector_link, device)
        self.device = device

    def snapshot(self, reported_angles_rad: torch.Tensor) -> KinSnapshot:
        joint_pos = reported_angles_rad.to(self.device)
        ee_pos, ee_quat = self.kin.forward(joint_pos)
        jacobian = self.kin.jacobian(joint_pos)
        return KinSnapshot(
            ee_pos=ee_pos,
            ee_quat=ee_quat,
            jacobian=jacobian,
            joint_pos=joint_pos,
        )

    def to_world(self, rel_pos, rel_quat):
        return rel_pos, rel_quat

    def to_local(self, world_pos, world_quat):
        return world_pos, world_quat