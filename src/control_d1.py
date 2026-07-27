"""Forward kinematics + Jacobian for D1 arm using pytorch_kinematics."""

from __future__ import annotations
import torch
import pytorch_kinematics as pk


class D1ArmKinematics:
    def __init__(self, urdf_path: str, end_effector_link: str = "Link6", device: str = "cpu"):
        with open(urdf_path, "rb") as f:
            urdf_data = f.read()

        self.chain = pk.build_serial_chain_from_urdf(
            urdf_data,
            end_link_name=end_effector_link,
            root_link_name="base_link",
        )
        self.chain = self.chain.to(device=device)
        self.device = device

    def joint_names(self) -> list[str]:
        return self.chain.get_joint_parameter_names()

    def forward(self, joint_pos: torch.Tensor):
        tf = self.chain.forward_kinematics(joint_pos)
        matrix = tf.get_matrix()
        ee_pos = matrix[:, :3, 3]
        ee_quat = pk.matrix_to_quaternion(matrix[:, :3, :3])
        return ee_pos, ee_quat

    def jacobian(self, joint_pos: torch.Tensor) -> torch.Tensor:
        return self.chain.jacobian(joint_pos)