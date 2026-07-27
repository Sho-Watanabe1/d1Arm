"""
Standalone Differential Inverse Kinematics controller.

Reimplements the public API of `isaaclab.controllers.DifferentialIKController`
/ `DifferentialIKControllerCfg` using plain PyTorch. No Isaac Sim, no PhysX,
no GPU required. You supply the robot Jacobian yourself (e.g. from a URDF via
`pytorch_kinematics`, `pinocchio`, or your own forward kinematics), instead of
getting it from the simulator.

Usage mirrors Isaac Lab:

    cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    controller = DifferentialIKController(cfg, num_envs=1, device="cpu")
    controller.set_command(command)
    joint_pos_des = controller.compute(ee_pos, ee_quat, jacobian, joint_pos)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import torch

from issaclabFiles.mathUtils import apply_delta_pose, compute_pose_error


@dataclass
class DifferentialIKControllerCfg:
    """Configuration for the differential IK controller."""

    command_type: Literal["position", "pose"] = "pose"
    use_relative_mode: bool = False
    ik_method: Literal["pinv", "svd", "trans", "dls"] = "dls"
    ik_params: dict = field(default_factory=dict)


class DifferentialIKController:
    r"""Differential inverse kinematics (IK) controller.

    Computes the change in joint positions that yields a desired change in
    end-effector pose:

        dq = J^+ dx
        q_desired = q_current + dq

    where J^+ is a (pseudo-)inverse of the Jacobian J, computed via one of:
      - "pinv":  Moore-Penrose pseudo-inverse
      - "svd":   truncated SVD-based pseudo-inverse (robust near singularities)
      - "trans": Jacobian transpose (cheap, less accurate)
      - "dls":   damped least-squares (Levenberg-Marquardt style damping)
    """

    def __init__(self, cfg: DifferentialIKControllerCfg, num_envs: int, device: str = "cpu"):
        self.cfg = cfg
        self.num_envs = num_envs
        self.device = device

        self._command_dim = 3 if cfg.command_type == "position" else 7
        self._command = torch.zeros(self.num_envs, self._command_dim, device=self.device)

        # default ik method params
        self._ik_params = {
            "pinv": {"k_val": 1.0},
            "svd": {"k_val": 1.0, "min_singular_value": 1e-5},
            "trans": {"k_val": 1.0},
            "dls": {"lambda_val": 0.05},
        }[cfg.ik_method]
        if cfg.ik_params:
            self._ik_params.update(cfg.ik_params)

    @property
    def action_dim(self) -> int:
        return self._command_dim

    def reset(self, env_ids: torch.Tensor | None = None) -> None:
        if env_ids is None:
            self._command[:] = 0.0
        else:
            self._command[env_ids] = 0.0

    def set_command(
        self,
        command: torch.Tensor,
        ee_pos: torch.Tensor | None = None,
        ee_quat: torch.Tensor | None = None,
    ) -> None:
        """Set the desired end-effector command.

        If `use_relative_mode` is True, `command` is a delta to be applied to
        the current pose (ee_pos, ee_quat must be given). Otherwise `command`
        is treated as an absolute target pose/position.
        """
        self._command[:] = command
        if self.cfg.use_relative_mode:
            if ee_pos is None or ee_quat is None:
                raise ValueError("ee_pos and ee_quat are required in relative mode.")
            if self.cfg.command_type == "position":
                self._command[:, 0:3] += ee_pos
            else:
                pos, quat = apply_delta_pose(ee_pos, ee_quat, self._command)
                self._command[:, 0:3] = pos
                # replace rotation part of internal 7-dim command with resulting quat
                if self._command_dim == 7:
                    self._command = torch.cat([pos, quat], dim=-1)

    def compute(
        self,
        ee_pos: torch.Tensor,
        ee_quat: torch.Tensor,
        jacobian: torch.Tensor,
        joint_pos: torch.Tensor,
    ) -> torch.Tensor:
        """Compute desired joint positions.

        Args:
            ee_pos: current end-effector position, shape (N, 3)
            ee_quat: current end-effector orientation (w,x,y,z), shape (N, 4)
            jacobian: geometric Jacobian, shape (N, 6, num_joints) if command_type
                is "pose", or (N, 3, num_joints) if "position"
            joint_pos: current joint positions, shape (N, num_joints)

        Returns:
            Desired joint positions, shape (N, num_joints)
        """
        pos_err, rot_err = self._compute_pose_error(ee_pos, ee_quat)
        if self.cfg.command_type == "position":
            pose_err = pos_err
        else:
            pose_err = torch.cat([pos_err, rot_err], dim=-1)

        delta_joint_pos = self._solve_delta_joint_pos(pose_err, jacobian)
        return joint_pos + delta_joint_pos

    def _compute_pose_error(self, ee_pos: torch.Tensor, ee_quat: torch.Tensor):
        if self.cfg.command_type == "position":
            if self.cfg.use_relative_mode:
                return self._command[:, 0:3], torch.zeros_like(ee_pos)
            return self._command[:, 0:3] - ee_pos, torch.zeros_like(ee_pos)
        else:
            target_pos = self._command[:, 0:3]
            target_quat = self._command[:, 3:7]
            return compute_pose_error(ee_pos, ee_quat, target_pos, target_quat, rot_error_type="axis_angle")

    def _solve_delta_joint_pos(self, pose_err: torch.Tensor, jacobian: torch.Tensor) -> torch.Tensor:
        method = self.cfg.ik_method
        dx = pose_err.unsqueeze(-1)  # (N, 6 or 3, 1)

        if method == "pinv":
            k = self._ik_params["k_val"]
            J_pinv = torch.linalg.pinv(jacobian)
            dq = k * J_pinv @ dx

        elif method == "svd":
            k = self._ik_params["k_val"]
            min_sv = self._ik_params["min_singular_value"]
            U, S, Vh = torch.linalg.svd(jacobian, full_matrices=False)
            S_inv = torch.where(S > min_sv, 1.0 / S, torch.zeros_like(S))
            J_pinv = torch.einsum("nij,nj,njk->nki", U, S_inv, Vh)
            dq = k * J_pinv @ dx

        elif method == "trans":
            k = self._ik_params["k_val"]
            J_T = jacobian.transpose(-2, -1)
            dq = k * J_T @ dx

        elif method == "dls":
            lam = self._ik_params["lambda_val"]
            J = jacobian
            J_T = J.transpose(-2, -1)
            n = J.shape[-2]
            eye = torch.eye(n, device=J.device).unsqueeze(0).expand(J.shape[0], -1, -1)
            dq = J_T @ torch.linalg.solve(J @ J_T + (lam**2) * eye, dx)

        else:
            raise ValueError(f"Unknown ik_method: {method}")

        return dq.squeeze(-1)