"""
Standalone quaternion / pose math utilities.

Reimplements the public API surface of `isaaclab.utils.math` (quat_mul,
quat_inv, quat_apply, quat_conjugate, apply_delta_pose, compute_pose_error)
using plain PyTorch. No Isaac Sim, no `omni.*` modules, no GPU required —
everything here runs fine on CPU tensors.

Quaternion convention: (w, x, y, z), same as Isaac Lab.
"""

from __future__ import annotations

from typing import Literal

import torch


def normalize(x: torch.Tensor, eps: float = 1e-9) -> torch.Tensor:
    """Normalize a tensor along its last dimension to unit length."""
    return x / x.norm(p=2, dim=-1, keepdim=True).clamp(min=eps)


def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    """Conjugate of a quaternion (w, x, y, z) -> (w, -x, -y, -z)."""
    q = q.clone()
    q[..., 1:] *= -1.0
    return q


def quat_inv(q: torch.Tensor) -> torch.Tensor:
    """Inverse of a quaternion. For unit quaternions this equals the conjugate;
    for non-unit quaternions we divide by the squared norm."""
    q_conj = quat_conjugate(q)
    norm_sq = (q * q).sum(dim=-1, keepdim=True).clamp(min=1e-9)
    return q_conj / norm_sq


def quat_mul(q1: torch.Tensor, q2: torch.Tensor) -> torch.Tensor:
    """Hamilton product of two quaternions (w, x, y, z)."""
    if q1.shape != q2.shape:
        raise ValueError(f"quat_mul: shape mismatch {q1.shape} vs {q2.shape}")
    shape = q1.shape
    q1 = q1.reshape(-1, 4)
    q2 = q2.reshape(-1, 4)

    w1, x1, y1, z1 = q1[:, 0], q1[:, 1], q1[:, 2], q1[:, 3]
    w2, x2, y2, z2 = q2[:, 0], q2[:, 1], q2[:, 2], q2[:, 3]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return torch.stack([w, x, y, z], dim=-1).view(shape)


def quat_apply(quat: torch.Tensor, vec: torch.Tensor) -> torch.Tensor:
    """Rotate a vector (x, y, z) by a quaternion (w, x, y, z)."""
    shape = vec.shape
    quat = quat.reshape(-1, 4)
    vec = vec.reshape(-1, 3)

    xyz = quat[:, 1:]
    t = xyz.cross(vec, dim=-1) * 2.0
    return (vec + quat[:, 0:1] * t + xyz.cross(t, dim=-1)).view(shape)


def axis_angle_from_quat(quat: torch.Tensor, eps: float = 1e-6) -> torch.Tensor:
    """Convert quaternion (w, x, y, z) to axis-angle vector (rotation vector)."""
    quat = normalize(quat)
    mag = torch.linalg.norm(quat[..., 1:], dim=-1)
    half_angle = torch.atan2(mag, quat[..., 0])
    angle = 2.0 * half_angle
    sin_half = torch.sin(half_angle)
    small = sin_half.abs() < eps
    scale = torch.where(small, 2.0 * torch.ones_like(sin_half), angle / sin_half.clamp(min=eps))
    return quat[..., 1:] * scale.unsqueeze(-1)


def compute_pose_error(
    t01: torch.Tensor,
    q01: torch.Tensor,
    t02: torch.Tensor,
    q02: torch.Tensor,
    rot_error_type: Literal["quat", "axis_angle"] = "axis_angle",
):
    """Position and orientation error between a current pose (t01, q01) and a
    target pose (t02, q02)."""
    pos_error = t02 - t01
    q_error = quat_mul(q02, quat_inv(q01))

    if rot_error_type == "quat":
        return pos_error, q_error
    elif rot_error_type == "axis_angle":
        return pos_error, axis_angle_from_quat(q_error)
    else:
        raise ValueError(f"Unsupported rot_error_type: {rot_error_type}")


def apply_delta_pose(
    source_pos: torch.Tensor,
    source_rot: torch.Tensor,
    delta_pose: torch.Tensor,
    eps: float = 1e-6,
):
    """Apply a 6D delta pose (dx, dy, dz, drx, dry, drz) to a source pose."""
    new_pos = source_pos + delta_pose[:, 0:3]

    rot_vec = delta_pose[:, 3:6]
    angle = torch.linalg.norm(rot_vec, dim=-1, keepdim=True).clamp(min=eps)
    axis = rot_vec / angle
    half_angle = angle * 0.5
    delta_quat = torch.cat([torch.cos(half_angle), axis * torch.sin(half_angle)], dim=-1)

    new_rot = quat_mul(delta_quat, source_rot)
    return new_pos, normalize(new_rot)