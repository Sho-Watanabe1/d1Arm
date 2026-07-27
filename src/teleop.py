#!/usr/bin/env python3
"""Safe teleop for D1 arm."""

import sys
import time
import math
import select
import termios
import tty

import torch

from d1_sdk.d1_client import D1Arm
from d1_ik_controller import D1CartesianController
from backend import PyTorchKinematicsBackend


POS_STEP = 0.005      # very small: 5mm
ROT_STEP = 0.03       # ~2 degrees
SMOOTHING = 0.2       # more smoothing = slower but stable

def multiply_quaternions(q1, q2):
    """
    Multiplies two quaternions q1 * q2.
    Assumes [w, x, y, z] format!
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ]

def apply_delta_rotation(current_quat, axis, angle_rad):
    """
    Creates a small delta rotation and multiplies it by the current orientation.
    """
    half_angle = angle_rad / 2.0
    s = math.sin(half_angle)
    c = math.cos(half_angle)

    # Build the delta quaternion [w, x, y, z]
    if axis == 'x':
        delta_q = [c, s, 0.0, 0.0]
    elif axis == 'y':
        delta_q = [c, 0.0, s, 0.0]
    elif axis == 'z':
        delta_q = [c, 0.0, 0.0, s]

    # Multiply current_quat * delta_q for LOCAL axis rotation
    return multiply_quaternions(current_quat, delta_q)

def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def setup_term():
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    return old


def restore_term(old):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def main():
    old = setup_term()
    try:
        print("=" * 60)
        print("D1 ARM TELEOP (SAFE MODE)")
        print("=" * 60)
        print("Controls:")
        print("  w/s = +X/-X    a/d = +Y/-Y    q/e = +Z/-Z")
        print("  i/k = pitch    j/l = yaw      u/o = roll")
        print("  r = reset to current pose    space = suspend")
        print("  Ctrl+C = quit")
        print("=" * 60)

        client = D1Arm(network_interface="eth0")
        if hasattr(client, 'set_motor_power'):
            client.set_motor_power(True)
            time.sleep(0.5)

        backend = PyTorchKinematicsBackend("d1.urdf")
        # print("URDF joint order:", backend.kin.joint_names())   # the joint order is correct
        controller = D1CartesianController(
            arm_client=client,
            backend=backend,
            num_envs=1,
            device="cpu",
            rate_hz=10.0,
            smoothing=SMOOTHING,
        )

        # CRITICAL: Start from where the arm ACTUALLY is
        pos, rot = controller.resume_from_arm()
        print(f"\nResumed from arm pose:")
        print(f"  pos={[round(x, 3) for x in pos]}")
        print(f"  rot={[round(x, 3) for x in rot]}")

        target_pos = list(pos)
        target_rot = list(rot)
        suspended = False

        print("\nReady. Press keys to move.\n")

        while True:
            dt = 0.01
            key = get_key()

            if key:
                if key == 'w': target_pos[0] += POS_STEP; print(f"  X+ = {target_pos[0]:.3f}")
                elif key == 's': target_pos[0] -= POS_STEP; print(f"  X- = {target_pos[0]:.3f}")
                elif key == 'a': target_pos[1] += POS_STEP; print(f"  Y+ = {target_pos[1]:.3f}")
                elif key == 'd': target_pos[1] -= POS_STEP; print(f"  Y- = {target_pos[1]:.3f}")
                elif key == 'q': target_pos[2] += POS_STEP; print(f"  Z+ = {target_pos[2]:.3f}")
                elif key == 'e': target_pos[2] -= POS_STEP; print(f"  Z- = {target_pos[2]:.3f}")
                elif key == 'r':
                    pos, rot = controller.resume_from_arm()
                    target_pos, target_rot = list(pos), list(rot)
                    print(f"  Reset to current pose")
                elif key == ' ':
                    if suspended:
                        pos, rot = controller.resume_from_arm()
                        target_pos, target_rot = list(pos), list(rot)
                        suspended = False
                        print("  Resumed")
                    else:
                        controller.suspend()
                        suspended = True
                        print("  Suspended")
                elif key == 'u': 
                    target_rot = apply_delta_rotation(target_rot, 'y', ROT_STEP)
                    print(f"  Roll+ (Local Y)")
                elif key == 'o': 
                    target_rot = apply_delta_rotation(target_rot, 'y', -ROT_STEP)
                    print(f"  Roll- (Local Y)")
                elif key == 'i': 
                    target_rot = apply_delta_rotation(target_rot, 'x', ROT_STEP)
                    print(f"  Pitch+ (Local X)")
                elif key == 'k': 
                    target_rot = apply_delta_rotation(target_rot, 'x', -ROT_STEP)
                    print(f"  Pitch- (Local X)")
                elif key == 'j': 
                    target_rot = apply_delta_rotation(target_rot, 'z', ROT_STEP)
                    print(f"  Yaw+ (Local Z)")
                elif key == 'l': 
                    target_rot = apply_delta_rotation(target_rot, 'z', -ROT_STEP)
                    print(f"  Yaw- (Local Z)")
                elif key == '\x03':
                    raise KeyboardInterrupt

            if not suspended:
                sent = controller.update(target_pos, target_rot, dt)
                if sent:
                    client.poll()
                    angles = client.get_joint_angles()
                    angles_rad = torch.tensor([[a * math.pi / 180.0 for a in angles]], dtype=torch.float32)
                    ee_pos, ee_quat = backend.kin.forward(angles_rad)
                    print(f"    [Sent] angles={[round(a, 1) for a in angles]}  "
                        f"ee_pos={[round(x,3) for x in ee_pos[0].tolist()]}  "
                        f"ee_quat={[round(x,3) for x in ee_quat[0].tolist()]}")

            time.sleep(dt)

    except KeyboardInterrupt:
        print("\n\nStopping...")
        controller.suspend()
    finally:
        restore_term(old)
        print("Done.")


if __name__ == "__main__":
    main()