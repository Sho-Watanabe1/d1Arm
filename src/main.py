#!/usr/bin/env python3
"""Run Cartesian IK control on the real D1 arm."""

import time

from d1_sdk.d1_client import D1Arm
from d1_ik_controller import D1CartesianController
from backend import PyTorchKinematicsBackend


def main():
    print("Connecting to D1 arm on eth0...", flush=True)
    client = D1Arm(network_interface="eth0")
    
    # Enable motor power if available
    if hasattr(client, 'set_motor_power'):
        print("Enabling motor power...", flush=True)
        client.set_motor_power(True)
        time.sleep(0.5)

    print("Loading URDF...", flush=True)
    backend = PyTorchKinematicsBackend(
        urdf_path="d1.urdf",
        end_effector_link="Link6",
        device="cpu",
    )
    print(f"URDF joints: {backend.kin.joint_names()}", flush=True)

    print("Creating controller...", flush=True)
    controller = D1CartesianController(
        arm_client=client,
        backend=backend,
        num_envs=1,
        device="cpu",
        rate_hz=10.0,
        smoothing=0.05,
    )

    print("Resetting controller...", flush=True)
    controller.reset(
        rel_pos=[0.3, 0.0, 0.2],
        rel_rot=[1.0, 0.0, 0.0, 0.0],
    )

    # Test: can we read live angles?
    print("\n--- Live angle test (3 seconds) ---", flush=True)
    for i in range(30):
        client.poll()
        angles = client.get_joint_angles()
        print(f"  {i*0.1:.1f}s: {angles}", flush=True)
        time.sleep(0.1)

    # Test: direct command
    print("\n--- Direct motor test ---", flush=True)
    print("Sending [10, 0, 0, 0, 0, 0]...", flush=True)
    client.set_all_joint_angles([10.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1.0)
    client.poll()
    print(f"After command: {client.get_joint_angles()}", flush=True)

    print("Returning to zero...", flush=True)
    client.set_all_joint_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(1.0)
    client.poll()
    print(f"After zero: {client.get_joint_angles()}", flush=True)

    # Now run IK loop
    target_pos = [0.3, 0.0, 0.2]
    target_rot = [1.0, 0.0, 0.0, 0.0]

    print("\n--- Starting IK loop. Ctrl+C to stop ---", flush=True)
    step = 0
    try:
        while True:
            dt = 0.01
            sent = controller.update(target_pos, target_rot, dt)
            step += 1

            if sent:
                client.poll()
                angles = client.get_joint_angles()
                print(f"[Step {step}] Command sent | Readback: {angles}", flush=True)

            time.sleep(dt)

    except KeyboardInterrupt:
        controller.suspend()
        print("\nStopped.", flush=True)


if __name__ == "__main__":
    main()