# This is a forward kinematics solver for the arm
import numpy as np
from fkine import forward_kinematics
import socket, struct
from scipy.spatial.transform import Rotation as R_


# Gets output from joint_angle_sender.cpp => run in build folder: ./joint_angle_sender
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 9999))

while True:
    data, _ = sock.recvfrom(1024)
    servo_angles = struct.unpack('7f', data)   # servo0..servo6
    q = servo_angles[:6]                        # arm joints -> forward_kinematics
    gripper = servo_angles[6]
    T = forward_kinematics(q)
    x, y, z = T[0][3], T[1][3], T[2][3]

    R = T[:3, :3]  # rotation submatrix

    # ZYX (yaw-pitch-roll) Euler angles from rotation matrix
    sy = np.hypot(R[0][0], R[1][0])
    singular = sy < 1e-6

    r = R_.from_matrix(T[:3, :3])
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)
    axis_angle = r.as_rotvec(degrees=True)   # no gimbal lock, good for debugging

    print(f"x={x*1000:+7.2f}  y={y*1000:+7.2f}  z={z*1000:+7.2f}  (mm)   "
        f"roll={roll:+7.2f}  pitch={pitch:+7.2f}  yaw={yaw:+7.2f}  (deg)")

    if abs(abs(pitch) - 90) < 5:
        print("  (near gimbal lock — roll/yaw individually unreliable here)")
