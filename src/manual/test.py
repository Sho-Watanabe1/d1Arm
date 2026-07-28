# This is a forward kinematics solver for the arm
import numpy as np
from fkine import forward_kinematics

import socket, struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 9999))

while True:
    data, _ = sock.recvfrom(1024)
    servo_angles = struct.unpack('7f', data)   # servo0..servo6
    q = servo_angles[:6]                        # arm joints -> forward_kinematics
    gripper = servo_angles[6]
    T = forward_kinematics(q)
    x, y, z = T[0][3], T[1][3], T[2][3]
    print(f"x={x*1000:+7.2f}  y={y*1000:+7.2f}  z={z*1000:+7.2f}  (mm)")
