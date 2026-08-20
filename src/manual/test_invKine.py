import json
import numpy as np
from invKine import invKine
import socket

def build_arm_command_json(q_rad, seq=4, mode=1):
    """
    Converts 6-DOF joint angles (radians) into Unitree's ArmString JSON payload.
    """
    # Convert radians from invKine to degrees
    q_deg = np.degrees(q_rad)
    
    payload = {
        "seq": seq,
        "address": 1,
        "funcode": 2,
        "data": {
            "mode": mode,
            "angle0": round(float(q_deg[0]), 2),
            "angle1": round(float(q_deg[1]), 2),
            "angle2": round(float(q_deg[2]), 2),
            "angle3": round(float(-q_deg[3]), 2),
            "angle4": round(float(q_deg[4]), 2),
            "angle5": round(float(q_deg[5]), 2),
            "angle6": 0,
        }
    }
    return json.dumps(payload)

# Example Usage:
# x= +73.79  y=+123.18  z=+260.87  (mm)   roll=-175.69  pitch= -79.24  yaw= +40.60  (deg)
x = 0.25
y = 0.05
z = 0.4


T06 = np.array([
    [ 0, 0, 1, x],   # x = 300 mm forward
    [ 0, 1, 0, y],   # y = 0 mm centered
    [ 1, 0, 0, z],   # z = 300 mm high
    [ 0, 0, 0,  1.0]
])


q_rad = invKine(T06)
json_str = build_arm_command_json(q_rad)
print(q_rad)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
sock.sendto(json_str.encode('utf-8'), ("127.0.0.1", 5005))