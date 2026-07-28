# This is a forward kinematics solver for the arm
import numpy as np

def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,     ca,    d],
        [0,        0,      0,    1]
    ])

def forward_kinematics(q_i):
    # Standard (distal) DH parameters: a [m], alpha [rad], d [m], theta_offset [rad]
    # theta_i(q) = theta_offset[i] + q_i
    theta_1 = q_i[0] * np.pi / 180
    theta_2 = q_i[1] * np.pi / 180
    theta_3 = q_i[2] * np.pi / 180
    theta_4 = q_i[3] * np.pi / 180
    theta_5 = q_i[4] * np.pi / 180
    theta_6 = q_i[5] * np.pi / 180

    theta = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]
    dh_table = [
        # (a,        alpha,          d,        theta_offset)
        (0.0000,  -np.pi/2,      0.1096,              theta_1),
        (0.2693,   0.0,         -0.0280,   -np.pi/2 + theta_2),
        (0.0420,  -np.pi/2,      0.0266,              theta_3),
        (0.0001,   np.pi/2,      0.19788,             theta_4),
        (0.0011,  -np.pi/2,      0.0001,     -np.pi + theta_5),
        (0.0000,   0.0,         -0.0825,              theta_6),
    ]

    dh_array = np.array(dh_table)  # shape (6,4) -> columns: a, alpha, d, theta_offset

    # Get ^{i-1}T_i
    T = []
    for index, row in enumerate(dh_array):
        T.append(dh_transform(row[0], row[1], row[2], row[3]))

    # Get ^{0}T_i
    T_from_origin = [T[0]]
    T_prev = T[0]
    for index, t in enumerate(T):
        if index == 0:
            continue
        newT = T_prev @ t
        T_from_origin.append(newT)
        T_prev = newT
    return T_from_origin[5]


