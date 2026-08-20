import math
import numpy as np
from fkine import dh_transform

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def invKine(T06, tol=1e-4):
    joint_limits_rad = np.radians(np.array([
        [-135.0, 135.0],
        [ -90.0,  90.0],
        [ -90.0,  90.0],
        [-135.0, 135.0],
        [ -90.0,  90.0],
        [-135.0, 135.0]
    ]))

    dh_table = np.array([
        (0.0000,   np.pi/2,  0.1316,   0),
        (0.2700,   0.0,      0.0000,   np.pi/2),
        (0.041325, np.pi/2,  0.0000,   0),
        (0.0000,   np.pi/2,  0.204675, 0),
        (0.0000,  -np.pi/2,  0.0000,   0),
        (0.0000,   0.0,      0.205744, 0),
    ])

    R06 = T06[0:3, 0:3]
    o6  = T06[0:3, 3]
    z6  = T06[0:3, 2]
    d6  = dh_table[5, 2]

    pc = o6 - d6 * z6
    xc, yc, zc = pc[0], pc[1], pc[2]

    l2 = dh_table[1, 0]
    a3 = dh_table[2, 0]
    l3 = dh_table[3, 2]
    d1 = dh_table[0, 2]

    l3_eff = math.hypot(a3, l3)
    delta = math.atan2(l3, a3)
    H = zc - d1

    theta1_fwd = wrap_to_pi(math.atan2(-yc, xc))
    theta1_bwd = wrap_to_pi(theta1_fwd + np.pi)

    base_candidates = [
        (theta1_fwd, math.sqrt(xc**2 + yc**2)),
        (theta1_bwd, -math.sqrt(xc**2 + yc**2))
    ]

    cos_beta = np.clip((xc**2 + yc**2 + H**2 - l2**2 - l3_eff**2) / (2 * l2 * l3_eff), -1.0, 1.0)
    beta_base = math.acos(cos_beta)
    elbow_solutions = [beta_base, -beta_base]

    q_min = joint_limits_rad[:, 0]
    q_max = joint_limits_rad[:, 1]

    valid_solutions = []
    all_candidates = []

    for theta1, r_xy in base_candidates:
        for beta in elbow_solutions:
            theta3 = wrap_to_pi(beta - delta)

            A = l2 + l3_eff * math.cos(beta)
            B = l3_eff * math.sin(beta)
            theta2 = math.atan2(r_xy, H) - math.atan2(B, A)
            theta2 = wrap_to_pi(theta2)

            T01 = dh_transform(dh_table[0, 0], dh_table[0, 1], dh_table[0, 2], -theta1 + dh_table[0, 3])
            T12 = dh_transform(dh_table[1, 0], dh_table[1, 1], dh_table[1, 2], -theta2 + dh_table[1, 3])
            T23 = dh_transform(dh_table[2, 0], dh_table[2, 1], dh_table[2, 2], -theta3 + dh_table[2, 3])

            R03 = (T01 @ T12 @ T23)[0:3, 0:3]
            R36 = R03.T @ R06

            r13, r23 = R36[0, 2], R36[1, 2]
            r31, r32, r33 = R36[2, 0], R36[2, 1], R36[2, 2]

            theta5_a = math.atan2(math.sqrt(max(0.0, 1.0 - r33**2)), r33)
            theta4_a = math.atan2(-r23, -r13)
            theta6_a = math.atan2(-r32, r31)

            theta5_b = -theta5_a
            theta4_b = wrap_to_pi(theta4_a + np.pi)
            theta6_b = wrap_to_pi(theta6_a + np.pi)

            wrist_candidates = [
                (theta4_a, theta5_a, theta6_a),
                (theta4_b, theta5_b, theta6_b)
            ]

            for t4, t5, t6 in wrist_candidates:
                q_cand = np.array([theta1, theta2, theta3, wrap_to_pi(t4), wrap_to_pi(t5), wrap_to_pi(t6)])
                all_candidates.append(q_cand)

                if np.all(q_cand >= (q_min - tol)) and np.all(q_cand <= (q_max + tol)):
                    q_clean = np.clip(q_cand, q_min, q_max)
                    valid_solutions.append(q_clean)

    if valid_solutions:
        return valid_solutions[0]

    violations = [np.sum(np.maximum(0, q_min - q) + np.maximum(0, q - q_max)) for q in all_candidates]
    best_candidate = all_candidates[np.argmin(violations)]

    return np.clip(best_candidate, q_min, q_max)