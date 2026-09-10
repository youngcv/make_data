import math

import numpy as np


def axis_angle_to_matrix(axis_xyz, angle):
    axis = np.array(axis_xyz, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm <= 1e-12:
        raise ValueError("joint axis cannot be zero")

    x, y, z = axis / axis_norm
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    one_minus_cos = 1.0 - cos_theta
    return np.array(
        [
            [
                cos_theta + x * x * one_minus_cos,
                x * y * one_minus_cos - z * sin_theta,
                x * z * one_minus_cos + y * sin_theta,
            ],
            [
                y * x * one_minus_cos + z * sin_theta,
                cos_theta + y * y * one_minus_cos,
                y * z * one_minus_cos - x * sin_theta,
            ],
            [
                z * x * one_minus_cos - y * sin_theta,
                z * y * one_minus_cos + x * sin_theta,
                cos_theta + z * z * one_minus_cos,
            ],
        ]
    )


def rpy_to_matrix(rpy_xyz):
    roll, pitch, yaw = rpy_xyz
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)

    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rot_z @ rot_y @ rot_x


def resolve_joint_value(value, lower, upper):
    if not isinstance(value, str):
        return float(value)

    modes = {
        "lower": lower,
        "upper": upper,
        "mid": 0.5 * (lower + upper),
        "zero": 0.0,
    }
    if value not in modes:
        raise ValueError(f"Unknown joint-value mode: {value}")
    return modes[value]
