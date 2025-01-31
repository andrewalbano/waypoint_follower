import math

import numpy as np

def quaternion_from_axis_angle(axis, angle):
    # returns quaternion q as a NumPy array with [a, b, c, d]
    q = np.zeros(4)
    q[0] = math.cos(angle / 2)
    q[1] = axis[0] * math.sin(angle / 2)
    q[2] = axis[1] * math.sin(angle / 2)
    q[3] = axis[2] * math.sin(angle / 2)
    return q

def quaternion_normalize(q1):
    # returns normalized quaternion q as a NumPy array
    norm = np.linalg.norm(q1)
    if norm == 0:
        return q1  # Avoid division by zero
    q_normalized = q1 / norm
    return q_normalized

def quaternion_multiply(q1, q2):
    # returns quaternion q as a NumPy array
    q = np.zeros(4)
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q

def quaternion_to_rotation_matrix(q):
    # returns 4x4 homogeneous rotation matrix as a NumPy array
    rotation = np.zeros((4, 4))
    rotation[0][0] = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2
    rotation[0][1] = 2 * (q[1] * q[2] - q[0] * q[3])
    rotation[0][2] = 2 * (q[0] * q[2] + q[1] * q[3])

    rotation[1][0] = 2 * (q[1] * q[2] + q[0] * q[3])
    rotation[1][1] = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2
    rotation[1][2] = 2 * (q[2] * q[3] - q[0] * q[1])

    rotation[2][0] = 2 * (q[1] * q[3] - q[0] * q[2])
    rotation[2][1] = 2 * (q[0] * q[1] + q[2] * q[3])
    rotation[2][2] = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2

    rotation[3][3] = 1  # To make it a homogeneous matrix

    return rotation

# # Example usage
# axis = [0, 0, 1]  # Rotation around the z-axis
# axis = np.array(axis)
# angle = math.pi / 2  # 90 degrees
# q = quaternion_from_axis_angle(axis, angle)
# q_normalized = quaternion_normalize(q)
# rotation_matrix = quaternion_to_rotation_matrix(q_normalized)

# print("Quaternion:", q)
# print("Normalized Quaternion:", q_normalized)
# print("Rotation Matrix:")
# print(rotation_matrix)

