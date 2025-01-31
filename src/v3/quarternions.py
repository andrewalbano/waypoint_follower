import math

import numpy as np


def quaternion_from_axis_angle(axis, angle):
    # returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    q = {}
    q['a'] = math.cos(angle / 2)
    q['b'] = axis[0] * math.sin(angle / 2)
    q['c'] = axis[1] * math.sin(angle / 2)
    q['d'] = axis[2] * math.sin(angle / 2)
    return q

def quaternion_normalize(q1):
    # returns quaternion q as dict, with q['a'] as real number,
    # q['b'] as i component, q['c'] as j component, q['d'] as k component
    q = {}
    norm = math.sqrt(q1['a']**2 + q1['b']**2 + q1['c']**2 + q1['d']**2)

    q['a'] = q1['a'] / norm
    q['b'] = q1['b'] / norm
    q['c'] = q1['c'] / norm
    q['d'] = q1['d'] / norm
    return q

def quaternion_multiply(q1, q2):
    # returns quaternion q as dict, with q['a'] as real number,
    # q['b'] as i component, q['c'] as j component, q['d'] as k component
    q = {}
    q['a'] = q1['a'] * q2['a'] - q1['b'] * q2['b'] - q1['c'] * q2['c'] - q1['d'] * q2['d']
    q['b'] = q1['a'] * q2['b'] + q1['b'] * q2['a'] + q1['c'] * q2['d'] - q1['d'] * q2['c']
    q['c'] = q1['a'] * q2['c'] - q1['b'] * q2['d'] + q1['c'] * q2['a'] + q1['d'] * q2['b']
    q['d'] = q1['a'] * q2['d'] + q1['b'] * q2['c'] - q1['c'] * q2['b'] + q1['d'] * q2['a']
    return q

def quaternion_to_rotation_matrix(q):
    # returns 4-by-4 homogenoous rotation matrix
    rotation = [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 1]]
    rotation[0][0] = q['a']**2 + q['b']**2 - q['c']**2 - q['d']**2
    rotation[0][1] = 2 * (q['b']*q['c'] - q['a']*q['d'])
    rotation[0][2] = 2 * (q['a']*q['c'] + q['b']*q['d'])

    rotation[1][0] = 2 * (q['b']*q['c'] + q['a']*q['d'])
    rotation[1][1] = q['a']**2 - q['b']**2 + q['c']**2 - q['d']**2
    rotation[1][2] = 2 * (q['c']*q['d'] - q['a']*q['b'])

    rotation[2][0] = 2 * (q['b']*q['d'] - q['a']*q['c'])
    rotation[2][1] = 2 * (q['a']*q['b'] + q['c']*q['d'])
    rotation[2][2] = q['a']**2 - q['b']**2 - q['c']**2 + q['d']**2
    
    rotation = np.array(rotation)

    return rotation


# sample q input for quarternion to rotation
q2 = {
    'a': np.sqrt(0.5),  # cos(π/4)
    'b': 0,
    'c': 0,
    'd': np.sqrt(0.5)   # sin(π/4)
}

# print(quaternion_to_rotation_matrix(q2))
