import numpy as np

def quaternion_from_axis_angle(axis, angle):
    """
    Converts an axis-angle representation to a quaternion.

    Parameters:
        axis (list or numpy.ndarray): The axis of rotation.
        angle (float): The angle of rotation in radians.
    
    Returns:
        numpy.ndarray: A quaternion (q0, q1, q2, q3) as a 4-element array.
    """
    axis = np.asarray(axis)
    q = np.zeros(4)
    q[0] = np.cos(angle / 2)
    q[1] = axis[0] * np.sin(angle / 2)
    q[2] = axis[1] * np.sin(angle / 2)
    q[3] = axis[2] * np.sin(angle / 2)
    return q

def quaternion_normalize(q1):
    """
    Normalizes a quaternion to unit length.

    Parameters:
        q1 (numpy.ndarray): The quaternion to normalize.
    
    Returns:
        numpy.ndarray: The normalized quaternion.
    """
    norm = np.linalg.norm(q1)
    if norm == 0:
        return q1  # Avoid division by zero
    q_normalized = q1 / norm
    return q_normalized

def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.

    Parameters:
        q1 (numpy.ndarray): The first quaternion.
        q2 (numpy.ndarray): The second quaternion.
    
    Returns:
        numpy.ndarray: The resulting quaternion from the multiplication.
    """
    q = np.zeros(4)
    q[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]
    q[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]
    q[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]
    q[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]
    return q

def quaternion_to_rotation_matrix(q):
    """
    Converts a quaternion into a 4x4 homogeneous rotation matrix.

    Parameters:
        q (numpy.ndarray): The quaternion to convert.
    
    Returns:
        numpy.ndarray: A 4x4 rotation matrix.
    """
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

# Example usage of the quaternion functions
def main(show_sample = False):
    if show_sample:
        axis = [0, 0, 1]  # Rotation around the z-axis
        angle = np.pi / 2  # 90 degrees in radians

        # Convert axis-angle to quaternion
        q = quaternion_from_axis_angle(axis, angle)
        q_normalized = quaternion_normalize(q)

        # Convert quaternion to rotation matrix
        rotation_matrix = quaternion_to_rotation_matrix(q_normalized)

        print("Quaternion:", q)
        print("Normalized Quaternion:", q_normalized)
        print("Rotation Matrix:")
        print(rotation_matrix)
        return 0
    else: 
        return 0

if __name__ == '__main__':
    # main(show_sample=True)
    main(show_sample=False)