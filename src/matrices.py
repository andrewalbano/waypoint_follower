import numpy as np

def rotation_matrix(axis, angle):
    """
    Generate a 3x3 rotation matrix for rotating about the specified axis by the specified angle.

    Parameters:
    axis (str): The axis to rotate about ('x', 'y', or 'z').
    angle_degrees (float): The rotation angle in radians.

    Returns:
    numpy.ndarray: A 3x3 rotation matrix.
    """
    
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)

    if axis == 'roll':
        return np.array([[1, 0, 0],
                         [0, cos_angle, -sin_angle],
                         [0, sin_angle, cos_angle]])
    elif axis == 'pitch':
        return np.array([[cos_angle, 0, sin_angle],
                         [0, 1, 0],
                         [-sin_angle, 0, cos_angle]])
    elif axis == 'yaw':
        return np.array([[cos_angle, -sin_angle, 0],
                         [sin_angle, cos_angle, 0],
                         [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Choose from 'roll', 'pitch', or 'yaw'.")

def transformation_matrix(axis=None, angle=np.pi/2, translation=[0,0,0]):
    """
    Generate a 4x4 transformation matrix for rotating about the specified axis
    and translating by the specified vector.

    Parameters:
    axis (str): The axis to rotate about ('x', 'y', or 'z').
    angle_degrees (float): The rotation angle in radians.
    translation (list or tuple): A 3-element list or tuple representing the translation vector [tx, ty, tz].

    Returns:
    numpy.ndarray: A 4x4 transformation matrix.
    """

    # Create the 3x3 rotation matrix
    if axis != None:
        # angle_radians = np.radians(angle)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        if axis == 'roll':
            rot_matrix = np.array([[1, 0, 0],
                                [0, cos_angle, -sin_angle],
                                [0, sin_angle, cos_angle]])
        elif axis == 'pitch':
            rot_matrix = np.array([[cos_angle, 0, sin_angle],
                                [0, 1, 0],
                                [-sin_angle, 0, cos_angle]])
        elif axis == 'yaw':
            rot_matrix = np.array([[cos_angle, -sin_angle, 0],
                                [sin_angle, cos_angle, 0],
                                [0, 0, 1]])
        else:
            raise ValueError("Invalid axis. Choose from 'roll', 'pitch', or 'yaw'.")
    else:
        rot_matrix = np.eye(3)
    
 
    # Create the 4x4 transformation matrix using the translation 
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rot_matrix
    transform_matrix[:3, 3] = translation


    return transform_matrix
