import numpy as np
from  quarternions import *


def make_homogenous(vector):
    return np.hstack((vector,np.ones(1)))

    
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


def generate_rotation_matrix(axis, angle):
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

    if axis == 'x':
        return np.array([[1, 0, 0],
                         [0, cos_angle, -sin_angle],
                         [0, sin_angle, cos_angle]])
    elif axis == 'x':
        return np.array([[cos_angle, 0, sin_angle],
                         [0, 1, 0],
                         [-sin_angle, 0, cos_angle]])
    elif axis == 'z':
        return np.array([[cos_angle, -sin_angle, 0],
                         [sin_angle, cos_angle, 0],
                         [0, 0, 1]])
    else:
        raise ValueError("Invalid axis. Choose from 'roll', 'pitch', or 'yaw'.")

def generate_translation_matrix(translation=[0,0,0]):
    transformation = np.eye(4)
    transformation[:3, 3] = translation
    return transformation
    

def generate_transformation_matrix(axis=None, angle=np.pi/2, translation=[0,0,0]):
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


def transform_waypoints(transform_matrix, waypoints):
    """
    Apply a 4x4 transformation matrix to an array of 3D waypoints.

    Parameters:
    transform_matrix (numpy.ndarray): A 4x4 transformation matrix.
    waypoints (numpy.ndarray): A Nx3 array of waypoints, where N is the number of waypoints.

    Returns:
    numpy.ndarray: A Nx3 array of transformed waypoints.
    """
    # Convert waypoints to homogeneous coordinates by adding a column of ones
    num_waypoints = waypoints.shape[0]
    waypoints_xyz = waypoints[:,:3] #added this
    waypoints_rpy = waypoints[:,3:] #added this
    homogeneous_waypoints = np.hstack((waypoints_xyz, np.ones((num_waypoints, 1))))
    
    # R = np.linalg.inv(transform_matrix[:3, :3])
    # waypoints_rpy = R @ waypoints_rpy.T
    
    # waypoints_rpy = waypoints_rpy.T
    
    rpy_global = euler_from_transformation_matrix(transform_matrix)
    
    # Repeat the single RPY row to match the number of position rows
    waypoints_rpy = np.tile(rpy_global, (num_waypoints, 1))
    
    # print(homogeneous_waypoints)

    # Apply the transformation matrix
    transformed_homogeneous_waypoints = transform_matrix @ homogeneous_waypoints.T

    # Convert back from homogeneous coordinates to 3D
    transformed_waypoints = transformed_homogeneous_waypoints[:3].T
    
    # Convert from 3d to pose
    transformed_waypoints = np.hstack((transformed_waypoints,waypoints_rpy))

    return transformed_waypoints

def apply_transform(rotation_matrix, waypoints):
    
    # Convert waypoints to homogeneous coordinates by adding a column of ones
    num_waypoints = waypoints.shape[0]
    waypoints_xyz = waypoints[:,:3] #added this
    waypoints_rpy = waypoints[:,3:] #added this
    homogeneous_waypoints = np.hstack((waypoints_xyz, np.ones((num_waypoints, 1))))
    
    # R = np.linalg.inv(transform_matrix[:3, :3])
    # waypoints_rpy = R @ waypoints_rpy.T
    
    # waypoints_rpy = waypoints_rpy.T
    
    # rpy_global = euler_from_transformation_matrix(transform_matrix)
    
    # Repeat the single RPY row to match the number of position rows
    # waypoints_rpy = np.tile(rpy_global, (num_waypoints, 1))
    
    # print(homogeneous_waypoints)

    # Apply the transformation matrix
    transformed_homogeneous_waypoints = rotation_matrix @ homogeneous_waypoints.T

    # Convert back from homogeneous coordinates to 3D
    transformed_waypoints = transformed_homogeneous_waypoints[:3].T
    
    # Convert from 3d to pose
    transformed_waypoints = np.hstack((transformed_waypoints,waypoints_rpy))

    return transformed_waypoints


# need to check this 
def euler_from_transformation_matrix(T):
    """Extract roll, pitch, yaw from a 4x4 transformation matrix."""

    # The rotation matrix is the top-left 3x3 part of the transformation matrix
    R = T[:3, :3]

    # Check for gimbal lock
    if R[2, 0] != 1 and R[2, 0] != -1:
        # Standard extraction of angles
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock; pitch is ±90 degrees
        yaw = 0  # Can set to 0 when ambiguous
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = yaw + np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = -yaw + np.arctan2(-R[0, 1], -R[0, 2])

    return np.array([roll, pitch, yaw])


def normalize_vector(v):
    # Convert the input list into a NumPy array if it isn't already
    v = np.array(v)
    # Calculate the magnitude (norm) of the vector
    magnitude = np.linalg.norm(v)
    # Divide each component by the magnitude
    if magnitude == 0:
        return v  # Return the vector as is if magnitude is 0 to avoid division by zero
    v_normalized = v / magnitude
    return v_normalized

# def body_to_global(rotation_matrix):
#     orientation_martrix = rotation_matrix.T
#     x_axis = 
    