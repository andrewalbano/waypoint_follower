import numpy as np
from classes import *
from  quarternions import *

# TO DO: need to test this in 3D but it works when i only adjust Yaw, see note on body frame, also dont appear to need the mstack
def generate_rotation_matrix(axis=[0, 0, 1], angle=0, mstack=np.eye(4)):
    """
    Generates a 3x3 rotation matrix from a given axis and angle.

    Parameters:
        axis (list or numpy.ndarray): The axis of rotation. Default is [0, 0, 1].
        angle (float): The angle of rotation in radians.
        mstack (numpy.ndarray): The reference frame stack. Defaults to the identity matrix.
        body_frame (bool): If True, interprets the axis in the body's frame.

    Returns:
        numpy.ndarray: A 3x3 rotation matrix.
    """
    
    # Convert axis to numpy array if not already
    axis = np.array(axis)

    # # Get global axis from local frame if body_frame is True... this is where i need the double check 
    # if body_frame:
    #     axis = mstack[:3, :3] @ axis  # Rotation part of the transformation (ignores translation)

    # Generate and normalize quaternion from axis-angle
    q = quaternion_from_axis_angle(axis, angle)
    q = quaternion_normalize(q)

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(q)

    return rotation_matrix

# TO DO, review the necessity of the body_frame transform, but currentlty works when true for my purposes
# TO DO: add documentation on my matrix frames pre vs post multiplication and review 
def generate_translation_matrix(axis=[1, 0, 0], translation=1, mstack=np.eye(4)):
    """
    Generates a 4x4 translation matrix in homogeneous coordinates.

    Parameters:
        axis (list or numpy.ndarray): The direction of translation. Default is [1, 0, 0].
        translation (float): The magnitude of translation.
        mstack (numpy.ndarray): The reference frame stack. Defaults to the identity matrix.
        body_frame (bool): If True, interprets the axis in the body's frame.

    Returns:
        numpy.ndarray: A 4x4 translation matrix in homogeneous coordinates.
    """
    
    # Convert axis to numpy array if not already
    axis = np.array(axis)

    # Normalize the axis vector
    unit_axis = axis / np.linalg.norm(axis)

    # # Get global axis from local frame if body_frame is false: review the necessity of this, but currentlty works when true for my purposes
    # if not body_frame:
    #     unit_axis = mstack[:3, :3] @ unit_axis

    # Create a 4x4 identity matrix and add translation components
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, 3] += translation * unit_axis

    return transformation_matrix

    
def euler_from_transformation_matrix(T):
    """
    Extracts roll, pitch, yaw from a 4x4 transformation matrix using ZYX Euler angles.

    Parameters:
        T (numpy.ndarray): A 4x4 transformation matrix.

    Returns:
        tuple: A tuple containing roll, pitch, and yaw angles in radians.
    """

    # Extract the rotation matrix
    R = T[:3, :3]

    # Check for a pure rotation of zero (identity matrix)
    if np.allclose(R, np.eye(3)):
        return 0.0, 0.0, 0.0

    # Standard angle extraction avoiding gimbal lock
    if R[2, 0] != 1 and R[2, 0] != -1:
        pitch = -np.arcsin(R[2, 0])
        roll = np.arctan2(R[2, 1] / np.cos(pitch), R[2, 2] / np.cos(pitch))
        yaw = np.arctan2(R[1, 0] / np.cos(pitch), R[0, 0] / np.cos(pitch))
    else:
        # Gimbal lock case - set yaw to 0 (or any constant) if ambiguous
        yaw = 0.0
        if R[2, 0] == -1:
            pitch = np.pi / 2
            roll = yaw + np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = -yaw + np.arctan2(-R[0, 1], -R[0, 2])

    return roll, pitch, yaw