import numpy as np
from classes import *
from  quarternions import *

# TO DO: need to test this in 3D but it works when i only adjust Yaw
def generate_rotation_matrix(axis=[0, 0, 1], angle=0):
    """
    Generates a 3x3 rotation matrix from a given axis and angle.

    Parameters:
        axis (list or numpy.ndarray): The axis of rotation. Default is [0, 0, 1].
        angle (float): The angle of rotation in radians.
    
    Returns:
        numpy.ndarray: A 3x3 rotation matrix.
    """
    
    # Convert axis to numpy array if not already
    axis = np.array(axis)

    # Generate and normalize quaternion from axis-angle
    q = quaternion_from_axis_angle(axis, angle)
    q = quaternion_normalize(q)

    # Convert quaternion to rotation matrix
    rotation_matrix = quaternion_to_rotation_matrix(q)

    return rotation_matrix

# TO DO: add documentation on my matrix frames pre vs post multiplication and review 
def generate_translation_matrix(axis=[1, 0, 0], translation=1):
    """
    Generates a 4x4 translation matrix in homogeneous coordinates.

    Parameters:
        axis (list or numpy.ndarray): The direction of translation. Default is [1, 0, 0].
        translation (float): The magnitude of translation.
    
    Returns:
        numpy.ndarray: A 4x4 translation matrix in homogeneous coordinates.
    """
    
    # Convert axis to numpy array if not already
    axis = np.array(axis)

    # Normalize the axis vector
    unit_axis = axis / np.linalg.norm(axis)
    
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


def get_transformation_from_pose(pose = Pose()):    # Calculate individual rotation matrices
    
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(pose.roll), -np.sin(pose.roll)],
                    [0, np.sin(pose.roll), np.cos(pose.roll)]])
    
    R_y = np.array([[np.cos(pose.pitch), 0, np.sin(pose.pitch)],
                    [0, 1, 0],
                    [-np.sin(pose.pitch), 0, np.cos(pose.pitch)]])
    
    R_z = np.array([[np.cos(pose.yaw), -np.sin(pose.yaw), 0],
                    [np.sin(pose.yaw), np.cos(pose.yaw), 0],
                    [0, 0, 1]])

    # Combine rotations
    R = R_z@(R_y@R_x)
    
    # Create the transformation matrix
    T = np.array([[R[0, 0], R[0, 1], R[0, 2], pose.x],
                  [R[1, 0], R[1, 1], R[1, 2], pose.y],
                  [R[2, 0], R[2, 1], R[2, 2], pose.z],
                  [0, 0, 0, 1]])

    return T


def get_rotation_from_waypoint(waypoint):
    # used to get the axis frames for quiver
    
    roll = waypoint[3]
    pitch = waypoint[4]
    yaw = waypoint[5]
    
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    # Combine rotations
    R = R_z@(R_y@R_x)


    # Create the transformation matrix
    R = np.array([[R[0, 0], R[0, 1], R[0, 2]],
                [R[1, 0], R[1, 1], R[1, 2]],
                [R[2, 0], R[2, 1], R[2, 2]]])

    return R
    
# sample usage and calling of functions
def main(show_sample = False):
    if show_sample:
            
        # Define an axis and angle for rotation
        axis = [0, 1, 0]  # Rotate around y-axis
        angle = np.pi / 4  # Angle in radians (45 degrees)

        # Generate rotation matrix
        rotation_matrix = generate_rotation_matrix(axis, angle)
        print("Rotation Matrix:\n", rotation_matrix)

        # Define translation properties
        translation_axis = [1, 0, 0]  # Translate along x-axis
        translation_distance = 5  # Translation magnitude

        # Generate translation matrix
        translation_matrix = generate_translation_matrix(translation_axis, translation_distance)
        print("\nTranslation Matrix:\n", translation_matrix)

        # Combining rotation and translation into a single transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix = transformation_matrix @ translation_matrix
        print("\nCombined Transformation Matrix:\n", transformation_matrix)

        # Extract Euler angles from the transformation matrix
        roll, pitch, yaw = euler_from_transformation_matrix(transformation_matrix)
        print("\nEuler Angles from Transformation Matrix:")
        print("Roll:", np.degrees(roll))
        print("Pitch:", np.degrees(pitch))
        print("Yaw:", np.degrees(yaw))
        return 0
    else:
        return 0

if __name__ == '__main__':
    main(show_sample = False)
    # main(show_sample = True)
