import numpy as np
from classes import Pose
from matrices import euler_from_transformation_matrix

def get_waypoint(mstack=np.eye(4), origin=Pose()):
    """
    Computes a new waypoint by applying a transformation matrix to a given origin Pose.

    Parameters:
        mstack (numpy.ndarray): A 4x4 transformation matrix. Defaults to the identity matrix.
        origin (Pose): The origin Pose object. Defaults to a Pose at the origin with no rotation.
    
    Returns:
        Pose: The resulting waypoint Pose after transformation.
    """
    origin_homogeneous = np.hstack((origin.xyz(), 1))
    xyz = mstack @ origin_homogeneous.T
    roll, pitch, yaw = euler_from_transformation_matrix(mstack)
    waypoint = Pose(xyz[0], xyz[1], xyz[2], roll, pitch, yaw)
    return waypoint


def add_global_waypoint(current_waypoints=None, new_waypoint=Pose()):
    """
    Adds a new global waypoint to the existing waypoints array.

    Parameters:
        current_waypoints (numpy.ndarray): An array of current waypoints. Defaults to None.
        new_waypoint (Pose): The new waypoint Pose to add. Defaults to a default Pose.
    
    Returns:
        numpy.ndarray: An updated array of waypoints including the new waypoint.
    """
    if current_waypoints is None:
        current_waypoints = new_waypoint.pose()
    else:
        current_waypoints = np.vstack((current_waypoints, new_waypoint.pose()))
    return current_waypoints


def add_waypoints(current_waypoints=None, new_waypoints=None):
    """
    Combines new waypoints with the existing array of waypoints.

    Parameters:
        current_waypoints (numpy.ndarray): An array of current waypoints. Defaults to None.
        new_waypoints (numpy.ndarray): An array of new waypoints to add. Must be provided.
    
    Raises:
        ValueError: If `new_waypoints` is None.
    
    Returns:
        numpy.ndarray: An updated array of waypoints including the new waypoints.
    """
    if new_waypoints is None:
        raise ValueError("New waypoints must be provided.")
        
    if current_waypoints is None:
        current_waypoints = new_waypoints
    else:
        current_waypoints = np.vstack((current_waypoints, new_waypoints))
    
    return current_waypoints


def update_orientation(T=np.eye(4), t=np.eye(4)):
    """
    Combines two transformation matrices into a single updated transformation.

    Parameters:
        T (numpy.ndarray): The current transformation matrix. Defaults to the identity matrix.
        t (numpy.ndarray): The incremental transformation matrix. Defaults to the identity matrix.
    
    Returns:
        numpy.ndarray: The resulting transformation matrix after combining T and t.
    """
    return T @ t


def remove_adjacent_duplicates(array):
    """
    Removes adjacent duplicate rows from an array.

    Parameters:
        array (numpy.ndarray): The input array to filter.
    
    Returns:
        numpy.ndarray: The filtered array without adjacent duplicate rows.
    """
    mask = np.any(np.diff(array, axis=0) != 0, axis=1)
    result = np.concatenate(([True], mask))
    filtered_array = array[result]
    return filtered_array