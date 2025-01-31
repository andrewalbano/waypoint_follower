import numpy as np
from classes import Pose
from matrices import euler_from_transformation_matrix


def get_waypoint(mstack = np.eye(4), origin = Pose()): 
    origin_homogenous = np.hstack((origin.xyz(), 1))
    xyz = mstack @ origin_homogenous.T
    roll,pitch,yaw = euler_from_transformation_matrix(mstack)
    waypoint = Pose(xyz[0],xyz[1],xyz[2],roll, pitch,yaw)
    return waypoint
    
    
def add_global_waypoint(current_waypoints =None, new_waypoint = Pose()):
    if current_waypoints is None:
        current_waypoints = new_waypoint.pose()
    else:
        current_waypoints = np.vstack((current_waypoints,new_waypoint.pose()))
    return current_waypoints

def add_waypoints(current_waypoints=None, new_waypoints=None):
    if new_waypoints is None:
        raise ValueError("New waypoints must be provided.")
        
    if current_waypoints is None:
        current_waypoints = new_waypoints
    else:
        current_waypoints = np.vstack((current_waypoints, new_waypoints))
    
    return current_waypoints
    
def update_orientation(T = np.eye(4), t =np.eye(4)):
    return T @ t

def remove_adjacent_duplicates(array):
    # Compute a boolean mask where each row is different from the next row
    mask = np.any(np.diff(array, axis=0) != 0, axis=1)

    # Use np.concatenate to ensure the result includes the last row
    result = np.concatenate(([True], mask))

    # Apply the mask to get the result without adjacent duplicate rows
    filtered_array = array[result]
    return filtered_array

        