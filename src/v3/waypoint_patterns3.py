import numpy as np
from classes3 import Pose
from matrices3 import *
    
# # Example parameters for two patterns
# # lawnmower pattern
# width1, height1, strip_width1, spacing1 = 30, 20, 5, 3
# center1, radius1, angle1, increments1 = np.array([0, 0]), 2, np.pi/2, 10 

#  probably dont need this 
def path_position_tolerance_check(pose, waypoint, tolerance=0.2):
    
    distance = np.sqrt((pose['x'] - waypoint[0]) ** 2 + (pose['y'] - waypoint[1]) ** 2)
    return distance < tolerance


def print_waypoints(waypoints):
    """
        Print each waypoint from the given list of waypoints where the elements of the list are defined as Pose objects
    """
    for pose in waypoints:
        print(pose)


def remove_adjacent_duplicates(array):
    # Compute a boolean mask where each row is different from the next row
    mask = np.any(np.diff(array, axis=0) != 0, axis=1)

    # Use np.concatenate to ensure the result includes the last row
    result = np.concatenate(([True], mask))

    # Apply the mask to get the result without adjacent duplicate rows
    filtered_array = array[result]
    return filtered_array

def chain_patterns(new_waypoints_local, waypoints_array, mstack, local_transform):
    # extract local frame 
    xyz = new_waypoints_local[:, :3]
    rpy = new_waypoints_local[:, 3:]
    # print(xyz)
    # transform waypoints to global frame
    new_waypoints_global= transform_waypoints(mstack, new_waypoints_local)
    # new_waypoints_global = np.hstack(xyz_global,)
  
    # add waypoints to global stack and remove duplicate adjacent waypoints
    waypoints_array = np.vstack((waypoints_array, new_waypoints_global))
    
    # remove duplicate adjacent waypoints
    waypoints_array= remove_adjacent_duplicates(waypoints_array)
    
    # update transform
    mstack = np.matmul(mstack,local_transform)
    
    return waypoints_array, mstack, new_waypoints_global
    

# DEFINED PATTERNS
    
def defined_square(size = 1):
    waypoints = []
    waypoints.append(Pose().get_list())
    waypoints.append(Pose(size,0,0,0,0,0).get_list())
    waypoints.append(Pose(size,0,0,0,0, np.pi/2).get_list())
    waypoints.append(Pose(size,size,0,0,0,np.pi/2).get_list())
    waypoints.append(Pose(size,size,0,0,0, np.pi).get_list())
    waypoints.append(Pose(0,size,0,0,0, np.pi).get_list())
    waypoints.append(Pose(0,size,0,0,0, 3*np.pi/2).get_list())
    waypoints.append(Pose(0,0,0,0,0,3*np.pi/2).get_list())

    waypoints = np.array(waypoints)

    return waypoints    

# FIXED HEADING PATTERNS RELATIVE TO THE INITIAL POSITION AND ORIENTATION (body frame)                 
def generate_forward_path(current_pose = Pose(0,0,0,0,0,0), distance=4, spacing=1):
    """
    Generates a series of waypoints that form a linear path along the x-axis(body frame of robot), starting from the
    origin of a coordinate frame and extending to a specified distance with a defined spacing between each waypoint. 
    If the final position does not exactly match the specified distance due to spacing, an additional waypoint
    is added at the exact distance to ensure coverage.

    Parameters:
        - distance (float): The total distance along the x-axis for which waypoints will be generated.
                      Defaults to 4 units.
        - spacing (float): The interval between consecutive waypoints along the x-axis. Defaults to 1 unit.

    Returns:
        - numpy.ndarray: A numpy array of waypoints, where each waypoint is represented as a list containing
                   [x, y, yaw]. The waypoints cover the path along the x-axis from the origin to the 
                   specified distance at the provided spacing intervals. The orientation (yaw) is constant 
                   at 0 for all waypoints, indicating no rotation from the starting frame.
    """
    # Start at the body frame origin
    x = current_pose.x
    next_x = x
    
    # Initialize an empty list to store waypoints
    waypoints = []
    
    # Initialize a waypoint as a pose 
    waypoint = current_pose
    waypoints.append(waypoint.get_list())

    # Generate waypoints until the specified distance is covered
    while next_x <= distance + x:
        # Update the waypoint with new values (keeping y, z, roll, pitch, yaw the same)
        waypoint.x = next_x
        # waypoint.update(next_x, current_pose.y, current_pose.z,
        #                 current_pose.roll, current_pose.pitch, current_pose.yaw)
        
        # Append it to the waypoints list as a list
        waypoints.append(waypoint.get_list()) 
        next_x += spacing
    
    # Ensure the last waypoint is within the tolerance of the desired distance
    if waypoint.x < distance + x:
        waypoint.x = distance
        # Update the waypoint for the final point if it did not reach it  
        # waypoint.update(distance, current_pose.y, current_pose.z,
        #                 current_pose.roll, current_pose.pitch, current_pose.yaw)
        waypoints.append(waypoint.get_list())
        
    # Convert to numpy array
    waypoints_array = np.array(waypoints)
    
    # Generate transformation matrix
    transformation = generate_transformation_matrix(translation=[distance, 0, 0])

    return waypoints_array, waypoint, transformation


def generate_rotation_waypoint(current_pose = Pose(0,0,0,0,0,0), axis = [0,0,1], angle = np.pi/2):
    # currentyl only handles yaw rotation but setting it up for any axis later 
    # initialize waypoints list 
    waypoints = []
    
    # adding current pose to the waypoint list
    waypoint = current_pose
    waypoints.append(current_pose.get_list())
    
    
    # applying transformation
    q = quaternion_from_axis_angle(axis, angle)
    q = quaternion_normalize(q)
    transformation = quaternion_to_rotation_matrix(q)
    
    # Note for later this works
    # rpy = euler_from_transformation_matrix(transformation)
    # print("yaw from matrix: ", rpy[2])
    
    waypoint.yaw += angle 
    
    waypoints.append(waypoint.get_list())
    
    waypoints = np.array(waypoints)
    
    return waypoints, waypoint, transformation
    
    
# def generate_rotation_waypoint(current_pose = Pose(0,0,0,0,0,0), rotation = np.pi/2, axis = [0,0,1], interval = False, rotation_interval = np.pi/4):
#     # Note assumes yaw axis only for now
    
#     #  note can update this to make it shorter by only updating along the dimension
#     # Start at the initial yaw
#     initial_yaw = current_pose.yaw
#     next_yaw = initial_yaw
    
#     # Initialize an empty list to store waypoints
#     waypoints = []
    
#     # initialize a waypoint as a pose and add it as a waypoint
#     waypoint = current_pose
    

#     if interval:
        
#         # Generate waypoints until the specified rotation angle is reached
#         while next_yaw <= initial_yaw + rotation:
#             # Update the waypoint with new yaw
#             waypoint.yaw = next_yaw
#             # waypoint.update(
#             #     current_pose.x,
#             #     current_pose.y,
#             #     current_pose.z,
#             #     current_pose.roll,
#             #     current_pose.pitch,
#             #     next_yaw
#             # )
#             # Append it to the waypoints list as a list
#             waypoints.append(waypoint.get_list())
            
#             next_yaw += rotation_interval
        
#         # Ensure the last waypoint is within the tolerance of the desired rotation
#         if  waypoint.yaw < initial_yaw + rotation:
            
#             # Update the waypoint for the final point if it did not reach it
#             waypoint.yaw = initial_yaw + rotation
#             # waypoint.update(
#             #     current_pose.x,
#             #     current_pose.y,
#             #     current_pose.z,
#             #     current_pose.roll,
#             #     current_pose.pitch,
#             #     initial_yaw + rotation
#             # )
#             waypoints.append(waypoint.get_list())
#     else:
        
#         # append starting point to the waypoints list as a list
#         waypoints.append(waypoint.get_list())
#         waypoint.yaw = initial_yaw + rotation
#         # Update the waypoint for the final point if it did not reach it and add it to the waypoint 
#         # waypoint.update(
#         #         current_pose.x,
#         #         current_pose.y,
#         #         current_pose.z,
#         #         current_pose.roll,
#         #         current_pose.pitch,
#         #         initial_yaw + rotation
#         #     )
#         waypoints.append(waypoint.get_list())  
        
    
        
#     # convert to numpy array
#     waypoints_array = np.array(waypoints)
    
#     transformation = generate_transformation_matrix(axis='yaw', angle=rotation)
    
#     return waypoints_array, waypoint, transformation
    
    

def generate_lawnmower_pattern_2(width, height, strip_width, spacing):
    """
    Generate a lawnmower pattern for a given width, height, strip width, and spacing.

    Parameters:
    - width: The width of the area to be covered (meters).
    - height: The height of the area to be covered (meters).
    - strip_width: The width of each mowing strip (meters).
    - spacing: The vertical spacing between each mow row (meters).

    Returns:
    - A 2D numpy array of shape (N, 2) representing the lawnmower pattern waypoints.
    """
    # Initialize an empty list to store waypoints
    waypoints = []

    # We will generate waypoints row by row
    y_position = 0  # Start from the top of the field (y = 0)
    
    while y_position < height:
        # Calculate how many strips we can fit horizontally in this row
        num_strips = int(np.ceil(width / strip_width))
        
        if int(y_position / spacing) % 2 == 0:
            # Left to right direction
            for i in range(num_strips):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                
                # TO DO: Need to do some thinking on how to assign Yaw
                yaw = 0
                waypoints.append(Pose(x_position, y_position, yaw))
                
        else:
            # Right to left direction
            for i in range(num_strips - 1, -1, -1):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                waypoints.append([x_position, y_position])
        
        # Move down by the spacing after completing a row
        y_position += spacing
        
    # Convert the list of waypoints into a numpy array
    waypoints_array = np.array(waypoints)

    return waypoints_array

def create_semicircle_2(center,radius,angle,increments):
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles)]) + center[:,None]
    return POSE.T

def orbit_mode_2(center,radius,angle,increments):
    # Modified version of create_semicircle, where the robot will always point towards the center of the arc
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles), ]) + center[:,None]
    return POSE.T



def generate_lawnmower_pattern(width, height, strip_width, spacing):
    """
    Generate a lawnmower pattern for a given width, height, strip width, and spacing.

    Parameters:
    - width: The width of the area to be covered (meters).
    - height: The height of the area to be covered (meters).
    - strip_width: The width of each mowing strip (meters).
    - spacing: The vertical spacing between each mow row (meters).

    Returns:
    - A 2D numpy array of shape (N, 2) representing the lawnmower pattern waypoints.
    """
    # Initialize an empty list to store waypoints
    waypoints = []

    # We will generate waypoints row by row
    y_position = 0  # Start from the top of the field (y = 0)
    
    while y_position < height:
        # Calculate how many strips we can fit horizontally in this row
        num_strips = int(np.ceil(width / strip_width))
        
        if int(y_position / spacing) % 2 == 0:
            # Left to right direction
            for i in range(num_strips):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                waypoints.append([x_position, y_position])
        else:
            # Right to left direction
            for i in range(num_strips - 1, -1, -1):
                x_position = i * strip_width
                # Ensure x_position doesn't exceed width
                x_position = min(x_position, width)
                waypoints.append([x_position, y_position])
        
        # Move down by the spacing after completing a row
        y_position += spacing
        
    # Convert the list of waypoints into a numpy array
    waypoints_array = np.array(waypoints)

    return waypoints_array

def create_semicircle(center,radius,angle,increments):
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles)]) + center[:,None]
    return POSE.T

def orbit_mode(center,radius,angle,increments):
    # Modified version of create_semicircle, where the robot will always point towards the center of the arc
    angles = np.linspace(-angle/2,angle/2,increments)
    # Only care about X,Y, and yaw return values
    POSE = np.array([radius*np.cos(angles), radius*np.sin(angles), ]) + center[:,None]
    return POSE.T

# def main():
#         # waypoints = generate_forward_path()
#         # # print_waypoints(waypoints)
#         # waypoint = Pose()
#         # print(waypoint.xyz)
        
# if __name__ == "__main__":
#     main()