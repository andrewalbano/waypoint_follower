import numpy as np
from classes import Pose
from matrices import *
from waypoint import *
    
    
def generate_rotation(initial_pose = Pose(), mstack= np.eye(4), angle = np.pi/2, axis = [0,0,1]):

    # GENERATE ROTATION USING ZAXIS (body frame) AND 90 DEGREE ROTATION and return the transform 
    transform = generate_rotation_matrix(axis=axis, angle=angle, mstack=mstack)

    # # UPDATE MSTACK    
    # mstack = update_orientation(T=transform, t = mstack)
    mstack = update_orientation(T=mstack, t = transform)

    # EXTRACT WAYPOINT FROM THE TRANSFORMATION
    waypoint = get_waypoint(mstack=mstack, origin=initial_pose)
    print(waypoint)
    
     # ADD WAYPOINT TO THE WAYPOINT ARRAY
    waypoints_array = add_global_waypoint(new_waypoint=waypoint)
    
    return waypoints_array, mstack

# currently works best for going forward, could probably be made more robus for different angles. realistically should only transalte one direction at a time for now anyways
def generate_line(initial_pose = Pose(), mstack= np.eye(4), trans = 1, axis = [1,0,0]):
    # GENERATE INITIAL WAYPOINT
    # waypoint_array = initial_pose.pose()

    # GENERATE TRANSLATION USING DEFAULT DIRECTION AND VALUES
    transform = generate_translation_matrix(axis= axis, translation = trans, mstack=mstack) 

    # UPDATE MSTACK    
    # mstack = update_orientation(T=transform, t = mstack)
    mstack = update_orientation(T=mstack, t = transform)

    # EXTRACT WAYPOINT FROM THE TRANSFORMATION
    waypoint = get_waypoint(mstack=mstack, origin=initial_pose)
    print(waypoint)
    
    # ADD WAYPOINT TO THE WAYPOINT ARRAY
    waypoints_array = add_global_waypoint(new_waypoint=waypoint)
    
    return waypoints_array, mstack
     

# # Example parameters for two patterns
# # lawnmower pattern
# width1, height1, strip_width1, spacing1 = 30, 20, 5, 3
# center1, radius1, angle1, increments1 = np.array([0, 0]), 2, np.pi/2, 10 

#  probably dont need this 
# def path_position_tolerance_check(pose, waypoint, tolerance=0.2):
    
#     distance = np.sqrt((pose['x'] - waypoint[0]) ** 2 + (pose['y'] - waypoint[1]) ** 2)
#     return distance < tolerance


# def print_waypoints(waypoints):
#     """
#         Print each waypoint from the given list of waypoints where the elements of the list are defined as Pose objects
#     """
#     for pose in waypoints:
#         print(pose)

# def chain_patterns(new_waypoints_local, waypoints_array, mstack, local_transform):
#     # extract local frame 
#     xyz = new_waypoints_local[:, :3]
#     rpy = new_waypoints_local[:, 3:]
#     # print(xyz)
#     # transform waypoints to global frame
#     new_waypoints_global= transform_waypoints(mstack, new_waypoints_local)
#     # new_waypoints_global = np.hstack(xyz_global,)
  
#     # add waypoints to global stack and remove duplicate adjacent waypoints
#     waypoints_array = np.vstack((waypoints_array, new_waypoints_global))
    
#     # remove duplicate adjacent waypoints
#     waypoints_array= remove_adjacent_duplicates(waypoints_array)
    
#     # update transform
#     mstack = np.matmul(mstack,local_transform)
    
#     return waypoints_array, mstack, new_waypoints_global
    

# # DEFINED PATTERNS
    
# def defined_square(size = 1):
#     waypoints = []
#     waypoints.append(Pose().pose()())
#     waypoints.append(Pose(size,0,0,0,0,0).pose()())
#     waypoints.append(Pose(size,0,0,0,0, np.pi/2).pose()())
#     waypoints.append(Pose(size,size,0,0,0,np.pi/2).pose()())
#     waypoints.append(Pose(size,size,0,0,0, np.pi).pose()())
#     waypoints.append(Pose(0,size,0,0,0, np.pi).pose()())
#     waypoints.append(Pose(0,size,0,0,0, 3*np.pi/2).pose()())
#     waypoints.append(Pose(0,0,0,0,0,3*np.pi/2).pose()())

#     waypoints = np.array(waypoints)

#     return waypoints    


# # FIXED HEADING PATTERNS RELATIVE TO THE INITIAL POSITION AND ORIENTATION (body frame)                 
# def generate_forward_path(current_pose = Pose(0,0,0,0,0,0), mstack = np.eye(4), distance=4, spacing=1):
#     print(current_pose)
    

# def generate_rotation_waypoint(current_pose = Pose(0,0,0,0,0,0), axis = [0,0,1], angle = np.pi/2):
#     # currentyl only handles yaw rotation but setting it up for any axis later 
#     # initialize waypoints list 
#     waypoints = []
    
#     # adding current pose to the waypoint list
#     waypoint = current_pose
#     waypoints.append(current_pose.pose()())
    
    
#     # applying transformation
#     q = quaternion_from_axis_angle(axis, angle)
#     q = quaternion_normalize(q)
#     transformation = quaternion_to_rotation_matrix(q)
    
#     # Note for later this works
#     # rpy = euler_from_transformation_matrix(transformation)
#     # print("yaw from matrix: ", rpy[2])
    
#     waypoint.yaw += angle 
    
#     waypoints.append(waypoint.pose())
    
#     waypoints = np.array(waypoints)
    
#     return waypoints, waypoint, transformation   

# def generate_lawnmower_pattern_2(width, height, strip_width, spacing):
#     """
#     Generate a lawnmower pattern for a given width, height, strip width, and spacing.

#     Parameters:
#     - width: The width of the area to be covered (meters).
#     - height: The height of the area to be covered (meters).
#     - strip_width: The width of each mowing strip (meters).
#     - spacing: The vertical spacing between each mow row (meters).

#     Returns:
#     - A 2D numpy array of shape (N, 2) representing the lawnmower pattern waypoints.
#     """
#     # Initialize an empty list to store waypoints
#     waypoints = []

#     # We will generate waypoints row by row
#     y_position = 0  # Start from the top of the field (y = 0)
    
#     while y_position < height:
#         # Calculate how many strips we can fit horizontally in this row
#         num_strips = int(np.ceil(width / strip_width))
        
#         if int(y_position / spacing) % 2 == 0:
#             # Left to right direction
#             for i in range(num_strips):
#                 x_position = i * strip_width
#                 # Ensure x_position doesn't exceed width
#                 x_position = min(x_position, width)
                
#                 # TO DO: Need to do some thinking on how to assign Yaw
#                 yaw = 0
#                 waypoints.append(Pose(x_position, y_position, yaw))
                
#         else:
#             # Right to left direction
#             for i in range(num_strips - 1, -1, -1):
#                 x_position = i * strip_width
#                 # Ensure x_position doesn't exceed width
#                 x_position = min(x_position, width)
#                 waypoints.append([x_position, y_position])
        
#         # Move down by the spacing after completing a row
#         y_position += spacing
        
#     # Convert the list of waypoints into a numpy array
#     waypoints_array = np.array(waypoints)

#     return waypoints_array

