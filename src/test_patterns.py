import numpy as np
from classes import Pose
from waypoint_pattern_quick_gen import *
from matrices import *
from quarternions import *
from waypoint import *


# TEST 1: Straight line 1 unit 
def line_test(initial_pose=Pose(), mstack=np.eye(4)):
    
    # waypoints_array = initial_pose.pose()
    new_waypoints, mstack = generate_line(initial_pose=initial_pose, mstack= mstack, trans=1, axis=[1,0,0])
    # waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    waypoints_array = add_waypoints(current_waypoints=None, new_waypoints=new_waypoints)
    return waypoints_array
    
    
# TEST 2: Rotation 90 degree
def rotation_test(initial_pose=Pose(), mstack=np.eye(4)):

    # waypoints_array = initial_pose.pose()
    
    new_waypoints, mstack = generate_rotation(initial_pose=initial_pose, mstack=mstack, angle = np.pi/2, axis = [0,0,1])
    # waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    waypoints_array = add_waypoints(current_waypoints=None, new_waypoints=new_waypoints)
    return waypoints_array

# Test 3: L shape 
def L_test(initial_pose = Pose(),mstack = np.eye(4)):
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    
    # the current location is the first waypoint 
    waypoints_array = initial_pose.pose()

    # generate translation in the x direction
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate 90 degree rotation
    new_waypoints, mstack = generate_rotation(initial_pose=initial_pose, mstack=mstack, angle=np.pi/2, axis=z_axis)

    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate translation
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
 
    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    return waypoints_array


# Test 4: square shape 
def square_test(initial_pose = Pose(),mstack = np.eye(4)):
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    
    # the current location is the first waypoint 
    # waypoints_array = initial_pose.pose()

    # generate translation in the x direction
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=None, new_waypoints=new_waypoints)
    
    # generate 90 degree rotation
    new_waypoints, mstack = generate_rotation(initial_pose=initial_pose, mstack=mstack, angle=np.pi/2, axis=z_axis)

    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate translation
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
 
    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
      
    # generate rotation
    new_waypoints, mstack = generate_rotation(initial_pose=initial_pose, mstack=mstack, angle=np.pi/2, axis=z_axis)

    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)

    # generate translation
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
   
    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate rotation
    new_waypoints, mstack = generate_rotation(initial_pose=initial_pose, mstack=mstack, angle=np.pi/2, axis=z_axis)

    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)

    # generate translation
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
   
    # add new waypoints 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)

    return waypoints_array

    
# Test 4: Draw a z with constant yaw
def z_test(initial_pose = Pose(),mstack = np.eye(4)):
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    # the current location is the first waypoint 
    # waypoints_array = initial_pose.pose()
    
    # generate translation in the x direction
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=None, new_waypoints=new_waypoints)
    
    
    # generate translation in direction and add waypoint
    axis = get_axis_from_angle(angle= -3*np.pi/4) # note the could also specify axis if you know direction of axis already, example [-1,-1,0]    
    new_waypoints, mstack = generate_angled_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=axis)
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate translation in direction and add waypoint
    new_waypoints, mstack = generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=[1,0,0])
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)

    return waypoints_array

# Test 5: Draw a z with non constant yaw and using generate transformation matrix
def z_test_2(initial_pose = Pose(),mstack = np.eye(4)):
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    # the current location is the first waypoint 
    # waypoints_array = initial_pose.pose()
    
    # generate translation in the x direction and rotation about z_axis
    new_waypoints, mstack= generate_transformation(initial_pose = initial_pose, mstack= mstack, trans=1, trans_axis=x_axis, rot_axis=z_axis, angle=-3*np.pi/4)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=None, new_waypoints=new_waypoints)
    
    # generate translation in the x,y direction and rotation about z_axis

    new_waypoints, mstack= generate_transformation(initial_pose = initial_pose, mstack= mstack, trans=1, trans_axis=x_axis, rot_axis=z_axis, angle=3*np.pi/4)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    return waypoints_array
