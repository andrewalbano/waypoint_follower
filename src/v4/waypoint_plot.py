import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from classes import Pose
from waypoint_patterns import *
from matrices import *
from quarternions import *
from waypoint import *


def plot_pattern(ax,waypoints,title, heading_defined = False):

    # visualize the desired heading
    if heading_defined:
        # Unpack the x and y coordinates, and yaw values
        x, y, yaw = waypoints[:, 0], waypoints[:, 1], waypoints[:,5]
    else:
        # Unpack the x and y coordinates
        x, y = waypoints[:, 0], waypoints[:, 1]
    
    # Setting subplot details
    ax.set_title(title)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.grid(True)
 
    # Plot the waypoints in a given subplot axis (ax)
    ax.plot(x, y, marker='o', linestyle='--')

 
    # check if the heading is defined
    if heading_defined:
        # Setting subplot aspect details for scaling arrows
        # define y-unit to x-unit ratio
        ratio = 1.0

        # setting axis intervals and aspects
        x_left, x_right = ax.get_xlim()
        y_low, y_high = ax.get_ylim()
        ax.set_aspect(abs((x_right-x_left)/(y_low-y_high))*ratio)
        
        # Get the x-axis ticks
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()

        # Calculate the interval between the first two ticks in x and y axis
        x_interval = xticks[1] - xticks[0]
        y_interval = yticks[1] - yticks[0]
        
        ax.quiver(x, y, x_interval*np.cos(yaw), y_interval*np.sin(yaw), 
                angles='xy', scale_units='xy', scale = 2, color='r', label='Yaw')

# TEST 1: Dtraight line 1 unit 
# currently works best for going forward, could probably be made more robus for different angles. realistically should only transalte one direction at a time for now anyways

def line_test():
    initial_pose=Pose()
    waypoints_array = initial_pose.pose()
    waypoints_array,_ = generate_line(initial_pose=initial_pose, mstack= np.eye(4), trans=1, axis=[1,0,0])
    
    # PLOTTING
    num_rows = 1
    num_cols = 2
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Row 1
    plot_pattern(axs[0], waypoints_array, 'line', heading_defined=True)
    axs[1].set_visible(False)
        
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()
    
    
# TEST 2: Rotation 90 degree

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

def rotation_test():
    initial_pose = Pose()
    
    waypoints_array = initial_pose.pose()
    
    waypoints_array,_ = generate_rotation(initial_pose = initial_pose, mstack= np.eye(4), angle = np.pi/2, axis = [0,0,1])
    
    # PLOTTING
    num_rows = 1
    num_cols = 2
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Row 1
    plot_pattern(axs[0], waypoints_array, 'line', heading_defined=True)

    axs[1].set_visible(False)
    
    
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()
    


# Test 3: L shape 
def L_test():
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    # initializing the start pose
    initial_pose = Pose()
    
    # initializing the rotation matrix stack
    mstack = np.eye(4)
    
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
        
    
    # PLOTTING
    num_rows = 1
    num_cols = 2
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Row 1
    plot_pattern(axs[0], waypoints_array, 'L test', heading_defined=True)
    axs[1].set_visible(False)
    
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()

    
# Test 4: square shape 
def square_test():
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    # initializing the start pose
    initial_pose = Pose()
    
    # initializing the rotation matrix stack
    mstack = np.eye(4)
    
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
    
    
    # PLOTTING
    
    
    # PLOTTING
    num_rows = 1
    num_cols = 2
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Row 1
    plot_pattern(axs[0], waypoints_array, 'square test', heading_defined=True)
    axs[1].set_visible(False)
    
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()

    
    
    
    
    
    


    
def main():
    # line_test()
    # rotation_test()
    square_test() 
    # L_test()
    return 0
    

    
    
    
    

    
if __name__ == "__main__":
    main()
   
