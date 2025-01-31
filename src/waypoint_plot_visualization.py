import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.gridspec as gridspec
from classes import Pose
from waypoint_pattern_quick_gen import *
from matrices import *
from quarternions import *
from waypoint import *
from test_patterns import *


def plot_pattern_backup(ax,waypoints,title, heading_defined = False):

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
        
        # Get the x-axis ticks
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()

        # Calculate the interval between the first two ticks in x and y axis
        x_interval = xticks[1] - xticks[0]
        y_interval = yticks[1] - yticks[0]
        
        scale=np.linalg.norm([x_interval,y_interval])
                
        ax.quiver(x, y, scale*np.cos(yaw), scale*np.sin(yaw), 
                angles='xy', scale_units='xy', scale = 3, color='r', label='Yaw')
    

def plot_pattern(ax, waypoints,title, heading_defined = True):

    # visualize the desired heading
    if heading_defined:
        if waypoints.ndim > 1:
            # Unpack the x and y coordinates, and yaw values
            x, y, yaw = waypoints[:, 0], waypoints[:, 1], waypoints[:,5]
        else:
            x, y, yaw = waypoints[0], waypoints[1], waypoints[5]
    else:
        if waypoints.ndim > 1:
            # Unpack the x and y coordinates
            x, y = waypoints[:, 0], waypoints[:, 1]
        else:
            x, y = waypoints[0], waypoints[1]
            
    
    # Setting subplot details
    ax.set_title(title)
    ax.set_xlabel('X position')
    ax.set_ylabel('Y position')
    ax.grid(True)
 
    # Plot the waypoints in a given subplot axis (ax)
    ax.plot(x, y, marker='o', linestyle='--', color = "b")


    # check if the heading is defined
    if heading_defined:
        
        # Get the x-axis ticks
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()

        # Calculate the interval between the first two ticks in x and y axis
        x_interval = xticks[1] - xticks[0]
        y_interval = yticks[1] - yticks[0]
        
        scale=np.linalg.norm([x_interval,y_interval])
                
        ax.quiver(x, y, scale*np.cos(yaw), scale*np.sin(yaw), 
                angles='xy', scale_units='xy', scale = 3, color='r', label='Yaw')
  
def plot_initial_frames(ax,points,heading_defined= True):

    # visualize the desired heading
    if heading_defined:
        if points.ndim > 1:
            # Unpack the x and y coordinates, and yaw values
            x, y, yaw = points[:, 0], points[:, 1], points[:,5]
        else:
            x, y, yaw = points[0], points[1], points[5]
    else:
        if points.ndim > 1:
            # Unpack the x and y coordinates
            x, y = points[:, 0], points[:, 1]
        else:
            x, y = points[0], points[1]
            
    # Plot the waypoints in a given subplot axis (ax)
    ax.plot(x, y, marker='o', linestyle='--', color='k')


    # check if the heading is defined
    if heading_defined:
        
        # Get the x-axis ticks
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()

        # Calculate the interval between the first two ticks in x and y axis
        x_interval = xticks[1] - xticks[0]
        y_interval = yticks[1] - yticks[0]
        
        scale=np.linalg.norm([x_interval,y_interval])
                
        ax.quiver(x, y, scale*np.cos(yaw), scale*np.sin(yaw), 
                angles='xy', scale_units='xy', scale = 3, color='g', label='Yaw')
  
def plot_3d_pattern(ax, initial_frames, waypoints_array):
    # Plot
    x, y, z= waypoints_array[:, 0], waypoints_array[:, 1], waypoints_array[:, 2]
    ax.plot(initial_frames[:,0], initial_frames[:,1], initial_frames[:,2], color ="k")      
    ax.plot(x, y, z, color = "b",marker='o', linestyle='--')
    elevation = -25
    azimuth = 66
    roll = 179
    ax.view_init(elev=elevation, azim=azimuth, roll=roll)
    ax.set_ylabel('y position')
    ax.set_xlabel('x position')
    ax.set_zlabel('depth')

  
def get_premade_test_waypoints(test="square", initial_pose = Pose(), mstack = np.eye(4)):
    if test == "square":
        print("visualizing square test")
        waypoints_array = square_test(initial_pose=initial_pose, mstack=mstack)
        
    elif test == "rotate":
        print("visualizing rotation")
        waypoints_array = rotation_test(initial_pose=initial_pose, mstack=mstack)
      
    elif test == "line":
        print("visualizing line test")
        waypoints_array = line_test(initial_pose=initial_pose, mstack=mstack)
      
    elif test ==  "z1":
        print("visualizing z1 test")
        waypoints_array = z_test(initial_pose=initial_pose, mstack=mstack)
        
    elif test ==  "z2":
        print("visualizing z2 test")
        waypoints_array = z_test_2(initial_pose=initial_pose, mstack=mstack)
    elif test == "new":
        print("visualizing new test")
        waypoints_array = new_test(initial_pose=initial_pose, mstack=mstack)
    else:
        return ValueError("No test associated with this option")
    return waypoints_array
    

def new_test(initial_pose = Pose(), mstack = np.eye(4)):
    # Initializing
    x_axis = [1,0,0]
    y_axis = [0,1,0]
    z_axis = [0,0,1]
    
    # the current location is the first waypoint 
    waypoints_array = initial_pose.pose()
    
    # generate translation in the x direction and rotation about z_axis
    new_waypoints, mstack= generate_transformation(initial_pose = initial_pose, mstack= mstack, trans=1, trans_axis=x_axis, rot_axis=z_axis, angle=-3*np.pi/4)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    # generate translation in the x,y direction and rotation about z_axis

    new_waypoints, mstack= generate_transformation(initial_pose = initial_pose, mstack= mstack, trans=1, trans_axis=x_axis, rot_axis=z_axis, angle=3*np.pi/4)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)
    
    new_waypoints, mstack= generate_line(initial_pose = initial_pose, mstack= mstack, trans=1, axis=x_axis)
        
    # add the new waypoint 
    waypoints_array = add_waypoints(current_waypoints=waypoints_array, new_waypoints=new_waypoints)


# note that the positions are global and each transformation request is body relative 
def main():
    
    simulate_global_origin = True
    test = "square"
  
    
    if simulate_global_origin:
           
        # Set up a figure twice as tall as it is wide
        fig = plt.figure()
        fig.suptitle("waypoint planner")


        # PLOTTING
        num_rows = 1
        num_cols = 2
        # First subplot
        ax = fig.add_subplot(num_rows, num_cols, 1)
        ax2 = fig.add_subplot(num_rows, num_cols, 2, projection='3d')
               
        
        # INITIALIZE global start pose.. simulate the point at which you drop into the water
        origin = Pose(0, 0, 0, 0, 0, 0)
        mstack = np.eye(4)  
        initial_frames = add_global_waypoint(current_waypoints=None, new_waypoint=origin)
        
                
        # Initialize body frame 
        body_pose = Pose(0, 0, 1, 0, 0, 0) # simulate reading the current pose 
        waypoint_array = body_pose.pose()
        
        # add the initial frames to the array (want this seperate from teh waypoints)
        initial_frames= add_global_waypoint(current_waypoints=initial_frames, new_waypoint=body_pose)
        transform = get_transformation_from_pose(pose=body_pose)
        mstack = update_orientation(mstack, transform)


        # get the waypoints
        new_waypoints = get_premade_test_waypoints(test=test,initial_pose=origin, mstack=mstack)
        waypoints_array = add_waypoints(current_waypoints=waypoint_array, new_waypoints=new_waypoints)
        
        # plot the points in 2d and 3d
        plot_pattern(ax, waypoints_array, f"{test} test pattern\n black shows global origin and inital body frame", heading_defined=True)  
        plot_initial_frames(ax, initial_frames)
        plot_3d_pattern(ax2,initial_frames=initial_frames, waypoints_array=waypoints_array)

        # Display all plots
        plt.tight_layout()
        plt.show()
    else:
        # PLOTTING
        num_rows = 1
        num_cols = 1
        fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)
        
        waypoints_array = get_premade_test_waypoints(test)
        plot_pattern(axs, waypoints_array, test, heading_defined=True)       
        
        # Display all plots
        plt.tight_layout()
        plt.show()
            
        
        return 0
    
if __name__ == "__main__":
    main()
    