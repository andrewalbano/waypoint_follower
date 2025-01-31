import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from classes3 import Pose
from waypoint_patterns3 import *

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

    
def test1():
    mstack = np.eye(4)
       
    # CHAINING MULTIPLE PATTERNS TOGETHER AND DISPLAYING IN LOCAL AND GLOBAL FRAMES
        
    # GENERATE FORWARD PATH
    # generate initial waypoints 
    waypoints1_local, _, local_transform= generate_forward_path() # note: don't really need last waypoint anymore

    # transform waypoints to global
    waypoints1_global = transform_waypoints(mstack, waypoints1_local)
    
    # add transformed waypoint to the global waypoints array
    waypoints_array_global = waypoints1_global
    
    # update transform
    mstack = np.matmul(mstack,local_transform)
    
    
    # GENERATE ROTATION
    # generate initial waypoints, since I am keeping track of mstack, I can use local rotation from body frame pose and then transform it using mstack
    waypoints2_local, _, local_transform = generate_rotation_waypoint(rotation=-np.pi/2, interval= False)
    
    # chain the new waypoint to the global chain
    waypoints_array_global, mstack, waypoints2_global = chain_patterns(waypoints2_local,waypoints_array_global,mstack,local_transform)
    
    # PLOTTING
    # Create subplots: 2 rows, 1 column
    num_rows = 2
    num_cols = 3
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)
    # Note notation for 2d axis plotting
    plot_pattern(axs[0,0], waypoints1_local, 'fwd path local',  heading_defined=True)
    plot_pattern(axs[0,1], waypoints2_local, 'Rotation local', heading_defined=True)
    # Hide unused subplots
    axs[0, 2].set_visible(False)
    plot_pattern(axs[1,0], waypoints1_global, 'fwd path global',  heading_defined=True)
    plot_pattern(axs[1,1], waypoints2_global, 'Rotation global', heading_defined=True)
    plot_pattern(axs[1,2], waypoints_array_global, 'L', heading_defined=True)
    

    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()

def test2():
    mstack = np.eye(4)
    print(mstack)
       
    # CHAINING MULTIPLE PATTERNS TOGETHER AND DISPLAYING IN LOCAL AND GLOBAL FRAMES
        
    # GENERATE FORWARD PATH
    # generate initial waypoints in in local frame 
    waypoints1_local, _, local_transform= generate_forward_path() # note: don't really need last waypoint anymore

    # transform waypoints to global
    waypoints1_global = transform_waypoints(mstack, waypoints1_local)
    
    # add transformed waypoint to the global waypoints array
    waypoints_array_global = waypoints1_global
    
    # update transform
    mstack = np.matmul(mstack,local_transform)
    
    print(mstack)
    # GENERATE ROTATION
    # generate initial waypoints in local frame
    waypoints2_local, _, local_transform = generate_rotation_waypoint(rotation=-np.pi/2, interval= False)
    
    # chain the new waypoint to the global chain
    waypoints_array_global, mstack, waypoints2_global = chain_patterns(waypoints2_local, waypoints_array_global, mstack, local_transform)
    
    print(mstack)
    # GENERATE FWD PATH
    # generate initial waypoints in local frame
    waypoints3_local, _, local_transform = generate_forward_path(distance=2)
    
    # chain the new waypoint to the global chain
    waypoints_array_global, mstack, waypoints3_global = chain_patterns(waypoints3_local,waypoints_array_global,mstack,local_transform)
    
      # GENERATE ROTATION
    # generate initial waypoints in local frame
    waypoints4_local, _, local_transform = generate_rotation_waypoint(rotation=-np.pi/2, interval= False)
    
    # chain the new waypoint to the global chain
    waypoints_array_global, mstack, waypoints4_global = chain_patterns(waypoints4_local, waypoints_array_global, mstack, local_transform)
    
    
    # PLOTTING
    # Create subplots: 3 rows, 3 column
    num_rows = 3
    num_cols = 4
    fig = plt.figure(figsize=(10, 10))
    gs = gridspec.GridSpec(num_rows, num_cols, figure=fig)


    # Row 1
    ax1 = fig.add_subplot(gs[0, 0])
    plot_pattern(ax1, waypoints1_local, 'fwd path local', heading_defined=True)

    ax2 = fig.add_subplot(gs[0, 1])
    plot_pattern(ax2, waypoints2_local, 'Rotation local', heading_defined=True)

    ax3 = fig.add_subplot(gs[0, 2])
    plot_pattern(ax3, waypoints3_local, 'fwd path local', heading_defined=True)

    ax4 = fig.add_subplot(gs[0, 3])
    plot_pattern(ax4, waypoints4_local, 'rot local', heading_defined=True)
    
    # Row 2
    ax5 = fig.add_subplot(gs[1, 0])
    plot_pattern(ax5, waypoints1_global, 'fwd path global', heading_defined=True)

    ax6 = fig.add_subplot(gs[1, 1])
    plot_pattern(ax6, waypoints2_global, 'Rotation global', heading_defined=True)

    ax7 = fig.add_subplot(gs[1, 2])
    plot_pattern(ax7, waypoints3_global, 'fwd path global', heading_defined=True)
    
    ax8 = fig.add_subplot(gs[1, 3])
    plot_pattern(ax8, waypoints4_global, 'fwd path global', heading_defined=True)

    # Row 3, span all three columns
    ax9 = fig.add_subplot(gs[2, :])
    plot_pattern(ax9, waypoints_array_global, 'U', heading_defined=True)
    
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()

def test():
    mstack=np.eye(4)
    # CHAINING MULTIPLE PATTERNS TOGETHER AND DISPLAYING IN LOCAL AND GLOBAL FRAMES
        
    # GENERATE FORWARD PATH
    # generate initial waypoints 
    waypoints1_local, _, local_transform= generate_forward_path() # note: don't really need last waypoint anymore
    print(waypoints1_local)
    # transform waypoints to global
    waypoints1_global = transform_waypoints(mstack, waypoints1_local)
    
    # add transformed waypoint to the global waypoints array
    waypoints_array_global = waypoints1_global
    
    # update transform
    mstack = np.matmul(mstack,local_transform)
    # PLOTTING
    # Create subplots: 2 rows, 1 column
    num_rows = 2
    num_cols = 3
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)
    # Note notation for 2d axis plotting
    plot_pattern(axs[0,0], waypoints1_local, 'fwd path local',  heading_defined=True)
    # plot_pattern(axs[0,1], waypoints2_local, 'Rotation local', heading_defined=True)
    # Hide unused subplots
    axs[0, 2].set_visible(False)
    plot_pattern(axs[1,0], waypoints1_global, 'fwd path global',  heading_defined=True)
    # plot_pattern(axs[1,1], waypoints2_global, 'Rotation global', heading_defined=True)
    # plot_pattern(axs[1,2], waypoints_array_global, 'L', heading_defined=True)
    

    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()
    
    
def square_test():
    
    # GROUND TRUTH
    defined_waypoints = defined_square()
    # print("defined_waypoints: \n", defined_waypoints)
    
    # GENERATE PATH
    # Initializing
    mstack = np.eye(4)
    start_pose = Pose()
    waypoints_array = np.array(start_pose.get_list())
    
    # generate fwd 1
    waypoints1, last_waypoint, local_transform= generate_forward_path(current_pose = start_pose, distance = 1)
    # print("waypoints1: \n", waypoints1)
    # print("last_waypoint: ", last_waypoint)
    mstack = mstack@local_transform
    # print("mstack: \n", mstack)
    # waypoints_array = np.vstack((waypoints_array,waypoints1))
    
    # generate rotation 1
    waypoints2,last_waypoint, local_transform = generate_rotation_waypoint(current_pose= last_waypoint)
    # print("last_waypoint: ", last_waypoint)
    
    mstack = mstack@local_transform 
    # print("mstack: \n", mstack)
    
    waypoints_array = np.vstack((waypoints_array,waypoints2))
    
    
    # generate fwd 2
    # waypoints3,last_waypoint, local_transform = generate_forward_path(current_pose = last_waypoint, distance = 1)
    waypoints3,last_waypoint, local_transform = generate_forward_path(current_pose = last_waypoint, distance = 1)
    print("waypoints1: \n", waypoints1)
    print("last_waypoint: ", last_waypoint)
    waypoints3 = apply_transform(mstack,waypoints3)
       
    
    
    # print("last_waypoint: ", last_waypoint)
    
    mstack = mstack@local_transform 
    # print("mstack: \n", mstack)
    waypoints_array = np.vstack((waypoints_array,waypoints3))
    
  
    # # remove duplicate adjacent waypoints
    waypoints_array= remove_adjacent_duplicates(waypoints_array)   
    

    
    # PLOTTING
    num_rows = 2
    num_cols = 2

    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Row 1
    plot_pattern(axs[0,0], defined_waypoints, 'defined square', heading_defined=True)
    plot_pattern(axs[0,1], waypoints3, 'step', heading_defined=True)
    
    plot_pattern(axs[1,0], waypoints_array, 'generated pattern', heading_defined=True)
    # Hide unused subplots
    # axs[1, 0].set_visible(False)
    axs[1, 1].set_visible(False)

    
    
    
    # Display all plots
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()

    
    
    
    
# note to self use np.vstack((waypoints1, waypoints2)) to concatenate
def main(): 
    square_test()
    

    
if __name__ == "__main__":
    main()
