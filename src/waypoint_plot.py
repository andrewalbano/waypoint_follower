import numpy as np
import matplotlib.pyplot as plt
from classes import Pose
from waypoint_patterns import *
        
# def plot_pattern(ax, waypoints, title,):
#     '''
#         Plot a set of waypoints on a given subplot axis.

#         This function takes a set of waypoints and plots them on a specified
#         Matplotlib axis. It is designed for plotting lawnmower pattern
#         waypoints or similar coordinate data. 

#         Parameters:
#             - ax: The Matplotlib axis object where the plot will be drawn. This
#             allows for plotting in a subplot environment.
#             - waypoints: A NumPy array of shape (N, 2) containing the (x, y)
#             coordinates of waypoints to be plotted.
#             - title: A string that sets the title of the subplot, giving context
#             to the plot.

#         Plot Characteristics:
#             - The waypoints are plotted as a line with 'o' markers denoting
#             individual waypoints.
#             - X and Y axes are labeled with "X position" and "Y position" 
#             respectively.
#             - A grid is shown to help with visual reference.
#     '''
#     # Unpack the x and y coordinates
#     x_coords, y_coords = waypoints[:, 0], waypoints[:, 1]
    
#     # Plot the waypoints in a given subplot axis (ax)
#     ax.plot(x_coords, y_coords, marker='o', linestyle='--')
    
#     # Set subplot details
#     ax.set_title(title)
#     ax.set_xlabel('X position')
#     ax.set_ylabel('Y position')
#     ax.grid(True)

def plot_pattern(ax,waypoints,title, heading_defined = False):

    # visualize the desired heading
    if heading_defined:
        # Unpack the x and y coordinates, and yaw values
        x, y, yaw = waypoints[:, 0], waypoints[:, 1], waypoints[:,2]
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

    
    
# note to self use np.vstack((waypoints1, waypoints2)) to concatenate
def main(): 
    
    # Example parameters for two patterns
    # lawnmower pattern
    width1, height1, strip_width1, spacing1 = 30, 20, 5, 3
    center1, radius1, angle1, increments1 = np.array([0, 0]), 2, np.pi/2, 10 
    
    
    # Generate waypoints for each pattern
    waypoints1 = generate_lawnmower_pattern(width1, height1, strip_width1, spacing1) 
    # print("size of waypoints 1: ", waypoints1.size)
    # waypoints2 = generate_lawnmower_pattern(width2, height2, strip_width2, spacing2)
    # waypoints2 = create_semicircle(center1, radius1, angle1, increments1)
    # waypoints3 = orbit_mode(center1, radius1, angle1, increments1)
    
    waypoints2, last_waypoint,_= generate_forward_path()
    # print("size of waypoints 2: ", waypoints2.size)
    print("waypoints 2: \n", waypoints2)
    print(last_waypoint)
    
    waypoints3,_,_= generate_rotation_waypoint(current_pose = last_waypoint, rotation=-np.pi/2, interval= False)
    print("waypoints 3: \n", waypoints3)

    # Create subplots: 2 rows, 1 column
    num_rows = 1
    num_cols = 3
    fig, axs = plt.subplots(num_rows, num_cols, figsize=(10, 10), sharex=False)

    # Plot each set of waypoints in a subplot
    # plot_pattern(ax, waypoints, title):
    # plot_pattern(axs[0], waypoints1, 'lawnmower pattern')
    # plot_pattern(axs[1], waypoints2, 'fwd path',  heading_defined=True)
    # plot_pattern(axs[2], waypoints3, 'rotation', heading_defined=True)
    
    
    waypoints = np.vstack((waypoints2,waypoints3))
    
    plot_pattern(axs[0], waypoints2, 'fwd path',  heading_defined=True)
    plot_pattern(axs[1], waypoints3, 'Rotation', heading_defined=True)
    plot_pattern(axs[2], waypoints, 'L', heading_defined=True)
    
    # Display all plots
    # plt.figure(figsize=(8, 8))
    plt.tight_layout()  # Optional, to improve spacing
    plt.show()
    
    
if __name__ == "__main__":
    main()