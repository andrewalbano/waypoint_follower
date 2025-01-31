
import numpy as np
import matplotlib.pyplot as plt
# import matplotlib.gridspec as gridspec
from classes import Pose
from waypoint_pattern_quick_gen import *
from matrices import *
from quarternions import *
from waypoint import *
from test_patterns import *
from waypoint_plot_visualization import *


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
       # fig, axs = plt.subplot( figsize=(10, 10), sharex=False)
       
       
       # INITIALIZE global start pose.. simulate the point at which you drop into the water
       origin = Pose(0, 0, 0, 0, 0, 0)
       mstack = np.eye(4)  
       initial_frames = add_global_waypoint(current_waypoints=None, new_waypoint=origin)
       
              
       # Initialize body frame 
       body_pose = Pose(0, 0, 1, 0, 0, np.pi/2) # simulate reading the current pose 
       waypoint_array = body_pose.pose()
       
       initial_frames= add_global_waypoint(current_waypoints=initial_frames, new_waypoint=body_pose)
       transform = get_transformation_from_pose(pose=body_pose)
       mstack = update_orientation(mstack, transform)


       new_waypoints = get_premade_test_waypoints(test=test,initial_pose=origin, mstack=mstack)
       waypoints_array = add_waypoints(current_waypoints=waypoint_array, new_waypoints=new_waypoints)
       # print(waypoints_array)
       # global_waypoints_array = waypoints_array
       
       
       plot_pattern(ax, waypoints_array, f"{test} test pattern\n black shows global origin and inital body frame", heading_defined=True)  
       plot_initial_frames(ax, initial_frames)
       # plot_pattern(axs[0], waypoints_array, test, heading_defined=True)
       #  plot_pattern(axs[1], waypoints_array, test, heading_defined=True)  

       # Plot
       ax2 = fig.add_subplot(num_rows, num_cols, 2, projection='3d')
       # ax2 = plt.subplot(1, 2, 2, projection='3d')  # specify 3D projection here
       x, y, z= waypoints_array[:, 0], waypoints_array[:, 1], waypoints_array[:, 2]
       
       ax2.plot(initial_frames[:,0], initial_frames[:,1], initial_frames[:,2])
       
       ax2.plot(x, y, z)
       # fig.add.subplot(axs[1], projection='3d')
       # subplot_kw={"projection": "3d"})
       # axs[1].plot(xs, ys, zs)

       elevation = -25
       azimuth = 143
       roll = 179
       ax2.view_init(elev=elevation, azim=azimuth, roll=roll)
       ax2.set(xticklabels=[],
       yticklabels=[],
       zticklabels=[])    
       
       
       ax2.set_ylabel('x position')
       ax2.set_xlabel('y position')
       ax2.set_zlabel('depth')
       
            
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
    