import numpy as np

class Pose:
    """
    A class to represent the pose of an object in a 3D space,
    characterized by its position and orientation.
    """

    def __init__(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Initializes a Pose instance.

        Parameters:
            x (float): X-coordinate of the position. Default is 0.0.
            y (float): Y-coordinate of the position. Default is 0.0.
            z (float): Z-coordinate of the position. Default is 0.0.
            roll (float): Roll angle (rotation around the x-axis). Default is 0.0.
            pitch (float): Pitch angle (rotation around the y-axis). Default is 0.0.
            yaw (float): Yaw angle (rotation around the z-axis). Default is 0.0.
        """
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def xyz(self):
        """
        Returns the position coordinates as a numpy array.

        Returns:
            numpy.ndarray: An array containing [x, y, z].
        """
        return np.array([self.x, self.y, self.z])

    def update(self, x, y, z, roll, pitch, yaw):
        """
        Updates the pose with new position and orientation values.

        Parameters:
            x (float): New X-coordinate of the position.
            y (float): New Y-coordinate of the position.
            z (float): New Z-coordinate of the position.
            roll (float): New roll angle.
            pitch (float): New pitch angle.
            yaw (float): New yaw angle.
        """
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def pose(self):
        """
        Returns the complete pose (position and orientation) as a numpy array.

        Returns:
            numpy.ndarray: An array containing [x, y, z, roll, pitch, yaw].
        """
        return np.array([self.x, self.y, self.z, self.roll, self.pitch, self.yaw])

    def __str__(self):
        """
        Returns a string representation of the Pose instance.

        Returns:
            str: A formatted string depicting the pose.
        """
        return (f"Pose(x={self.x}, y={self.y}, z={self.z}, "
                f"roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})")
        



# Example usage of the Pose class
def main(show_sample = False):
    if show_sample:
            
        # Create a default Pose object
        default_pose = Pose()

        # Print the default Pose
        print("Default Pose:", default_pose)

        # Create a Pose object with specific values
        custom_pose = Pose(1.0, 2.0, 3.0, 0.1, 0.2, 0.3)

        # Print the custom Pose
        print("Custom Pose:", custom_pose)

        # Update the custom pose with new values
        custom_pose.update(4.0, 5.0, 6.0, 0.4, 0.5, 0.6)

        # Print the updated Pose
        print("Updated Pose:", custom_pose)

        # Get the position of the updated Pose as a numpy array
        position = custom_pose.xyz()
        print("Position (x, y, z):", position)

        # Get the complete Pose as a numpy array
        complete_pose = custom_pose.pose()
        print("Complete Pose (x, y, z, roll, pitch, yaw):", complete_pose)
        return 0
    else:
        return 0
    
    
# Example usage of the Pose class
if __name__ == '__main__':
    main(show_sample=False)
    # main(show_sample=True)