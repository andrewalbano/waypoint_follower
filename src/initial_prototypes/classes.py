import numpy as np
# Defining a Pose
class Pose:
    """
        Pose class represents a 2D position and orientation
        in a plane. It stores the x and y coordinates, as well as
        the yaw (orientation) angle.

        Attributes:
        -----------
            - x : float, The x-coordinate of the pose.
            - y : float, The y-coordinate of the pose.
            - yaw : float, The orientation of the pose in radians.

        Methods:
        --------
        update(x, y, yaw):
            - Updates the x, y, and yaw values of the pose.
        
        __str__():
            - Returns a string representation of the current pose.
    """
        
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def update(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        
    def get_list(self):
        return [self.x, self.y, self.yaw]
    
    def __str__(self):
        return f"Pose(x={self.x}, y={self.y}, yaw={self.yaw})"