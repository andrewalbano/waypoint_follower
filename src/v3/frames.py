import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from classes3 import Pose
from waypoint_patterns3 import *
from matrices3 import *
from quarternions2 import *
    
import numpy as np
import numpy as np

class ReferenceFrame3D:
    def __init__(self, origin=None, orientation=None):
        self.origin = np.array(origin if origin is not None else [0.0, 0.0, 0.0])
        self.orientation = np.array(orientation if orientation is not None else np.eye(3))
    
    def set_origin(self, origin):
        self.origin = np.array(origin)
    
    def set_orientation(self, orientation):
        assert orientation.shape == (3, 3), "Orientation must be a 3x3 matrix."
        self.orientation = np.array(orientation)
    
    def transform_point(self, point):
        point = np.array(point)
        return self.orientation @ point + self.origin

    def inverse_transform_point(self, point):
        point = np.array(point)
        return np.linalg.inv(self.orientation) @ (point - self.origin)
    
    def get_x_axis(self):
        return self.orientation[:, 0]
    
    def get_y_axis(self):
        return self.orientation[:, 1]

    def get_z_axis(self):
        return self.orientation[:, 2]
    
    def rotate_about_x(self, angle):
        """Rotate the frame about the X-axis by a given angle in radians."""
        # angle_radians = np.radians(angle_degrees)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        rotation_matrix = np.array([
            [1, 0, 0],
            [0, cos_angle, -sin_angle],
            [0, sin_angle, cos_angle]
        ])
        
        self.orientation = rotation_matrix @ self.orientation

    def rotate_about_y(self, angle):
        """Rotate the frame about the Y-axis by a given angle in radians."""
        # angle_radians = np.radians(angle_degrees)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        rotation_matrix = np.array([
            [cos_angle, 0, sin_angle],
            [0, 1, 0],
            [-sin_angle, 0, cos_angle]
        ])
        
        self.orientation = rotation_matrix @ self.orientation
    
    def rotate_about_z(self, angle):
        """Rotate the frame about the Z-axis by a given angle in degrees."""
        # angle_radians = np.radians(angle_degrees)
        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)
        
        rotation_matrix = np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle, cos_angle, 0],
            [0, 0, 1]
        ])
        
        self.orientation = rotation_matrix @ self.orientation
 
    def __repr__(self):
        return f"ReferenceFrame3D(\n  Origin: {self.origin},\n  Orientation:\n{self.orientation}\n)"

# Example usage:
frame = ReferenceFrame3D()

# Print the initial reference frame
print("Initial frame:")
print(frame)

# Rotate the frame 90 degrees about the X-axis
frame.rotate_about_x(90)
print("\nFrame after rotating 90 degrees about the X-axis:")
print(frame)

frame2 = ReferenceFrame3D()
# Rotate the frame 90 degrees about the X-axis
frame2.rotate_about_z(90)
print("\nFrame after rotating 90 degrees about the X-axis:")
print(frame2)





# # Define a point in the original reference frame
# original_point = np.array([1, 0, 0])
# print(f"\nOriginal point in the reference frame: {original_point}")

# # Transform the point using the current reference frame
# transformed_point = frame.transform_point(original_point)
# print(f"Transformed point with the initial orientation: {transformed_point}")

# # Rotate the frame 90 degrees about the X-axis
# frame.rotate_about_x(90)
# print("\nFrame after rotating 90 degrees about the X-axis:")
# print(frame)

# # Transform the original point again after rotation
# transformed_point_after_x = frame.transform_point(original_point)
# print(f"Transformed point after X-axis rotation: {transformed_point_after_x}")

# # Rotate the frame 90 degrees about the Y-axis
# frame.rotate_about_y(90)
# print("\nFrame after rotating 90 degrees about the Y-axis:")
# print(frame)

# # Transform the original point again after the second rotation
# transformed_point_after_y = frame.transform_point(original_point)
# print(f"Transformed point after Y-axis rotation: {transformed_point_after_y}")

# # Finally, rotate the frame 90 degrees about the Z-axis
# frame.rotate_about_z(90)
# print("\nFrame after rotating 90 degrees about the Z-axis:")
# print(frame)

# # Transform the original point again after the third rotation
# transformed_point_after_z = frame.transform_point(original_point)
# print(f"Transformed point after Z-axis rotation: {transformed_point_after_z}")

# # Inverse transformation to go back to the original point (should approximate the original)
# inverse_point = frame.inverse_transform_point(transformed_point_after_z)
# print(f"\nInverse transformed point: {inverse_point}")