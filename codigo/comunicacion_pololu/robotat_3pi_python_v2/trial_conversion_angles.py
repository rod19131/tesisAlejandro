from transformations import euler_from_quaternion
import math
def quaternion_to_euler(quaternion):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
    qx, qy, qz, qw (float): The quaternion components.

    Returns:
    roll, pitch, yaw (float): The Euler angles in radians.
    """
    #quaternion = [qx, qy, qz, qw]
    euler_angles = euler_from_quaternion(quaternion)
    #XYZ
    #[roll, pitch, yaw] = euler_angles
    angles_degrees = [math.degrees(angle) for angle in euler_angles]
    return angles_degrees

# Example usage:
quat = [0.7071, 0.0, 0.0, 0.7071]  # Example quaternion
listaxyz = quaternion_to_euler(quat)
print(listaxyz)
"""
print("Roll:", roll)
print("Pitch:", pitch)
print("Yaw:", yaw)
"""