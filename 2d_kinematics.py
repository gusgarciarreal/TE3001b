# Imports
from roboticstoolbox import ET2
import roboticstoolbox as rtb
import math

# Joint angles
pi = math.pi
q = [pi/6, 0.8]  # Joint angles in radians

# Links
a1 = 1  # Length of the first link
a2 = 1  # Length of the second link

# 2-DOF Robot using HTM (Homogeneous Transformation Matrices)
# Transformation from frame 0 to frame 1
T_01 = ET2.R(q[0], 'rad') * ET2.tx(a1)  # Rotation by q[0] and translation by a1 along x-axis

# Transformation from frame 1 to frame 2
T_12 = ET2.R(q[1], 'rad') * ET2.tx(a2)  # Rotation by q[1] and translation by a2 along x-axis

# Transformation from frame 0 to frame 2
T_02 = T_01 * T_12  # Combined transformation from frame 0 to frame 2

# Forward kinematics to find the end-effector position
K_2d = T_02.fkine([])  # Compute the forward kinematics
print(f"2D Kinematics: {K_2d}")

# Create a 2-DOF planar robot model
robot1 = rtb.models.ETS.Planar2()

# Forward kinematics using the robot model
T = robot1.fkine(q)
print(f"2D Kinematics: {T}")

# Plot the robot
robot1.plot(q, block=True)