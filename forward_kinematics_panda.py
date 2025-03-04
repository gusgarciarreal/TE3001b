# Imports
import roboticstoolbox as rtb
import numpy as np

panda = rtb.models.Panda()  # Create a Panda robot model
print(f"Default angular position qr: \n{panda.qr}")
print(f"Homogeneous Transformation Matrix: \n{panda.fkine(panda.qr)}")

# Denavit-Hartenberg parameters
panda_dh = rtb.models.DH.Panda()  # Create a Panda robot model using DH parameters
print(f"DH Parameters: \n{panda_dh}")

# Plot the robot
panda.plot(panda.qr, block=True, backend='pyplot')  # Plot the robot in the default configuration

# Plot new Pose
new_pos = np.array([0, 0, 0, 0, 0, 0, 0])  # New pose of the robot
panda.plot(new_pos, block=True, backend='pyplot')  # Plot the robot in the new pose
print(f"New Pose: \n{panda.fkine(new_pos)}")