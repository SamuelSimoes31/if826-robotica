import roboticstoolbox as rtb
import numpy as np
from _2_kinematics import traj_joint, traj_euclidean

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# Plot robot's details
print(robot)

# theta1 = np.radians([0, 90])
# theta2 = np.radians([0, -90])
# q_traj = traj_joint(theta1[0], theta2[0], theta1[1], theta2[1])

q_traj = traj_euclidean(0, 0, -1, -1)

# theta1 = np.radians([0, -45])
# theta2 = np.radians([0, -90])
# q_traj= traj_joint(theta1[0], theta2[0], theta1[1], theta2[1])

# Visualize the trajectory
robot.plot(q_traj, backend='pyplot', movie='cartesiano_eixo_x.gif', dt=0.01)
