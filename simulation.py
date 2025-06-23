import roboticstoolbox as rtb
import numpy as np
from kinematics import traj_joint

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# Plot robot's details
print(robot)

theta1 = np.radians([0, 45])
theta2 = np.radians([0, -45])
q_traj = traj_joint(theta1[0], theta2[0], theta1[1], theta2[1])

# Visualize the trajectory
robot.plot(q_traj, backend='pyplot', movie='q_traj.gif')
