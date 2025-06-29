import roboticstoolbox as rtb
from traj_eucl import traj_euclidean
from traj_joint import traj_joint
import numpy as np

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# Exemplo de trajetória euclideana
# q_traj_eucl = traj_euclidean(0, 0, 0, 0.5, ds=0.01)


# Exemplo de trajetória articular
theta1 = np.radians([0, 45])
theta2 = np.radians([0, 90])
q_traj_joint, _, _, _  = traj_joint(theta1[0], theta2[0], theta1[1], theta2[1], ts=0.01)
# print(q_traj_joint)

# Visualize the trajectory
# robot.plot(q_traj_eucl, backend='pyplot', movie='cartesiano_eucl.gif', dt=0.01)
robot.plot(q_traj_joint, backend='pyplot', movie='cartesiano_joint.gif', dt=0.01)
