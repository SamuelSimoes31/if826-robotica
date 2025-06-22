import roboticstoolbox as rtb
import numpy as np
from kinematics import traj_joint

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# Plot robot's details
print(robot)

# Create a joint trajectory (must be replaced by your function)
# nbSteps = 11
# qt = np.zeros((nbSteps, 2))

# for idx in range(nbSteps):
#     qt[idx, :] = [idx * 0.1] * 2

q_traj, v_traj, a_traj, t_traj = traj_joint(0, 0, 90, 120)

# Visualize the trajectory
robot.plot(q_traj, backend='pyplot', movie='q_traj3.gif')
