import roboticstoolbox as rtb
import numpy as np

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# Plot robot's details
print(robot)

# Create a joint trajectory (must be replaced by your function)
nbSteps = 11
qt = np.zeros((nbSteps, 2))

for idx in range(nbSteps):
    qt[idx, :] = [idx * 0.1] * 2

# Visualize the trajectory
robot.plot(qt, backend='pyplot', movie='RR.gif')
