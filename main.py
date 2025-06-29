import roboticstoolbox as rtb
from _3_traj_eucl import traj_eucl
from _3_traj_joint import traj_joint
import numpy as np
from utils import plot_trajectory_euclidean, plotar_series_temporais_completo

# Create the RR robot object
robot = rtb.models.DH.Planar2()

# trajetória no espaço de configuração
initial_pos = [0, -1.5]
final_pos = [0.4, 0]
q_traj_eucl, pos_traj_eucl, t_traj_eucl = traj_eucl(initial_pos[0], initial_pos[1], final_pos[0], final_pos[1], ds=0.01)
plot_trajectory_euclidean(pos_traj_eucl, q_traj_eucl, t_traj_eucl)
robot.plot(q_traj_eucl, backend='pyplot', movie='cartesiano_eucl.gif', dt=0.01)

# trajetória no espaço das juntas
theta1 = np.radians([0, 45])
theta2 = np.radians([0, 90])
q_traj_joint, v_traj_joint, a_traj_joint, t_traj = traj_joint(theta1[0], theta2[0], theta1[1], theta2[1], ts=0.01)
plotar_series_temporais_completo(q_traj_joint, v_traj_joint, a_traj_joint, t_traj)
robot.plot(q_traj_joint, backend='pyplot', movie='cartesiano_joint.gif', dt=0.01)
