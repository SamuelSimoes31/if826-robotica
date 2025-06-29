import math as mt
from _2_kinematics import ik
import matplotlib.pyplot as plt
import numpy as np
from utils import plot_trajectory_euclidean

def traj_eucl(x_init, y_init, x_final, y_final, ds=0.01):
    init_final = [[x_init, x_final], [y_init, y_final]]
    dist_axis = [abs(x_final - x_init), abs(y_final - y_init)]
    dist_sync = max(dist_axis[0], dist_axis[1])
    xy = []

    for i in range(2):
        sign = 1 if init_final[i][1] >= init_final[i][0] else -1
        if dist_axis[i] == dist_sync:
            arr = np.arange(init_final[i][0], init_final[i][1], sign * ds)
            xy.append(arr)
        else:
            if dist_axis[i] == 0:
                arr = np.empty(int(np.ceil(dist_sync / ds)))
                arr.fill(init_final[i][0])
                xy.append(arr)
            else:
                ds_sync = (dist_axis[i] * ds) / dist_axis[0 if i == 1 else 1]
                arr = np.arange(init_final[i][0], init_final[i][1], sign * ds_sync)
                xy.append(arr)

    q = []
    pos = []
    last_q = None

    for i in range(len(xy[0])):
        x, y = xy[0][i], xy[1][i]
        sols = ik(x, y)  # Espera-se que 'ik' retorne uma lista de soluções

        if sols is None or len(sols) == 0:
            raise ValueError("Fora da área")

        if last_q is None:
            sol = sols[0]
        else:
            sol = min(sols, key=lambda q_: np.linalg.norm(np.array(q_) - np.array(last_q)))

        q.append(sol)
        pos.append([x, y])
        last_q = sol

    q = np.array(q)
    pos = np.array(pos)
    t = np.arange(len(q)) * ds

    return q, pos, t


def main():
    q_traj, pos_traj, t_traj = traj_eucl(0.1, 0.1, 1, 1, ds=0.01)
    plot_trajectory_euclidean(pos_traj, q_traj, t_traj)

if __name__ == "__main__":
    main()
    