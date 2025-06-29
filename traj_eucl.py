import math as mt
from kinematics import ik
import matplotlib.pyplot as plt
import numpy as np

import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import matplotlib.pyplot as plt

import numpy as np
import matplotlib.pyplot as plt

def traj_euclidean(x_init, y_init, x_final, y_final, ds=0.01):
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

    # === PLOT: Dois gráficos lado a lado ===
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # 1. Trajetória no espaço cartesiano
    axes[0].plot(pos[:, 0], pos[:, 1], 'b-', linewidth=2, label='Trajetória XY')
    axes[0].scatter(pos[0, 0], pos[0, 1], color='green', label='Início')
    axes[0].scatter(pos[-1, 0], pos[-1, 1], color='red', label='Fim')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Trajetória do End-Effector')
    axes[0].axis('equal')
    axes[0].grid(True)
    axes[0].legend()

    # 2. Ângulos das juntas ao longo do tempo
    axes[1].plot(t, (q[:, 0]), label='Junta 1 (θ1)')
    axes[1].plot(t, (q[:, 1]), label='Junta 2 (θ2)')
    axes[1].set_xlabel('Tempo (s)')
    axes[1].set_ylabel('Ângulo (rad)')
    axes[1].set_title('Movimento das Juntas')
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    plt.show()

    return q


def main():
	# Exemplo de uso da função traj_euclidean
    q_traj = traj_euclidean(0.1, 0.1, 1, 1, ds=0.01)
    # print(q_traj)

if __name__ == "__main__":
    main()
    