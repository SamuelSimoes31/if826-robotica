import numpy as np
import math as mt
from atv1_sssf2_tcsr import *
from utils import plotar_series_temporais_completo
a1 = a2 = 1
vmax = 10  # graus/s
amax = 10  # graus/s^2

# Jacobiano inverso (com verificação de singularidade)
def inverse_jacobian(q1, q2):
    sin_q2 = mt.sin(q2)
    if abs(sin_q2) < 1e-6:
        raise ValueError("Jacobian is singular (q2 ≈ 0), cannot invert.")

    denom = a1 * a2 * sin_q2
    return (1 / denom) * np.array(
        [
            [a2 * mt.cos(q1 + q2), a2 * mt.sin(q1 + q2)],
            [
                -a1 * mt.cos(q1) - a2 * mt.cos(q1 + q2),
                -a1 * mt.sin(q1) - a2 * mt.sin(q1 + q2),
            ],
        ]
    )


def calculate_trajectory_params(init, final, v_max, a_max):
    s_total = abs(final - init)

    t_acc_ideal = v_max / a_max
    s_acc_ideal = 0.5 * a_max * t_acc_ideal**2

    # Caso 1: Perfil Trapezoidal (atinge v_max)
    if 2 * s_acc_ideal <= s_total:
        t_acc = t_acc_ideal
        s_acc = s_acc_ideal
        s_const = s_total - 2 * s_acc
        t_const = s_const / v_max
        v_peak = v_max
    # Caso 2: Perfil Triangular (não atinge v_max)
    else:
        t_acc = (s_total / a_max) ** 0.5
        s_acc = 0.5 * a_max * t_acc**2
        t_const = 0
        s_const = 0
        v_peak = a_max * t_acc

    t_total = 2 * t_acc + t_const

    return {
        "t_acc": t_acc,
        "s_acc": s_acc,
        "t_const": t_const,
        "s_const": s_const,
        "t_total": t_total,
        "s_total": s_total,
        "v_peak": v_peak,
    }


def traj_joint_single_axis_sync(params, a_max, t_sync):
    # Resolve v_peak^2 - v_peak*a*t_sync + a*s_total = 0
    discriminant = (a_max * t_sync) ** 2 - 4 * a_max * params["s_total"]
    v_peak_adj = (a_max * t_sync - np.sqrt(discriminant)) / 2.0

    t_acc_adj = v_peak_adj / a_max
    t_const_adj = t_sync - 2 * t_acc_adj
    params["t_acc"] = t_acc_adj
    params["t_const"] = t_const_adj
    params["v_peak"] = v_peak_adj

    return params

# Função principal da trajetória
def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final, ts=0.01):
    q_init = np.array([theta1_init, theta2_init])
    q_final = np.array([theta1_final, theta2_final])
    vmax_j = np.array([vmax, vmax])
    amax_j = np.array([amax, amax])

    # ângulo está aumentando ou diminuindo?
    directions = np.sign(q_final - q_init)

    # 1. Calcular perfis individuais para cada junta
    params1 = calculate_trajectory_params(q_init[0], q_final[0], vmax_j[0], amax_j[0])
    params2 = calculate_trajectory_params(q_init[1], q_final[1], vmax_j[1], amax_j[1])

    # 2. Encontrar o tempo de sincronização (a junta limitante)
    t_sync = max(params1["t_total"], params2["t_total"])

    if t_sync == 0:  # Se não há movimento
        return np.array([q_init]), np.array([[0, 0]]), np.array([[0, 0]]), np.array([0])

    # 3. Recalcular os parâmetros da(s) junta(s) mais rápida(s) para que durem t_sync
    all_params = [params1, params2]
    adjusted_params = []
    for i in range(2):
        if all_params[i]["t_total"] < t_sync:
            # Esta junta precisa ser desacelerada. Recalculamos sua v_peak.
            adjusted_params.append(
                traj_joint_single_axis_sync(all_params[i], amax_j[i], t_sync)
            )
        else:
            # Esta é a junta limitante, usamos seus parâmetros originais
            adjusted_params.append(all_params[i])

    # 4. Gerar os pontos da trajetória com os parâmetros sincronizados
    time_points = np.arange(0, t_sync, ts)
    q_traj = []

    for t in time_points:
        q_t = []
        for i in range(2):
            p = adjusted_params[i]
            direction = directions[i]
            q0 = q_init[i]
            a_max = amax_j[i]

            t_acc = p["t_acc"]
            t_const = p["t_const"]
            v_peak = p["v_peak"]

            # Fase 1: Aceleração
            if t <= t_acc:
                accel = a_max * direction
                vel = accel * t
                pos = q0 + 0.5 * accel * t**2
            # Fase 3: Desaceleração
            elif t > t_acc + t_const:
                accel = -a_max * direction
                t_in_phase = t - (t_acc + t_const)
                # Posição e velocidade no início da desaceleração
                pos_start_d = q0 + direction * (
                    0.5 * a_max * t_acc**2 + v_peak * t_const
                )
                vel_start_d = v_peak * direction
                vel = vel_start_d + accel * t_in_phase
                pos = (
                    pos_start_d
                    + (vel_start_d * t_in_phase)
                    + (0.5 * accel * t_in_phase**2)
                )
            # Fase 2: Velocidade Constante
            else:
                accel = 0
                vel = v_peak * direction
                t_in_phase = t - t_acc
                pos_start_c = q0 + direction * (0.5 * a_max * t_acc**2)
                pos = pos_start_c + vel * t_in_phase

            q_t.append(pos)
        q_traj.append(q_t)
    # Adicionar o ponto final para garantir precisão
    q_traj.append(q_final.tolist())

    q_traj = np.array(q_traj)
    t_all = np.arange(len(q_traj)) * ts

    q_dot = np.zeros_like(q_traj)
    q_dot[1:] = (q_traj[1:] - q_traj[:-1]) / ts

    q_ddot = np.zeros_like(q_traj)
    q_ddot[2:] = (q_dot[2:] - q_dot[1:-1]) / ts
    plotar_series_temporais_completo(q_traj, q_dot, q_ddot, t_all)

    return q_traj, q_dot, q_ddot, t_all


def main():
	theta1 = np.radians([0, -45])
	theta2 = np.radians([0, -90])
	q_traj, q_dot, q_ddot, t_all = traj_joint(theta1[0], theta2[0], theta1[1], theta2[1], ts=0.01)
	
	# Visualize the trajectory
	# print(q_traj)
	plotar_series_temporais_completo(q_traj, q_dot, q_ddot, t_all)

if __name__ == "__main__":
	main()
