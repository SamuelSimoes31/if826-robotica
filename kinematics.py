import math as mt
from atv1_sssf2_tcsr import *
from utils import plotar_series_temporais_completo, plotar_series_temporais

a1 = a2 = 1
ts = 0.01
vmax = 75  # graus/s
amax = 100  # graus/s^2


def fk(theta1, theta2):
    E1 = SE2_theta(mt.degrees(theta1)) @ SE2_xy(a1, 0)
    E2 = SE2_theta(mt.degrees(theta2)) @ SE2_xy(a2, 0)
    E = E1 @ E2

    x = E[0, 2]
    y = E[1, 2]
    theta = mt.atan2(y, x)
    return np.array([x, y])
    # return x , y, mt.degrees(theta)


def ik(x, y):
    xy_squared = x**2 + y**2
    R_squared = (a1 + a2) ** 2

    # se o ponto está dentro da área de atuação
    if xy_squared > R_squared:
        return None

    # se ele tiver exatamente na borda da área de atuação (1 resposta)
    if xy_squared == R_squared:
        theta1 = mt.atan2(y, x)
        theta2 = 0.0
        return [(theta1, theta2)]

    # se ele está dentro da da borda (2 respostas)
    if xy_squared < R_squared:
        theta2_a = mt.acos((x**2 + y**2 - a1**2 - a2**2) / (2 * a1 * a2))
        theta1_a = mt.atan2(y, x) - mt.atan2(
            a2 * mt.sin(theta2_a), a1 + a2 * mt.cos(theta2_a)
        )

        theta2_b = -theta2_a
        theta1_b = mt.atan2(y, x) - mt.atan2(
            a2 * mt.sin(theta2_b), a1 + a2 * mt.cos(theta2_b)
        )
        return [(theta1_a, theta2_a), (theta1_b, theta2_b)]


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


# angulos em radianos
def traj_joint_single_axis_params(theta_init, theta_final, v_max, a_max):
    s_total = abs(theta_final - theta_init)
    # s_total = mt.radians(theta_final - theta_init)
    # v_max = mt.radians(v_max)
    # a_max = mt.radians(a_max)

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
def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final):
    q_init = np.radians([theta1_init, theta2_init])
    q_final = np.radians([theta1_final, theta2_final])
    vmax_j = np.radians([vmax, vmax])
    amax_j = np.radians([amax, amax])

    # ângulo está aumentando ou diminuindo?
    directions = np.sign(q_final - q_init)

    # 1. Calcular perfis individuais para cada junta
    params1 = traj_joint_single_axis_params(q_init[0], q_final[0], vmax_j[0], amax_j[0])
    params2 = traj_joint_single_axis_params(q_init[1], q_final[1], vmax_j[1], amax_j[1])

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
    q_traj, v_traj, a_traj = [], [], []

    for t in time_points:
        q_t, v_t, a_t = [], [], []
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
            v_t.append(vel)
            a_t.append(accel)

        q_traj.append(q_t)
        v_traj.append(v_t)
        a_traj.append(a_t)

    # Adicionar o ponto final para garantir precisão
    q_traj.append(q_final.tolist())
    v_traj.append([0.0, 0.0])
    a_traj.append([0.0, 0.0])
    time_points = np.append(time_points, t_sync)

    return np.array(q_traj), np.array(v_traj), np.array(a_traj), time_points


def main():
    q_traj, v_traj, a_traj, t_traj = traj_joint(300, 0, 30, 1)

    print(f"Movimento concluído em {t_traj[-1]:.2f} segundos.")
    plotar_series_temporais_completo(q_traj, v_traj, a_traj, t_traj)


if __name__ == "__main__":
    main()

