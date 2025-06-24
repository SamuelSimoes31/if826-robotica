import math as mt
from atv1_sssf2_tcsr import *
from utils import plotar_series_temporais_completo, plotar_series_temporais

a1 = a2 = 1
ts = 0.01
vmax = 75  # graus/s
amax = 100  # graus/s^2

vmax_euclidean = 2.5 # m/s
amax_euclidean = 12.5 # m/s^2

vmax = mt.radians(vmax)
amax = mt.radians(amax)

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
        return [[theta1, theta2]]

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
        return [[theta1_a, theta2_a], [theta1_b, theta2_b]]


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
def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final):
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

    return np.array(q_traj)

def traj_euclidean(x_init, y_init, x_final, y_final):
    s_total = np.sqrt((x_final - x_init)**2 + (y_final - y_init)**2)
    direction = np.array([x_final - x_init, y_final - y_init])
    direction = direction / s_total
    q_init = ik(x_init, y_init)
    if q_init is None:
        raise ValueError("Ponto inicial fora da área de atuação")
    q_final = ik(x_final, y_final)
    if q_final is None:
        print("Ponto final fora da área de atuação")
    q_init = q_init[0] # primeira solução
    q_final = q_final[0] # primeira solução

    params = calculate_trajectory_params(0, s_total, vmax_euclidean, amax_euclidean)
    t_acc = params["t_acc"]
    t_const = params["t_const"]
    v_peak = params["v_peak"]
    a_max = amax_euclidean
    
    print(params["t_total"])
    

    time_points = np.arange(0, params["t_total"], ts)
    q_traj = [[q_init[0], q_init[1]]]


    for t in time_points:
        # Fase 1: Aceleração
        if t <= t_acc:
            accel = a_max
            vel_mag = accel * t
            vel_j = vel_mag * direction
        # Fase 3: Desaceleração
        elif t > t_acc + t_const:
            accel = -a_max
            t_in_phase = t - (t_acc + t_const)
            # Posição e velocidade no início da desaceleração
            vel_mag = accel * t_in_phase
            vel_j = vel_mag * direction
        # Fase 2: Velocidade Constante
        else:
            accel = 0
            vel_mag = v_peak
            vel_j = vel_mag * direction

        inv_j = inverse_jacobian(q_traj[-1][0], q_traj[-1][1])
        q_dot = inv_j @ vel_j
        q = q_traj[-1] + q_dot * ts
        q_traj.append(q)

    return np.array(q_traj)

def main():
    q_traj = traj_euclidean(0.2, 0, 0.8, 0)
    # print(q_traj)

if __name__ == "__main__":
    main()

