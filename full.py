def traj_joint_full_profile(theta1_init, theta2_init, theta1_final, theta2_final):
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
