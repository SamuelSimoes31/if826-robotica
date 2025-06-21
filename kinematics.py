import math as mt
from atv1_sssf2_tcsr import *
from utils import imprimir_simples, plotar_series_temporais
a1 = a2 = 1
vmax = 0.2
amax = 0.1
ts = 0.01
v_modulo = 0.1

def fk(theta1,theta2):
    E1 = SE2_theta(mt.degrees(theta1)) @ SE2_xy(a1, 0)
    E2 = SE2_theta(mt.degrees(theta2)) @ SE2_xy(a2, 0)
    E = E1 @ E2

    x = E[0,2]
    y = E[1,2]
    theta = mt.atan2(y, x)
    return np.array([x, y])
    # return x , y, mt.degrees(theta)

def ik(x,y):
    xy_squared = x**2 + y**2
    R_squared = (a1 + a2)**2

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
        theta1_a = mt.atan2(y, x) - mt.atan2(a2 * mt.sin(theta2_a), a1 + a2 * mt.cos(theta2_a))

        theta2_b = -theta2_a
        theta1_b = mt.atan2(y, x) - mt.atan2(a2 * mt.sin(theta2_b), a1 + a2 * mt.cos(theta2_b))
        return [(theta1_a, theta2_a), (theta1_b, theta2_b)]

# Jacobiano inverso (com verificação de singularidade)
def inverse_jacobian(q1, q2):
    sin_q2 = mt.sin(q2)
    if abs(sin_q2) < 1e-6:
        raise ValueError("Jacobian is singular (q2 ≈ 0), cannot invert.")
    
    denom = a1 * a2 * sin_q2
    return (1 / denom) * np.array([
        [a2 * mt.cos(q1 + q2), a2 * mt.sin(q1 + q2)],
        [-a1 * mt.cos(q1) - a2 * mt.cos(q1 + q2), -a1 * mt.sin(q1) - a2 * mt.sin(q1 + q2)]
    ])

# Função principal da trajetória
def traj_joint(theta1_init, theta2_init, theta1_final, theta2_final):
    # Conversão para radianos
    theta1_init = mt.radians(theta1_init)
    theta2_init = mt.radians(theta2_init)
    theta1_final = mt.radians(theta1_final)
    theta2_final = mt.radians(theta2_final)

    # Inicialização
    q = np.array([[theta1_init, theta2_init]])
    q_dot_list = []
    q_ddot_list = []

    # Trajetória cartesiana (reta no espaço)
    p_start = fk(theta1_init, theta2_init)
    p_end = fk(theta1_final, theta2_final)
    s_total = np.linalg.norm(p_end - p_start)

    # Perfil trapezoidal
    t_acc = vmax / amax
    s_acc = 0.5 * amax * t_acc**2

    if 2 * s_acc >= s_total:
        # Perfil triangular
        t_acc = (s_total / amax)**0.5
        t_const = 0
        v_peak = amax * t_acc
    else:
        # Perfil trapezoidal
        s_const = s_total - 2 * s_acc
        t_const = s_const / vmax
        v_peak = vmax

    t_total = 2 * t_acc + t_const
    t = 0
    s_traj = 0

    while q[-1][0] < theta1_final and t < t_total + ts:
        q1, q2 = q[-1]

        # Posição atual
        p1 = fk(q1, q2)
        p2 = fk(q1 + 0.001, q2)
        v_unit = (p2 - p1)
        v_unit /= np.linalg.norm(v_unit)

        # Perfil trapezoidal: calcula v_modulo e a_modulo
        if t < t_acc:
            v_modulo = amax * t
            a_modulo = amax
        elif t < t_acc + t_const:
            v_modulo = v_peak
            a_modulo = 0
        elif t < t_total:
            v_modulo = v_peak - amax * (t - t_acc - t_const)
            a_modulo = -amax
        else:
            v_modulo = 0
            a_modulo = 0

        v = v_unit * v_modulo
        a = v_unit * a_modulo

        try:
            J_inv = inverse_jacobian(q1, q2)
        except ValueError as e:
            print("Erro no Jacobiano:", e)
            break

        q_dot = J_inv @ v
        q_ddot = J_inv @ a

        q_dot_list.append(q_dot)
        q_ddot_list.append(q_ddot)

        # Integração
        next_q = q[-1] + q_dot * ts
        q = np.vstack((q, next_q))

        s_traj += v_modulo * ts
        t += ts

    return q, np.array(q_dot_list), np.array(q_ddot_list)

def main():
    q, q_dot_list, q_ddot_list = traj_joint(0, 50, 120, 90)
    q_deg = np.rad2deg(q)

    plotar_series_temporais(q_deg)
    plotar_series_temporais(q_dot_list)
    plotar_series_temporais(q_ddot_list)

if __name__ == '__main__':
    main()


# ele consegue chegar na velcodiade maxima?

# tempo necessario pra chegar na vel max
# distancia perc nesse tempo.

# enviar a cada ts a posição q ele precisa atingir

# os dois acabar no mesmo momento
# uma solução pro 1 motor, 1 solução pro 2º motor. Pegar o mais lento e adapatar o mais rpaido pra ficar mias lento.
# Pra isso, via mudar a velocidade máxima pra ser mais baixa.

# 1dof_traj(theta, theta, vmax, amax, ts)

# dof_time(ts)