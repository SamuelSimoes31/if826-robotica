import math as mt
from atv1_sssf2_tcsr import *
from utils import imprimir_simples, plotar_series_temporais
a1 = a2 = 1
vmax = 1
amax = 0.01
ts = 0.01

def fk(theta1,theta2):
    E1 = SE2_theta(mt.degrees(theta1)) @ SE2_xy(a1, 0)
    E2 = SE2_theta(mt.degrees(theta2)) @ SE2_xy(a2, 0)
    E = E1 @ E2

    x = E[0,2]
    y = E[1,2]
    theta = mt.atan2(y, x)
    return x , y, mt.degrees(theta)

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

def inverse_jacobian(q1,q2):
    return 1/(a1*a2*mt.sin(q2)) * np.array([[a2*mt.cos(q1 + q2), a2*mt.sin(q1 + q2)],
                                            [-a1*mt.cos(q1) - a2*mt.cos(q1 + q2), -a1*mt.sin(q1) - a2*mt.sin(q1 + q2)]])

def traj_joint(theta1_init,theta2_init, theta1_final,theta2_final):
    theta1_init = mt.radians(theta1_init)
    theta2_init = mt.radians(theta2_init)
    theta1_final = mt.radians(theta1_final)
    theta2_final = mt.radians(theta2_final)

    v = np.array([0 , 0])
    v_max = np.array([vmax , vmax])
    a = np.array([amax , amax])
    q = np.array([[theta1_init,theta2_init]])

    # fase 1: aceleração
    while q[-1][0] < theta1_final and v[0] < v_max[0]:
        J_inv = inverse_jacobian(q[-1][0], q[-1][1])
        q_dot = J_inv @ v
        next_q = q[-1] + q_dot*ts
        q = np.vstack((q, next_q))
        v = v + a*ts
    q_phase1 = q[-1].copy()
    v_max = np.array([vmax , vmax])

    # fase 2: velocidade constante
    while q[-1][0] < theta1_final and theta1_final - q[-1][0] < q_phase1[0]:
        J_inv = inverse_jacobian(q[-1][0], q[-1][1])
        q_dot = J_inv @ v_max
        next_q = q[-1] + q_dot*ts
        q = np.vstack((q, next_q))

    # fase 3: desaceleração
    while q[-1][0] < theta1_final:
        J_inv = inverse_jacobian(q[-1][0], q[-1][1])
        q_dot = J_inv @ v
        next_q = q[-1] + q_dot*ts
        q = np.vstack((q, next_q))
        v = v - a*ts

    return q #sequencia de theta1 e theta2

def main():
    # print(fk(0,0))
    # print(fk(mt.pi/2, 0))
    # print(fk(0,mt.pi/2))
    # print(fk(mt.pi/2,mt.pi/2))

    # test_points = [
    #     (1, 1),
    #     (1, -1),
    #     (-1, 1),
    #     (-1, -1),
    #     (2, 1),
    #     (2, 0),
    #     (0, 2),
    #     (-2, 0)
    # ]

    # print("Testing ik(x, y) for various points:")
    # for x, y in test_points:
    #     result = ik(x, y)
    #     print(f"ik({x}, {y}) = {result}")

    q = traj_joint(0, 90, 180, 0)
    q_deg = np.rad2deg(q)
    imprimir_simples(q_deg)
    plotar_series_temporais(q_deg)

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