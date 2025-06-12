import math as mt
from atv1_sssf2_tcsr import *
a1 = a2 = 1

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

def main():
    print(fk(0,0))
    print(fk(mt.pi/2, 0))
    print(fk(0,mt.pi/2))
    print(fk(mt.pi/2,mt.pi/2))

    test_points = [
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
        (2, 1),
        (2, 0),
        (0, 2),
        (-2, 0)
    ]

    print("Testing ik(x, y) for various points:")
    for x, y in test_points:
        result = ik(x, y)
        print(f"ik({x}, {y}) = {result}")

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