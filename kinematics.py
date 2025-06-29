import math as mt
from _1_transformation_matrix import *
a1 = a2 = 1

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
