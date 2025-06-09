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
    # se o ponto está dentro da área de atuação
    # se ele tiver exatamente na borda da área de atuação (1 resposta)
    # se ele está dentro da da borda (2 respostas)

    return theta1, theta2

def main():
    print(fk(0,0))
    print(fk(mt.pi/2, 0))
    print(fk(0,mt.pi/2))
    print(fk(mt.pi/2,mt.pi/2))

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