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

def main():
    print(fk(0,0))
    print(fk(mt.pi/2, 0))
    print(fk(0,mt.pi/2))
    print(fk(mt.pi/2,mt.pi/2))

if __name__ == '__main__':
    main()