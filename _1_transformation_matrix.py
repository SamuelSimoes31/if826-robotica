import numpy as np
import math as mt
from utils import pretty_matrix_multiplication

def SE2_xy(Dx,Dy):
    return np.array([
        [1, 0, Dx],
        [0, 1, Dy],
        [0, 0, 1]
    ])

def SE2_theta(DthetaDeg):
    DthetaRad = np.deg2rad(DthetaDeg)
    cos = mt.cos(DthetaRad)
    sin = mt.sin(DthetaRad)
    return np.array([
        [cos, -sin, 0],
        [sin, cos, 0],
        [0, 0, 1]
    ])


def point(x,y):
    return np.array([[x],[y],[1]])

def translation(xi, yi, xf, yf):
    H = SE2_xy(xf - xi, yf - yi)
    P = point(0.5, 0.5)
    return H @ P

def translation_example():
    H12 = SE2_xy(1, 0.25)
    P1R2 = point(0.5, 0.5)
    P1R1 = H12 @ P1R2
    pretty_matrix_multiplication(P1R1, H12, P1R2)


    H21 = np.linalg.inv(H12)
    P2R1 = point(0.5, 0.5)
    P2R2 = H21 @ P2R1
    pretty_matrix_multiplication(P2R2, H21, P2R1)


def rotation_example():
    H12 = SE2_xy(1, 0.25) @ SE2_theta(45)
    P3R2 = point(0.5, 0.5)
    P3R1 = H12 @ P3R2
    pretty_matrix_multiplication(P3R1, H12, P3R2)

    H21 = np.linalg.inv(H12)
    P4R1 = point(0.5, 0.5)
    P4R2 = H21 @ P4R1
    pretty_matrix_multiplication(P4R2, H21, P4R1)


def main():
    translation_example()
    rotation_example()

if __name__ == '__main__':
    main()