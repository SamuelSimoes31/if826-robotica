import numpy as np
import math as mt

def pretty_matrix_multiplication(A, H, P):
    A = np.array(A).reshape(-1, 1)
    H = np.array(H)
    P = np.array(P).reshape(-1, 1)

    output = ""
    for i in range(H.shape[0]):
        line = ""

        # Matriz H
        line += "| " + " ".join(f"{H[i, j]:>6.3f}" for j in range(H.shape[1])) + " |"

        # Multiplicador
        line += "   *" if i == 1 else "    "

        # Matriz P
        line += "   | {:>6.3f} |".format(P[i, 0])

        # Igualdade e resultado
        line += "  =" if i == 1 else "   "
        line += "  | {:>7.3f} |".format(A[i, 0])

        output += line + "\n"

    print(output)



def SE2_xy(Dx,Dy):
    return np.array([
        [1, 0, Dx],
        [0, 1, Dy],
        [0, 0, 1]
    ])

def SE2_theta(DthetaDeg):
    DthetaRad = np.deg2rad(DthetaDeg)
    return np.array([
        [mt.cos(DthetaRad), -mt.sin(DthetaRad), 0],
        [mt.sin(DthetaRad), mt.cos(DthetaRad), 0],
        [0, 0, 1]
    ])


def point(x,y):
    return np.array([[x],[y],[1]])

def translation():
    H12 = SE2_xy(1, 0.25)
    P1R2 = point(0.5, 0.5)
    P1R1 = H12 @ P1R2
    pretty_matrix_multiplication(P1R1, H12, P1R2)


    H21 = np.linalg.inv(H12)
    P2R1 = point(0.5, 0.5)
    P2R2 = H21 @ P2R1
    pretty_matrix_multiplication(P2R2, H21, P2R1)


def rotation():
    H12 = SE2_xy(1, 0.25) @ SE2_theta(45)
    P3R2 = point(0.5, 0.5)
    P3R1 = H12 @ P3R2
    pretty_matrix_multiplication(P3R1, H12, P3R2)

    H21 = np.linalg.inv(H12)
    P4R1 = point(0.5, 0.5)
    P4R2 = H21 @ P4R1
    pretty_matrix_multiplication(P4R2, H21, P4R1)


def main():
    translation()
    rotation()

if __name__ == '__main__':
    main()