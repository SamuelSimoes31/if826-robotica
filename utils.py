import numpy as np

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