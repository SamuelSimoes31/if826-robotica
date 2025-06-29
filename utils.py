import numpy as np
import matplotlib.pyplot as plt

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

def imprimir_simples(matriz: np.ndarray):
    for linha in matriz:
        print(f"{linha[0]:.2f}, {linha[1]:.2f}")

# --- Função para plotar o gráfico ---
def plotar_series_temporais(matriz: np.ndarray):
    """
    Plota as duas colunas da matriz como séries temporais separadas.
    O eixo X é o índice da linha (tempo 't').
    """
    # Pega o número de linhas (pontos no tempo)
    num_linhas = matriz.shape[0]

    # Cria o eixo do tempo 't' (ex: [0, 1, 2, ..., n-1])
    t = np.arange(num_linhas)

    # Extrai as colunas de dados
    dados_coluna_0 = matriz[:, 0]
    dados_coluna_1 = matriz[:, 1]

    # --- Cria a figura com 2 subplots (2 linhas, 1 coluna) ---
    # sharex=True faz com que os dois gráficos usem o mesmo eixo X (tempo)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    fig.suptitle('Dados da Matriz em Função do Tempo (t)', fontsize=16)

    # Gráfico 1: Coluna 0 vs. Tempo
    ax1.plot(t, dados_coluna_0, marker='o', color='royalblue')
    ax1.set_ylabel("Valores da Coluna 0")
    ax1.set_title("Série Temporal da Coluna 0")
    ax1.grid(True)

    # Gráfico 2: Coluna 1 vs. Tempo
    ax2.plot(t, dados_coluna_1, marker='o', color='seagreen')
    ax2.set_ylabel("Valores da Coluna 1")
    ax2.set_xlabel("Tempo (índice da linha 't')") # Label do eixo X fica no de baixo
    ax2.set_title("Série Temporal da Coluna 1")
    ax2.grid(True)

    # Ajusta o layout para evitar sobreposição de títulos
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('grafico_duplo.png')
    plt.show()


def plotar_series_temporais_completo(q, v, a, t):
    q_deg = np.rad2deg(q)
    v_deg = np.rad2deg(v)
    a_deg = np.rad2deg(a)

    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    fig.suptitle('Perfil da Trajetória Sincronizada no Espaço de Juntas', fontsize=16)

    axs[0].plot(t, q_deg[:, 0], label='Junta 1 (q1)')
    axs[0].plot(t, q_deg[:, 1], label='Junta 2 (q2)', linestyle='--')
    axs[0].set_ylabel('Posição (graus)')
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(t, v_deg[:, 0], label='Junta 1 (v1)')
    axs[1].plot(t, v_deg[:, 1], label='Junta 2 (v2)', linestyle='--')
    axs[1].set_ylabel('Velocidade (graus/s)')
    axs[1].legend()
    axs[1].grid(True)

    axs[2].plot(t, a_deg[:, 0], label='Junta 1 (a1)')
    axs[2].plot(t, a_deg[:, 1], label='Junta 2 (a2)', linestyle='--')
    axs[2].set_ylabel('Aceleração (graus/s²)')
    axs[2].set_xlabel('Tempo (s)')
    axs[2].legend()
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.savefig('cartesiano_joint.png')
    plt.show()

def plot_trajectory_euclidean(pos, q, t):
    """
    Plota a trajetória euclidiana com dois gráficos lado a lado:
    1. Trajetória no espaço cartesiano (XY)
    2. Ângulos das juntas ao longo do tempo

    Args:
        pos: array com posições [x, y] do end-effector
        q: array com ângulos das juntas [θ1, θ2]
        t: array com tempos
    """
    # === PLOT: Dois gráficos lado a lado ===
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # 1. Trajetória no espaço cartesiano
    axes[0].plot(pos[:, 0], pos[:, 1], 'b-', linewidth=2, label='Trajetória XY')
    axes[0].scatter(pos[0, 0], pos[0, 1], color='green', label='Início')
    axes[0].scatter(pos[-1, 0], pos[-1, 1], color='red', label='Fim')
    axes[0].set_xlabel('X (m)')
    axes[0].set_ylabel('Y (m)')
    axes[0].set_title('Trajetória do End-Effector')
    axes[0].axis('equal')
    axes[0].grid(True)
    axes[0].legend()

    # 2. Ângulos das juntas ao longo do tempo
    axes[1].plot(t, np.degrees(q[:, 0]), label='Junta 1 (θ1)')
    axes[1].plot(t, np.degrees(q[:, 1]), label='Junta 2 (θ2)')
    axes[1].set_xlabel('Tempo (s)')
    axes[1].set_ylabel('Ângulo (graus)')
    axes[1].set_title('Movimento das Juntas')
    axes[1].grid(True)
    axes[1].legend()

    plt.tight_layout()
    plt.savefig('cartesiano_eucl.png')
    plt.show()