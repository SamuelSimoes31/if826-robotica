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

# --- Função para imprimir do jeito que você quer ---
def imprimir_simples(matriz: np.ndarray):
    """
    Imprime a matriz no formato '0.00, 0.00', linha por linha.
    """
    for linha in matriz:
        # Formata cada número com 2 casas decimais (: .2f)
        # e une com ', '
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
    
    # Salva a imagem em um arquivo
    plt.savefig('grafico_duplo.png')
    
    # Exibe o gráfico
    plt.show()