# Projeto de Robótica - IF826

Este repositório contém o projeto desenvolvido para a disciplina de Robótica (IF826). O projeto implementa conceitos fundamentais de robótica, incluindo matrizes de transformação, cinemática e planejamento de trajetórias.

## Alunos

[Samuel Simões](https://github.com/SamuelSimoes31) (sssf2)

[Thallys Christ](https://github.com/ThallysRocha) (tcsr)

## Estrutura do Projeto

A estrutura de arquivos segue a numeração dos requisitos do projeto:

- `_1_transformation_matrix.py` - (Questão 1) Implementação de matrizes de transformação
- `_2_kinematics.py` - (Questão 2) Implementação da cinemática do robô
- `_3_traj_eucl.py` - (Questão 3) Planejamento de trajetória no espaço euclidiano
- `_3_traj_joint.py` - (Questão 3) Planejamento de trajetória no espaço das juntas
- `main.py` - Arquivo principal para testes e simulações
- `utils.py` - Funções utilitárias para visualização e plotagem
- `requirements.txt` - Dependências do projeto

## Instalação e Configuração

### Pré-requisitos
- Python 3.8 ou superior
- pip (gerenciador de pacotes Python)

### Configuração do Ambiente Virtual

1. **Clone o repositório:**
   ```bash
   git clone https://github.com/SamuelSimoes31/if826-robotica
   cd if826-robotica
   ```

2. **Crie um ambiente virtual:**
   ```bash
   python -m venv venv
   ```

3. **Ative o ambiente virtual:**
   
   **No Windows:**
   ```bash
   venv\Scripts\activate
   ```
   
   **No Linux/Mac:**
   ```bash
   source venv/bin/activate
   ```

4. **Instale as dependências:**
   ```bash
   pip install -r requirements.txt
   ```

## Uso do Projeto

### Executando Simulações

O arquivo `main.py` é o ponto de entrada principal para executar testes e simulações:

```bash
python main.py
```

### Testando

Mude o arquivo main.py conforme quiser alterando ângulos e comentando o que não desejar executar no momento.

### Visualização dos Resultados

Todos os gráficos e simulações são salvos automaticamente como imagens na pasta raiz do projeto:
- `cartesiano_eucl.png` - Gráfico de XY e Juntas x tempo
- `cartesiano_eucl.gif` - Animação da trajetória no espaço euclidiano
- `cartesiano_joint.png` - Perfil de posição, velocidade e aceleração das juntas x tempo
- `cartesiano_joint.gif` - Animação da trajetória no espaço das juntas

## Dependências Principais

- **roboticstoolbox-python**: Biblioteca para simulação de robôs
- **matplotlib**: Visualização de gráficos e animações
- **numpy**: Computação numérica


## Notas Importantes

- Sempre ative o ambiente virtual antes de executar o projeto
- As animações são salvas como arquivos GIF na pasta raiz
- Os gráficos são salvos como PNG na pasta raiz
- O robô utilizado é um robô planar RR (2 graus de liberdade)
- Todos os ângulos são especificados em radianos mas os gráficos estão em graus para melhor entendimento
