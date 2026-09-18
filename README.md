# QTree-RRT: Improving RRT through Quadtree-Guided Sampling for Path Planning

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Sobre o Projeto

Este repositório apresenta a implementação do **QTree-RRT**, uma abordagem híbrida para **planejamento de caminhos em ambientes bidimensionais com obstáculos**.

O método combina a **decomposição espacial por Quadtree** com o planejamento baseado em **Rapidly-exploring Random Tree (RRT)**. Inicialmente, o ambiente é decomposto em regiões por meio de uma Quadtree, permitindo identificar uma sequência de regiões livres que conecta o ponto inicial ao objetivo. Essa sequência forma um **corredor macro de navegação**, utilizado posteriormente para orientar a amostragem e o refinamento da trajetória.

A implementação foi desenvolvida em **Python** e utiliza mapas em escala de cinza como representação dos ambientes de planejamento.

---

## Estrutura do Repositório

    QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning/
    │
    ├── algorithms/
    │   └── qtree-rrt.py
    │
    ├── dataset/
    │   ├── mapa01.png
    │   ├── mapa02.png
    │   ├── ...
    │   └── mapa50.png
    │
    ├── LICENSE
    └── README.md

- **`algorithms/`** — implementação do QTree-RRT.
- **`dataset/`** — conjunto de 50 mapas PNG utilizados nos experimentos.
- **`LICENSE`** — licença MIT do projeto.

---

## Pré-requisitos

- Python **3.8 ou superior**
- NumPy
- Pillow

Para verificar a versão instalada do Python:

    python3 --version

---

## Instalação

### 1. Clonar o repositório

    git clone https://github.com/SEU-USUARIO/QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning.git

    cd QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning

### 2. Criar um ambiente virtual

    python3 -m venv venv

No Linux/macOS:

    source venv/bin/activate

No Windows:

    venv\Scripts\activate

### 3. Instalar as dependências

    pip install numpy pillow

---

## Execução

A implementação pode ser executada diretamente pelo script principal:

    python3 algorithms/qtree-rrt.py

O algoritmo processa os mapas disponíveis no diretório `dataset/` conforme os parâmetros definidos no código.

---

## Métricas

Durante a execução, podem ser obtidas métricas relacionadas à estrutura de planejamento e à trajetória encontrada:

| Métrica | Descrição |
|---|---|
| **Vértices** | Quantidade de pontos incorporados à estrutura de planejamento |
| **Arestas** | Quantidade de conexões estabelecidas entre os vértices |
| **Tempo de Execução** | Tempo necessário para realizar o planejamento |
| **Custo da Rota** | Comprimento ou custo acumulado da trajetória encontrada |
| **Waypoints** | Quantidade de pontos intermediários utilizados na trajetória |

Essas métricas podem ser utilizadas para a análise experimental do comportamento do método em diferentes ambientes.

---

## Mapas

O diretório `dataset/` contém **50 mapas em escala de cinza** utilizados como ambientes de planejamento.

A representação dos mapas considera:

- **Branco:** espaço livre;
- **Preto:** obstáculos.

Os mapas são identificados sequencialmente de `mapa01.png` a `mapa50.png`.

---

## Tecnologias

- **Python** — implementação do algoritmo;
- **NumPy** — operações numéricas;
- **Pillow** — leitura e processamento dos mapas;
- **Quadtree** — decomposição espacial;
- **RRT** — planejamento baseado em amostragem.

---

## Licença

Este projeto está disponível sob a **Licença MIT**.

Consulte o arquivo [`LICENSE`](LICENSE) para obter os termos completos da licença.
