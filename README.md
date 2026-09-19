# QTree-RRT: Improving RRT through Quadtree-Guided Sampling for Path Planning

[![Python](https://img.shields.io/badge/Python-3.8%2B-blue.svg?logo=python\&logoColor=white)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## About the Project

This repository presents the implementation of **QTree-RRT**, a hybrid approach for **path planning in two-dimensional environments with obstacles**.

The method combines **Quadtree-based spatial decomposition** with planning based on the **Rapidly-exploring Random Tree (RRT)**. Initially, the environment is decomposed into regions using a Quadtree, allowing the identification of a sequence of free regions connecting the starting point to the goal. This sequence forms a **macro navigation corridor**, which is subsequently used to guide sampling and trajectory refinement.

The implementation was developed in **Python** and uses grayscale maps as the representation of the planning environments.


**Paper:** [QTree-RRT: Improving RRT Through Quadtree-Guided Sampling for Path Planning](https://ieeexplore.ieee.org/abstract/document/11249615/)
---

## Repository Structure

```
QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning/
│
├── algorithms/
│   └── qtree-rrt.py
│
├── SMPP dataset/
│   ├── mapa01.png
│   ├── mapa02.png
│   ├── ...
│   └── mapa50.png
│
├── LICENSE
└── README.md
```

* **`algorithms/`** — QTree-RRT implementation.
* **`SMPP dataset/`** — dataset containing 50 PNG maps used in the experiments.
* **`LICENSE`** — MIT license for the project.

---

## Requirements

* Python **3.8 or higher**
* NumPy
* Pillow

To check the installed Python version:

```
python3 --version
```

---

## Installation

### 1. Clone the repository

```
git clone https://github.com/YOUR-USERNAME/QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning.git

cd QTree-RRT-Improving-RRT-through-Quadtree-Guided-Sampling-for-Path-Planning
```

### 2. Create a virtual environment

```
python3 -m venv venv
```

On Linux/macOS:

```
source venv/bin/activate
```

On Windows:

```
venv\Scripts\activate
```

### 3. Install the dependencies

```
pip install numpy pillow
```

---

## Execution

The implementation can be executed directly using the main script:

```
python3 algorithms/qtree-rrt.py
```

The algorithm processes the maps available in the `SMPP dataset/` directory according to the parameters defined in the code.

---

## Metrics

During execution, metrics related to the planning structure and the resulting trajectory can be obtained:

| Metric             | Description                                               |
| ------------------ | --------------------------------------------------------- |
| **Vertices**       | Number of points incorporated into the planning structure |
| **Edges**          | Number of connections established between vertices        |
| **Execution Time** | Time required to perform the planning                     |
| **Route Cost**     | Length or accumulated cost of the resulting trajectory    |
| **Waypoints**      | Number of intermediate points used in the trajectory      |

These metrics can be used for the experimental analysis of the method's behavior across different environments.

---

## SMPP dataset

The `SMPP dataset/` directory contains the **SMPP (Synthetic Maps for Path Planning)** dataset, consisting of **50 grayscale maps** used as planning environments.

The map representation considers:

* **White:** free space;
* **Black:** obstacles.

The maps are sequentially identified from `mapa01.png` to `mapa50.png`.

---

## Technologies

* **Python** — algorithm implementation;
* **NumPy** — numerical operations;
* **Pillow** — map reading and processing;
* **Quadtree** — spatial decomposition;
* **RRT** — sampling-based planning.

---

## Citation

Use the following bibtex code to cite our QTree-RRT algorithm and our SMPP dataset.

```bibtex
@INPROCEEDINGS{11249615,
  author={Bianor, Jhorlen Souza and de Abreu Dias, Lucas Matos and Drews-Jr, Paulo L. J. and Tello Gamarra, Daniel Fernando and Cukla, Anselmo Rafael and de Oliveira, Felipe Gomes},
  booktitle={2025 Brazilian Symposium on Robotics (SBR) and 2025 Workshop on Robotics in Education (WRE)}, 
  title={QTree-RRT: Improving RRT Through Quadtree-Guided Sampling for Path Planning}, 
  year={2025},
  volume={},
  number={},
  pages={170-175},
  keywords={Costs;Navigation;Trees (botanical);Conferences;Education;Focusing;Path planning;Mobile robots;Reliability;Autonomous vehicles;Path Planning;RRT;Quadtree;Autonomous Navigation;Mobile Robotics},
  doi={10.1109/SBR/WRE66973.2025.11249615}
}
```

---

## License

This project is available under the **MIT License**.

See the [`LICENSE`](LICENSE) file for the complete license terms.
