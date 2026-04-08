# Inverted Pendulum on Cart - State Space + LQR Control

Graph

<img width="1497" height="835" alt="image" src="https://github.com/user-attachments/assets/a0f10987-1e91-40e1-bda3-ed7658f11de4" />


A complete **modern control systems** project demonstrating **state-space modeling**, stability analysis, and **optimal control using LQR** on the classic **Inverted Pendulum** system.

Built as part of Robotics & AI coursework at Thapar Institute.

---

## ✨ Features

- Full **linearized state-space model** (4 states)
- Stability analysis (eigenvalue/pole analysis)
- **LQR (Linear Quadratic Regulator)** optimal controller
- Professional comparison between **Open-loop (Unstable)** vs **Closed-loop (Stabilized)**
- **Live real-time animation** showing cart and pendulum dynamics
- Clean, well-commented code in **Python** and **MATLAB**
- Ready for academic reports, portfolio, and placements

---

## 🎯 Project Overview

The Inverted Pendulum is a classic **unstable** benchmark problem in control theory.  
Keeping the pendulum upright while controlling the cart position is highly challenging due to its unstable nature.

This project:
- Derives the **state-space representation**
- Proves the open-loop system is **unstable**
- Designs an **LQR controller** to stabilize it optimally
- Visualizes the dramatic difference using live animation

**Applications**: Self-balancing robots (Segway, two-wheel robots), rocket stabilization, humanoid balance control.

---


## 🛠️ Technologies & Concepts Used

- **Python 3.10+** with `control`, `matplotlib`, `numpy`
- **MATLAB** + Control System Toolbox
- State-Space Modeling
- Eigenvalue Analysis (Stability)
- **LQR Optimal Control**
- Real-time Animation using Matplotlib



Key Results

Open-loop poles: One positive real pole → Unstable
LQR Gain (K): Automatically computed optimal feedback gain
With LQR, the pendulum stabilizes within ~3–4 seconds even from an initial tilt.

Tuning Parameters:

State cost matrix Q = diag([10, 1, 100, 1])
Control cost R = 1
