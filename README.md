# Dynamic Pathfinding Agent

This project implements a graphical pathfinding agent using informed search algorithms.
It allows users to visualize and compare A* Search and Greedy Best-First Search using
Manhattan and Euclidean heuristics in both static and dynamic environments.

---

## Features
- Grid-based GUI using Tkinter
- A* Search (f(n) = g(n) + h(n))
- Greedy Best-First Search (f(n) = h(n))
- Manhattan & Euclidean heuristics
- Dynamic obstacles with automatic replanning
- Performance metrics (nodes visited, path cost, execution time)

---

## Installation & Running

### Requirements
- Python 3.8+

### Run the Program
```bash
python src/pathfinding_gui.py
