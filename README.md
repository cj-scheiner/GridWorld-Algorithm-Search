# Path Finder in a Grid World (DFS, BFS, GBFS, A*)

## Overview

This project visualizes and animates four classical search algorithms operating in a **50×50 grid world**:

* Depth First Search (DFS)
* Breadth First Search (BFS)
* Greedy Best-First Search (GBFS)
* A* Search

The grid environment contains:

* **Enclosures** – impassable obstacles that block movement
* **Turfs** – tiles with a higher traversal cost

The program calculates paths from a source location to a destination while respecting these constraints and displays the resulting path through an animated visualization.

Users can choose between a **default grid world** provided for testing or a **custom-made world configuration**, and can either use preset coordinates or specify their own start and destination points.

---

# Algorithms Implemented

### Depth First Search (DFS)

Explores as far as possible along a branch before backtracking.

* Path cost based on **step count**
* Does not guarantee the shortest path.

### Breadth First Search (BFS)

Explores nodes level by level outward from the starting position.

* Path cost based on **step count**
* Guarantees the **shortest path in terms of number of steps** when no weighted costs are considered.

### Greedy Best-First Search (GBFS)

Selects the next node using a **heuristic estimate** of distance to the goal.

* Faster exploration toward the goal
* May not produce the optimal path.

### A* Search

Combines the **actual path cost** and the **heuristic estimate** to determine the best path.

* Uses weighted traversal costs (turfs)
* Typically finds the **optimal path efficiently**.

---

# Requirements

## Python Version

Python **3.11 or higher** is recommended.
(The project was developed using Python 3.11.)

## Required Libraries

* `matplotlib` – Used for visualization and grid rendering
* `numpy` – Used for mathematical calculations within the heuristic

Install dependencies using:

```bash
pip install matplotlib numpy
```

---

# Project File Structure

All files should be located within the **same root directory**.

```
project-folder/
│
├── search.py
├── grid.py
├── utils.py
│
└── TestingGrid/
    ├── world1_enclosures.txt
    ├── world1_turfs.txt
    ├── world2_enclosures.txt
    └── world2_turfs.txt
```

### File Descriptions

| File           | Purpose                                           |
| -------------- | ------------------------------------------------- |
| `search.py`    | Main program containing the search algorithms     |
| `grid.py`      | Utility module for rendering and drawing the grid |
| `utils.py`     | Utility data structures used by the algorithms    |
| `TestingGrid/` | Contains grid world definitions for testing       |

---

# How to Run the Program

### 1. Start the Program

Run the following command from a terminal:

```bash
python search.py
```

You can also run the file directly from an IDE.

---

### 2. Select a World

When prompted, choose a grid world:

* `1` – Default world (provided with the assignment)
* `2` – Custom-made world

---

### 3. Select Start and Destination Points

You can either:

* **Use default coordinates**

  * Source: `(8, 10)`
  * Destination: `(43, 45)`

* **Enter custom coordinates**

The program validates that:

* Coordinates are within the **0–49 grid range**
* The selected location **is not inside an enclosure**

---

### 4. Choose a Search Algorithm

Select one of the following options:

| Option | Algorithm |
| ------ | --------- |
| 1      | DFS       |
| 2      | BFS       |
| 3      | GBFS      |
| 4      | A*        |

---

### 5. View the Results

Once an algorithm is selected:

* The program calculates the path
* A **visual animation of the search process** appears
* The window title and graph header display:

  * Path taken
  * Total cost
  * Number of nodes expanded

The console output will also display:

* Path taken
* Path cost
* Nodes expanded
* Execution duration of the algorithm

---

# Summary

This project demonstrates how different search strategies behave in a constrained grid environment. By comparing DFS, BFS, GBFS, and A*, users can observe differences in:

* Search efficiency
* Path optimality
* Node expansion behavior
* Computational performance
