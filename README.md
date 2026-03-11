# Path Finder in a Grid World

A Python program that visualizes four search algorithms on a 50x50 grid. The grid contains obstacles (enclosures) and weighted tiles (turfs), and the program animates the search process from a start point to a destination.

## Algorithms

- **DFS** — explores depth-first, does not guarantee shortest path
- **BFS** — explores level by level, shortest path by step count
- **Greedy Best-First** — uses a heuristic to move toward the goal, faster but not always optimal
- **A\*** — combines actual cost and heuristic, generally finds the optimal path

## Requirements

Python 3.11 or higher

```bash
pip install matplotlib numpy
```

## How to run

```bash
python search.py
```

You'll be prompted to:
1. Select a grid world (default or custom)
2. Enter start and destination coordinates, or use the defaults — `(8, 10)` to `(43, 45)`
3. Choose an algorithm

The program animates the search and prints the path, total cost, nodes expanded, and execution time to the console.

## Files

- `search.py` — main program and search algorithms
- `grid.py` — grid rendering
- `utils.py` — data structures used by the algorithms
- `TestingGrid/` — sample grid world definitions
