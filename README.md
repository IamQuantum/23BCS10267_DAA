
# Maze Solver Visualizer

This repository contains a simple interactive Maze Solver Visualizer implemented in Python using Pygame. The visualizer lets you draw walls, set a start and end node, then run common pathfinding algorithms (BFS, Dijkstra's, and A*) and watch them explore the grid and reconstruct the shortest path.

## Files

- `maze_solver.py` — Main visualizer and algorithm implementations (BFS, Dijkstra, A*). Uses a square grid of nodes and Pygame for rendering and interaction.
- `experiment5.cpp`, `experiment6.cpp`, ... `stack.cpp`, `LinkedList.cpp` — other course experiments included in the repository (not related to the visualizer).

## Requirements

- Python 3.8+ (tested with Python 3.8+)
- Pygame

Install dependencies with pip:

```powershell
pip install pygame
```

## How to run

Open a terminal in the repository folder and run:

```powershell
python maze_solver.py
```

The Pygame window will open and show a blank grid.

## Controls and Interaction

- Left mouse button (click / hold):
	- First left-click sets the start node (orange).
	- Second left-click sets the end node (turquoise).
	- Subsequent left-clicks (or dragging) place walls (black) on cells.
- Right mouse button: erase a node (resets to empty). If you erase the start or end node, they are cleared.
- Keyboard:
	- `1` — Run BFS (Breadth-First Search)
	- `2` — Run Dijkstra's algorithm
	- `3` — Run A* Search (uses Manhattan heuristic)
	- `c` — Clear the board (reset grid)
	- `q` — Quit the program

While an algorithm runs the grid is animated: green nodes are open/explored, red nodes are closed/finished, and the final path is shown in purple.

## Algorithms

- BFS: Explores nodes in breadth-first order, finds shortest path in an unweighted grid.
- Dijkstra: Classic shortest-path algorithm using a priority queue (uniform edge cost in this grid).
- A*: Uses g + h scoring where h is the Manhattan distance to the goal. Faster than Dijkstra in many cases on grids when admissible heuristic is used.

## Implementation notes

- Grid is implemented as a 2D array of `Node` objects. Each node tracks color, neighbors, and scores for graph algorithms.
- `Node.update_neighbors(grid)` checks four-directional neighbors (up/down/left/right). Diagonal movement is not enabled.
- The code assumes uniform movement cost between adjacent cells (cost = 1).
- Path reconstruction uses `previous_node` pointers set during search.

## Small contract / behavior

- Inputs: interactive mouse + keyboard. Program reads grid state from user-drawn walls and chosen start/end nodes.
- Output: Pygame window with animated search and reconstructed path (or message when no path found).
- Error / edge cases: If start or end are not set, algorithms won't run. If no path exists, the algorithm will terminate without constructing a path.

## Notes and possible improvements

- Add the ability to save/load mazes.
- Add weighted cells to demonstrate Dijkstra's behavior with non-uniform costs.
- Add diagonal movement option and alternative heuristics for A*.

---

If you want, I can also add a small example maze preset, a GIF/Screenshot, or a requirements file. Which would you prefer next?
