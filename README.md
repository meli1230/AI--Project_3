# **Pacman Complex Search**
This project extends **search algorithms** to solve more complex multi-goal search problems in Pacman, optimizing food collection and efficient pathfinding.

## **License**
This project is for educational purposes and follows the **Berkeley AI Pacman Project** framework. <br/>
Please note that the project has been solved in teams of 2. My work is marked under `@Author: Melisa Marian`.

## **Overview**
This project builds on basic search techniques, introducing **multi-goal search problems** and **heuristic optimizations** to enhance efficiency. Pacman must now navigate mazes while solving more complex constraints.

### **Implemented Algorithms**
- **Corners Problem Search:** Finds the shortest path through all four maze corners
- **A* with Heuristics:** Implements heuristics for optimal multi-goal pathfinding
- **Food Search Problem:** Optimizes food collection using search strategies
- **Greedy Search for Food:** Implements a suboptimal agent that eats the closest food first

## **How to Run the Search Agents**
Test different search strategies by running:

- **Corners Problem Search (BFS):**
  ```bash
  python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
  ```
- **A* for Corners Problem:**
  ```bash
  python pacman.py -l mediumCorners -p AStarCornersAgent -z 0.5
  ```
- **Food Search with A***:
  ```bash
  python pacman.py -l testSearch -p AStarFoodSearchAgent
  ```
- **Greedy Search for Food:**
  ```bash
  python pacman.py -l bigSearch -p ClosestDotSearchAgent -z 0.5
  ```

- Use `-h` for a list of available options:
  ```bash
  python pacman.py -h
  ```
  
- Run the autograder to test the given implementation:
```bash
python autograder.py
```

## **File Structure**
- **`search.py`** – Implements complex search algorithms
- **`searchAgent.py`** – Defines search-based agents using advanced search techniques
- **`pacman.py`** – Main game engine
- **`util.py`** – Helper functions for data structures
