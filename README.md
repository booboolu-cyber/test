# Bot Path Planning Module

This module provides path planning functionality for bot navigation using various algorithms including DFS, Dijkstra, and A*.

## Features

- **Multiple Algorithms**: DFS, DFS_Shortest, Dijkstra, A*
- **Visual Output**: Path visualization on maze images
- **Flexible Input**: Works with any graph representation
- **Cost-based Planning**: Supports weighted edges for optimal path finding

## Installation

1. Install the required dependencies:
```bash
pip install -r requirements.txt
```

## Usage

### Basic Usage

```python
from bot_pathplanner import bot_pathplanner

# Create path planner instance
planner = bot_pathplanner()

# Define your graph, start, end, and maze image
graph = {...}  # Your graph representation
start = (0, 0)  # Starting coordinates
end = (5, 5)    # Goal coordinates
maze = np.array(...)  # Your maze image

# Find path using A* algorithm
planner.find_path_nd_display(graph, start, end, maze, method="a_star")

# Access results
path = planner.path_to_goal
visualization = planner.img_shortest_path
```

### Available Methods

- `"DFS"` - Depth-First Search (finds any path)
- `"DFS_Shortest"` - DFS with cost optimization
- `"dijkstra"` - Dijkstra's shortest path algorithm
- `"a_star"` - A* algorithm with heuristic optimization

### Graph Format

The graph should be a dictionary where:
- Keys are node coordinates (tuples)
- Values are dictionaries of connected nodes with costs
- Each connection has a 'cost' key

Example:
```python
graph = {
    (0, 0): {
        (0, 1): {'cost': 1},
        (1, 0): {'cost': 1},
        'case': 'start'
    },
    (0, 1): {
        (0, 0): {'cost': 1},
        (0, 2): {'cost': 1},
        'case': 'node'
    }
    # ... more nodes
}
```

## Running the Demo

To see the module in action:

```bash
python bot_pathplanner.py
```

This will:
1. Create a sample 3x3 maze
2. Find the shortest path from (0,0) to (2,2)
3. Display the original maze and the path visualization
4. Show the computed path coordinates

## Output

- `path_to_goal`: List of coordinates representing the path
- `img_shortest_path`: Image with the path overlaid on the maze
- `choosen_route`: Clean path visualization without maze background

## Algorithm Details

### DFS (Depth-First Search)
- Finds any path from start to goal
- Not guaranteed to be shortest
- Fast but may explore unnecessary paths

### Dijkstra
- Guarantees shortest path
- Uses priority queue for efficiency
- Good for uniform cost graphs

### A*
- Combines Dijkstra with heuristic
- Uses Euclidean distance as heuristic
- Often faster than Dijkstra for large graphs
- Guarantees optimal path

## Dependencies

- OpenCV (cv2) - Image processing and visualization
- NumPy - Numerical operations and array handling

## Author

Based on original code by Haider Abbasi, adapted for modern Python and enhanced with additional features.