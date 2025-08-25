# ROSbot Path Planning System

This package provides an integrated path planning system for ROSbot that combines localization, mapping, and path planning capabilities.

## Features

- **Localization**: Bot position detection and tracking
- **Mapping**: Maze to graph conversion with node detection
- **Path Planning**: Multiple algorithms including:
  - DFS (Depth-First Search)
  - DFS with cost optimization
  - Dijkstra's algorithm
  - A* search algorithm
- **Visualization**: Color-coded path display with real-time updates

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <your-repo-url> rosbot_pathplanning
```

2. Install dependencies:
```bash
sudo apt install ros-humble-cv-bridge python3-opencv python3-numpy
```

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select rosbot_pathplanning
source install/setup.bash
```

## Usage

### Running the Path Planning Node

```bash
# Basic usage
ros2 run rosbot_pathplanning pathplanning

# With custom parameters
ros2 run rosbot_pathplanning pathplanning --ros-args \
  -p image_topic:=/camera/image \
  -p goal_x:=200 \
  -p goal_y:=200
```

### Using the Launch File

```bash
# Launch with default parameters
ros2 launch rosbot_pathplanning pathplanning.launch.py

# Launch with custom goal position
ros2 launch rosbot_pathplanning pathplanning.launch.py \
  goal_x:=250 \
  goal_y:=250
```

### Running the Integrated Demo

```bash
# Run the standalone demo (no ROS2 required)
python3 rosbot_pathplanning/integrated_pathplanning.py
```

## Architecture

### Core Classes

1. **`bot_localizer`**: Handles bot position detection
   - Background extraction
   - Bot localization
   - Coordinate transformations

2. **`bot_mapper`**: Converts maze to graph representation
   - Skeletonization
   - Node detection (endpoints, junctions)
   - Graph construction

3. **`bot_pathplanner`**: Main path planning orchestrator
   - Algorithm selection
   - Path visualization
   - Integration with other modules

### Path Planning Algorithms

- **DFS**: Simple depth-first search for all possible paths
- **DFS_Shortest**: DFS with cost optimization
- **Dijkstra**: Guaranteed shortest path using priority queue
- **A***: Heuristic-based search with Euclidean distance

## Configuration

### Parameters

- `image_topic`: Camera image topic (default: `/camera/image`)
- `goal_x`: Goal X coordinate (default: 150)
- `goal_y`: Goal Y coordinate (default: 150)

### Topics

- **Subscribed**: `/camera/image` (sensor_msgs/Image)
- **Published**: None (uses OpenCV windows for visualization)

## Integration with Your Code

To integrate with your existing localization and mapping files:

1. **Import your modules**:
```python
from .bot_localizer import bot_localizer
from .bot_mapper import bot_mapper
```

2. **Use your existing classes**:
```python
# In the main function
localizer = bot_localizer()
mapper = bot_mapper()
pathplanner = bot_pathplanner()

# Process camera frame
bot_location = localizer.localize_bot(frame, frame_disp)
mapper.graphify(localizer.maze_og)

# Find path
pathplanner.find_path_nd_display(
    mapper.graph,
    bot_location,
    goal_location,
    localizer.maze_og,
    method="dijisktra"
)
```

## Customization

### Adding New Path Planning Algorithms

1. Create a new class inheriting from the base algorithm
2. Implement the required methods
3. Add the algorithm to the `bot_pathplanner` class

### Modifying Visualization

The `draw_path_on_maze` method handles path visualization. You can customize:
- Color schemes
- Line thickness
- Window properties
- Additional overlays

## Troubleshooting

### Common Issues

1. **No path found**: Check if start/goal positions are in the graph
2. **Import errors**: Ensure package is built and sourced correctly
3. **OpenCV windows not showing**: Check display settings and X11 forwarding

### Debug Mode

Enable debug output by setting environment variables:
```bash
export ROS_LOG_LEVEL=DEBUG
```

## Examples

### Basic Path Planning

```python
from rosbot_pathplanning.pathplanning import bot_pathplanner

# Create path planner
planner = bot_pathplanner()

# Find path
planner.find_path_nd_display(
    graph,           # Your graph from mapping
    (50, 50),       # Start position
    (250, 250),     # Goal position
    maze_image,      # Maze image
    method="dijisktra"
)
```

### Custom Goal Setting

```python
# Set goal dynamically
goal_x = 300
goal_y = 200
goal_position = (goal_x, goal_y)

# Update path
planner.find_path_nd_display(graph, start, goal_position, maze, "a_star")
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Support

For issues and questions:
- Create an issue on GitHub
- Check the documentation
- Review the example code

## Acknowledgments

- Based on the original path planning implementation by Haider Abbasi
- Integrates with ROS2 and OpenCV for robotics applications
- Uses efficient algorithms for real-time path planning