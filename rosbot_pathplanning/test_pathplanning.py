#!/usr/bin/env python3
"""
Simple test script for the path planning system.
This script demonstrates the core functionality without ROS2.
"""

import cv2
import numpy as np
from numpy import sqrt
import sys
import os

# Add the package to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'rosbot_pathplanning'))

from pathplanning import bot_pathplanner

def create_test_maze():
    """Create a simple test maze for demonstration"""
    maze_size = 400
    maze = np.ones((maze_size, maze_size), dtype=np.uint8) * 255
    
    # Create walls/obstacles
    cv2.rectangle(maze, (100, 100), (150, 150), 0, -1)  # Top-left obstacle
    cv2.rectangle(maze, (250, 200), (300, 250), 0, -1)  # Center obstacle
    cv2.rectangle(maze, (150, 300), (200, 350), 0, -1)  # Bottom obstacle
    
    # Create some corridors
    cv2.rectangle(maze, (50, 50), (350, 80), 255, -1)   # Top corridor
    cv2.rectangle(maze, (50, 320), (350, 350), 255, -1) # Bottom corridor
    cv2.rectangle(maze, (50, 50), (80, 350), 255, -1)   # Left corridor
    cv2.rectangle(maze, (320, 50), (350, 350), 255, -1) # Right corridor
    
    return maze

def create_graph_from_maze(maze):
    """Create a simple graph from the maze"""
    graph = {}
    rows, cols = maze.shape
    
    # Create nodes at regular intervals
    for i in range(0, rows, 40):
        for j in range(0, cols, 40):
            if maze[i, j] > 0:  # If pixel is white (path)
                node = (i, j)
                graph[node] = {}
                
                # Add connections to neighboring nodes
                for di in [-40, 0, 40]:
                    for dj in [-40, 0, 40]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if (0 <= ni < rows and 0 <= nj < cols and 
                            maze[ni, nj] > 0):
                            neighbor = (ni, nj)
                            # Calculate cost as Euclidean distance
                            cost = sqrt(di*di + dj*dj)
                            graph[node][neighbor] = {"cost": cost}
    
    return graph

def test_pathplanning():
    """Test the path planning system"""
    print("Creating test maze...")
    maze = create_test_maze()
    
    print("Creating graph from maze...")
    graph = create_graph_from_maze(maze)
    print(f"Graph created with {len(graph)} nodes")
    
    # Set start and goal positions
    start_pos = (60, 60)
    goal_pos = (340, 340)
    
    print(f"Start position: {start_pos}")
    print(f"Goal position: {goal_pos}")
    
    # Check if positions are in graph
    if start_pos not in graph:
        print(f"Warning: Start position {start_pos} not in graph")
        # Find closest node
        start_pos = min(graph.keys(), key=lambda x: sqrt((x[0]-60)**2 + (x[1]-60)**2))
        print(f"Using closest start position: {start_pos}")
    
    if goal_pos not in graph:
        print(f"Warning: Goal position {goal_pos} not in graph")
        # Find closest node
        goal_pos = min(graph.keys(), key=lambda x: sqrt((x[0]-340)**2 + (x[1]-340)**2))
        print(f"Using closest goal position: {goal_pos}")
    
    # Create path planner
    print("Creating path planner...")
    pathplanner = bot_pathplanner()
    
    # Find path using Dijkstra's algorithm
    print("Finding path using Dijkstra's algorithm...")
    pathplanner.find_path_nd_display(
        graph, 
        start_pos, 
        goal_pos, 
        maze, 
        method="dijisktra"
    )
    
    # Show the original maze
    cv2.imshow("Test Maze", maze)
    
    # Show the path
    if pathplanner.img_shortest_path is not None:
        cv2.imshow("Path Planning Result", pathplanner.img_shortest_path)
    
    print("Press any key to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    print("Test completed successfully!")

if __name__ == "__main__":
    try:
        test_pathplanning()
    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback
        traceback.print_exc()