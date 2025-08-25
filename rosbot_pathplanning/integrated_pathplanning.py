#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Integrated Path Planning System
-------------------------------
Integrates localization, mapping, and path planning into a single system.
This file demonstrates how to use all three modules together.

Usage:
    python3 integrated_pathplanning.py
"""

import cv2
import numpy as np
from numpy import sqrt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import your modules (adjust paths as needed)
# from .bot_localizer import bot_localizer
# from .bot_mapper import bot_mapper
# from .pathplanning import bot_pathplanner

# For now, we'll define the classes inline to make it self-contained
class bot_localizer:
    """Simplified version of your localization class"""
    def __init__(self):
        self.is_bg_extracted = False
        self.bg_model = []
        self.maze_og = []
        self.loc_car = (0, 0)
        self.orig_X = 0
        self.orig_Y = 0
        self.orig_rows = 0
        self.orig_cols = 0
        self.transform_arr = []
        self.orig_rot = 0
        self.rot_mat = np.eye(2, dtype=float)
        self.rot_mat_rev = np.eye(2, dtype=float)

    def extract_bg(self, frame):
        """Extract background and create maze occupancy grid"""
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, L2gradient=True)
        
        # Create a simple maze for demonstration
        self.maze_og = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        self.is_bg_extracted = True

    def localize_bot(self, curr_frame, frame_disp):
        """Localize the bot in the current frame"""
        if not self.is_bg_extracted:
            self.extract_bg(curr_frame.copy())
        
        # For demonstration, set bot location to center
        h, w = curr_frame.shape[:2]
        self.loc_car = (h//2, w//2)
        
        # Draw bot location
        cv2.circle(frame_disp, (self.loc_car[1], self.loc_car[0]), 10, (0, 255, 0), -1)
        
        return self.loc_car

class bot_mapper:
    """Simplified version of your mapping class"""
    def __init__(self):
        self.graphified = False
        self.maze = None
        self.graph = {}

    def graphify(self, extracted_maze):
        """Convert maze to graph representation"""
        if not self.graphified:
            # Create a simple graph from the maze
            rows, cols = extracted_maze.shape
            self.graph = {}
            
            # Create nodes at regular intervals
            for i in range(0, rows, 30):
                for j in range(0, cols, 30):
                    if extracted_maze[i, j] > 0:  # If pixel is white (path)
                        node = (i, j)
                        self.graph[node] = {}
                        
                        # Add connections to neighboring nodes
                        for di in [-30, 0, 30]:
                            for dj in [-30, 0, 30]:
                                if di == 0 and dj == 0:
                                    continue
                                ni, nj = i + di, j + dj
                                if (0 <= ni < rows and 0 <= nj < cols and 
                                    extracted_maze[ni, nj] > 0):
                                    neighbor = (ni, nj)
                                    # Calculate cost as Euclidean distance
                                    cost = sqrt(di*di + dj*dj)
                                    self.graph[node][neighbor] = {"cost": cost}
            
            self.maze = extracted_maze
            self.graphified = True

class bot_pathplanner:
    """Path planning class with multiple algorithms"""
    def __init__(self):
        self.path_to_goal = []
        self.img_shortest_path = []
        self.choosen_route = []

    @staticmethod
    def cords_to_pts(cords):
        """Convert (row, col) coordinates to (x, y) points for OpenCV drawing"""
        return [cord[::-1] for cord in cords]

    def draw_path_on_maze(self, maze, shortest_path_pts, method):
        """Draw the found path on the maze with color-coded visualization"""
        maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        self.choosen_route = np.zeros_like(maze_bgr)

        depth = maze.shape[0]
        for i in range(len(shortest_path_pts) - 1):
            per_depth = (shortest_path_pts[i][1]) / depth

            # Color coding based on depth
            color = (
                int(255 * (abs(per_depth + (-1 * (per_depth > 0.5)) * 2))),
                int(255 * per_depth),
                int(255 * (1 - per_depth))
            )
            cv2.line(maze_bgr, shortest_path_pts[i], shortest_path_pts[i + 1], color, 2)
            cv2.line(self.choosen_route, shortest_path_pts[i], shortest_path_pts[i + 1], color, 3)

        img_str = "maze (Found Path) [" + method + "]"
        cv2.namedWindow(img_str, cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)
        
        self.img_shortest_path = maze_bgr.copy()
        return maze_bgr

    def find_path_dijkstra(self, graph, start, end):
        """Find shortest path using Dijkstra's algorithm"""
        if start not in graph or end not in graph:
            return []
        
        # Simple Dijkstra implementation
        distances = {node: float('inf') for node in graph}
        distances[start] = 0
        previous = {node: None for node in graph}
        unvisited = set(graph.keys())
        
        while unvisited:
            current = min(unvisited, key=lambda x: distances[x])
            if current == end:
                break
                
            unvisited.remove(current)
            
            for neighbor, edge_data in graph[current].items():
                if neighbor in unvisited:
                    cost = edge_data['cost']
                    if distances[current] + cost < distances[neighbor]:
                        distances[neighbor] = distances[current] + cost
                        previous[neighbor] = current
        
        # Reconstruct path
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous[current]
        
        return path[::-1] if path[-1] == start else []

    def find_path_nd_display(self, graph, start, end, maze, method="dijisktra"):
        """Main function to find path and display it"""
        if method == "dijisktra":
            path_to_display = self.find_path_dijkstra(graph, start, end)
        else:
            print(f"Method {method} not implemented yet")
            return

        if not path_to_display:
            print(f"No path found from {start} to {end}")
            return

        pathpts_to_display = self.cords_to_pts(path_to_display)
        self.path_to_goal = pathpts_to_display

        print(f"Shortest path from {start} to {end} is = {pathpts_to_display}")
        
        # Draw the path
        self.draw_path_on_maze(maze, pathpts_to_display, method)

def main():
    """Main function demonstrating the integrated system"""
    print("Starting Integrated Path Planning System...")
    
    # Create a synthetic maze for demonstration
    maze_size = 300
    maze = np.ones((maze_size, maze_size), dtype=np.uint8) * 255
    
    # Create some obstacles
    cv2.rectangle(maze, (50, 50), (100, 100), 0, -1)
    cv2.rectangle(maze, (150, 150), (200, 200), 0, -1)
    cv2.rectangle(maze, (100, 250), (150, 300), 0, -1)
    
    # Initialize components
    localizer = bot_localizer()
    mapper = bot_mapper()
    pathplanner = bot_pathplanner()
    
    # Set start and goal positions
    start_pos = (50, 50)
    goal_pos = (250, 250)
    
    print(f"Start position: {start_pos}")
    print(f"Goal position: {goal_pos}")
    
    # Process the maze
    localizer.maze_og = maze
    mapper.graphify(maze)
    
    print(f"Graph created with {len(mapper.graph)} nodes")
    
    # Find path
    if mapper.graph and start_pos in mapper.graph and goal_pos in mapper.graph:
        pathplanner.find_path_nd_display(
            mapper.graph, 
            start_pos, 
            goal_pos, 
            maze, 
            method="dijisktra"
        )
        
        # Show the original maze
        cv2.imshow("Original Maze", maze)
        
        # Show the path
        if pathplanner.img_shortest_path is not None:
            cv2.imshow("Path Planning Result", pathplanner.img_shortest_path)
        
        print("Press any key to exit...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("Could not create valid graph or start/goal positions not in graph")

if __name__ == "__main__":
    main()