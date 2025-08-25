#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 3: Path Planning (DFS, Dijkstra, A*)
------------------------------------------
Module to perform pathplanning from source to destination using provided methods:
- DFS (Depth First Search)
- DFS_Shortest (DFS with shortest path selection)
- Dijkstra (Dijkstra's shortest path algorithm)
- A* (A-star heuristic search algorithm)

Usage:
1) Import the class (bot_pathplanner)
2) Create its object
3) Access the object's function find_path_nd_display()
   Example: self.bot_pathplanner.find_path_nd_display(graph, start, end, maze, method="a_star")

Inputs:
1) Graph extracted in mapping stage
2) Source & Destination coordinates
3) Maze Image (skeleton from mapping)
4) Method to Use ["DFS", "DFS_Shortest", "dijkstra", "a_star"]

Outputs:
1) self.path_to_goal      => Computed Path from Source to destination [List of Coordinates]
2) self.img_shortest_path => Found path Overlayed (In Color) on Image

Author: Generated for ROSBot Navigation
Date: 2024
"""

import cv2
import numpy as np
from numpy import sqrt
import networkx as nx
from collections import deque


class bot_pathplanner:
    def __init__(self):
        self.DFS = DFS()
        self.dijkstra = Dijkstra()
        self.astar = AStar()
        
        self.path_to_goal = []
        self.img_shortest_path = []
        self.chosen_route = []
        
        # Debug flag
        self.debug = True
        self.debug_pathplanning = True

    @staticmethod
    def coords_to_pts(coords):
        """Convert coordinates to points (swap x,y)"""
        return [coord[::-1] for coord in coords]

    def build_graph_from_skeleton(self, skeleton):
        """
        Build a graph from the skeleton image for pathfinding.
        Returns a networkx graph with nodes and edges.
        """
        graph = nx.Graph()
        rows, cols = skeleton.shape
        
        # Find all white pixels (path pixels)
        path_pixels = np.where(skeleton > 0)
        
        # Add nodes for all path pixels
        for i in range(len(path_pixels[0])):
            y, x = path_pixels[0][i], path_pixels[1][i]
            graph.add_node((y, x))
        
        # Add edges between adjacent path pixels
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        
        for i in range(len(path_pixels[0])):
            y, x = path_pixels[0][i], path_pixels[1][i]
            current_node = (y, x)
            
            for dy, dx in directions:
                ny, nx = y + dy, x + dx
                neighbor_node = (ny, nx)
                
                # Check if neighbor is within bounds and is a path pixel
                if (0 <= ny < rows and 0 <= nx < cols and 
                    skeleton[ny, nx] > 0 and neighbor_node in graph.nodes):
                    
                    # Calculate edge weight (Euclidean distance)
                    weight = sqrt(dy*dy + dx*dx)
                    graph.add_edge(current_node, neighbor_node, weight=weight)
        
        return graph

    def find_nearest_path_point(self, skeleton, target_point):
        """Find the nearest path point to the given target point"""
        path_pixels = np.where(skeleton > 0)
        min_dist = float('inf')
        nearest_point = None
        
        for i in range(len(path_pixels[0])):
            y, x = path_pixels[0][i], path_pixels[1][i]
            dist = sqrt((target_point[0] - y)**2 + (target_point[1] - x)**2)
            if dist < min_dist:
                min_dist = dist
                nearest_point = (y, x)
        
        return nearest_point

    def draw_path_on_maze(self, maze, shortest_path_pts, method):
        """Draw the found path on the maze image with color coding"""
        if len(maze.shape) == 2:  # Convert grayscale to BGR
            maze_bgr = cv2.cvtColor(maze, cv2.COLOR_GRAY2BGR)
        else:
            maze_bgr = maze.copy()
            
        self.chosen_route = np.zeros_like(maze_bgr)
        
        if len(shortest_path_pts) < 2:
            self.img_shortest_path = maze_bgr.copy()
            return
        
        depth = maze.shape[0]
        
        for i in range(len(shortest_path_pts) - 1):
            per_depth = shortest_path_pts[i][1] / depth
            
            # Color coding based on depth
            color = (
                int(255 * (abs(per_depth + (-1 * (per_depth > 0.5))) * 2)),
                int(255 * per_depth),
                int(255 * (1 - per_depth))
            )
            
            cv2.line(maze_bgr, shortest_path_pts[i], shortest_path_pts[i + 1], color, 2)
            cv2.line(self.chosen_route, shortest_path_pts[i], shortest_path_pts[i + 1], color, 3)
        
        # Draw start and end points
        if shortest_path_pts:
            cv2.circle(maze_bgr, shortest_path_pts[0], 8, (0, 255, 0), -1)  # Green start
            cv2.circle(maze_bgr, shortest_path_pts[-1], 8, (0, 0, 255), -1)  # Red end
        
        img_str = f"maze (Found Path) [{method}]"
        if self.debug and self.debug_pathplanning:
            cv2.namedWindow(img_str, cv2.WINDOW_NORMAL)
            cv2.imshow(img_str, maze_bgr)
        
        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self, skeleton, start, end, method="DFS"):
        """
        Main function to find path and display results
        
        Args:
            skeleton: Binary skeleton image from mapping stage
            start: Start coordinates (row, col)
            end: End coordinates (row, col)  
            method: Algorithm to use ["DFS", "DFS_Shortest", "dijkstra", "a_star"]
        """
        # Build graph from skeleton
        graph = self.build_graph_from_skeleton(skeleton)
        
        if graph.number_of_nodes() == 0:
            print("Error: No path pixels found in skeleton")
            return
        
        # Find nearest path points to start and end
        start_node = self.find_nearest_path_point(skeleton, start)
        end_node = self.find_nearest_path_point(skeleton, end)
        
        if start_node is None or end_node is None:
            print("Error: Could not find valid start or end points on path")
            return
        
        if start_node not in graph.nodes or end_node not in graph.nodes:
            print("Error: Start or end points not in graph")
            return
        
        print(f"Planning path from {start_node} to {end_node} using {method}")
        
        path_to_display = []
        Path_str = "Path"
        
        try:
            if method == "DFS":
                path_to_display = self.DFS.get_path(graph, start_node, end_node)
                
            elif method == "DFS_Shortest":
                paths = self.DFS.get_all_paths(graph, start_node, end_node)
                if paths:
                    # Find shortest path by length
                    path_to_display = min(paths, key=len)
                    Path_str = "Shortest " + Path_str
                    
            elif method == "dijkstra":
                path_to_display = self.dijkstra.find_shortest_path(graph, start_node, end_node)
                Path_str = "Shortest " + Path_str
                
            elif method == "a_star":
                path_to_display = self.astar.find_shortest_path(graph, start_node, end_node)
                Path_str = "Shortest " + Path_str
                
            else:
                print(f"Unknown method: {method}")
                return
                
        except Exception as e:
            print(f"Error in pathfinding: {e}")
            return
        
        if not path_to_display:
            print("No path found!")
            return
        
        # Convert coordinates to points for display
        pathpts_to_display = self.coords_to_pts(path_to_display)
        self.path_to_goal = pathpts_to_display
        
        if self.debug and self.debug_pathplanning:
            print(f"{Path_str} from {start_node} to {end_node} is = {pathpts_to_display}")
        
        # Draw path on maze
        self.draw_path_on_maze(skeleton, pathpts_to_display, method)


class DFS:
    """Depth First Search implementation"""
    
    def get_path(self, graph, start, end, path=None):
        """Get a single path using DFS"""
        if path is None:
            path = []
        
        path = path + [start]
        
        if start == end:
            return path
        
        for neighbor in graph.neighbors(start):
            if neighbor not in path:
                new_path = self.get_path(graph, neighbor, end, path)
                if new_path:
                    return new_path
        
        return None
    
    def get_all_paths(self, graph, start, end, path=None):
        """Get all possible paths using DFS"""
        if path is None:
            path = []
        
        path = path + [start]
        
        if start == end:
            return [path]
        
        paths = []
        for neighbor in graph.neighbors(start):
            if neighbor not in path:
                new_paths = self.get_all_paths(graph, neighbor, end, path)
                paths.extend(new_paths)
        
        return paths


class Dijkstra:
    """Dijkstra's shortest path algorithm implementation"""
    
    def find_shortest_path(self, graph, start, end):
        """Find shortest path using Dijkstra's algorithm"""
        try:
            path = nx.shortest_path(graph, start, end, weight='weight')
            return path
        except nx.NetworkXNoPath:
            print("No path exists between start and end points")
            return []
        except Exception as e:
            print(f"Error in Dijkstra: {e}")
            return []


class AStar:
    """A* heuristic search algorithm implementation"""
    
    @staticmethod
    def heuristic(node1, node2):
        """Euclidean distance heuristic"""
        return sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)
    
    def find_shortest_path(self, graph, start, end):
        """Find shortest path using A* algorithm"""
        try:
            path = nx.astar_path(graph, start, end, heuristic=self.heuristic, weight='weight')
            return path
        except nx.NetworkXNoPath:
            print("No path exists between start and end points")
            return []
        except Exception as e:
            print(f"Error in A*: {e}")
            return []


# ---------- ROS2 Node Entry ----------
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point
    from cv_bridge import CvBridge
    
    class PathPlanningNode(Node):
        def __init__(self):
            super().__init__("pathplanning")
            
            # Parameters
            self.declare_parameter("image_topic", "/camera/image")
            self.declare_parameter("method", "a_star")
            self.declare_parameter("start_x", 50)
            self.declare_parameter("start_y", 50)
            self.declare_parameter("end_x", 200)
            self.declare_parameter("end_y", 200)
            
            # Get parameters
            topic = self.get_parameter("image_topic").get_parameter_value().string_value
            self.method = self.get_parameter("method").get_parameter_value().string_value
            self.start_x = self.get_parameter("start_x").get_parameter_value().integer_value
            self.start_y = self.get_parameter("start_y").get_parameter_value().integer_value
            self.end_x = self.get_parameter("end_x").get_parameter_value().integer_value
            self.end_y = self.get_parameter("end_y").get_parameter_value().integer_value
            
            # Initialize components
            self.bridge = CvBridge()
            self.pathplanner = bot_pathplanner()
            
            # Subscription
            self.sub = self.create_subscription(Image, topic, self._image_callback, 10)
            
            # Publishers for path visualization
            self.path_pub = self.create_publisher(Image, "/path_image", 10)
            
            self.get_logger().info(f"Path Planning Node initialized")
            self.get_logger().info(f"Subscribed to: {topic}")
            self.get_logger().info(f"Method: {self.method}")
            self.get_logger().info(f"Start: ({self.start_x}, {self.start_y})")
            self.get_logger().info(f"End: ({self.end_x}, {self.end_y})")
            
        def _image_callback(self, msg: Image):
            try:
                # Convert ROS image to OpenCV
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                
                # Convert to grayscale and threshold to get skeleton
                if len(frame.shape) == 3:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    gray = frame
                
                # Threshold to get binary skeleton
                _, skeleton = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
                
                # Define start and end points
                start = (self.start_y, self.start_x)  # (row, col)
                end = (self.end_y, self.end_x)        # (row, col)
                
                # Find path
                self.pathplanner.find_path_nd_display(skeleton, start, end, self.method)
                
                # Publish result if path was found
                if len(self.pathplanner.img_shortest_path) > 0:
                    path_msg = self.bridge.cv2_to_imgmsg(self.pathplanner.img_shortest_path, "bgr8")
                    self.path_pub.publish(path_msg)
                
                # Show images
                cv2.imshow("Original", frame)
                cv2.imshow("Skeleton", skeleton)
                if len(self.pathplanner.img_shortest_path) > 0:
                    cv2.imshow("Path Result", self.pathplanner.img_shortest_path)
                cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f"Error in image callback: {e}")

    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()