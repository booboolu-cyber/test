#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Path Planning Example
--------------------
Standalone example demonstrating how to use the path planning module
with a sample maze skeleton.

This example shows:
1. Creating a sample maze skeleton
2. Using different pathfinding algorithms
3. Visualizing the results
4. Comparing algorithm performance

Usage:
    python3 pathplanning_example.py
"""

import cv2
import numpy as np
import time
from pathplanning import bot_pathplanner


def create_sample_maze(width=400, height=400):
    """Create a sample maze skeleton for testing"""
    maze = np.zeros((height, width), dtype=np.uint8)
    
    # Create main corridors
    # Horizontal corridors
    maze[50:60, 50:350] = 255    # Top corridor
    maze[200:210, 50:350] = 255  # Middle corridor  
    maze[350:360, 50:350] = 255  # Bottom corridor
    
    # Vertical corridors
    maze[50:360, 50:60] = 255    # Left corridor
    maze[50:360, 200:210] = 255  # Middle corridor
    maze[50:360, 340:350] = 255  # Right corridor
    
    # Add some connecting passages
    maze[60:200, 100:110] = 255  # Connect top to middle
    maze[210:350, 150:160] = 255 # Connect middle to bottom
    maze[100:110, 60:200] = 255  # Connect left to middle
    maze[300:310, 210:340] = 255 # Connect middle to right
    
    # Add some dead ends and branches
    maze[80:120, 280:290] = 255  # Dead end branch
    maze[280:320, 80:90] = 255   # Dead end branch
    maze[150:190, 260:270] = 255 # Side branch
    
    return maze


def compare_algorithms(pathplanner, skeleton, start, end):
    """Compare different pathfinding algorithms"""
    algorithms = ["DFS", "DFS_Shortest", "dijkstra", "a_star"]
    results = {}
    
    print("\n" + "="*60)
    print("ALGORITHM COMPARISON")
    print("="*60)
    print(f"Start: {start}")
    print(f"End: {end}")
    print("-"*60)
    
    for algorithm in algorithms:
        print(f"\nTesting {algorithm}...")
        start_time = time.time()
        
        # Find path
        pathplanner.find_path_nd_display(skeleton, start, end, algorithm)
        
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Convert to milliseconds
        
        path_length = len(pathplanner.path_to_goal)
        
        results[algorithm] = {
            'path_length': path_length,
            'execution_time': execution_time,
            'path_found': path_length > 0
        }
        
        print(f"  Path length: {path_length} waypoints")
        print(f"  Execution time: {execution_time:.2f} ms")
        print(f"  Path found: {results[algorithm]['path_found']}")
        
        # Show result for a moment
        cv2.waitKey(1000)
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for alg, result in results.items():
        if result['path_found']:
            print(f"{alg:12}: {result['path_length']:3d} waypoints, "
                  f"{result['execution_time']:6.2f} ms")
        else:
            print(f"{alg:12}: No path found")
    print("="*60)
    
    return results


def interactive_path_planning(pathplanner, skeleton):
    """Interactive path planning with mouse clicks"""
    print("\n" + "="*60)
    print("INTERACTIVE PATH PLANNING")
    print("="*60)
    print("Instructions:")
    print("1. Click on the maze to set start point (GREEN)")
    print("2. Click again to set end point (RED)")
    print("3. Press 'd' for DFS")
    print("4. Press 's' for DFS_Shortest")  
    print("5. Press 'j' for Dijkstra")
    print("6. Press 'a' for A*")
    print("7. Press 'c' to clear points")
    print("8. Press 'q' to quit")
    print("-"*60)
    
    # Create display image
    display = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
    start_point = None
    end_point = None
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal start_point, end_point, display
        
        if event == cv2.EVENT_LBUTTONDOWN:
            # Check if click is on a white pixel (path)
            if skeleton[y, x] > 0:
                if start_point is None:
                    start_point = (y, x)  # (row, col)
                    display = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
                    cv2.circle(display, (x, y), 8, (0, 255, 0), -1)  # Green start
                    print(f"Start point set: {start_point}")
                elif end_point is None:
                    end_point = (y, x)    # (row, col)
                    cv2.circle(display, (x, y), 8, (0, 0, 255), -1)  # Red end
                    print(f"End point set: {end_point}")
                else:
                    # Reset points
                    start_point = (y, x)
                    end_point = None
                    display = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
                    cv2.circle(display, (x, y), 8, (0, 255, 0), -1)  # Green start
                    print(f"Start point reset: {start_point}")
            else:
                print("Please click on a white path pixel!")
    
    cv2.namedWindow("Interactive Path Planning")
    cv2.setMouseCallback("Interactive Path Planning", mouse_callback)
    
    while True:
        cv2.imshow("Interactive Path Planning", display)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('c'):
            start_point = None
            end_point = None
            display = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
            print("Points cleared")
        elif start_point is not None and end_point is not None:
            method = None
            if key == ord('d'):
                method = "DFS"
            elif key == ord('s'):
                method = "DFS_Shortest"
            elif key == ord('j'):
                method = "dijkstra"
            elif key == ord('a'):
                method = "a_star"
            
            if method:
                print(f"\nPlanning path using {method}...")
                pathplanner.find_path_nd_display(skeleton, start_point, end_point, method)
                
                # Update display with path
                if len(pathplanner.path_to_goal) > 0:
                    display = pathplanner.img_shortest_path.copy()
                    # Re-add start/end markers
                    cv2.circle(display, (start_point[1], start_point[0]), 8, (0, 255, 0), -1)
                    cv2.circle(display, (end_point[1], end_point[0]), 8, (0, 0, 255), -1)
                    print(f"Path found with {len(pathplanner.path_to_goal)} waypoints")
                else:
                    print("No path found!")


def main():
    """Main function demonstrating path planning capabilities"""
    print("Path Planning Module Demonstration")
    print("="*50)
    
    # Create pathplanner instance
    pathplanner = bot_pathplanner()
    
    # Create sample maze
    print("Creating sample maze...")
    skeleton = create_sample_maze()
    
    # Show the maze
    cv2.imshow("Sample Maze Skeleton", skeleton)
    print("Sample maze created. Press any key to continue...")
    cv2.waitKey(0)
    
    # Define test points
    start = (55, 55)    # (row, col) - top-left corner
    end = (355, 345)    # (row, col) - bottom-right corner
    
    # Compare algorithms
    results = compare_algorithms(pathplanner, skeleton, start, end)
    
    # Interactive mode
    print("\nStarting interactive mode...")
    interactive_path_planning(pathplanner, skeleton)
    
    # Cleanup
    cv2.destroyAllWindows()
    print("\nDemo completed!")


if __name__ == "__main__":
    main()