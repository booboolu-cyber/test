#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 3: Path Planning (Shortest Path Finding)
----------------------------------------------
- Integrates with bot_localizer and bot_mapper
- Supports multiple algorithms: DFS, DFS_Shortest, Dijkstra, A*
- Finds shortest path from current bot location to goal
- Visualizes path with color-coded overlay

Usage:
    ros2 run rosbot_description pathplanning
"""

import cv2
import numpy as np
from numpy import sqrt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class bot_pathplanner:
    def __init__(self):
        self.DFS = DFS()
        self.dijisktra = dijisktra()
        self.astar = a_star()

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

            # Color coding based on depth:
            # Blue: 0-depthperc-0
            # Green: depthperc
            # Red: 100-depthperc
            color = (
                int(255 * (abs(per_depth + (-1 * (per_depth > 0.5)) * 2))),
                int(255 * per_depth),
                int(255 * (1 - per_depth))
            )
            cv2.line(maze_bgr, shortest_path_pts[i], shortest_path_pts[i + 1], color)
            cv2.line(self.choosen_route, shortest_path_pts[i], shortest_path_pts[i + 1], color, 3)

        img_str = "maze (Found Path) [" + method + "]"
        cv2.namedWindow(img_str, cv2.WINDOW_FREERATIO)
        cv2.imshow(img_str, maze_bgr)

        if method == "dijisktra":
            self.dijisktra.shortest_path_overlayed = maze_bgr
        elif method == "a_star":
            self.astar.shortest_path_overlayed = maze_bgr

        self.img_shortest_path = maze_bgr.copy()

    def find_path_nd_display(self, graph, start, end, maze, method="dijisktra"):
        """Main function to find path and display it"""
        Path_str = "Path"

        if method == "DFS":
            paths = self.DFS.get_paths(graph, start, end)
            path_to_display = paths[0] if paths else []

        elif method == "DFS_Shortest":
            paths_N_costs = self.DFS.get_paths_cost(graph, start, end)
            paths = paths_N_costs[0]
            costs = paths_N_costs[1]
            if paths and costs:
                min_cost = min(costs)
                path_to_display = paths[costs.index(min_cost)]
                Path_str = "Shortest " + Path_str
            else:
                path_to_display = []

        elif method == "dijisktra":
            if not self.dijisktra.shortestpath_found:
                print("Finding Shortest Routes using Dijkstra")
                self.dijisktra.find_best_routes(graph, start, end)

            path_to_display = self.dijisktra.shortest_path
            Path_str = "Shortest " + Path_str

        elif method == "a_star":
            if not self.astar.shortestpath_found:
                print("Finding Shortest Routes using A*")
                self.astar.find_best_routes(graph, start, end)

            path_to_display = self.astar.shortest_path
            Path_str = "Shortest " + Path_str

        if not path_to_display:
            print(f"No path found from {start} to {end}")
            return

        pathpts_to_display = self.cords_to_pts(path_to_display)
        self.path_to_goal = pathpts_to_display

        print(f"{Path_str} from {start} to {end} is = {pathpts_to_display}")

        if method == "dijisktra":
            if not self.dijisktra.shortest_path_overlayed:
                self.draw_path_on_maze(maze, pathpts_to_display, method)
            else:
                cv2.imshow("maze (Found Path) [dijisktra]", self.dijisktra.shortest_path_overlayed)

        elif method == "a_star":
            if not self.astar.shortest_path_overlayed:
                self.draw_path_on_maze(maze, pathpts_to_display, method)
            else:
                cv2.imshow("maze (Found Path) [a_star]", self.astar.shortest_path_overlayed)


class DFS:
    """Depth-First Search implementation for path finding"""

    @staticmethod
    def get_paths(graph, start, end, path=[]):
        """Get all possible paths from start to end using DFS"""
        path = path + [start]

        if start == end:
            return [path]

        if start not in graph.keys():
            return []

        paths = []
        for node in graph[start].keys():
            if (node not in path) and (node != "case"):
                new_paths = DFS.get_paths(graph, node, end, path)
                for p in new_paths:
                    paths.append(p)

        return paths

    @staticmethod
    def get_paths_cost(graph, start, end, path=[], cost=0, trav_cost=0):
        """Get all possible paths with their costs"""
        path = path + [start]
        cost = cost + trav_cost

        if start == end:
            return [path], [cost]

        if start not in graph.keys():
            return [], []

        paths = []
        costs = []

        for node in graph[start].keys():
            if (node not in path) and (node != "case"):
                new_paths, new_costs = DFS.get_paths_cost(
                    graph, node, end, path, cost, graph[start][node]['cost']
                )
                for p in new_paths:
                    paths.append(p)
                for c in new_costs:
                    costs.append(c)

        return paths, costs


class Heap:
    """Min-heap implementation for priority queue in Dijkstra and A*"""

    def __init__(self):
        self.array = []
        self.size = 0
        self.posOfVertices = []

    def new_minHeap_node(self, v, dist):
        return [v, dist]

    def swap_nodes(self, a, b):
        temp = self.array[a]
        self.array[a] = self.array[b]
        self.array[b] = temp

    def minHeapify(self, node_idx):
        smallest = node_idx
        left = (node_idx * 2) + 1
        right = (node_idx * 2) + 2

        if (left < self.size) and (self.array[left][1] < self.array[smallest][1]):
            smallest = left
        if (right < self.size) and (self.array[right][1] < self.array[smallest][1]):
            smallest = right

        if smallest != node_idx:
            self.posOfVertices[self.array[node_idx][0]] = smallest
            self.posOfVertices[self.array[smallest][0]] = node_idx
            self.swap_nodes(node_idx, smallest)
            self.minHeapify(smallest)

    def extractmin(self):
        if self.size == 0:
            return

        root = self.array[0]
        lastNode = self.array[self.size - 1]
        self.array[0] = lastNode

        self.posOfVertices[root[0]] = self.size - 1
        self.posOfVertices[lastNode[0]] = 0

        self.size -= 1
        self.minHeapify(0)
        return root

    def decreaseKey(self, vertx, dist):
        idxofvertex = self.posOfVertices[vertx]
        self.array[idxofvertex][1] = dist

        while (idxofvertex > 0) and (self.array[idxofvertex][1] < self.array[(idxofvertex - 1) // 2][1]):
            self.posOfVertices[self.array[idxofvertex][0]] = (idxofvertex - 1) // 2
            self.posOfVertices[self.array[(idxofvertex - 1) // 2][0]] = idxofvertex
            self.swap_nodes(idxofvertex, (idxofvertex - 1) // 2)
            idxofvertex = (idxofvertex - 1) // 2

    def isInMinHeap(self, v):
        if self.posOfVertices[v] < self.size:
            return True
        return False


class dijisktra:
    """Dijkstra's algorithm implementation"""

    def __init__(self):
        self.shortestpath_found = False
        self.shortest_path = []
        self.shortest_path_overlayed = []
        self.minHeap = Heap()
        self.idxs2vrtxs = {}
        self.vrtxs2idxs = {}
        self.dijiktra_nodes_visited = 0

    def ret_shortestroute(self, parent, start, end, route):
        """Reconstruct shortest path from parent array"""
        route.append(self.idxs2vrtxs[end])

        if end == start:
            return

        end = parent[end]
        self.ret_shortestroute(parent, start, end, route)

    def find_best_routes(self, graph, start, end):
        """Find shortest path using Dijkstra's algorithm"""
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0]
        print(f"Index of search key: {start_idx}")

        dist = []
        parent = []
        self.minHeap.size = len(graph.keys())

        for idx, v in enumerate(graph.keys()):
            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)
            parent.append(-1)
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

        dist[start_idx] = 0
        self.minHeap.decreaseKey(start_idx, dist[start_idx])

        while self.minHeap.size != 0:
            self.dijiktra_nodes_visited += 1

            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            for v in graph[u]:
                if v != "case":
                    print(f"Vertex adjacent to {u} is {v}")
                    v_idx = self.vrtxs2idxs[v]

                    if (self.minHeap.isInMinHeap(v_idx) and (dist[u_idx] != 1e7) and
                        ((graph[u][v]["cost"] + dist[u_idx]) < dist[v_idx])):
                        dist[v_idx] = graph[u][v]["cost"] + dist[u_idx]
                        self.minHeap.decreaseKey(v_idx, dist[v_idx])
                        parent[v_idx] = u_idx

            if u == end:
                break

        shortest_path = []
        self.ret_shortestroute(parent, start_idx, self.vrtxs2idxs[end], shortest_path)
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True


class a_star(dijisktra):
    """A* algorithm implementation inheriting from Dijkstra"""

    def __init__(self):
        super().__init__()
        self.astar_nodes_visited = 0

    @staticmethod
    def euc_d(a, b):
        """Euclidean distance heuristic"""
        return sqrt(pow((a[0] - b[0]), 2) + pow((a[1] - b[1]), 2))

    def find_best_routes(self, graph, start, end):
        """Find shortest path using A* algorithm"""
        start_idx = [idx for idx, key in enumerate(graph.items()) if key[0] == start][0]
        print(f"Index of search key: {start_idx}")

        cost2node = []
        dist = []
        parent = []
        self.minHeap.size = len(graph.keys())

        for idx, v in enumerate(graph.keys()):
            cost2node.append(1e7)
            dist.append(1e7)
            self.minHeap.array.append(self.minHeap.new_minHeap_node(idx, dist[idx]))
            self.minHeap.posOfVertices.append(idx)
            parent.append(-1)
            self.vrtxs2idxs[v] = idx
            self.idxs2vrtxs[idx] = v

        cost2node[start_idx] = 0
        dist[start_idx] = cost2node[start_idx] + self.euc_d(start, end)
        self.minHeap.decreaseKey(start_idx, dist[start_idx])

        while self.minHeap.size != 0:
            self.astar_nodes_visited += 1

            curr_top = self.minHeap.extractmin()
            u_idx = curr_top[0]
            u = self.idxs2vrtxs[u_idx]

            for v in graph[u]:
                if v != "case":
                    print(f"Vertex adjacent to {u} is {v}")
                    v_idx = self.vrtxs2idxs[v]

                    if (self.minHeap.isInMinHeap(v_idx) and (dist[u_idx] != 1e7) and
                        ((graph[u][v]["cost"] + cost2node[u_idx]) < cost2node[v_idx])):
                        cost2node[v_idx] = graph[u][v]["cost"] + cost2node[u_idx]
                        dist[v_idx] = cost2node[v_idx] + self.euc_d(v, end)
                        self.minHeap.decreaseKey(v_idx, dist[v_idx])
                        parent[v_idx] = u_idx

            if u == end:
                break

        shortest_path = []
        self.ret_shortestroute(parent, start_idx, self.vrtxs2idxs[end], shortest_path)
        self.shortest_path = shortest_path[::-1]
        self.shortestpath_found = True


# ---------- ROS2 Node Entry ----------
def main(args=None):
    """Main function that integrates localization, mapping, and path planning"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    class PathPlanningNode(Node):
        def __init__(self):
            super().__init__("pathplanning")
            self.declare_parameter("image_topic", "/camera/image")
            topic = self.get_parameter("image_topic").get_parameter_value().string_value
            
            self.bridge = CvBridge()
            self.pathplanner = bot_pathplanner()
            
            # Subscribe to camera feed
            self.sub = self.create_subscription(Image, topic, self._cb, 10)
            self.get_logger().info(f"Subscribed to: {topic}")
            
            # Initialize state
            self.maze_og = None
            self.bot_location = None
            self.goal_location = None
            self.graph = None
            
            # Set goal location (you can modify this or make it configurable)
            self.goal_location = (150, 150)  # Example goal coordinates

        def _cb(self, msg: Image):
            """Process incoming camera frames"""
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Convert to grayscale and threshold to get maze
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, maze_og = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
            
            # For demonstration, create a simple graph from the maze
            # In a real implementation, this would come from your mapping module
            if self.graph is None:
                self.graph = self._create_simple_graph(maze_og)
            
            # For demonstration, set bot location to a known position
            # In a real implementation, this would come from your localization module
            if self.bot_location is None:
                self.bot_location = (50, 50)  # Example start position
            
            # Find path using Dijkstra's algorithm
            if self.graph and self.bot_location and self.goal_location:
                self.pathplanner.find_path_nd_display(
                    self.graph, 
                    self.bot_location, 
                    self.goal_location, 
                    maze_og, 
                    method="dijisktra"
                )
            
            # Show the maze
            cv2.imshow("Maze", maze_og)
            cv2.waitKey(1)

        def _create_simple_graph(self, maze):
            """Create a simple graph for demonstration purposes"""
            # This is a simplified graph creation
            # In your real implementation, this would come from the mapping module
            graph = {}
            
            # Create nodes at regular intervals
            rows, cols = maze.shape
            for i in range(0, rows, 20):
                for j in range(0, cols, 20):
                    if maze[i, j] > 0:  # If pixel is white (path)
                        node = (i, j)
                        graph[node] = {}
                        
                        # Add connections to neighboring nodes
                        for di in [-20, 0, 20]:
                            for dj in [-20, 0, 20]:
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