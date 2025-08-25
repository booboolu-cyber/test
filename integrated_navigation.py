#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Integrated Navigation System
----------------------------
Complete integration of localization, mapping, and path planning modules.
This demonstrates how to use all three components together for autonomous navigation.

Usage:
    ros2 run rosbot_description integrated_navigation

The system will:
1. Extract background and localize the robot
2. Create a skeleton map with interest points
3. Plan optimal paths using various algorithms
4. Display real-time navigation visualization

Author: ROSBot Navigation Integration
Date: 2024
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge

# Import your custom modules (assuming they're in the same package)
from localization import bot_localizer
from mapping import bot_mapper
from pathplanning import bot_pathplanner


class IntegratedNavigationNode(Node):
    def __init__(self):
        super().__init__("integrated_navigation")
        
        # Parameters
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pathplanning_method", "a_star")
        self.declare_parameter("goal_x", 200)
        self.declare_parameter("goal_y", 200)
        self.declare_parameter("navigation_enabled", True)
        
        # Get parameters
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").get_parameter_value().string_value
        self.method = self.get_parameter("pathplanning_method").get_parameter_value().string_value
        self.goal_x = self.get_parameter("goal_x").get_parameter_value().integer_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().integer_value
        self.navigation_enabled = self.get_parameter("navigation_enabled").get_parameter_value().bool_value
        
        # Initialize components
        self.bridge = CvBridge()
        self.localizer = bot_localizer()
        self.mapper = bot_mapper()
        self.pathplanner = bot_pathplanner()
        
        # State variables
        self.current_frame = None
        self.robot_position = (0, 0)
        self.goal_position = (self.goal_y, self.goal_x)  # (row, col)
        self.current_path = []
        self.path_index = 0
        
        # ROS2 interfaces
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, cmd_vel_topic, 10)
        
        # Publishers for visualization
        self.localization_pub = self.create_publisher(Image, "/localization_viz", 10)
        self.mapping_pub = self.create_publisher(Image, "/mapping_viz", 10)
        self.pathplanning_pub = self.create_publisher(Image, "/pathplanning_viz", 10)
        self.integrated_pub = self.create_publisher(Image, "/integrated_viz", 10)
        
        # Timer for navigation control
        self.nav_timer = self.create_timer(0.1, self.navigation_callback)
        
        self.get_logger().info("Integrated Navigation System initialized")
        self.get_logger().info(f"Image topic: {image_topic}")
        self.get_logger().info(f"Path planning method: {self.method}")
        self.get_logger().info(f"Goal position: ({self.goal_x}, {self.goal_y})")
        self.get_logger().info(f"Navigation enabled: {self.navigation_enabled}")

    def image_callback(self, msg: Image):
        """Main image processing callback"""
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.current_frame = frame.copy()
            frame_disp = frame.copy()
            
            # Step 1: Localization
            self.localizer.localize_bot(frame, frame_disp)
            self.robot_position = self.localizer.loc_car
            
            # Publish localization visualization
            if len(frame_disp) > 0:
                loc_msg = self.bridge.cv2_to_imgmsg(frame_disp, "bgr8")
                self.localization_pub.publish(loc_msg)
            
            # Step 2: Mapping (only if localization is complete)
            if self.localizer.is_bg_extracted and len(self.localizer.maze_og) > 0:
                self.mapper.graphify(self.localizer.maze_og)
                
                # Publish mapping visualizations
                if self.mapper.maze_interestPts is not None:
                    map_interest_msg = self.bridge.cv2_to_imgmsg(
                        self.mapper.maze_interestPts, "bgr8")
                    self.mapping_pub.publish(map_interest_msg)
            
            # Step 3: Path Planning (only if mapping is complete)
            if (self.mapper.graphified and self.mapper.maze is not None and 
                len(self.current_path) == 0):  # Only plan once
                
                self.plan_path_to_goal()
            
            # Step 4: Create integrated visualization
            self.create_integrated_visualization()
            
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")

    def plan_path_to_goal(self):
        """Plan path from current robot position to goal"""
        try:
            if self.mapper.maze is None:
                return
            
            # Use robot's current position as start
            start_pos = self.robot_position
            goal_pos = self.goal_position
            
            self.get_logger().info(f"Planning path from {start_pos} to {goal_pos}")
            
            # Find path using selected algorithm
            self.pathplanner.find_path_nd_display(
                self.mapper.maze, start_pos, goal_pos, self.method)
            
            # Store the planned path
            if len(self.pathplanner.path_to_goal) > 0:
                self.current_path = self.pathplanner.path_to_goal
                self.path_index = 0
                self.get_logger().info(f"Path planned with {len(self.current_path)} waypoints")
                
                # Publish path planning visualization
                if len(self.pathplanner.img_shortest_path) > 0:
                    path_msg = self.bridge.cv2_to_imgmsg(
                        self.pathplanner.img_shortest_path, "bgr8")
                    self.pathplanning_pub.publish(path_msg)
            else:
                self.get_logger().warning("No path found to goal")
                
        except Exception as e:
            self.get_logger().error(f"Error in path planning: {e}")

    def navigation_callback(self):
        """Navigation control loop"""
        if not self.navigation_enabled or len(self.current_path) == 0:
            return
        
        try:
            # Get current target waypoint
            if self.path_index >= len(self.current_path):
                # Reached the end of path
                self.publish_stop_command()
                self.get_logger().info("Goal reached!")
                return
            
            target_point = self.current_path[self.path_index]
            current_pos = self.robot_position
            
            # Calculate distance to target
            distance = np.sqrt((target_point[0] - current_pos[0])**2 + 
                             (target_point[1] - current_pos[1])**2)
            
            # Check if we've reached the current waypoint
            if distance < 10:  # Threshold in pixels
                self.path_index += 1
                return
            
            # Calculate control commands
            cmd_vel = self.calculate_control_commands(current_pos, target_point)
            self.cmd_vel_pub.publish(cmd_vel)
            
        except Exception as e:
            self.get_logger().error(f"Error in navigation: {e}")

    def calculate_control_commands(self, current_pos, target_point):
        """Calculate velocity commands to reach target point"""
        cmd = Twist()
        
        # Simple proportional controller
        dx = target_point[0] - current_pos[0]
        dy = target_point[1] - current_pos[1]
        
        # Calculate angle to target
        angle_to_target = np.arctan2(dy, dx)
        
        # Simple control law
        linear_gain = 0.5
        angular_gain = 1.0
        
        distance = np.sqrt(dx*dx + dy*dy)
        
        cmd.linear.x = min(linear_gain * distance / 100.0, 0.5)  # Scale and limit
        cmd.angular.z = angular_gain * angle_to_target
        
        return cmd

    def publish_stop_command(self):
        """Publish stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def create_integrated_visualization(self):
        """Create a comprehensive visualization showing all components"""
        try:
            if self.current_frame is None:
                return
            
            # Create a large canvas for multiple views
            canvas_height = 600
            canvas_width = 1200
            canvas = np.zeros((canvas_height, canvas_width, 3), dtype=np.uint8)
            
            # Original view (top-left)
            if self.current_frame is not None:
                h, w = self.current_frame.shape[:2]
                scale = min(300/w, 200/h)
                resized = cv2.resize(self.current_frame, None, fx=scale, fy=scale)
                rh, rw = resized.shape[:2]
                canvas[10:10+rh, 10:10+rw] = resized
                cv2.putText(canvas, "Original View", (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Localization view (top-right)
            if hasattr(self.localizer, 'bg_model') and len(self.localizer.bg_model) > 0:
                h, w = self.localizer.bg_model.shape[:2]
                scale = min(300/w, 200/h)
                resized = cv2.resize(self.localizer.bg_model, None, fx=scale, fy=scale)
                rh, rw = resized.shape[:2]
                canvas[10:10+rh, 320:320+rw] = resized
                cv2.putText(canvas, "Background Model", (320, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Mapping view (bottom-left)
            if self.mapper.maze_interestPts is not None:
                h, w = self.mapper.maze_interestPts.shape[:2]
                scale = min(300/w, 200/h)
                resized = cv2.resize(self.mapper.maze_interestPts, None, fx=scale, fy=scale)
                rh, rw = resized.shape[:2]
                canvas[220:220+rh, 10:10+rw] = resized
                cv2.putText(canvas, "Map (Interest Points)", (10, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Path planning view (bottom-right)
            if len(self.pathplanner.img_shortest_path) > 0:
                h, w = self.pathplanner.img_shortest_path.shape[:2]
                scale = min(300/w, 200/h)
                resized = cv2.resize(self.pathplanner.img_shortest_path, None, fx=scale, fy=scale)
                rh, rw = resized.shape[:2]
                canvas[220:220+rh, 320:320+rw] = resized
                cv2.putText(canvas, f"Path Planning ({self.method})", (320, 240), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Status information (right panel)
            status_x = 650
            status_y = 50
            line_height = 30
            
            status_info = [
                f"Robot Position: {self.robot_position}",
                f"Goal Position: {self.goal_position}",
                f"Path Length: {len(self.current_path)}",
                f"Current Waypoint: {self.path_index}",
                f"Method: {self.method}",
                f"Navigation: {'ON' if self.navigation_enabled else 'OFF'}",
                "",
                "Controls:",
                "- Set goal_x, goal_y parameters",
                "- Change pathplanning_method",
                "- Toggle navigation_enabled"
            ]
            
            for i, info in enumerate(status_info):
                cv2.putText(canvas, info, (status_x, status_y + i * line_height),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Publish integrated visualization
            integrated_msg = self.bridge.cv2_to_imgmsg(canvas, "bgr8")
            self.integrated_pub.publish(integrated_msg)
            
            # Show the integrated view
            cv2.imshow("Integrated Navigation System", canvas)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error creating integrated visualization: {e}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = IntegratedNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down navigation system...")
    finally:
        try:
            cv2.destroyAllWindows()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()