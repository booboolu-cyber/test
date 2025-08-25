#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Utilities for ROSBot Navigation
-------------------------------
Common utility functions used across localization, mapping, and path planning modules.

This module provides helper functions for:
- Image processing operations
- Contour analysis
- Coordinate transformations
- Visualization utilities
"""

import cv2
import numpy as np


def ret_smallest_obj(contours):
    """
    Return index of the smallest contour by area.
    
    Args:
        contours: List of contours from cv2.findContours()
    
    Returns:
        int: Index of smallest contour, or -1 if no contours
    """
    if not contours:
        return -1
    
    min_area = float('inf')
    min_idx = -1
    
    for idx, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area < min_area:
            min_area = area
            min_idx = idx
    
    return min_idx


def ret_largest_obj(binary_image):
    """
    Return mask and contour of the largest connected component.
    
    Args:
        binary_image: Binary image (0/255)
    
    Returns:
        tuple: (largest_mask, largest_contour) or (None, None) if no contours
    """
    # Find contours
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None, None
    
    # Find largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Create mask for largest contour
    mask = np.zeros_like(binary_image)
    cv2.drawContours(mask, [largest_contour], -1, 255, -1)
    
    return mask, largest_contour


def create_circular_kernel(radius):
    """
    Create a circular morphological kernel.
    
    Args:
        radius: Radius of the circular kernel
    
    Returns:
        np.ndarray: Circular kernel
    """
    diameter = 2 * radius + 1
    kernel = np.zeros((diameter, diameter), dtype=np.uint8)
    center = radius
    
    for y in range(diameter):
        for x in range(diameter):
            if (x - center) ** 2 + (y - center) ** 2 <= radius ** 2:
                kernel[y, x] = 1
    
    return kernel


def apply_morphology(image, operation, kernel_size=5, iterations=1):
    """
    Apply morphological operations to binary image.
    
    Args:
        image: Binary input image
        operation: 'open', 'close', 'erode', 'dilate', 'gradient', 'tophat', 'blackhat'
        kernel_size: Size of morphological kernel
        iterations: Number of iterations
    
    Returns:
        np.ndarray: Processed image
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    
    operations = {
        'erode': cv2.MORPH_ERODE,
        'dilate': cv2.MORPH_DILATE,
        'open': cv2.MORPH_OPEN,
        'close': cv2.MORPH_CLOSE,
        'gradient': cv2.MORPH_GRADIENT,
        'tophat': cv2.MORPH_TOPHAT,
        'blackhat': cv2.MORPH_BLACKHAT
    }
    
    if operation not in operations:
        raise ValueError(f"Unknown operation: {operation}")
    
    return cv2.morphologyEx(image, operations[operation], kernel, iterations=iterations)


def filter_contours_by_area(contours, min_area=0, max_area=float('inf')):
    """
    Filter contours by area range.
    
    Args:
        contours: List of contours
        min_area: Minimum area threshold
        max_area: Maximum area threshold
    
    Returns:
        list: Filtered contours
    """
    filtered = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if min_area <= area <= max_area:
            filtered.append(contour)
    return filtered


def get_contour_center(contour):
    """
    Get the center point of a contour using moments.
    
    Args:
        contour: OpenCV contour
    
    Returns:
        tuple: (cx, cy) center coordinates, or (0, 0) if invalid
    """
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return (0, 0)
    
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return (cx, cy)


def distance_between_points(p1, p2):
    """
    Calculate Euclidean distance between two points.
    
    Args:
        p1: First point (x, y) or (row, col)
        p2: Second point (x, y) or (row, col)
    
    Returns:
        float: Euclidean distance
    """
    return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def angle_between_points(p1, p2):
    """
    Calculate angle from p1 to p2 in radians.
    
    Args:
        p1: Start point (x, y)
        p2: End point (x, y)
    
    Returns:
        float: Angle in radians (-π to π)
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return np.arctan2(dy, dx)


def rotate_point(point, center, angle_rad):
    """
    Rotate a point around a center by given angle.
    
    Args:
        point: Point to rotate (x, y)
        center: Center of rotation (x, y)
        angle_rad: Rotation angle in radians
    
    Returns:
        tuple: Rotated point (x, y)
    """
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)
    
    # Translate to origin
    x = point[0] - center[0]
    y = point[1] - center[1]
    
    # Rotate
    x_rot = x * cos_a - y * sin_a
    y_rot = x * sin_a + y * cos_a
    
    # Translate back
    x_final = x_rot + center[0]
    y_final = y_rot + center[1]
    
    return (int(x_final), int(y_final))


def create_visualization_grid(images, titles=None, grid_size=None):
    """
    Create a grid visualization from multiple images.
    
    Args:
        images: List of images (can be different sizes)
        titles: List of titles for each image
        grid_size: Tuple (rows, cols) for grid layout, auto-calculated if None
    
    Returns:
        np.ndarray: Combined grid image
    """
    if not images:
        return np.zeros((100, 100, 3), dtype=np.uint8)
    
    n_images = len(images)
    
    # Auto-calculate grid size
    if grid_size is None:
        cols = int(np.ceil(np.sqrt(n_images)))
        rows = int(np.ceil(n_images / cols))
        grid_size = (rows, cols)
    
    rows, cols = grid_size
    
    # Standardize image sizes and convert to BGR
    processed_images = []
    max_height = max_width = 0
    
    for img in images:
        if img is None:
            img = np.zeros((100, 100, 3), dtype=np.uint8)
        
        if len(img.shape) == 2:  # Grayscale
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        elif len(img.shape) == 3 and img.shape[2] == 4:  # RGBA
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        
        max_height = max(max_height, img.shape[0])
        max_width = max(max_width, img.shape[1])
        processed_images.append(img)
    
    # Resize all images to the same size
    target_height = max_height
    target_width = max_width
    
    for i in range(len(processed_images)):
        img = processed_images[i]
        if img.shape[:2] != (target_height, target_width):
            processed_images[i] = cv2.resize(img, (target_width, target_height))
    
    # Create grid
    grid_height = rows * target_height + (rows - 1) * 10  # 10px spacing
    grid_width = cols * target_width + (cols - 1) * 10    # 10px spacing
    grid = np.zeros((grid_height, grid_width, 3), dtype=np.uint8)
    
    # Place images in grid
    for i, img in enumerate(processed_images):
        if i >= rows * cols:
            break
        
        row = i // cols
        col = i % cols
        
        y_start = row * (target_height + 10)
        y_end = y_start + target_height
        x_start = col * (target_width + 10)
        x_end = x_start + target_width
        
        grid[y_start:y_end, x_start:x_end] = img
        
        # Add title if provided
        if titles and i < len(titles):
            cv2.putText(grid, titles[i], (x_start, y_start - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return grid


def draw_path_on_image(image, path_points, color=(0, 255, 0), thickness=2):
    """
    Draw a path on an image.
    
    Args:
        image: Input image
        path_points: List of points [(x, y), ...]
        color: Line color (B, G, R)
        thickness: Line thickness
    
    Returns:
        np.ndarray: Image with path drawn
    """
    if len(path_points) < 2:
        return image.copy()
    
    result = image.copy()
    
    # Draw path lines
    for i in range(len(path_points) - 1):
        pt1 = tuple(map(int, path_points[i]))
        pt2 = tuple(map(int, path_points[i + 1]))
        cv2.line(result, pt1, pt2, color, thickness)
    
    # Draw start and end points
    if path_points:
        start_pt = tuple(map(int, path_points[0]))
        end_pt = tuple(map(int, path_points[-1]))
        cv2.circle(result, start_pt, 8, (0, 255, 0), -1)  # Green start
        cv2.circle(result, end_pt, 8, (0, 0, 255), -1)    # Red end
    
    return result


def create_color_map(value, min_val=0, max_val=1, colormap=cv2.COLORMAP_JET):
    """
    Create a color from a value using OpenCV colormap.
    
    Args:
        value: Input value
        min_val: Minimum value for normalization
        max_val: Maximum value for normalization
        colormap: OpenCV colormap constant
    
    Returns:
        tuple: (B, G, R) color values
    """
    # Normalize value to 0-255 range
    normalized = int(255 * (value - min_val) / (max_val - min_val))
    normalized = np.clip(normalized, 0, 255)
    
    # Create single pixel image and apply colormap
    pixel = np.array([[normalized]], dtype=np.uint8)
    colored = cv2.applyColorMap(pixel, colormap)
    
    return tuple(map(int, colored[0, 0]))


# Configuration flags (can be imported and modified)
class Config:
    def __init__(self):
        self.debug = True
        self.debug_localization = True
        self.debug_mapping = True
        self.debug_pathplanning = True
        self.show_intermediate_steps = True
        self.save_debug_images = False
        self.debug_image_path = "./debug_images/"


# Global config instance
config = Config()