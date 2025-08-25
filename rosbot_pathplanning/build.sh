#!/bin/bash

echo "Building ROSbot Path Planning Package..."

# Check if we're in a ROS2 workspace
if [ ! -f "setup.py" ]; then
    echo "Error: setup.py not found. Please run this script from the package directory."
    exit 1
fi

# Create build directory if it doesn't exist
mkdir -p build
cd build

# Build the package
echo "Building package..."
python3 ../setup.py build

# Install the package
echo "Installing package..."
python3 ../setup.py install --user

echo "Build completed successfully!"
echo ""
echo "To test the package:"
echo "  python3 test_pathplanning.py"
echo ""
echo "To run with ROS2:"
echo "  ros2 run rosbot_pathplanning pathplanning"
echo ""
echo "To launch with ROS2:"
echo "  ros2 launch rosbot_pathplanning pathplanning.launch.py"