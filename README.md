# qt_gui_test

This project provides a Qt-based GUI example for a ROS2 workspace.

## Features
- Demonstrates Qt widget integration with ROS2 nodes
- Basic GUI components for quick testing

## Topics
- Publishes to “/chatter_pub” for sending commands
- Subscribes to “/chatter_sub” for receiving status updates
- Enables real-time GUI interaction with ROS2 messages

## Build and Run
1. Navigate to the workspace root.
2. Run: colcon build
3. Source the setup script.
4. Launch the node with: ros2 run qt_gui_test qt_gui_test

## License
Distributed under the MIT license.