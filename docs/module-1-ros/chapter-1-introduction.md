---
sidebar_position: 1
title: "Introduction to ROS 2 Architecture"
---

# Introduction to ROS 2 Architecture

## Concept Overview

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms. Unlike traditional operating systems, ROS 2 is middleware that provides services designed specifically for a heterogeneous computer cluster.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROS 2 Architecture                          │
├─────────────────────────────────────────────────────────────────┤
│  Applications Layer                                             │
│  ┌─────────────────┬─────────────────┬─────────────────┐       │
│  │   Navigation    │   Perception    │   Manipulation  │       │
│  │   Package       │   Package       │   Package       │       │
│  └─────────────────┴─────────────────┴─────────────────┘       │
├─────────────────────────────────────────────────────────────────┤
│  Client Library Layer                                          │
│  ┌─────────────┬─────────────┬─────────────┬─────────────┐    │
│  │    rclpy    │    rclcpp   │    rclrs    │    rcljs    │    │
│  │  (Python)   │   (C++)     │  (Rust)     │  (JavaScript)│    │
│  └─────────────┴─────────────┴─────────────┴─────────────┘    │
├─────────────────────────────────────────────────────────────────┤
│  ROS 2 Core Layer                                              │
│  ┌─────────────────┬─────────────────┬─────────────────┐       │
│  │   DDS Abstraction│   Lifecycle    │   Parameters    │       │
│  │      Layer      │    Manager     │     Service     │       │
│  └─────────────────┴─────────────────┴─────────────────┘       │
├─────────────────────────────────────────────────────────────────┤
│  DDS Implementation Layer (Connext, FastDDS, CycloneDDS)       │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                    DDS Middleware                      │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## Key Components

### Nodes
A node is a process that performs computation. ROS 2 is designed to be a system of multiple nodes working together. In ROS 2, nodes are created within a context, which allows for multiple nodes to be run from a single process.

### Topics and Messages
Topics are named buses over which nodes exchange messages. A topic has a name and a message type, and allows for data to be passed between nodes that are subscribed to that topic.

### Services
Services provide a request/response interaction between nodes. A service consists of a request and a response, each with a specific message type.

### Actions
Actions are like services, but designed for long-running tasks. They include feedback during execution and the ability to cancel the task.

### Parameters
Parameters are named, typed values that belong to a node. They can be configured at startup or changed while the node is running.

## Lab Instructions

### Setting up your first ROS 2 workspace

1. Create a new workspace directory:
   ```bash
   mkdir -p ~/robotics_ws/src
   cd ~/robotics_ws
   ```

2. Build the workspace:
   ```bash
   colcon build
   ```

3. Source the setup file:
   ```bash
   source install/setup.bash
   ```

### Creating a simple publisher node

1. Navigate to your workspace source directory:
   ```bash
   cd ~/robotics_ws/src
   ```

2. Create a new package:
   ```bash
   ros2 pkg create --build-type ament_python py_publisher
   ```

3. Navigate to the package directory and create a publisher script in `py_publisher/py_publisher/publisher_member_function.py`:
   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MinimalPublisher(Node):
       def __init__(self):
           super().__init__('minimal_publisher')
           self.publisher_ = self.create_publisher(String, 'topic', 10)
           timer_period = 0.5  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0

       def timer_callback(self):
           msg = String()
           msg.data = 'Hello World: %d' % self.i
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

4. Run the publisher:
   ```bash
   ros2 run py_publisher publisher_member_function
   ```

## Learning Objectives

After completing this chapter, you should be able to:
- Explain the core concepts of ROS 2 architecture
- Identify the main components of a ROS 2 system
- Set up a basic ROS 2 workspace
- Create and run a simple publisher node