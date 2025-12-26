---
sidebar_position: 1
title: "Gazebo Simulation Basics"
---

# Gazebo Simulation Basics

## Concept Overview

Gazebo is a powerful open-source robotics simulator that provides realistic physics simulation, high-quality graphics, and convenient programmatic interfaces. It enables safe and reproducible testing of robotics algorithms without the need for physical hardware. Gazebo is widely used in robotics research and development, particularly for testing navigation, manipulation, and perception algorithms.

## Architecture Diagram: Gazebo Integration

```
┌─────────────────────────────────────────────────────────────────┐
│                    Gazebo Integration                          │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    Gazebo    ┌─────────────────┐         │
│  │   ROS 2 Nodes   │◄──Interface──►│   Simulator     │         │
│  │                 │              │                 │         │
│  │  ┌───────────┐  │              │  ┌───────────┐  │         │
│  │  │ Publisher │  │              │  │ Physics   │  │         │
│  │  │   Node    │  │              │  │ Engine    │  │         │
│  │  └───────────┘  │              │  └───────────┘  │         │
│  │  ┌───────────┐  │              │  ┌───────────┐  │         │
│  │  │ Subscriber│  │              │  │ Rendering │  │         │
│  │  │   Node    │  │              │  │ Engine    │  │         │
│  │  └───────────┘  │              │  └───────────┘  │         │
│  └─────────────────┘              │  ┌───────────┐  │         │
│                                   │  │ Sensors   │  │         │
│                                   │  │ Models    │  │         │
│                                   │  └───────────┘  │         │
│                                   └─────────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

## Gazebo World File Structure

A basic Gazebo world file defines the environment, lighting, physics properties, and initial models:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Include the sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Include the ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Your robot model -->
    <include>
      <name>simple_humanoid</name>
      <pose>0 0 1 0 0 0</pose>
      <uri>model://simple_humanoid</uri>
    </include>

    <!-- Additional objects -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.5 0.8</size>
            </box>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Gazebo Launch File for ROS 2

```xml
<launch>
  <!-- Arguments -->
  <arg name="world" default="empty"/>
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- Gazebo server -->
  <node name="gzserver" pkg="gazebo_ros" exec="gzserver"
        args="$(find-pkg-share my_robot_gazebo)/worlds/my_world.sdf
               -e ode
               $(unless $(var gui) --verbose)"
        respawn="false" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Gazebo client -->
  <node name="gzclient" pkg="gazebo_ros" exec="gzclient"
        respawn="false" output="screen"
        unless="$(var headless)"/>

  <!-- Spawn robot -->
  <node name="spawn_robot" pkg="gazebo_ros" exec="spawn_entity.py"
        args="-entity simple_humanoid -topic robot_description -x 0 -y 0 -z 1"
        respawn="false" output="screen"/>
</launch>
```

## Sensor Configuration for Humanoid Robot

Here's how to add various sensors to your humanoid robot model:

```xml
<link name="head_camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="head_camera_joint" type="fixed">
  <parent link="head"/>
  <child link="head_camera_link"/>
  <origin xyz="0.05 0 0" rpy="0 0 0"/>
</joint>

<!-- Camera sensor -->
<gazebo reference="head_camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_link</frame_name>
      <topic_name>head_camera/image_raw</topic_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU sensor -->
<gazebo reference="torso">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <frame_name>torso</frame_name>
    <topic_name>imu/data</topic_name>
    <body_name>torso</body_name>
  </plugin>
</gazebo>

<!-- LiDAR sensor -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.05"/>
    </geometry>
  </visual>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="torso"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
  </sensor>
  <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
    <frame_name>lidar_link</frame_name>
  </plugin>
</gazebo>
```

## Lab Instructions

### Exercise 1: Basic Gazebo Simulation

1. Create a simple world file with your humanoid robot:
   ```bash
   mkdir -p ~/robotics_ws/src/my_robot_gazebo/worlds
   touch ~/robotics_ws/src/my_robot_gazebo/worlds/basic_world.sdf
   ```

2. Add the world content from the example above to your world file

3. Launch Gazebo with your world:
   ```bash
   ros2 launch gazebo_ros empty_world.launch.py world:=~/robotics_ws/src/my_robot_gazebo/worlds/basic_world.sdf
   ```

### Exercise 2: Sensor Integration

1. Add the sensor configurations to your robot's URDF/Xacro file
2. Launch your robot in Gazebo with sensors enabled
3. Subscribe to sensor topics to verify they're publishing data:
   ```bash
   ros2 topic echo /head_camera/image_raw
   ros2 topic echo /imu/data
   ros2 topic echo /scan
   ```

### Exercise 3: Physics Configuration

1. Experiment with different physics engine parameters
2. Adjust the max_step_size and real_time_update_rate
3. Observe the impact on simulation stability and performance

## Troubleshooting Guide

### Common Issues and Solutions

1. **Robot falls through the ground**: Check that your robot has proper collision geometries and inertial properties defined.

2. **Simulation runs too slow**: Reduce the real_time_update_rate or simplify collision geometries.

3. **Sensors not publishing**: Verify that the Gazebo plugins are correctly loaded and that the frame names match between URDF and plugin configuration.

4. **Controllers not working**: Make sure the joint names in your controller configuration match the URDF joint names.

## Learning Objectives

After completing this chapter, you should be able to:
- Create and configure basic Gazebo world files
- Integrate ROS 2 with Gazebo simulation
- Add various sensors (camera, IMU, LiDAR) to your robot model
- Launch and troubleshoot Gazebo simulations
- Configure physics properties for realistic simulation