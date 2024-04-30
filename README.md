# Synthetic ROS Package

```
# 패키지 이름은 synthetic_rospkg 입니다. 폴더 이름을 이렇게 잡아주세요
git clone https://github.com/PLAIF-dev/sw_synthetic_rospkg.git synthetic_rospkg
```

## Overview

This ROS package, `synthetic_rospkg`, facilitates the generation and management of synthetic data for robotics
applications. It includes tools for editing textures, object modeling, and creating dynamic environments for simulation
and testing purposes.

## Repository Setup

To clone this repository, use the following command:

```bash
git clone https://github.com/PLAIF-dev/sw_synthetic_rospkg.git synthetic_rospkg
```

## Features

- **3D Object Models**: Includes detailed 3D models for various objects with customizable textures.
- **Texture Editing**: Tools for adjusting and applying textures to 3D models to simulate various environmental
  conditions.
- **Dynamic Scene Generation**: Scripts to automate the generation of synthetic scenes with configurable parameters.

## Structure

The package structure includes:

- `script`: Contains Python scripts like `syn_launcher.py` for launching the synthetic data generator
  and `wait_capture.py` for handling synchronization with data capture systems.
- `vs_synthetic_generator`: A comprehensive set of tools and models for generating synthetic data, including detailed
  object models and textures.

## Dependencies

This package requires ROS Kinetic or newer and depends on:

- `rospy`
- `sensor_msgs`
- `std_msgs`

## Building the Package

Ensure the following dependencies are included in your `CMakeLists.txt`:

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)
```

## Usage

To generate synthetic data, run the `syn_launcher.py` script located in the `script` directory. Adjust the
configurations in `labels.json` to modify the data generation process according to your needs.
