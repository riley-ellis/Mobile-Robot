# Mobile Robot Project - Prototype III

## Overview
This repository contains the ROS2 nodes and scripts for the third prototype of my mobile robot project. This advanced prototype integrates remote control via a PlayStation 4 controller, GPS and IMU data collection, RPLIDAR A1 sensor for mapping, and a dataset generation for future machine learning model development.

## Features
- **Remote Control**: Control the robot using a PlayStation 4 controller over Bluetooth.
- **GPS Following Mode**: Track a companion device with GPS (ESP32) and enable the robot to follow its position.
- **IMU Data Collection**: Utilize IMU for orientation and heading data.
- **Lidar Mapping**: Experiment with RPLIDAR A1 for mapping and environmental data collection.
- **Data-Driven Control**: Create a dataset by mapping lidar data to control inputs for future machine learning model development.

## Components
The project includes the following ROS2 nodes:
- **`control_kevin.py`**: Manages PS4 controller inputs to control the robot.
- **`data_collector_node.py`**: Collects GPS and IMU data for storage and analysis.
- **`gps_car_node.py`**: Acquires GPS data for the robot.
- **`imu_node.py`**: Publishes IMU heading data.
- **`lidar_to_csv_node.py`**: Processes and logs LiDAR and control data.
- **`lidar_scan_node.py`**: Interfaces with RPLIDAR and publishes `LaserScan` messages.
- **`kevin_components.py`**: Core functionalities including motor controls.

## Clone the repository:
git clone https://github.com/riley-ellis/Mobile-Robot.git

### BerryIMU
This project uses BerryIMU for IMU interfacing. The BerryIMU code is not included in this repository. Visit the [BerryIMU website](http://ozzmaker.com/berryimu) for installation instructions and place it in `/robot_controller/BerryIMU`.

## Data Collection
Detail the procedures for collecting GPS, IMU, and LiDAR data, including instructions on initiating data logging and the location where data is stored.

## Future Work
- Integration of GPS following mode.
- Development of mapping functionalities using RPLIDAR A1.
- Machine learning model for autonomous navigation.
