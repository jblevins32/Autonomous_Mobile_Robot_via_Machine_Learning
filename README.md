# Autonomous Mobile Robot via Machine Learning - CS7641 - Machine Learning

The objective of this project is to develop autonomous, non-holonomic mobile robot ML algorithms that must navigate a Turtlebot3 through an environment to capture a care package

## Table of Contents
- [Features](##Features)
- [File Structure] (##File Structure)
- [Changelog](##Changelog)

## Features
<!-- - Data Collection -->
- Filtering and cleaning sensor data
- Object Detection
- Mapping
- Control

## File Structure
- `data`: folder containing the data from the simulations
    - `raw_dataset_lidar.csv`: lidar sensor data
    - `raw_dataset_odom.csv`: encoder data
    - `merged_lidar_odom_data.csv`: merx`ged data
- `features`
    - `radar_plot_frames`: contains frames of the lidar data
    - `plotting.ipynb`: (Depricated) preprocessing data
- `figs`: Folder containing figures for the report
    - `gantt.png`: GANTT chart
    - `objective_fig.png`: Project objective graph
- `preprocessing`: Preprocessing of data
    - `lidar_preprocessing`: Notebook containing the pipeline to generate the radar graph
    - `radar_plot.gif`: Image of the simulation data from the LIDAR sensor
- `reports`:
    - `figs`: Folder containing figures for the report
        - `gantt.png`: GANTT chart
        - `objective_fig.png`: Project objective graph
    - `main.pdf`: PDF version of the report
    - `main.tex`: Latex file of the report
- `src`: folder containing the ML pipelines
    - `ml_robotics_interfaces`: ROS program code
    - `ml_robotics_project`: ROS program code
    - `object_detection`: Object detection pipeline
        - `predict`: Script used for predition
        - `yolo_training.py`: Script used for Object detection training using YOLO
    - `turtlebot3_example_code`: Pre-build Turtlebot ROS program code for reference
    - `q_learning.ipynb`: Q-Learning for path planning and control
- `rosbag_to_csv.py`: Python script to generate the data from the simulation
- `simulation.md`: Readme to run a simulation


## Changelog
All the changes in the repository will be documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

### Version 0.3
This is the final deliverable version


### Version 0.2

This is the midterm deliverable version
- Added data
- Preprocessing and Cleaned Data
- Deployed ML algorithms
- Generated initial results and graphs

### Version 0.1

This is the initial push of our project.
- Completed the project proposal
- Initialized the project page on Github
- Compiled a guide to run simulations
