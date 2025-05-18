# Autonomous Vehicle and Robotics 

An educational simulation and visualization tool for autonomous vehicles and robotics with path planning, sensor simulation, and real-time telemetry.

![Autonomous Vehicle Simulation](assets/robot.svg)

## Overview

This application is an educational simulation and visualization tool for autonomous vehicles and robotics. It serves as an interactive platform to explore core concepts in robotics navigation, sensor simulation, and path planning algorithms.

## Main Features

### 1. Interactive Simulation Environment
- **Dynamic Grid System**: A customizable 2D grid environment where an autonomous vehicle navigates
- **Obstacle Management**: Add, remove, or randomly generate obstacles that the vehicle must avoid
- **Real-time Visualization**: Watch the vehicle navigate through the environment with intuitive graphics

### 2. Path Planning Algorithms
- **A* Algorithm**: An efficient pathfinding algorithm that uses a heuristic to estimate the distance to the goal
- **Dijkstra's Algorithm**: A graph search algorithm that finds the shortest path from a starting point to all other points
- **Algorithm Comparison**: Test and compare the performance of these algorithms in different environments

### 3. Sensor Simulation
- **LiDAR Simulation**: Visualize how LiDAR sensors detect obstacles by casting rays in various directions
- **Proximity Detection**: Experience how vehicles detect nearby obstacles to avoid collisions
- **Sensor Visualization**: See the vehicle's "perception" of its environment through intuitive radar-like displays

### 4. Dual Control Modes
- **Autonomous Mode**: Let the vehicle navigate automatically using the selected pathfinding algorithm
- **Manual Mode**: Control the vehicle yourself using directional controls to compare human vs. algorithmic performance

### 5. Telemetry and Data Analysis
- **Position Tracking**: Monitor the vehicle's position, heading, and velocity in real-time
- **Telemetry Graphs**: Visualize the trajectory and movement data as the simulation progresses
- **Performance Metrics**: Analyze distance traveled, time taken, and efficiency of navigation

### 6. Database Integration
- **Save/Load Maps**: Create and store custom environments for later use
- **Record Simulations**: Capture entire simulation runs to replay or analyze later
- **Analytics Dashboard**: Compare different algorithms and control strategies with data visualizations

## How It Works

1. **Environment Setup**: The application creates a grid environment where you can place obstacles and set start/goal positions
2. **Path Planning**: When in autonomous mode, the selected algorithm (A* or Dijkstra) calculates the optimal path
3. **Sensor Integration**: The vehicle uses simulated sensors to detect obstacles in its environment
4. **Navigation Logic**: Based on sensor data and the planned path, the vehicle makes movement decisions
5. **Data Collection**: Throughout the simulation, telemetry data is collected and can be stored in the database
6. **Visualization**: All aspects of the simulation are visually represented in an intuitive interface

## Getting Started

### Prerequisites
```
streamlit
numpy
matplotlib
pandas
plotly
opencv-python
sqlalchemy
psycopg2-binary
```

### Installation

1. Clone the repository
```bash
git clone https://github.com/yourusername/autonomous-vehicle-simulation.git
cd autonomous-vehicle-simulation
```

2. Install required packages
```bash
pip install -r requirements.txt
```

3. Set up the database
```bash
python init_db.py
```

4. Run the application
```bash
streamlit run app.py
```

## Educational Value

This application helps students and hobbyists understand the fundamental concepts of autonomous navigation, obstacle avoidance, and sensor systems without requiring physical robots. It's perfect for educational purposes, allowing users to experiment with different algorithms and environments to understand the challenges of autonomous vehicle navigation.