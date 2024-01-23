# Dynamic Window Approach (DWA) Algorithm Simulation

## Overview

This Python-based simulation implements the Dynamic Window Approach (DWA) algorithm for collision avoidance. The DWA algorithm is a widely-used technique in robotics for real-time navigation and obstacle avoidance. The foundation of this implementation is based on the paper titled "The Dynamic Window Approach to Collision Avoidance" by Dieter Fox, Wolfram Burgard, and Sebastian Thrun.


![2201](https://github.com/oz182/DWA-in-python/assets/91877982/65ed82f4-0def-4954-b5b2-344047b062b2)


## Getting Started

### Prerequisites

Ensure you have the following prerequisites installed:

- Python (version 3.8.X)
- NumPy
- Matplotlib (for visualization, optional)

### Installation

Clone the repository to your local machine:

```bash
git clone https://github.com/your-username/dwa-algorithm-simulation.git
cd dwa-algorithm-simulation
```

### Usage
Run the simulation script:

```bash
python main.py
```
This will execute the DWA algorithm simulation, showcasing its behavior in a simulated environment.

### Algorithm Overview
The DWA algorithm operates on the principles of velocity space and evaluates potential candidate velocities based on dynamic constraints and collision predictions. The mathematical foundation of the algorithm involves the following key components:

1. Kinematic Model
The robot's kinematic model is represented by the state vector [x, y, theta], where x and y are the coordinates, and theta is the orientation.

2. Dynamic Window
The dynamic window is a set of feasible velocities that the robot can achieve within a short time horizon. It is determined by considering the robot's maximum linear and angular accelerations.

3. Trajectory Prediction
Collision predictions are made by simulating trajectories within the dynamic window. These predictions take into account the robot's current state and estimate future positions.

4. Objective Function
The DWA algorithm utilizes an objective function that evaluates trajectories based on proximity to the goal, closeness to obstacles, and adherence to velocity constraints.
