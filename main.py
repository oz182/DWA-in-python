# DWA navigation Simulation

# Import necessary libraries and modules

import numpy as np
import matplotlib.pyplot as plt
import time

# Define classes and functions for the simulation

class Robot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.ay = 0

    def update(self, dt): # Update the robot position and velocities based on the acceleration and time interval
        # Velocity update
        self.vx = self.vx + self.ax * dt
        self.vy = self.vy + self.ay * dt
        # Position update
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt

class Enviourment:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

    def add_obstacle(self, obstacle):
        # Add an obstacle to the environment
        self.obstacles.append(obstacle) # obstacle might need to be a class of itself?

def animate(robot, env):
    # Create a plot to visualize the simulation
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])
    ax.set_aspect('equal')
    plt.ion()
    plt.show()

    # Update the robot's position and plot it
    while True:
        robot.update(dt=0.1)
        ax.clear()
        for obstacle in env.obstacles:
            ax.add_patch(obstacle.get_patch())
        ax.add_patch(robot.get_patch())
        plt.draw()
        plt.pause(0.001)

# Main code for the simulation

def main():
    # Create a new environment and add obstacles to it
    #env = Environment(width=10, height=10)
    #env.add_obstacle(RectangleObstacle(x
    print("Hello world!")
    pass

if __name__ == "__main__":
    main()