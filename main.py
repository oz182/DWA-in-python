# DWA navigation Simulation

# Import necessary libraries and modules

import numpy as np
import matplotlib.pyplot as plt
import time
from math import *

# Define classes and functions for the simulation

dt = 0.1  # need to think about the right place for this value. and to make it a constant


class Robot:

    # This class create an instance of a robot.
    # The robot has a position, heading (theta), and a maximum velocity and rotational velocity
    # The robot also should have a maximum acceleration, I should add it.

    def __init__(self, x, y, theta, Vmax, Wmax):
        self.x = x
        self.y = y
        self.theta = theta
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.ay = 0
        self.W = 0

        self.Vmax = Vmax
        self.Wmax = Wmax
        self.acc_max = 1
        self.omegaMax = 1

    def update(self, dt):  # Update the robot position and velocities based on the acceleration and time interval
        # Velocity update
        self.vx = self.vx + self.ax * dt
        self.vy = self.vy + self.ay * dt
        # Position update
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        # Heading update
        self.theta = self.theta + self.W * dt


class Env:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []

    def add_obstacle(self, obstacle):
        # Add an obstacle to the environment
        self.obstacles.append((obstacle.x, obstacle.y, obstacle.radius))  # obstacle might need to be a class of itself? What is an obstacle?

class obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


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
            # ax.add_patch(obstacle.get_patch())  # Not clear what is get_patch??
            pass
        # ax.add_patch(robot.get_patch())
        plt.draw()
        plt.pause(0.001)


# Define the weights od the DWA algorithm

HEADING = 1
SPEED = 1
AVOIDANCE = 1
SIGMA = 1


def dynammic_window(robot, dt):
    # This function creates the dynamic window. This window can be imagine as a two axis graph where:
    # y axis - vertical speed
    # x axis - rotational speed

    # Function input: Robot object, time interval dt
    # Function output: array of values contains the vertical and rotational speeds that creates the window

            #----Stright speed axis------#
    V_searchspace = [0, robot.Vmax]
    # array from min speed (negative max speed), max speed, with some steps

    V_admirable = [robot.vx - (robot.acc_max * dt), robot.vx + (robot.acc_max * dt)] # Only Vx... Might needs to be Vxy, or add another list of Vy

    V_dynamic = [min(V_searchspace[0], V_admirable[0]), max(V_searchspace[1], V_admirable[1])]

            #----Rotational speed axis-----#
    W_search = [-robot.Wmax, robot.Wmax]
    W_avoid = []
    W_dynamic = []

    return V_dynamic, W_dynamic

    pass


def dwa_planner(robot, env):


    pass


def distFromObs(robot, env, obstacle):
    # Function that calculates the current distance from an obstacle

    dist = sqrt(robot.x + obstacle.x)



    pass


def main():
    # Create a new environment and add obstacles to it

    envFrame = Env(10, 10)
    # env.add_obstacle(RectangleObstacle(x))

    robot_proto = Robot(0.5, 0.5, 45, 1, 1)

    # animate(robot_proto, envFrame)

    print("Hello world!")

    print(dynammic_window(robot_proto, 0.1))

    pass


if __name__ == "__main__":
    main()
