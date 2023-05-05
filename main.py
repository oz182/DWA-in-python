# DWA navigation Simulation

# Import necessary libraries and modules

import numpy as np
import matplotlib.pyplot as plt
import time
from math import *

TIME_STEP = 0.01  # Simulation Time step


# ---------------------- Define classes and functions for the simulation ---------------------

class Config:

    # This function will store all the configuration parameters for the simulation and for other main purposes

    def __init__(self):
        self.dt = 0.1


class Robot:

    # This class create an instance of a robot.
    # The robot has a position (x,y), heading (theta), and a maximum velocity and rotational velocity
    # The robot also should have a maximum acceleration.

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
        self.RotAccMax = 1

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
        self.goal = ()

    def add_obstacle(self, obstacle):
        # Add an obstacle to the environment
        self.obstacles.append((obstacle))

    def RanddomObstacles(self, NumberOfObs, Env):
        # The function creates a random obstacles (In a loop) and using the insert them into the environment object

        # Output: ----------------
        #
        pass

    def SetGoal(self, x, y):
        # In this function the user can set the goal position for the robot (just x and y coordinates)
        self.goal = (x, y)
        pass


class obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


# ------------------------------ Option for algorithm classes --------------------------------


class DWA_Config:
    def __init__(self):
        self.HEADING = 1
        self.SPEED = 1
        self.AVOIDANCE = 1
        self.SIGMA = 1
        self.speed_Res = 0.01
        self.PredictTime = 3


class Trajectory:
    def __init__(self, V, W):
        self.xPos = []
        self.yPos = []
        self.AnglePos = []
        self.V = V
        self.W = W

    def NewTraj(self, x, y):
        self.xPos.append(x)
        self.yPos.append(y)


# ----------- Define functions of the algorithm (Except the simulation function) -------------

def simulation(robot, env, TIME_STEP):
    # Create a plot to visualize the simulation
    fig = plt.figure()
    ax = plt.subplot()
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])
    plt.grid()

    # ax.set_aspect('equal')
    # plt.ion()
    # plt.plot(robot.x, robot.y, marker="o", markersize=15, markeredgecolor="red", markerfacecolor="green")
    # plt.show()

    # Update the robot's position and plot it
    while True:
        robot.update(TIME_STEP)
        ax.clear()
        ax.set_xlim([0, env.width])
        ax.set_ylim([0, env.height])
        plt.plot(robot.x, robot.y, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
        for obs in env.obstacles:
            plt.plot(obs.x, obs.y, marker='o', markersize=obs.radius, markeredgecolor="black", markerfacecolor="red")
        plt.plot(env.goal[0], env.goal[1], marker="s", markersize=10, markeredgecolor="blue", markerfacecolor="green")
        plt.grid()
        plt.draw()
        plt.pause(0.001)


def dynamic_window(robot, dt):
    # This function creates the dynamic window. This window can be imagine as a two axis graph where:
    # y axis - vertical speed
    # x axis - rotational speed

    # Function input: Robot object, time interval dt
    # Function output: array of values contains the vertical and rotational speeds that creates the window

    # ----Straight speed axis------#
    V_searchspace = [0, robot.Vmax]
    # array from min speed (negative max speed - for now i kept it as zero), max speed, with some steps

    V_admirable = [robot.vx - (robot.acc_max * dt),
                   robot.vx + (robot.acc_max * dt)]  # Only Vx... Might needs to be Vxy, or add another list of Vy

    V_dynamic = [max(V_searchspace[0], V_admirable[0]), min(V_searchspace[1], V_admirable[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and admirable)
    # MIN between the maximum speeds (search and admirable)

    # ----Rotational speed axis-----#
    W_searchspace = [-robot.Wmax, robot.Wmax]

    W_admirable = [robot.W - (robot.RotAccMax * dt),
                   robot.W + (robot.RotAccMax * dt)]
    W_dynamic = [max(W_searchspace[0], W_admirable[0]), min(W_searchspace[1], W_admirable[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and admirable)
    # MIN between the maximum speeds (search and admirable)

    return V_dynamic, W_dynamic

    pass


def dynamic_window_v2(robot, dt):
    # Construction of the dynamic window will be as follow:
    # V_ss (Search space), V_a (Collision detection), V_d (The robot's dynamics), V_r (intersection between all)

    V_ss = [0, robot.Vmax, -robot.Wmax, robot.Wmax]

    # V_a will be added in advanced steps

    V_d = [robot.vx - robot.acc_max * dt, robot.vx + robot.acc_max * dt,
           robot.W - robot.acc_max * dt, robot.W + robot.acc_max * dt]

    V_r = [max(V_ss[0], V_d[0]), min(V_ss[1], V_d[1]),
           max(V_ss[2], V_d[2]), min(V_ss[3], V_d[3])]

    return V_r

    pass


def create_and_choose_trajectory(robot, dynamic_win, dwa_param):
    # This Function will generate each nominated trajectory, and choose the best one so far in each inside loop.

    # Input: robot class, the dynamic window, and speed resolution
    # Output: The function returns a lists of trajectory instances

    straight_vel_list = list(np.arange(dynamic_win[0][0], dynamic_win[0][1], dwa_param.speed_Res))
    # list of all the velocities in the V axis, by a defined resolution - straight velocities

    rotational_vel_list = list(np.arange(dynamic_win[1][0], dynamic_win[1][1], dwa_param.speed_Res))
    # list of all the velocities in the W axis, by a defined resolution - rotational

    for vel in straight_vel_list:

        for omega in rotational_vel_list:
            PredictTraj = trajectory_prediction(robot, vel, omega, TIME_STEP, dwa_param)

        pass

    pass

    return 0


def trajectory_prediction(robot, vel, omega, dt, dwa_param):
    # The following for loop will have to be done on some list that will contain all the changes
    # It shouldn't have permission to change something on the actual robot

    Traj_X_PositionList = []
    Traj_Y_PositionList = []
    Traj_THETA_PositionList = []

    Temp_robotX = robot.x
    Temp_robotY = robot.y
    Temp_robotTHETA = robot.theta

    for step_num in np.arange(0, dwa_param.PredictTime, dt):
        Temp_robotX = Temp_robotX + (vel * cos(robot.theta) * dt)
        Traj_X_PositionList.append(Temp_robotX)

        Temp_robotY = Temp_robotY + (vel * sin(robot.theta) * dt)
        Traj_Y_PositionList.append(Temp_robotY)

        Temp_robotTHETA = Temp_robotTHETA + (omega * dt)
        Traj_THETA_PositionList.append(Temp_robotTHETA)

    return [Traj_X_PositionList, Traj_Y_PositionList, Traj_THETA_PositionList]
    # Should it be a list or a tuple? Does it matter?


def choose_trajectory(robot, dwa_param):
    pass


def dwa_planner(env, dwa_param, robot, dt):
    Dyn_Win_edges = dynamic_window_v2(robot, dt)

    All_Trajectories = create_and_choose_trajectory(robot, Dyn_Win_edges, dwa_param)

    pass


def dist_from_obs(robot, env, obstacle):
    # Function that calculates the current distance from an obstacle

    dist = sqrt(robot.x + obstacle.x)

    pass


def motion_planner(robot):
    # while robot position is NOT close to the goal position.
    # Close definition will be given with some tolerance.

    # This function will be universal and can be use in other projects.

    pass


def main():
    # Create a new environment and add obstacles to it

    envFrame = Env(10, 10)
    envFrame.add_obstacle(obstacle(x=8, y=4, radius=15))  # For comparison, the size of the robot is 10
    envFrame.add_obstacle(obstacle(x=3, y=7, radius=15))
    envFrame.SetGoal(8, 8)

    DWA_Parameters = DWA_Config()

    robot_proto = Robot(5, 5, 45, 1, 1)
    robot_proto.ax = 0.05  # This value is the one that makes the movement, For now this line is only for the simulation

    dwa_planner(envFrame, DWA_Parameters, robot_proto, TIME_STEP)

    simulation(robot_proto, envFrame, TIME_STEP)  # Simulation test

    pass


if __name__ == "__main__":
    main()
