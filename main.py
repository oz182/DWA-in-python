# DWA navigation Simulation

# Import necessary libraries and modules

import numpy as np
import matplotlib.pyplot as plt
import time
from math import *

TIME_STEP = 0.1  # Simulation Time step


# ---------------------- Define classes and functions for the simulation ---------------------

class Config:

    # This function will store all the configuration parameters for the simulation and for other main purposes

    def __init__(self):
        self.dt = 0.1


class Robot:

    # This class create an instance of a robot.
    # The robot has a position (x,y), heading (theta), and a maximum velocity and rotational velocity
    # The robot also should have a maximum acceleration.

    def __init__(self, x, y, theta):
        self.x = x  # Set x initial position
        self.y = y  # Set y initial position
        self.theta = theta  # Set angle initial position
        self.vx = 0
        self.vy = 0
        self.ax = 0
        self.ay = 0
        self.W = 0

        self.Traj = []

        self.Vmax = 20  # [m/s]
        self.Wmax = 40.0 * pi / 180.0  # [rad/s]
        self.acc_max = 2  # [m/ss]
        self.RotAccMax = 40.0 * pi / 180.0  # [rad/ss]

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
        self.ArriveTolerance = 0.3


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
    ax = plt.subplot()

    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])
    plt.grid()

    # ax.set_aspect('equal')
    # plt.ion()
    # plt.plot(robot.x, robot.y, marker="o", markersize=15, markeredgecolor="red", markerfacecolor="green")
    # plt.show()

    # Update the robot's position and plot it
    # robot.update(TIME_STEP)

    ax.clear()
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])

    plt.plot(robot.x, robot.y, marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")

    for i in range(len(robot.Traj[0])):
        plt.plot((robot.Traj[0][i] * cos(robot.Traj[2][i])), (robot.Traj[1][i] * sin(robot.Traj[2][i])),
                 color="red")

    #  The plot trajectory is not working

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
    V_ss = [-robot.Vmax, robot.Vmax]
    # array from min speed (negative max speed - for now i kept it as zero), max speed, with some steps

    V_d = [robot.vx - (robot.acc_max * dt),
           robot.vx + (robot.acc_max * dt)]  # Only Vx... Might needs to be Vxy, or add another list of Vy

    V_r = [max(V_ss[0], V_d[0]), min(V_ss[1], V_d[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and admirable)
    # MIN between the maximum speeds (search and admirable)

    # ----Rotational speed axis-----#
    W_ss = [-robot.Wmax, robot.Wmax]

    W_d = [robot.W - (robot.RotAccMax * dt),
           robot.W + (robot.RotAccMax * dt)]
    W_r = [max(W_ss[0], W_d[0]), min(W_ss[1], W_d[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and admirable)
    # MIN between the maximum speeds (search and admirable)

    return V_r, W_r

    pass


def goal_cost(trajectory, goal):
    delta_x = abs(goal[0] - trajectory[0][-1])  # trajectory[x position][last value] - Take the end of the trajectory
    delta_y = abs(goal[1] - trajectory[1][-1])

    delta_head = atan2(delta_y, delta_x)

    GoalCostValue = abs(delta_head - trajectory[2][-1])  # trajectory[Theta Pos][Last Value]

    return GoalCostValue


def obstacle_cost(trajectory, obs):
    return 0


def speed_cost(robot, vel):
    SpeedCostValue = robot.Vmax - vel

    return SpeedCostValue


def create_and_choose_trajectory(goal, robot, dynamic_win, dwa_param):
    # This Function will generate each nominated trajectory, and choose the best one so far in each inside loop.

    # Input: robot class, the dynamic window, and speed resolution
    # Output: The function returns a lists of trajectory instances

    straight_vel_list = list(np.arange(dynamic_win[0][0], dynamic_win[0][1], dwa_param.speed_Res))
    # list of all the velocities in the V axis, by a defined resolution - straight velocities

    rotational_vel_list = list(np.arange(dynamic_win[1][0], dynamic_win[1][1], dwa_param.speed_Res))
    # list of all the velocities in the W axis, by a defined resolution - rotational

    MinTotalCost = float(inf)  # Define the initial cost to infinity - for minimizing the cost function

    for vel in straight_vel_list:

        for omega in rotational_vel_list:
            PredictTraj = trajectory_prediction(robot, vel, omega, TIME_STEP, dwa_param)

            TotalCost = dwa_param.SIGMA * ((goal_cost(PredictTraj, goal) * dwa_param.HEADING) +
                                           (speed_cost(robot, vel) * dwa_param.SPEED) +
                                           (obstacle_cost(PredictTraj, goal) * dwa_param.AVOIDANCE))

            if MinTotalCost >= TotalCost:
                MinTotalCost = TotalCost

                Best_U_vector = [vel, omega]
                BestTraj = PredictTraj

            # This trajectory needs to be compared to the last trajectory (In terms of cost)
            # The most valuable trajectory will be chosen and returned from this function
    print(Best_U_vector)
    return Best_U_vector, BestTraj

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


def arrived_to_goal(robot, env, dwa_param):
    CloseToGoal_X = abs(robot.x - env.goal[0])
    CloseToGoal_Y = abs(robot.y - env.goal[1])

    if (CloseToGoal_X <= dwa_param.ArriveTolerance) and (CloseToGoal_Y <= dwa_param.ArriveTolerance):
        # If arrived to goal
        return 1
    else:
        return 0


def dwa_planner(env, dwa_param, robot, dt):
    Dyn_Win_edges = dynamic_window(robot, dt)
    u, BestTraj = create_and_choose_trajectory(env.goal, robot, Dyn_Win_edges, dwa_param)

    robot.vx = u[0] * cos(robot.theta)
    robot.vy = u[0] * sin(robot.theta)
    robot.W = u[1]

    robot.update(TIME_STEP)

    robot.Traj = BestTraj


def dist_from_obs(robot, env, obstacle):
    # Function that calculates the current distance from an obstacle

    dist = sqrt(robot.x + obstacle.x)

    pass


def main():
    # Create a new environment, add obstacles and goal.
    envFrame = Env(10, 10)
    envFrame.add_obstacle(obstacle(x=8, y=4, radius=15))  # For comparison, the size of the robot is 10
    envFrame.add_obstacle(obstacle(x=3, y=7, radius=15))
    envFrame.SetGoal(2, 8)

    DWA_Parameters = DWA_Config()  # Create the algorithm configuration object

    robot_proto = Robot(1, 2, (45 * pi / 180))  # Create the Robot entity

    # -------------- Motion planner part -----------------------------------
    # This part can be placed inside it's own function.

    while not arrived_to_goal(robot_proto, envFrame, DWA_Parameters):
        dwa_planner(envFrame, DWA_Parameters, robot_proto, TIME_STEP)

        simulation(robot_proto, envFrame, TIME_STEP)

    print("Arrived To Goal!")


if __name__ == "__main__":
    main()
