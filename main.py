# DWA navigation Simulation

# Import necessary libraries and modules

import numpy as np
import matplotlib.pyplot as plt
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
        self.V = sqrt(self.vx ^ 2 + self.vy ^ 2)
        self.ax = 0
        self.ay = 0
        self.W = 0

        self.Traj = []

        self.Vmax = 3  # [m/s]
        self.Wmax = 90.0 * pi / 180.0  # [rad/s]
        self.acc_max = 10  # [m/ss]
        self.RotAccMax = 120.0 * pi / 180.0  # [rad/ss]
        self.Dimensions = 10  # Circle Radius

    def update(self, dt):  # Update the robot position and velocities based on the acceleration and time interval
        # Velocity update
        self.vx = self.vx + self.ax * dt
        self.vy = self.vy + self.ay * dt
        # Position update
        self.x = self.x + self.vx * dt
        self.y = self.y + self.vy * dt
        # Heading update
        self.theta = self.theta + self.W * dt

    def dist_to_obs(self, obs):
        DistRobotToObs = sqrt(((self.x - obs.x) ** 2) + (self.y - obs.y) ** 2) + (obs.radius / 10)
        return DistRobotToObs


class Env:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.obstacles = []
        self.goal = ()

    def add_obstacle(self, obs):
        # Add an obstacle to the environment
        self.obstacles.append(obs)

    def make_random_obstacles(self, NumberOfObs):
        # The function creates a random obstacles (In a loop) and using the insert them into the environment object

        # Output: ----------------
        #
        pass

    def set_goal(self, x, y):
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
        self.HEADING = 0.4
        self.SPEED = 0.5
        self.AVOIDANCE = 0
        self.SIGMA = 1
        self.speed_Res = 0.1
        self.PredictTime = 3
        self.ArriveTolerance = 0.3


# ----------- Define functions of the algorithm (Except the simulation function) -------------

def simulation(robot, env):
    # Create a plot to visualize the simulation
    ax = plt.subplot()

    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])
    plt.grid()

    # Update the robot's position and plot it
    # robot.update(TIME_STEP)

    ax.clear()
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])

    plt.plot(robot.x, robot.y, marker="o", markersize=robot.Dimensions, markeredgecolor="red", markerfacecolor="green")

    for i in range(len(robot.Traj[0][:])):
        plt.plot((robot.Traj[0][i]), (robot.Traj[1][i]),
                 color="red", marker='*', markersize=2)

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

    V_d = [robot.V - (robot.acc_max * dt),
           robot.V + (robot.acc_max * dt)]

    V_r = [max(V_ss[0], V_d[0]), min(V_ss[1], V_d[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and dynamic)
    # MIN between the maximum speeds (search and dynamic)

    # ----Rotational speed axis-----#
    W_ss = [-robot.Wmax, robot.Wmax]

    W_d = [robot.W - (robot.RotAccMax * dt),
           robot.W + (robot.RotAccMax * dt)]

    W_r = [max(W_ss[0], W_d[0]), min(W_ss[1], W_d[1])]
    # Explanation for the intersection above:
    # MAX between the minimum speeds (search and dynamic)
    # MIN between the maximum speeds (search and dynamic)

    return V_r, W_r


def goal_cost(trajectory, goal):
    delta_x = (goal[0] - trajectory[0][-1])  # trajectory[x position][last value] - Take the end of the trajectory
    delta_y = (goal[1] - trajectory[1][-1])

    delta_head = atan2(delta_y, delta_x)

    GoalCostValue = abs(delta_head - trajectory[2][-1])  # trajectory[Theta Pos][Last Value]

    return GoalCostValue


def obstacle_cost(DistToObs):
    DistFromObsCost = 1 / DistToObs
    return DistFromObsCost


def speed_cost(robot, vel):
    SpeedCostValue = robot.Vmax - vel

    return SpeedCostValue


def create_and_choose_trajectory(env, robot, dynamic_win, dwa_param):
    # This Function will generate each nominated trajectory, and choose the best one so far in each inside loop.

    # Input: robot class, the dynamic window, and speed resolution
    # Output: The function returns a lists of trajectory instances

    straight_vel_list = list(np.arange(dynamic_win[0][0], dynamic_win[0][1], dwa_param.speed_Res))
    # list of all the velocities in the V axis, by a defined resolution - straight velocities

    rotational_vel_list = list(np.arange(dynamic_win[1][0], dynamic_win[1][1], dwa_param.speed_Res))
    # list of all the velocities in the W axis, by a defined resolution - rotational

    Best_U_vector = [0, 0]  # Initial values
    BestTraj = [[]]

    #  Both of the speeds V_a and W_a, define as admissible velocities. These velocities are the last component of the
    #  window. instead of integrate it in the dynamic window function, for me, it made more sense to use them here.
    #  The formula comes form the kinematic equations of motion of a two wheeled robot on a 2d plane.
    DistToNearObs, NearestObs = find_nearest_obs(robot, env)
    V_a = [sqrt(2 * DistToNearObs * robot.acc_max)]
    W_a = [sqrt(2 * DistToNearObs * robot.RotAccMax)]

    MinTotalCost = float(inf)  # Define the initial cost to infinity - for minimizing the cost function

    for vel in straight_vel_list:

        for omega in rotational_vel_list:

            if vel < V_a and omega < W_a:  # Take these velocities only if it is safe for the robot to be able to stop
                # before collision

                PredictTraj = trajectory_prediction(robot, vel, omega, TIME_STEP, dwa_param)

                TotalCost = dwa_param.SIGMA * ((goal_cost(PredictTraj, env.goal) * dwa_param.HEADING) +
                                               (speed_cost(robot, vel) * dwa_param.SPEED) +
                                               (obstacle_cost(DistToNearObs) * dwa_param.AVOIDANCE))

                if MinTotalCost >= TotalCost:
                    MinTotalCost = TotalCost
                    Best_U_vector = [vel, omega]
                    BestTraj = PredictTraj

            # This trajectory needs to be compared to the last trajectory (In terms of cost)
            # The most valuable trajectory will be chosen and returned from this function
    robot.Traj = BestTraj
    return Best_U_vector, BestTraj


def trajectory_prediction(robot, vel, omega, dt, dwa_param):
    # The following for loop will have to be done on some list that will contain all the changes
    # It shouldn't have permission to change something on the actual robot

    Traj_X_PositionList = []
    Traj_Y_PositionList = []
    Traj_THETA_PositionList = []

    StepNumList = []

    Temp_robotX = robot.x
    Temp_robotY = robot.y
    Temp_robotTHETA = robot.theta

    for step_num in np.arange(0, dwa_param.PredictTime, dt):
        Temp_robotX = Temp_robotX + (vel * cos(Temp_robotTHETA) * dt)
        Traj_X_PositionList.append(Temp_robotX)

        Temp_robotY = Temp_robotY + (vel * sin(Temp_robotTHETA) * dt)
        Traj_Y_PositionList.append(Temp_robotY)

        Temp_robotTHETA = Temp_robotTHETA + (omega * dt)
        Traj_THETA_PositionList.append(Temp_robotTHETA)

        StepNumList.append(step_num)  # Not used

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
    # Function that runs the algorithm, for one time step.
    # The function gets a given state (of the robot and the environment).
    # The function outputs the speed command for the robot (Forward and rotational velocity)

    Dyn_Win_edges = dynamic_window(robot, dt)
    u, BestTraj = create_and_choose_trajectory(env, robot, Dyn_Win_edges, dwa_param)

    robot.vx = u[0] * cos(robot.theta)
    robot.vy = u[0] * sin(robot.theta)
    robot.W = u[1]

    robot.update(TIME_STEP)

    robot.Traj = BestTraj


def find_nearest_obs(robot, env):
    Dist = float('inf')
    closest_distance = float('inf')
    closest_obs = 0

    for obs in env.obstacles:
        Dist = robot.dist_to_obs(obs)
        if Dist < closest_distance:
            closest_distance = Dist
            closest_obs = obs
    return Dist, closest_obs


def obstacles_on_traj(AllObstaclesList, PredictedTraj):

    for obs in AllObstaclesList:

        for x, y in zip(PredictedTraj[0], PredictedTraj[1]):

            if (abs(x - obs.x) < obs.radius) and (abs(y - obs.y) < obs.radius):
                return 1
            else:
                return 0


def main():
    # Create a new environment, add obstacles and goal.
    envFrame = Env(10, 10)
    envFrame.add_obstacle(obstacle(x=8, y=4, radius=15))  # For comparison, the size of the robot is 10
    envFrame.add_obstacle(obstacle(x=3, y=4, radius=15))
    envFrame.add_obstacle(obstacle(x=6, y=8, radius=15))
    envFrame.add_obstacle(obstacle(x=3.5, y=6, radius=15))
    envFrame.add_obstacle(obstacle(x=5, y=2, radius=15))
    envFrame.set_goal(8, 2)

    DWA_Parameters = DWA_Config()  # Create the algorithm configuration object

    robot_proto = Robot(1, 2, (45 * pi / 180))  # Create the Robot entity - gets [x, y, theta]

    # -------------- Motion planner part -----------------------------------
    # This part can be placed inside it's own function.

    while not arrived_to_goal(robot_proto, envFrame, DWA_Parameters):

        dwa_planner(envFrame, DWA_Parameters, robot_proto, TIME_STEP)

        simulation(robot_proto, envFrame)

    print("Arrived To Goal!")

    simulation(robot_proto, envFrame)  # Keep showing the simulation after arrived to goal
    plt.show()


if __name__ == "__main__":
    main()
