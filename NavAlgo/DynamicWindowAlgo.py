# Import necessary libraries and modules

import numpy as np
from math import *

TIME_STEP = 0.1  # Simulation Time step


# ------------------------------ Option for algorithm classes --------------------------------

class DWA_Config:
    def __init__(self):
        self.HEADING = 0.4
        self.SPEED = 0.5
        self.AVOIDANCE = 0.1
        self.SIGMA = 1
        self.speed_Res = 0.1
        self.PredictTime = 3
        self.ArriveTolerance = 0.3

        self.dt = 0.1


# ----------- Define functions of the algorithm (Except the simulation function) -------------


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

    # delta_head_deg = delta_head * (180 / pi)

    # traj_head_deg = trajectory[2][-1] * (180 / pi)

    GoalCostValue = abs(delta_head - trajectory[2][-1])  # trajectory[Theta Pos][Last Value]
    GoalCostValue_deg = GoalCostValue * (180 / pi)

    Norm_GoalCost = GoalCostValue_deg / 180

    return Norm_GoalCost


def obstacle_cost(DistToObs):
    DistFromObsCost = 1 / DistToObs
    return DistFromObsCost


def speed_cost(robot, vel):
    SpeedCostValue = abs(robot.Vmax - vel)

    # Normalization for the result:
    Norm_SpeedCost = SpeedCostValue / robot.Vmax

    return Norm_SpeedCost


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

                GoalCost = (goal_cost(PredictTraj, env.goal)) * dwa_param.HEADING
                SpeedCost = (speed_cost(robot, vel)) * dwa_param.SPEED
                ObstacleCost = (obstacle_cost(DistToNearObs)) * dwa_param.AVOIDANCE

                TotalCost = dwa_param.SIGMA * (GoalCost + SpeedCost + ObstacleCost)

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

    robot.update(dt)

    robot.Traj = BestTraj


def find_nearest_obs(robot, env):
    Dist = float('inf')
    closest_distance = float('inf')
    closest_obs = 0

    for obs in env.obstacles:
        Dist = robot.dist_to_obs(obs)  # The robot has the ability to tell the distance from an obstacle
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
