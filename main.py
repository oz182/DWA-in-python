# DWA navigation Simulation

# Import necessary libraries and modules

from GeneralObjects.EnvironmentClass import *
from GeneralObjects.RobotClass import *
from NavAlgo.DynamicWindowAlgo import *
from Simulation import *

TIME_STEP = 0.1  # Algorithm Time step


def main():
    # Collect the current frame from the simulation
    SimFrames = []

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

    while not arrived_to_goal(robot_proto, envFrame, DWA_Parameters):
        dwa_planner(envFrame, DWA_Parameters, robot_proto, TIME_STEP)

        SimCurrentFrame = simulation(robot_proto, envFrame)

        SimFrames.append(SimCurrentFrame)

    print("Arrived To Goal!")

    # Keep showing the simulation after arrived to goal (Hold the last frame)
    simulation(robot_proto, envFrame)

    # Un comment to save the animation video
    # sim_movie(SimFrames)  # Outputs mp4 animation file
    # plt.show() # Uncomment if you want to keep the last frame presented on the screen


if __name__ == "__main__":
    main()
