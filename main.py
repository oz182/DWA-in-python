# DWA navigation Simulation

# Import necessary libraries and modules

from GeneralObjects.EnvironmentClass import *
from GeneralObjects.RobotClass import *
from NavAlgo.DynamicWindowAlgo import *
from Simulation import *

TIME_STEP = 0.1  # Simulation Time step


# ---------------------- Define classes and functions for the simulation ---------------------

class Config:

    # This function will store all the configuration parameters for the simulation and for other main purposes

    def __init__(self):
        self.dt = 0.1


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
