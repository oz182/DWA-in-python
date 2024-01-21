from math import sqrt, pi


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
        # The robot has the ability to tell the distance from an obstacle

        DistRobotToObs = sqrt(((self.x - obs.x) ** 2) + (self.y - obs.y) ** 2) + (obs.radius / 10)
        return DistRobotToObs
    
    def is_obs_on_path(self, obs):
        # The robot needs to have the abilty to tell if there is an obstacle ahead of it on the desierd path
        
        pass
