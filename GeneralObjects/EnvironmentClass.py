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


class obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius
