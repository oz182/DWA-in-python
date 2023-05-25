import matplotlib.pyplot as plt


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
