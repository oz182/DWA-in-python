import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation  # FFMpegWriter

# import imageio  # Activate for function "movie_sim2"

fig, ax = plt.subplots()


def simulation(robot, env):
    # Create a plot to visualize the simulation
    # Update the robot's position and plot it
    # robot.update(TIME_STEP)

    plt.grid()
    ax.clear()
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])

    # Robot draw - updated position
    plt.plot(robot.x, robot.y, marker="o", markersize=robot.Dimensions, markeredgecolor="red", markerfacecolor="green")

    # Trajectory plot:
    for i in range(len(robot.Traj[0][:])):
        plt.plot((robot.Traj[0][i]), (robot.Traj[1][i]),
                 color="red", marker='*', markersize=2)

    for obs in env.obstacles:
        plt.plot(obs.x, obs.y, marker='o', markersize=obs.radius, markeredgecolor="black", markerfacecolor="red")

    plt.plot(env.goal[0], env.goal[1], marker="s", markersize=10, markeredgecolor="blue", markerfacecolor="green")

    plt.grid()
    plt.draw()
    plt.pause(0.001)

    return fig


def sim_movie(fig_list):
    # Create the animation
    animation_fig = plt.figure()
    print("Creating animation file: ")

    # Define the animation function
    def animate(i):
        animation_fig.clear()
        plt.imshow(fig_list[i].canvas.renderer.buffer_rgba(), origin='upper')

    # Create the animation object
    ani = FuncAnimation(animation_fig, animate, frames=len(fig_list), interval=200, blit=False)
    print("Collecting simulation data, Please wait...")
    # Save the animation as a video file (e.g., MP4)
    ani.save('animation.gif', writer='pillow')
    print("Animation file has created!")


# Another way to save the animation
# Not working

"""
def sim_movie2(fig_list):
    frame_paths = []
    # Save each figure as a separate image file
    print("Creating animation file: ")
    for i, figu in enumerate(fig_list):
        frame_path = f"frame_{i}.png"  # Provide a file path for each frame
        figu.savefig(frame_path)
        plt.close(figu)  # Close the figure to release memory (optional)
        frame_paths.append(frame_path)

    # Create the animation from the saved frames
    print("Collecting simulation data, Please wait...")
    images = []
    for frame_path in frame_paths:
        images.append(imageio.imread(frame_path))
    imageio.mimsave('animation.gif', images, duration=0.2)
    print("Animation file has created!")
"""
