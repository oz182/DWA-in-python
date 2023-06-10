import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter  # ArtistAnimation  # FFMpegWriter

# import imageio  # Activate for function "movie_sim2"

fig, ax = plt.subplots(figsize=(6, 6))

metadata = dict(title="movie", artist="RobotAlgo")
writer = PillowWriter(fps=15, metadata=metadata)


def simulation(robot, env):
    # Create a plot to visualize the simulation
    # Update the robot's position and plot it
    # robot.update(TIME_STEP)
    ax.clear()
    ax.set_xlim([0, env.width])
    ax.set_ylim([0, env.height])

    # Robot draw - updated position
    ax.plot(robot.x, robot.y, marker="o", markersize=robot.Dimensions, markeredgecolor="red", markerfacecolor="green")

    # Trajectory plot:
    for i in range(len(robot.Traj[0][:])):
        ax.plot((robot.Traj[0][i]), (robot.Traj[1][i]),
                color="red", marker='*', markersize=2)

    for obs in env.obstacles:
        ax.plot(obs.x, obs.y, marker='o', markersize=obs.radius, markeredgecolor="black", markerfacecolor="red")

    ax.plot(env.goal[0], env.goal[1], marker="s", markersize=10, markeredgecolor="blue", markerfacecolor="green")

    ax.grid()
    plt.draw()
    plt.pause(0.01)

    CurrentFig = plt.gcf()
    plt.cla()

    return CurrentFig


def sim_movie(fig_list):
    # Create the animation
    print("Creating animation file: ")

    # Define the animation function
    def animate(i):
        # animation_fig.clear()
        return plt.imshow(fig_list[i].canvas.renderer.buffer_rgba(), origin='upper')

    # Create the animation object
    ani = FuncAnimation(fig_list[0], animate, frames=len(fig_list), interval=1000, blit=False)
    # ani = ArtistAnimation(animation_fig, fig_list)  #  Save a list of images
    print("Collecting simulation data, Please wait...")

    # Save the animation as a video file (e.g., MP4)
    ani.save('animation.gif', writer='pillow')  # ---------- Uncomment to save the .gif file -----------
    print("Animation file has created!")


# Another way to save the animation
# Not working so good

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
