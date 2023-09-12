import matplotlib.pyplot as plt
import pandas as pd

# Load the trajectory data from the CSV file
df = pd.read_csv("robot_trajectory.csv", delimiter=',')

# Extract the x and y positions for each robot
robot_positions = df.values

# Number of robots (assuming all have the same number of data points)
num_robots = robot_positions.shape[1] // 2

# Split the data into x and y positions for each robot
x_positions = robot_positions[:, :num_robots]
y_positions = robot_positions[:, num_robots:]

# Create a plot for each robot's trajectory
plt.figure(figsize=(10, 6))
for i in range(num_robots):
    plt.plot(x_positions[:, i], y_positions[:, i], label=f"Robot {i + 1}")

# Add labels and legend
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.legend()

# Show the plot
plt.title("Robot Trajectories")
plt.grid(True)
plt.show()
