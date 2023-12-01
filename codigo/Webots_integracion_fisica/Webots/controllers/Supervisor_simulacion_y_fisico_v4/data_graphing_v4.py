import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
#trajectory = np.load('robot_trajec.npy')
#trajectory = np.load('trial.npy')
#trajectory = np.load('setup_traj0_formacion_2.npy')

data = np.load('trial0.npz')
#print(data)

#to find out the files within the .npz
"""
data = np.load('trial.npz', allow_pickle=True)
print(data.files)
"""

trajectory_data = data['trajectory_data']
velocity_data = data['velocity_data']
normV_data = data['normV_data']
obj_data = data['obj_data']
obs_data = data['obs_data']
total_cycle = data['total_cycle']
form_cycle = data['form_cycle']
obj_cycle = data['obj_cycle']
quantO = data['quantO']
ciclos = data['total_cycle']
posObsAct = data['posObsAct']
sizeO = data['sizeO']
NStart = data['NStart']
N = data['N']
pObjVec = data['pObjVec']
begin_alg_time = data['begin_alg_time']
fisico = data['fisico']
r_initial_conditions = data['r_initial_conditions']
r_obs = data['r_obs']
r_obj = data['r_obj']
TIME_STEP = data['TIME_STEP']
agent_setup = data['agent_setup']
obs_active = data['obs_active']
initial_pos_setup = data['initial_pos_setup']
r = data['r']
R = data['R']
MAX_SPEED = data['MAX_SPEED']
form_shape = data['form_shape']
rigidity_level = data['rigidity_level']
total_agent_number = data['total_agent_number']
NMax = data['NMax']
obj_marker = data['obj_marker']
obs_start_marker = data['obs_start_marker']
graphCycleStart = begin_alg_time
graphCycleEnd = ciclos

# Extract x and y coordinates for each element
x_positions = trajectory_data[:, 0, :]
y_positions = trajectory_data[:, 1, :]
x_posObsAct = posObsAct[0, :]
y_posObsAct = posObsAct[1, :]
x_pObjVec = pObjVec[0]
y_pObjVec = pObjVec[1]


#print(x_positions)
vx_positions = velocity_data[:, 0, :]
vy_positions = velocity_data[:, 1, :]
time_steps = np.arange(graphCycleStart, graphCycleEnd, 1)
#print(time_steps)
plt.figure(figsize=(3.8*1.5, 4.8*1.5))
# plotting the points 
# trajectory
plt.xlim(-1.9, 1.9)  # Set the x-axis limits (from 0 to 6)
plt.ylim(-2.4, 2.4)  # Set the y-axis limits (from 0 to 6)

factor_m = 0.0003528 # m por punto de scatter
diam_agente = 0.0375 #diametro promedio agentes

num_agents = x_positions.shape[1]  # Get the number of agents
print(NStart)
print(N)
NStart = NStart - 1
labels = [f'Agente {i + 1}' for i in range(NStart,num_agents)] 
#labels = [f'Agente {i}' for i in range(NStart,N+1)] 
#NStart_calc = NStart-1
#print(x_positions[:,NStart:N])

plt.plot(x_positions[graphCycleStart:graphCycleEnd,NStart:N], y_positions[graphCycleStart:graphCycleEnd,NStart:N], linestyle='--', zorder=4, label=labels)
plt.scatter(x_positions[graphCycleStart,NStart:N], y_positions[graphCycleStart,NStart:N], marker='o', facecolor='none', edgecolor='red', label='Pos Iniciales', zorder=5, s = diam_agente/factor_m)
plt.scatter(x_positions[-1,NStart:N], y_positions[-1,NStart:N], marker='o', color='green', label='Pos Finales', zorder=4, s = diam_agente/factor_m)
plt.scatter(x_posObsAct, y_posObsAct, marker='o', color='purple', label='Obstáculos', zorder=3, s = sizeO/factor_m) # s para convertir a m
plt.scatter(x_pObjVec, y_pObjVec, marker='*', color='yellow', label='Objetivo', zorder=2, s = 0.1/factor_m) 

# naming the x axis
plt.xlabel('Eje x (m)')
# naming the y axis
plt.ylabel('Eje y (m)')
  
# giving a title to my graph
plt.title('Trayectorias')

legend = plt.legend(markerscale=0.5)
for text in legend.get_texts():
    text.set_fontsize(7)

plt.subplots_adjust(right=0.8)
legend.set_bbox_to_anchor((0.99, 1))
plt.grid()
plt.figure()

# velocityHist in x
plt.plot(time_steps,vx_positions[graphCycleStart:graphCycleEnd,NStart:N], label = labels)
plt.ylim([-20, 20])
# naming the x axis
plt.xlabel('t (ciclos)')
# naming the y axis
plt.ylabel('Vx (m/s)')
  
# giving a title to my graph
plt.title('Velocidades en X')

legend = plt.legend()
for text in legend.get_texts():
    text.set_fontsize(7)

plt.subplots_adjust(right=0.8)
legend.set_bbox_to_anchor((0.99, 1))

plt.grid()

plt.figure()

# velocityHist in y
plt.plot(time_steps,vy_positions[graphCycleStart:graphCycleEnd,NStart:N], label = labels)
plt.ylim([-20, 20])
# naming the x axis
plt.xlabel('t (ciclos)')
# naming the y axis
plt.ylabel('Vy (m/s)')
  
# giving a title to my graph
plt.title('Velocidades en Y')

legend = plt.legend()
for text in legend.get_texts():
    text.set_fontsize(7)

plt.subplots_adjust(right=0.8)
legend.set_bbox_to_anchor((0.99, 1))

plt.grid()

# function to show the plot
plt.show()
# Create a figure and axis
fig, ax = plt.subplots(figsize=(3.8*1.5, 4.8*1.5))
ax.set_xlim(-1.9, 1.9)
ax.set_ylim(-2.4, 2.4)

# Initialize the scatter plots for the agents, obstacles, and goal
scatter_agents = ax.scatter([], [], marker='o', color ='green', label='Agents', zorder=5, s = diam_agente/factor_m)
scatter_goal = ax.scatter([], [], marker='*', color='yellow', label='Objetivo', zorder=2, s = 0.1/factor_m)
scatter_obstacles = ax.scatter([], [], marker='o', color='purple', label='Obstáculos', zorder=3, s = sizeO/factor_m)

# Plot the initial positions
scatter_agents.set_offsets(np.column_stack([x_positions[graphCycleStart, NStart:N], y_positions[graphCycleStart, NStart:N]]))
scatter_goal.set_offsets(np.column_stack([x_pObjVec, y_pObjVec]))
scatter_obstacles.set_offsets(np.column_stack([x_posObsAct, y_posObsAct]))

# Function to update the scatter plots in each animation frame
def update(frame):
    scatter_agents.set_offsets(np.column_stack([x_positions[frame, NStart:N], y_positions[frame, NStart:N]]))
    scatter_goal.set_offsets(np.column_stack([x_pObjVec, y_pObjVec]))
    scatter_obstacles.set_offsets(np.column_stack([x_posObsAct, y_posObsAct]))

# Create the animation
animation = FuncAnimation(fig, update, frames=np.arange(graphCycleStart, graphCycleEnd+1, 8), interval = 50, blit=False)

# Display the plot
plt.xlabel('Eje x (m)')
plt.ylabel('Eje y (m)')
plt.title('Trayectorias')
legend = plt.legend(markerscale=0.5)
for text in legend.get_texts():
    text.set_fontsize(7)
animation.save('animation.gif', writer='Pillow')
plt.subplots_adjust(right=0.8)
legend.set_bbox_to_anchor((1.3, 1))
plt.grid()
plt.show()



