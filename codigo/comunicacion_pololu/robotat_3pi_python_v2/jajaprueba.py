import numpy as np
from funciones_conjunto import *
desfases_euler = np.load('calibracion_markers_inicial.npy')
desfases_euler = quat2eul(desfases_euler,'zyx')
agents_pose = np.load('calibracion_markers_inicial.npy')
print(desfases_euler)
agents_pose = quat2eul(agents_pose,'zyx')
"""
for marker in range(len(agents_pose)):
        agents_pose[marker,3] = agents_pose[marker,3] - desfases_euler[marker,3]
"""
for marker in range(len(agents_pose)):
    print(agents_pose[marker][3])