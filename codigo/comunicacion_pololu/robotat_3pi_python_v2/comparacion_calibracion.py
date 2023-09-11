import pickle
from funciones_conjunto import *
#calibracion inicial de markers 1 al 15 del robotat
# Load variables from the file
poses_markers = np.load('calibracion_markers_inicial.npy')
    #pose = pickle.load(file)

#eul1 = quaternion_to_euler(list(pose_por_agente[0]),'xyz')
markers_eul = quat2eul(poses_markers,'zyx') 

#print(pose_por_agente)
print("conversion")
i = 0
i_agente = 0
for marker in markers_eul:
    i_agente = i_agente+1


    print(f"agent: {i_agente}\n angle compensation: {markers_eul[i,3]}°")
    i = i+1
#print(eul1)
#print(markers_eul)
#print(pose)