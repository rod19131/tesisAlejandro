import pickle
from funciones_conjunto import *
#teoria 2
#lista = [1,2]
#lista2 = [lista]
#print(lista)
#print(lista2)
# Load variables from the file
with open('first_setup.pickle', 'rb') as file:
    pose_por_agente = pickle.load(file)
    #pose = pickle.load(file)

print(pose_por_agente)

#eul1 = quaternion_to_euler(list(pose_por_agente[0]),'xyz')
eul = quat2eul(pose_por_agente,'xyz')

print(pose_por_agente)
print("conversion")
#print(eul1)
print(eul)
#print(pose)
