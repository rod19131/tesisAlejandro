import pickle

# Load variables from the file
with open('first_setup.pickle', 'rb') as file:
    pose_por_agente = pickle.load(file)
    pose = pickle.load(file)
    
print(pose_por_agente)
print(pose)