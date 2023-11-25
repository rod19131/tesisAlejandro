import numpy as np
import pickle

with open('first_setup.pickle', 'rb') as file:
    poses_markers = pickle.load(file)

np.save('first_setup.npy', poses_markers)

#poses = np.load('markers_alineados_zyx2np.npy')
#print(poses)
