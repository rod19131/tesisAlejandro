from funciones_conjunto import *
import numpy as np
import pickle

robotat = robotat_connect()
if robotat:
    print("connected to robotat")
else:
    print("error")
try:
    print(robotat)
    agentes = [1,2]
    n = len(agentes)
    print("Number of agents:\n",n)
    #disconnect = int(input("desconectar?"))
    pose = robotat_get_pose(robotat, agentes)
    print("Pose per agent\n:", pose)
    """
    print(type(pose))
    print("all agent poses:\n", pose)
    pose_por_agente = np.array(pose).reshape(n,7)
    print(type(pose_por_agente))
    print("Pose per agent\n:", pose_por_agente)
    """

except:
    pass
finally:
    robotat_disconnect(robotat)
    """
    with open('first_setup.pickle', 'wb') as f:
        pickle.dump(pose_por_agente, f)
        pickle.dump(pose, f)
    """

    