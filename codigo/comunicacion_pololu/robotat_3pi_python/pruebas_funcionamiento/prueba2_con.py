from funciones_conjunto import *
import numpy as np
import pickle
# Adquisición de múltiples poses, múltiples veces de los marcadores del robotat al conectarse con el servidor.
robotat = robotat_connect()
if robotat:
    print("connected to robotat")
else:
    print("error")
try:
    print(robotat)
    agentes = [1,2,3,4,5,6,7,8,9,10]
    n = len(agentes)
    print("Number of agents:\n",n)
    disconnect = int(input("desconectar?"))
    pose = robotat_get_pose(robotat, agentes)
    print("Pose per agent\n:", pose)
    pose = robotat_get_pose(robotat, [1,2,3,4,5])
    print("Pose per agent\n:", pose)
   # eul = quaternion_to_euler(list(pose[0]),'xyz')

    #print(pose[0])
    #print(eul)
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
    markers_eul = quat2eul(pose,'zyx') 
    
    #np.save('calibracion_markers_inicial.npy', pose)
    
    """
    with open('markers_alineados_zyx2.pickle', 'wb') as f:
        pickle.dump(pose, f)

    """