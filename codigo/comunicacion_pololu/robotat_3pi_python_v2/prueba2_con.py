from funciones_conjunto import *

robotat = robotat_connect()
if robotat:
    print("connected to robotat")
else:
    print("error")
try:
    print(robotat)
    #disconnect = int(input("desconectar?"))
    pose = robotat_get_pose(robotat, [1])
    print(pose)
except:
    pass
finally:
    robotat_disconnect(robotat)