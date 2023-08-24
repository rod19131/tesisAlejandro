from robotat_connect import *
from robotat_disconnect import *
from robotat_get_pose import *
prueba = 1
if (prueba == 1):
    robotat = robotat_connect("192.168.50.200")
while(prueba==1):
    disconnect = int(input("desconectar?"))
    if disconnect == 0:
        #pose = robotat_get_pose(robotat, 1, "XYZ")
        #print(pose)
        robotat_disconnect(robotat)
        prueba = 0
print("desconectado")
        
