from robotat_connect import *
from robotat_disconnect import *
prueba = 1
if (prueba == 1):
    robotat = robotat_connect("192.168.50.200")
while(prueba):
    disconnect = int(input("desconectar?"))
    if disconnect == 0:
        robotat_disconnect(robotat)
        prueba = 0
        
