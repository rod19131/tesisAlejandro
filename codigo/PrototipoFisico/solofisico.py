""" =========================================================================
% CONTROLADOR DE AGENTES PARA TRANSFORMACIÓN DE VELOCIDADES LINEALES A
% VELOCIDADES RADIALES DE LAS RUEDAS
% =========================================================================
% Autor: Andrea Maybell Peña Echeverría
% Última modificación: 27/09/2019
% =========================================================================
% El siguiente script implementa las transformaciones necesarias para que
% los robots E-Puck se muevan a la velocidad determinada por el Supervisor.
% Es un controlador del tipo robot. 
========================================================================="""

"""pruebaMatrizDifeomorfismo controller."""

# Import de librerías
import math
import numpy as np
import pickle
from funciones_conjunto_3pi import *
from funciones_conjunto import *
TIME_STEP = 64
MAX_SPEED = 50
#MAX_SPEED = 0.2
fisico = 0
V = np.zeros([2,1])
desfases = np.load('calibracion_markers_inicial.npy')
desfases_euler = quat2eul(desfases,'zyx')


def update_data():
    try: 
        robotat = robotat_connect()
        if robotat:
            #print(robotat)
            agentes = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
            n_ag = len(agentes)
            print("Number of agents:\n",n_ag)
            pose = robotat_get_pose(robotat, agentes)
        else:
            print("error")
    except:
        print("error")
    finally:
        robotat_disconnect(robotat)
        pose_eul = quat2eul(pose,'zyx')
        print(pose_eul)
        return n_ag, pose_eul

nag,agents_pose = update_data()
for marker in range(len(agents_pose)):
    agents_pose[marker,3] = agents_pose[marker,3] - desfases_euler[marker,3]

if (fisico == 0):
    # Dimensiones robot
    r = 0.0205
    l = 0.0355
    a = 0.0355
    ciclos = 0
    robot2 = robotat_3pi_connect(2)
    # Robot instance.
    # Main loop:
    while ciclos < 1000:
        ciclos = ciclos + 1
        nag, agents_pose = update_data()
        for marker in range(len(agents_pose)):
            agents_pose[marker,3] = agents_pose[marker,3] - desfases_euler[marker,3]  
        V[0][0] = V[0][0] - 1*(agents_pose[0][1]+2.0)
        V[1][0] = V[1][0] - 1*(agents_pose[0][0]+2.0)      
        # Posiciones actuales y finales según Supervisor
        # print("AGENTE",argc)
        """
        try:
            with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos.pickle','rb') as f:
                    posActuales = pickle.load(f)
            with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos2.pickle','rb') as f:
                    posNuevas = pickle.load(f)
            with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos3.pickle','rb') as f:
                    V = pickle.load(f)
        except:
            print("fallo")
        """
        
        # Velocidades
        
        # Orientación robot
        theta_o = agents_pose[0,3]
        
    	# Transformación de velocidad lineal y velocidad angular
        v = (V[0][0])*(math.cos(theta_o*math.pi/180)) + (V[1][0])*(math.sin(theta_o*math.pi/180))
        w = (V[0][0])*(-math.sin(theta_o*math.pi/180)/a) + (V[1][0])*(math.cos(theta_o*math.pi/180)/a)
        
        # Cálculo de velocidades de las ruedas   
        phi_r = (v+(w*l))/r
        # print(phi_r)
        phi_l = (v-(w*l))/r
        # print(phi_l)
        
        # Truncar velocidades a la velocidad maxima
        if(phi_r > 0):
            if(phi_r > MAX_SPEED):
                phi_r = MAX_SPEED
        else:
            if(phi_r < -MAX_SPEED):
                phi_r = -MAX_SPEED
                
        if(phi_l > 0):
            if(phi_l > MAX_SPEED):
                phi_l = MAX_SPEED
        else:
            if(phi_l < -MAX_SPEED):
                phi_l = -MAX_SPEED
        robotat_3pi_set_wheel_velocities(robot2, phi_l, phi_r)
    	# Asignación de velocidades a las ruedas
        pass
robotat_3pi_force_stop(robot2)
robotat_3pi_disconnect(robot2)
