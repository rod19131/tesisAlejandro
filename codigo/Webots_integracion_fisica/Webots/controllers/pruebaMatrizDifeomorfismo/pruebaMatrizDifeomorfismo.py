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
from controller import Robot, Compass, Motor
import math
import numpy as np
import pickle
from multiprocessing import shared_memory, Lock
from funciones_conjunto_3pi import *
import time
TIME_STEP = 64
MAX_SPEED = 6.28
MAX_SPEED_f = 50
#MAX_SPEED = 0.5
shm1 = shared_memory.SharedMemory(name="my_shared_memory1")
shm2 = shared_memory.SharedMemory(name="my_shared_memory2")
lock = Lock()
fisico = 0
if (fisico == 0):

    # Dimensiones robot
    r = 0.0205
    l = 0.0355
    a = 0.0355
    
    # Robot instance.
    robot = Robot()
    argc = int(robot.getCustomData())
    
    # Enable compass
    compass = robot.getDevice("compass")
    compass.enable(TIME_STEP)
    robot.step(TIME_STEP)
    
    # get a handler to the motors and set target position to infinity (speed control)
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    # Main loop:
    while robot.step(TIME_STEP) != -1:
        # Posiciones actuales y finales según Supervisor
        # print("AGENTE",argc)
        lock.acquire()
        pick_V = shm1.buf[:shm1.size]
        lock.release()
        V = pickle.loads(pick_V)
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
        
        # Posición nueva/final
        #posFinal = np.asarray([posNuevas[1][argc], posNuevas[0][argc], -6.39203e-05])
        # print("posFinal",posFinal)
        
        # Posición actual
        #posAct = np.asarray([posActuales[1, argc], posActuales[0, argc], -6.39203e-05])
        # print("posAct",posAct)
        
        # Velocidades
        
        # Orientación robot
        comVal = compass.getValues()
        angRad = math.atan2(comVal[0],comVal[1]) # se ingresan como (x,y) en lugar de (y,x) para compensar el desfase de 90° en físico
        angDeg = (angRad/math.pi)*180
        if(angDeg < 0):
            angDeg = angDeg + 360
        theta_o = angDeg
        
    	# Transformación de velocidad lineal y velocidad angular
        v = (V[0][argc])*(math.cos(theta_o*math.pi/180)) + (V[1][argc])*(math.sin(theta_o*math.pi/180))
        w = (V[0][argc])*(-math.sin(theta_o*math.pi/180)/a) + (V[1][argc])*(math.cos(theta_o*math.pi/180)/a)
        
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
        
    	# Asignación de velocidades a las ruedas
        leftMotor.setVelocity(phi_l)
        rightMotor.setVelocity(phi_r)
        pass
    
elif (fisico == 1):
    # Dimensiones Pololu3Pi+
    r = 0.017
    l = 0.0485
    a = 0.0485
    
    # Robot instance.
    robot = Robot()
    
    # Enable compass
    robot.step(TIME_STEP)
    
    # get a handler to the motors and set target position to infinity (speed control)
    # Main loop:
    while robot.step(TIME_STEP) != -1:
        # Posiciones actuales y finales según Supervisor
        # print("AGENTE",argc)
        lock.acquire()
        pick_V = shm1.buf[:shm1.size]
        pick_agents_pose = shm2.buf[:shm2.size]
        lock.release()
        V = pickle.loads(pick_V)
        agents_pose = pickle.loads(pick_agents_pose)
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
        
        # Posición nueva/final
        #posFinal = np.asarray([posNuevas[1][argc], posNuevas[0][argc], -6.39203e-05])
        # print("posFinal",posFinal)
        
        # Posición actual
        #posAct = np.asarray([posActuales[1, argc], posActuales[0, argc], -6.39203e-05])
        # print("posAct",posAct)
        
        # Velocidades
        
        # Orientación robot
        theta_o = agents_pose[0][3]
        
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
        
    	# Asignación de velocidades a las ruedas
        leftMotor.setVelocity(phi_l)
        rightMotor.setVelocity(phi_r)
        pass
        
elif (fisico == 2):

    # Dimensiones robot
    r = 0.0205
    l = 0.0355
    a = 0.0355
    r_f = 0.017
    l_f = 0.0485
    a_f = 0.0485
    
    # Robot instance.
    robot = Robot()
    argc = int(robot.getCustomData())
    
    agente = argc + 1
    #if (argc == 1 or argc == 2 or argc == 3 or argc == 4 or argc == 5 or argc == 6 or argc == 7 or argc == 8 or argc == 9):
    if (argc == 1 or argc == 2 or argc == 3  or argc == 4  or argc == 5  or argc == 6 or argc == 7): 
        try:
            pololu = robotat_3pi_connect(agente)
        except:
            print("error, no se pudo conectar al pololu")
            pass
    """
    elif (argc == 3):
        try:
            pololu = robotat_3pi_connect(agente)
        except:
            print("error, no se pudo conectar al pololu")
            pass
    """
    # Enable compass
    compass = robot.getDevice("compass")
    compass.enable(TIME_STEP)
    robot.step(TIME_STEP)
    
    # get a handler to the motors and set target position to infinity (speed control)
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    
    # Main loop:
    while robot.step(TIME_STEP) != -1:
        # Posiciones actuales y finales según Supervisor
        # print("AGENTE",argc)
        lock.acquire()
        pick_V = shm1.buf[:shm1.size]
        pick_agents_pose = shm2.buf[:shm2.size]
        lock.release()
        V = pickle.loads(pick_V)
        agents_pose = pickle.loads(pick_agents_pose)
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
        
        # Posición nueva/final
        #posFinal = np.asarray([posNuevas[1][argc], posNuevas[0][argc], -6.39203e-05])
        # print("posFinal",posFinal)
        
        # Posición actual
        #posAct = np.asarray([posActuales[1, argc], posActuales[0, argc], -6.39203e-05])
        # print("posAct",posAct)
        
        # Velocidades
        
        # Orientación robot
        comVal = compass.getValues()
        angRad = math.atan2(comVal[0],comVal[1])
        angDeg = (angRad/math.pi)*180
        if(angDeg < 0):
            angDeg = angDeg + 360
        theta_o = angDeg
        
        theta_o_f = agents_pose[argc][3]+90 #desfase de 90 grados por la orientación en la vida real
        if(theta_o_f < 0):
            thetha_o_f = theta_o_f + 360
    	# Transformación de velocidad lineal y velocidad angular
        v = (V[0][argc])*(math.cos(theta_o*math.pi/180)) + (V[1][argc])*(math.sin(theta_o*math.pi/180))
        w = (V[0][argc])*(-math.sin(theta_o*math.pi/180)/a) + (V[1][argc])*(math.cos(theta_o*math.pi/180)/a)
        
        # Transformación de velocidad lineal y velocidad angular
        v_f = (V[0][argc])*(math.cos(theta_o_f*math.pi/180)) + (V[1][argc])*(math.sin(theta_o_f*math.pi/180))
        w_f = (V[0][argc])*(-math.sin(theta_o_f*math.pi/180)/a_f) + (V[1][argc])*(math.cos(theta_o_f*math.pi/180)/a_f)
        
        # Cálculo de velocidades de las ruedas   
        phi_r = (v+(w*l))/r
        # print(phi_r)
        phi_l = (v-(w*l))/r
        # print(phi_l)
        
        phi_r_f = (v_f+(w_f*l_f))/r_f
        # print(phi_r)
        phi_l_f = (v_f-(w_f*l_f))/r_f
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
        
        # Truncar velocidades a la velocidad maxima
        if(phi_r_f > 0):
            if(phi_r_f > MAX_SPEED_f):
                phi_r_f = MAX_SPEED_f
        else:
            if(phi_r_f < -MAX_SPEED_f):
                phi_r_f = -MAX_SPEED_f
                
        if(phi_l_f > 0):
            if(phi_l_f > MAX_SPEED_f):
                phi_l_f = MAX_SPEED_f
        else:
            if(phi_l_f < -MAX_SPEED_f):
                phi_l_f = -MAX_SPEED_f
    
    	# Asignación de velocidades a las ruedas
        leftMotor.setVelocity(phi_l)
        rightMotor.setVelocity(phi_r)
        #if (argc == 1 or argc == 2 or argc == 3 or argc == 4 or argc == 5 or argc == 6 or argc == 7 or argc == 8 or argc == 9):
        if (argc == 1 or argc == 2 or argc == 3  or argc == 4  or argc == 5  or argc == 6 or argc == 7):
            try:
                robotat_3pi_set_wheel_velocities(pololu, phi_l_f, phi_r_f)
                #time.sleep(1)
            except:
                #print("error, no se pudo conectar al pololu")
                pass
        """
        elif (argc == 3):
            try:
                robotat_3pi_set_wheel_velocities(pololu, phi_l_f, phi_r_f)
                #time.sleep(1)
            except:
                #print("error, no se pudo conectar al pololu")
                pass
        """
            
        if (robot.step(TIME_STEP) == -1):
            #if (argc == 1 or argc == 2 or argc == 3 or argc == 4 or argc == 5 or argc == 6 or argc == 7 or argc == 8 or argc == 9):
            if (argc == 1 or argc == 2 or argc == 3  or argc == 4  or argc == 5  or argc == 6 or argc == 7):    
                try:
                    robotat_3pi_force_stop(pololu)
                    robotat_3pi_disconnect(pololu)
                except:
                    #print("error, no se pudo conectar al pololu")
                    pass
            break
            """
            elif (argc == 3):
                try:
                    robotat_3pi_force_stop(pololu)
                    robotat_3pi_disconnect(pololu)
                except:
                    #print("error, no se pudo conectar al pololu")
                    pass
            """
        pass