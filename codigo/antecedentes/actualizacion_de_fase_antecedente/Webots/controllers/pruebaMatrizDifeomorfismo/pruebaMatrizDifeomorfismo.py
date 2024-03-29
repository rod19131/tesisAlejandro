""" =========================================================================
% CONTROLADOR DE AGENTES PARA TRANSFORMACIÓN DE VELOCIDADES LINEALES A
% VELOCIDADES RADIALES DE LAS RUEDAS
% =========================================================================
# Autor: José Alejandro Rodríguez Porras
# Código basado en el trabajo previo de: Andrea Maybell Peña Echeverría
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
TIME_STEP = 64
MAX_SPEED = 6.28
#MAX_SPEED = 0.2
shm1 = shared_memory.SharedMemory(name="my_shared_memory1")
shm2 = shared_memory.SharedMemory(name="my_shared_memory2")
shm3 = shared_memory.SharedMemory(name="my_shared_memory3")
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
        pick_posActuales = shm1.buf[:shm1.size]
        pick_posNuevas = shm2.buf[:shm2.size]
        pick_V = shm3.buf[:shm3.size]
        lock.release()
        posActuales = pickle.loads(pick_posActuales)
        posNuevas = pickle.loads(pick_posNuevas)
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
        posFinal = np.asarray([posNuevas[1][argc], posNuevas[0][argc], -6.39203e-05])
        # print("posFinal",posFinal)
        
        # Posición actual
        posAct = np.asarray([posActuales[1, argc], posActuales[0, argc], -6.39203e-05])
        # print("posAct",posAct)
        
        # Velocidades
        
        # Orientación robot
        comVal = compass.getValues()
        angRad = math.atan2(comVal[1],comVal[0])
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
        pick_posActuales = shm1.buf[:shm1.size]
        pick_posNuevas = shm2.buf[:shm2.size]
        pick_V = shm3.buf[:shm3.size]
        lock.release()
        posActuales = pickle.loads(pick_posActuales)
        posNuevas = pickle.loads(pick_posNuevas)
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
        posFinal = np.asarray([posNuevas[1][argc], posNuevas[0][argc], -6.39203e-05])
        # print("posFinal",posFinal)
        
        # Posición actual
        posAct = np.asarray([posActuales[1, argc], posActuales[0, argc], -6.39203e-05])
        # print("posAct",posAct)
        
        # Velocidades
        
        # Orientación robot
        comVal = compass.getValues()
        angRad = math.atan2(comVal[1],comVal[0])
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