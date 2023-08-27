""" =========================================================================
% SIMULACIÓN MODELO DINÁMICO CON CONTROL DE FORMACIÓN, USANDO COSENO
% HIPERBÓLICO, Y EVASIÓN DE COLISIONES INCLUYENDO LÍMITES DE VELOCIDAD
% =========================================================================
% Autor: Andrea Maybell Peña Echeverría
% Última modificación: 27/09/2019
% (MODELO 6)
% =========================================================================
% El siguiente script implementa la simulación del modelo dinámico de
% modificación a la ecuación de consenso utilizando evasión de obstáculos y
% luego una combinación de control de formación con una función de coseno 
% hiperbólico para grafos mínimamente rígidos y evasión de obstáculos.
% Además incluye cotas de velocidad para que no se sobrepasen los límites
% físicos de los agentes. 
% Es un controlador del tipo supervisor. 
========================================================================="""

"""Supervisor3 controller."""

# Imports de librerías
from controller import Robot, Supervisor
import numpy as np
import random
import math
import pickle
from funVel import Fmatrix
import funciones
from multiprocessing import shared_memory, Lock

shm1 = shared_memory.SharedMemory(name="my_shared_memory1", create=True, size=1024)
shm2 = shared_memory.SharedMemory(name="my_shared_memory2", create=True, size=1024)
shm3 = shared_memory.SharedMemory(name="my_shared_memory3", create=True, size=1024)
lock = Lock()
ciclo = 0

TIME_STEP = 64
# Se crea instancia de supervisor
supervisor = Supervisor()

""" ARENA """
arena = supervisor.getFromDef("Arena")
size = arena.getField("floorSize")
sizeVec = size.getSFVec2f()				# vector con el tamaño de la arena

""" OBSTACULOS """
cantO = 3												# cantidad de obstáculos
obs1 = supervisor.getFromDef("Obs1")
obs2 = supervisor.getFromDef("Obs2")
obs3 = supervisor.getFromDef("Obs3")
Obstaculos = [obs1, obs2, obs3]
sizeO = 2.5*obs1.getField("majorRadius").getSFFloat()	# tamaño del obstáculo

## Posiciones de los agentes ##
posO1 = obs1.getField("translation")
posO2 = obs2.getField("translation")
posO3 = obs3.getField("translation")
posObs = [posO1, posO2, posO3]

""" Objetivo """
objetivo = supervisor.getFromDef("OBJ")
pObj = objetivo.getField("translation")
pObjVec = pObj.getSFVec3f()

""" AGENTES """
N = 10									# cantidad de agentes
r = 0.1								 	# radio a considerar para evitar colisiones
R = 2									# rango del radar
MAX_SPEED = 6.28						# velocidad máxima
agente0 = supervisor.getFromDef("Agente0")
agente1 = supervisor.getFromDef("Agente1")
agente2 = supervisor.getFromDef("Agente2")
agente3 = supervisor.getFromDef("Agente3")
agente4 = supervisor.getFromDef("Agente4")
agente5 = supervisor.getFromDef("Agente5")
agente6 = supervisor.getFromDef("Agente6")
agente7 = supervisor.getFromDef("Agente7")
agente8 = supervisor.getFromDef("Agente8")
agente9 = supervisor.getFromDef("Agente9")

Agentes = [agente0, agente1, agente2, agente3, agente4, agente5, agente6, agente7, agente8, agente9]

## Posiciones de los agentes ##
pos0 = agente0.getField("translation")
pos1 = agente1.getField("translation")
pos2 = agente2.getField("translation")
pos3 = agente3.getField("translation")
pos4 = agente4.getField("translation")
pos5 = agente5.getField("translation")
pos6 = agente6.getField("translation")
pos7 = agente7.getField("translation")
pos8 = agente8.getField("translation")
pos9 = agente9.getField("translation")

PosTodos = [pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9]
X = np.empty([2,N])

# Asignar posiciones random a cada agente
for a in range(0,N):
    X[0,a] = random.uniform(sizeVec[1]/2-0.1,-sizeVec[1]/2+0.1) #0.1 para que el carro no esté pegado a la pared
    X[1,a] = random.uniform(sizeVec[0]/2-0.1,-sizeVec[0]/2+0.1)
print("X",X)

# Revisión de las posiciones    
cW1 = 2						# contador de agentes sobre agentes
while(cW1 > 1 or cW2 > 1):
    cW1 = 0
    cW2 = 0
    # Asegurar que los agentes no empiecen uno sobre otro
    contR = 1				# contador de intersecciones
    while(contR > 0):
        contR = 0
        for i in range(1, N):
            for j in range(1, N-i):
                resta = math.sqrt((X[0,i]-X[0,i+j])**2+(X[1,i]-X[1,i+j])**2)	# diferencia entre las posiciones
                if(abs(resta) < r):
                    X[0,i+j] = random.uniform(sizeVec[1]/2-0.1,-sizeVec[1]/2+0.1)
                    X[1,i+j] = random.uniform(sizeVec[0]/2-0.1,-sizeVec[0]/2+0.1)									# hay intersección
                    contR = contR+1
        cW1 = cW1+1
    
    contRO = 1			# contador de intersecciones con obstáculos
    while(contRO > 0):
        contRO = 0
        for i in range(1,N):
            for j in range(1,cantO):
				# distancia agente obstáculo
                resta = math.sqrt((X[0,i]-posObs[j].getSFVec3f()[1])**2 + (X[1,i]-posObs[j].getSFVec3f()[0])**2)	
                if(abs(resta) < sizeO):
	         # cambio de posición
                    X[0,i+j] = random.uniform(sizeVec[1]/2-0.1,-sizeVec[1]/2+0.1)
                    X[1,i+j] = random.uniform(sizeVec[0]/2-0.1,-sizeVec[0]/2+0.1)
                    contRO = contRO + 1			# hay intersección
        cW2 = cW2 + 1        

Xi = X
  
# Asignar posiciones revisadas  
for b in range(0, N):
    PosTodos[b].setSFVec3f([X[1,b], X[0,b], -6.39203e-05])
    pass

# Posiciones actuales
posActuales = np.zeros([2,N])
posNuevas = np.zeros([2,N])

# Matriz de velocidades
V = np.empty([2,N])

# Matriz de formación
d = Fmatrix(2,1)
print(d)

# Main loop:
cambio = 0						# variable para cambio de control 
while supervisor.step(TIME_STEP) != -1:
    print("cambio",cambio)
	
	# Se obtienen posiciones actuales
    for c in range(0,N):
        posC = Agentes[c].getField("translation")
        posActuales[0][c] = posC.getSFVec3f()[1]
        posActuales[1][c] = posC.getSFVec3f()[0]        
    
    for g in range(0,N):
        E0 = 0
        E1 = 0
        for h in range(0,N):
            dist = np.asarray([posActuales[0][g]-posActuales[0][h], posActuales[1][g]-posActuales[1][h]])	# vector xi - xj   
            mdist = math.sqrt(dist[0]**2 + dist[1]**2)														# norma euclidiana vector xi - xj
            dij = 0.2*d[g][h]																				# distancia deseada entre agentes i y j
            
			# Peso añadido a la ecuación de consenso
            if(mdist == 0 or mdist >= R):
                w = 0
            else:
                if(cambio == 0): 										# inicio: acercar a los agentes sin chocar
                    #print("collision avoidance")
                    w = (mdist - (2*(r+0.05)))/(mdist - (r+0.05))**2 	# collision avoidance
                else:
                    if(dij == 0):										# si no hay arista, se usa función plana como collision avoidance
                        print("cosh")
                        w = 0.15*math.sinh(15*mdist-6)/mdist 		
                    else:												# collision avoidance & formation control
                        print("formacion")
                        w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)**2)/(mdist*(mdist - r)**2)
                
            # Tensión de aristas entre agentes 
            E0 = E0 + 2*w*dist[0]
            E1 = E1 + 2*w*dist[1]
        # Collision avoidance con obstáculos
        for j in range(0,cantO):
            distO0 = posActuales[0,g] - posObs[j].getSFVec3f()[1]
            distO1 = posActuales[1,g] - posObs[j].getSFVec3f()[0]  
            mdistO = math.sqrt(distO0**2 + distO1**2) - sizeO

            if(abs(mdistO) < 0.0001):
                mdistO = 0.0001
            w = -1/(mdistO**2)

            E0 = E0 + 0.1*w*distO0
            E1 = E1 + 0.1*w*distO1    
        # Actualización de velocidad
        V[0][g] = -1*(E0)*TIME_STEP/1000 
        V[1][g] = -1*(E1)*TIME_STEP/1000 

	# Al llegar muy cerca de la posición deseada realizar cambio de control
    normV2 = 0
    for m in range(0,N):
        nV2 = V[0][m]**2 + V[1][m]**2
        normV2 = normV2 + nV2
    normV = math.sqrt(normV2)
    print("normV", normV)
    
    if(normV < 3 and cambio < 1):
        cambio = cambio + 1
    if (ciclo > 400):
        V[0][4] = V[0][0] - (posActuales[0][0]-pObjVec[1])
        V[1][4] = V[1][0] - (posActuales[1][0]-pObjVec[0])
    lock.acquire()
    pick_posActuales = pickle.dumps(posActuales)
    shm1.buf[:len(pick_posActuales)] = pick_posActuales
    pick_posNuevas = pickle.dumps(posNuevas)
    shm2.buf[:len(pick_posNuevas)] = pick_posNuevas
    pick_V = pickle.dumps(V)
    shm3.buf[:len(pick_V)] = pick_V
    lock.release()
    print(ciclo)    
    ciclo = ciclo + 1 
    
    if (supervisor.step(TIME_STEP) == -1):
        shm1.close()
        #shm1.unlink()
        shm2.close()
        #shm2.unlink()
        shm3.close()
        break
       
    if(ciclo > 3000):
        #lock.acquire()
        shm1.close()
        #shm1.unlink()
        shm2.close()
        #shm2.unlink()
        shm3.close()
        #shm3.unlink()
        #del shm1
        #del shm2
        #del shm3
        #lock.release()
        break
        
    
     
    """    
    # Guardar datos necesarios para asignar velocidad a cada agente  
    with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos.pickle','wb') as f:
        pickle.dump(posActuales, f)
    with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos2.pickle','wb') as f:
        pickle.dump(posNuevas, f)
    with open('D:/AlejandroDigital/tesisAlejandro/codigo/antecedentes/prueba_antecedentes_propia/Webots/controllers/Datos3.pickle','wb') as f:
        pickle.dump(V, f)
    pass
    """
