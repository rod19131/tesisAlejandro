""" =========================================================================
% SIMULACIÓN MODELO DINÁMICO CON CONTROL DE FORMACIÓN, USANDO COSENO
% HIPERBÓLICO, Y EVASIÓN DE COLISIONES INCLUYENDO LÍMITES DE VELOCIDAD
% =========================================================================
# Autor: José Alejandro Rodríguez Porras
# Código basado en el trabajo previo de: Andrea Maybell Peña Echeverría
% =========================================================================
% El siguiente script implementa la simulación del modelo dinámico de
% modificación a la ecuación de consenso utilizando evasión de obstáculos y
% luego una combinación de control de formación con una función de coseno 
% hiperbólico para grafos mínimamente rígidos y evasión de obstáculos.
% Además incluye cotas de velocidad para que no se sobrepasen los límites
% físicos de los agentes. 
% Es un controlador del tipo supervisor. 
========================================================================="""

"""Supervisor_simulacion_y_fisico_v3 controller."""

# Imports de librerías
from controller import Robot, Supervisor
import numpy as np
import random
import math
import pickle
import keyboard
import time
from funVel import Fmatrix
import funciones
from multiprocessing import shared_memory, Lock
from funciones_conjunto import *

shm1 = shared_memory.SharedMemory(name="my_shared_memory1", create=True, size=1024)
shm2 = shared_memory.SharedMemory(name="my_shared_memory2", create=True, size=4096)

lock = Lock()
ciclo = 0   
TIME_STEP = 64
# Se crea instancia de supervisor
supervisor = Supervisor()

fisico = 1  # 0 to use Webots, 1 to use Robotat
r_initial_conditions = 0 # 0 para simulación nueva 1 para simulación basada en condiciones iniciales físicas
r_obs = 0 # 0 para obstaculos virtuales 1 para obstaculos reales (markers)
r_obj = 0 # 0 para objetivo virtual 1 para objetivo real

# Matriz de formación
form_shape = 1
rigidity_level = 8
""" AGENTES """
NMax = 10
NStart = 4
NStart = NStart-1
N = 6				# cantidad de agentes
r = 0.07								 	# radio a considerar para evitar colisiones
R = 4	
setup_pos = np.zeros((NMax, 6))
setup_shape = 0
setup_shape_space = 1.5
# Specify the starting point
setup_starting_point = np.array([-1.0, -1.5])


if (setup_shape == 0):
    for i in range(NStart,N):
        setup_pos[i, 0] = setup_starting_point[0] + i * 0.3 
        setup_pos[i, 1] = setup_starting_point[1] + i * 0.3 * 0
        setup_pos[i, 2] = 0.5

elif (setup_shape == 1):
    for i in range(NStart,N):
        angle = 2 * np.pi * i / (N-NStart)  # Calculate the angle for each marker
        setup_shape_radius = setup_shape_space/2
        setup_pos[i, 0] = setup_starting_point[0] + setup_shape_radius * np.cos(angle)  # x-coordinate
        setup_pos[i, 1] = setup_starting_point[1] + setup_shape_radius * np.sin(angle)  # y-coordinate
        setup_pos[i, 2] = 0.5  # constant z-coordinate

"""    
for i in range(3):
    obs_pos[i, 0] = i * 0.3 -1.3
    obs_pos[i, 1] = -1.5
    obs_pos[i, 2] = 0.5
"""

								# rango del radar
MAX_SPEED = 30						# velocidad máxima
total_agent_number = N-NStart
total_agent_weight = (N-NStart)/NMax
begin_alg_time = 400 # valor arbitrario para inicializar variable    
    
    
    
trajectory = []
velocityHist = []
normVHist = []
objHist = []
obsHist = []
PosRealAgents = 0
RotRealAgents = 0 
form_cycle = 0
obj_cycle = 0
agents_pose = []

def update_data():
    try: 
        robotat = robotat_connect()
        if robotat:
            #print(robotat)
            agentes = [1,2,3,4,5,6,7,8,9,10,11,12]
            n_ag = len(agentes)
            #print("Number of agents:\n",n_ag)
            pose = robotat_get_pose(robotat, agentes)
        else:
            print("error")
    except:
        print("error")
    finally:
        robotat_disconnect(robotat)
        pose_eul = quat2eul(pose,'zyx')
        #print(pose_eul)
        return pose_eul   

if (fisico == 1):
    desfases = np.load('calibracion_markers_inicial.npy')
    desfases_euler = quat2eul(desfases,'zyx')
    agents_pose = update_data()
    for marker in range(len(agents_pose)):
            agents_pose[marker,3] = agents_pose[marker,3] - desfases_euler[marker,3]
    
""" ARENA """
arena = supervisor.getFromDef("Arena")
size = arena.getField("floorSize")
sizeVec = size.getSFVec2f()				# vector con el tamaño de la arena

""" OBSTACULOS """
cantO = 3  # Adjust this to the number of obstacles you have
Obstaculos = []
posObs = []
posObsAct = np.empty([2,cantO])
sizeObsIn = np.empty([2,cantO])

for i in range(0, cantO):
    obstacle_name = f"Obs{i}"
    obstacle = supervisor.getFromDef(obstacle_name)
    pos_obstacle = obstacle.getField("translation")
    Obstaculos.append(obstacle)
    posObs.append(pos_obstacle)
    posObsAct[0][i] = posObs[i].getSFVec3f()[0]
    posObsAct[1][i] = posObs[i].getSFVec3f()[1]

sizeO = 3*Obstaculos[0].getField("majorRadius").getSFFloat() # tamaño del obstáculo
#print(sizeO) 0.175m
#print(Obstaculos)

""" Objetivo """

objetivo = supervisor.getFromDef("OBJ")
pObj = objetivo.getField("translation")
pObjVec = pObj.getSFVec3f()

""" inicialización agentes """

Agents = []
PosTodos = []
RotTodos = []

PosTodosVec = []
RotTodosVec = []


initialPositions = []
posIniPos = []
posIniPosVec = []


for i in range(0, NMax):
    inipos_name = f"IniPos{i+1}"
    inipos = supervisor.getFromDef(inipos_name)
    inipos_pos = inipos.getField("translation")
    inipos_pos_vec = inipos_pos.getSFVec3f()
    initialPositions.append(inipos)
    posIniPos.append(inipos_pos)
    posIniPosVec.append(inipos_pos_vec)
    agent_name = f"Agent{i+1}"
    agent = supervisor.getFromDef(agent_name)
    agent_pos = agent.getField("translation")
    agent_rot = agent.getField("rotation")
    pos_todos_vec = agent_pos.getSFVec3f()
    rot_todos_vec = agent_rot.getSFVec3f()
    Agents.append(agent)
    PosTodos.append(agent_pos)
    RotTodos.append(agent_rot)
    PosTodosVec.append(pos_todos_vec)
    RotTodosVec.append(rot_todos_vec)
 
X = np.empty([2,N])

# Asignar posiciones random a cada agente
for a in range(0, NMax):
    if a in range(NStart,N):
        X[0,a] = random.uniform(sizeVec[1]/2-0.4,-sizeVec[1]/2+0.4) #0.4 para que el carro no esté pegado a la pared
        X[1,a] = random.uniform(sizeVec[0]/2-0.4,-sizeVec[0]/2+0.4)

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
        for i in range(NStart+1, N):
            for j in range(NStart+1, N-i):
                resta = math.sqrt((X[0,i]-X[0,i+j])**2+(X[1,i]-X[1,i+j])**2)	# diferencia entre las posiciones
                if(abs(resta) < r):
                    X[0,i+j] = random.uniform(sizeVec[1]/2-0.4,-sizeVec[1]/2+0.4)
                    X[1,i+j] = random.uniform(sizeVec[0]/2-0.4,-sizeVec[0]/2+0.4)									# hay intersección
                    contR = contR+1
        cW1 = cW1+1
    
    contRO = 1			# contador de intersecciones con obstáculos
    while(contRO > 0):
        contRO = 0
        for i in range(NStart,N):
            for j in range(1,cantO):
				# distancia agente obstáculo
                resta = math.sqrt((X[0,i]-posObs[j].getSFVec3f()[1])**2 + (X[1,i]-posObs[j].getSFVec3f()[0])**2)	
                if(abs(resta) < sizeO):
	         # cambio de posición
                    X[0,i+j] = random.uniform(sizeVec[1]/2-0.4,-sizeVec[1]/2+0.4)
                    X[1,i+j] = random.uniform(sizeVec[0]/2-0.4,-sizeVec[0]/2+0.4)
                    contRO = contRO + 1			# hay intersección
        cW2 = cW2 + 1        

Xi = X
  
# Asignar posiciones revisadas
agent_setup = 5
obs_active = 1
initial_pos_setup = 1 #inicialización de markers 0: aleatorio 1: planificado

for i in range(0, 3):
    if (obs_active == 0):
        posObs[i].setSFVec3f([-3.3, i*0.9, -6.39203e-05])

for b in range(0, NMax):
    if (b<NStart or b>=N):
        PosTodos[b].setSFVec3f([-2.3, b*0.3, 0.3])
        posIniPos[b].setSFVec3f([-2.3, b*0.3, -6.39203e-05])

if (r_initial_conditions == 1):
    desfases = np.load('calibracion_markers_inicial.npy')
    desfases_euler = quat2eul(desfases,'zyx')
    initial_data = np.load('trial_distance_1A_2.npz')
    PosRealAgents = initial_data['PosRealAgents']
    RotRealAgents = initial_data['RotRealAgents']
    PosRealIniPosVec = initial_data['posIniPosVec']
    posObsAct = initial_data['posObsAct']
    pObjVec = initial_data['pObjVec']
    N = initial_data['N']
    NStart = initial_data['NStart']
    pObj.setSFVec3f([pObjVec[0], pObjVec[1], -6.39203e-05])

if (r_initial_conditions == 1):
    for i in range(0, cantO):
        posObs[i].setSFVec3f([posObsAct[0,i], posObsAct[1,i], -6.39203e-05])

for b in range(NStart, N):
    if (agent_setup == 1): # random agent position spawn
        PosTodos[b].setSFVec3f([X[1,b], X[0,b], -6.39203e-05])
        
    elif (agent_setup == 2): # random initial position markers spawn
        posIniPos[b].setSFVec3f([X[1,b], X[0,b], 0.3])
        posIniPosVec[b] = posIniPos[b].getSFVec3f()
                
    elif (agent_setup == 3): # instant agent position based on saved setup
        with open('D:/AlejandroDigital/tesisAlejandro/codigo/comunicacion_pololu/first_setup.pickle','rb') as f:
            setup_pos = pickle.load(f)
        PosTodos[b].setSFVec3f([setup_pos[b,0], setup_pos[b,1], -6.39203e-05])
    
    elif (agent_setup == 4): # custom agent positioning
    
        posIniPos[b].setSFVec3f([setup_pos[b,0],setup_pos[b,1], setup_pos[b,2]])
        posIniPosVec[b] = posIniPos[b].getSFVec3f()
        if (fisico == 0):  
            PosTodos[b].setSFVec3f([X[1,b], X[0,b], -6.39203e-05])
            """
            PosTodosVec[b] = PosTodos[b].getSFVec3f()
            RotTodosVec[b] = RotTodos[b].getSFVec3f()
            PosRealAgents = np.array(PosTodosVec)
            RotRealAgents = np.array(RotTodosVec)
            np.savez('initial_conditions_f.npz', PosRealAgents = PosRealAgents, RotRealAgents = RotRealAgents)
            """
        elif (fisico == 1):#probarfisico 
            PosTodos[b].setSFVec3f([agents_pose[b,0], agents_pose[b,1], -6.39203e-05])
            RotTodos[b].setSFRotation([0, 0, 1, agents_pose[b,3]])
            PosTodosVec[b] = [agents_pose[b,0], agents_pose[b,1], -6.39203e-05]
            RotTodosVec[b] = [0, 0, 1, agents_pose[b,3]]
    
    elif (agent_setup == 5):
        if (initial_pos_setup == 0):
            posIniPos[b].setSFVec3f([X[1,b], X[0,b], 0.3])
            
        elif (initial_pos_setup == 1):
            posIniPos[b].setSFVec3f([setup_pos[b,0],setup_pos[b,1], setup_pos[b,2]])
            
        if (fisico == 0):
            if (r_initial_conditions == 0): 
                PosTodos[b].setSFVec3f([X[1,b], X[0,b], -6.39203e-05])
                if (initial_pos_setup == 1):
                    PosTodos[b].setSFVec3f([setup_pos[b,0],setup_pos[b,1], -6.39203e-05])
                
            if (r_initial_conditions == 1):
                PosTodos[b].setSFVec3f([PosRealAgents[b,0], PosRealAgents[b,1], -6.39203e-05])
                RotTodos[b].setSFRotation([0, 0, 1, RotRealAgents[b,3]])
                posIniPos[b].setSFVec3f([PosRealIniPosVec[b,0], PosRealIniPosVec[b,1], 0.3])
            
        if (fisico == 1):  #try elif later
            PosTodos[b].setSFVec3f([agents_pose[b,0], agents_pose[b,1], -6.39203e-05])
            RotTodos[b].setSFRotation([0, 0, 1, agents_pose[b,3]])
            PosTodosVec[b] = [agents_pose[b,0], agents_pose[b,1], -6.39203e-05]
            RotTodosVec[b] = [0, 0, 1, agents_pose[b,3]]
            
        posIniPosVec[b] = posIniPos[b].getSFVec3f()
    #pObjs[b].setSFVec3f([X[1,b], X[0,b], 0.3])
    #PosTodos[b].setSFVec3f([X[1,b], X[0,b], -6.39203e-05])
    #PosTodos[b].setSFVec3f([setup_pos[b,0], setup_pos[b,1], -6.39203e-05])
  

if (fisico == 1):
    PosRealAgents = np.array(PosTodosVec)
    RotRealAgents = np.array(RotTodosVec)
    np.savez('initial_conditions_f.npz', PosRealAgents = PosRealAgents, RotRealAgents = RotRealAgents, posIniPosVec = posIniPosVec,  posObsAct = posObsAct, pObjVec = pObjVec, NStart = NStart, N = N)
    
"""    
for obstacle in range(0, len(posObs)):
    posObs[obstacle].setSFVec3f([setup_pos[obstacle+10,0], setup_pos[obstacle+10,1], -6.39203e-05])
    
    pass
pObj.setSFVec3f([setup_pos[len(setup_pos)-1,0], setup_pos[len(setup_pos)-1,1], -6.39203e-05])
"""
# Posiciones actuales
posActuales = np.zeros([2,N])


# Matriz de velocidades
V = np.zeros([2,N])

# Matriz de obstaculos
#posObsAct = np.empty([2,cantO])

d = Fmatrix(form_shape,rigidity_level)
print(d)

# Main loop:
cambio = 0	
if (r_initial_conditions):
    cambio = 1
					# variable para cambio de control 
while supervisor.step(TIME_STEP) != -1:
    #print(posActuales)
    if (fisico == 0):
        for obs in range(0,cantO):
            posObsAct[0][obs] = posObs[obs].getSFVec3f()[0]
            posObsAct[1][obs] = posObs[obs].getSFVec3f()[1]
        
        pObjVec = pObj.getSFVec3f()
    if (fisico == 1):
        #lock.acquire()
        
        try:#probar locks y quitar try
            agents_pose = update_data()
        except:
            agents_pose = agents_pose_old
        agents_pose_old = agents_pose
        #lock.release()
        #agents_pose = update_data()
        
        for marker in range(len(agents_pose)):
            agents_pose[marker,3] = agents_pose[marker,3] - desfases_euler[marker,3]
        if (r_obs == 1):
            for obs in range(0,cantO):
                posObsAct[0][obs] = agents_pose[obs+10,0]
                posObsAct[1][obs] = agents_pose[obs+10,1]
        
        if (r_obs == 0):
            for obs in range(0,cantO):
                posObsAct[0][obs] = posObs[obs].getSFVec3f()[0]
                posObsAct[1][obs] = posObs[obs].getSFVec3f()[1]
        
        if (r_obj == 1):
            pObjVec[0] = agents_pose[0,0]
            pObjVec[1] = agents_pose[0,1]
        
        if (r_obj == 0):
            pObjVec = pObj.getSFVec3f() 
        
    #print("cambio",cambio)
    	
	# Se obtienen posiciones actuales
    for c in range(NStart,N):
        if (fisico == 0):
            posC = Agents[c].getField("translation")
            posActuales[0][c] = posC.getSFVec3f()[0]
            posActuales[1][c] = posC.getSFVec3f()[1]
        elif (fisico == 1):
            posActuales[0][c] = agents_pose[c][0]
            posActuales[1][c] = agents_pose[c][1]    
    
    for g in range(NStart,N):
        E0 = 0
        E1 = 0
        for h in range(NStart,N):
            dist = np.asarray([posActuales[0][g]-posActuales[0][h], posActuales[1][g]-posActuales[1][h]])	# vector xi - xj   
            mdist = math.sqrt(dist[0]**2 + dist[1]**2)														# norma euclidiana vector xi - xj
            dij = 0.3*d[g][h]																				# distancia deseada entre agentes i y j
            
			# Peso añadido a la ecuación de consenso
            if(mdist == 0 or mdist >= R):
                w = 0
            else:
                if(cambio == 1 or cambio == 0): 										# inicio: acercar a los agentes sin chocar
                    #print("collision avoidance")
                    w = 0.05*(mdist - (2*(r+0.05)))/(mdist - (r+0.05))**2 
                    #w = (mdist - (2*(r+0.05)))/(mdist - (r+0.05))**2 	# collision avoidance
                elif (cambio == 2 or cambio == 3):
                    if(dij == 0):										# si no hay arista, se usa función plana como collision avoidance
                        #print("cosh")
                        w = 0.15*math.sinh(15*mdist-6)/mdist 		
                    else:												# collision avoidance & formation control
                        #print("formacion")
                        w = (4*(mdist - dij)*(mdist - r) - 2*(mdist - dij)**2)/(mdist*(mdist - r)**2)
                
            # Tensión de aristas entre agentes 
            E0 = E0 + 5*w*dist[0]
            E1 = E1 + 5*w*dist[1]
        # Collision avoidance con obstáculos
        for j in range(0,cantO):
            distO0 = posActuales[0,g] - posObsAct[0][j]
            distO1 = posActuales[1,g] - posObsAct[0][j]  
            mdistO = math.sqrt(distO0**2 + distO1**2) - sizeO

            if(abs(mdistO) < 0.0001):
                mdistO = 0.0001
            w = -1/(mdistO**2)

            E0 = E0 + 0.6*w*distO0
            E1 = E1 + 0.6*w*distO1    
        # Actualización de velocidad
        V[0][g] = -1*(E0)*TIME_STEP/1000 
        V[1][g] = -1*(E1)*TIME_STEP/1000 

	# Al llegar muy cerca de la posición deseada realizar cambio de control
    normV2 = 0
    for m in range(NStart,N):
        nV2 = V[0][m]**2 + V[1][m]**2
        normV2 = normV2 + nV2
    normV = math.sqrt(normV2)
    print("normV", normV)
    
    if(normV < 0.5 and cambio == 1):
        form_cycle = ciclo
        cambio = 2
    
    elif(normV < 2 and cambio == 2):
        obj_cycle = ciclo
        cambio = 3    
    
    if (cambio == 0):
        ready_ini_pos = 0
        cont_N = 0
        for obj in range(NStart,N):
            cont_N = cont_N + 1
            V[0][obj] = V[0][obj] - 5*(posActuales[0][obj]-posIniPosVec[obj][0])
            V[1][obj] = V[1][obj] - 5*(posActuales[1][obj]-posIniPosVec[obj][1])
            if ((posActuales[0][obj]-posIniPosVec[obj][0])<0.05 and (posActuales[1][obj]-posIniPosVec[obj][1])<0.05):
                ready_ini_pos = ready_ini_pos + 1
        if (ready_ini_pos == cont_N):
            begin_alg_time = ciclo
            cambio = 1
            if (r_initial_conditions == 1):
                for b in range(NStart, N):
                    PosTodosVec[b] = [agents_pose[b,0], agents_pose[b,1], -6.39203e-05]
                    RotTodosVec[b] = [0, 0, 1, agents_pose[b,3]]
                PosRealAgents = np.array(PosTodosVec)
                RotRealAgents = np.array(RotTodosVec)                            
        #print(V)
        
    if (cambio == 3):
        if (posActuales[0][NStart]-pObjVec[0] < 0.6):     
            V[0][NStart] = V[0][NStart] - total_agent_weight*4*(posActuales[0][NStart]-pObjVec[0])
            V[1][NStart] = V[1][NStart] - total_agent_weight*4*(posActuales[1][NStart]-pObjVec[1])
        elif (posActuales[0][NStart]-pObjVec[0] >= 0.6):
            V[0][NStart] = V[0][NStart] - 10*(posActuales[0][NStart]-pObjVec[0])
            V[1][NStart] = V[1][NStart] - 10*(posActuales[1][NStart]-pObjVec[1])
    

    lock.acquire()
    pick_V = pickle.dumps(V)
    shm1.buf[:len(pick_V)] = pick_V
    pick_agents_pose = pickle.dumps(agents_pose)
    shm2.buf[:len(pick_agents_pose)] = pick_agents_pose
    lock.release()
    trajectory.append(posActuales.copy())
    velocityHist.append(V.copy())
    normVHist.append(normV)
    objHist.append(pObjVec)
    obsHist.append(posObsAct.copy())
    print(ciclo)
    print(cambio)    
    ciclo = ciclo + 1     
    
    if keyboard.is_pressed('a'):
        V = np.zeros([2,N])
        lock.acquire()
        pick_V = pickle.dumps(V)
        shm1.buf[:len(pick_V)] = pick_V
        pick_agents_pose = pickle.dumps(agents_pose)
        shm2.buf[:len(pick_agents_pose)] = pick_agents_pose
        trajectory_data = np.array(trajectory)
        velocity_data = np.array(velocityHist)
        normV_data = np.array(normVHist) 
        obj_data = np.array(objHist) 
        obs_data = np.array(obsHist)
        np.savez('trial0.npz', trajectory_data = trajectory_data, velocity_data = velocity_data, normV_data = normV_data, obj_data = obj_data, obs_data = obs_data, total_cycle = ciclo, form_cycle = form_cycle, obj_cycle = obj_cycle, quantO = cantO, posObsAct = posObsAct, sizeO = sizeO, NStart = NStart, N = N, pObjVec = pObjVec, PosRealAgents = PosRealAgents, RotRealAgents = RotRealAgents, begin_alg_time = begin_alg_time, posIniPosVec = posIniPosVec, fisico = fisico, r_initial_conditions = r_initial_conditions, r_obs = r_obs, r_obj = r_obj, TIME_STEP = TIME_STEP, agent_setup = agent_setup, obs_active = obs_active, initial_pos_setup = initial_pos_setup, r = r, R = R, MAX_SPEED = MAX_SPEED, form_shape = form_shape, rigidity_level = rigidity_level, total_agent_number = total_agent_number, NMax = NMax)       
        lock.release()
        shm1.close()
        shm2.close()  
        break

    
     