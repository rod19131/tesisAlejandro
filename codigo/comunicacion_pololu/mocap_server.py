from NatNetClient import NatNetClient
from _thread import *
import socket
import json
import time
import numpy as np
from pymycobot.mycobot import MyCobot
from threading import Lock

# Mocap data table
mocap_data = np.empty([100, 7]);

# TCP server info
host = '192.168.50.200'
port = 1883
ThreadCount = 0

# Se inicializan los objetos MyCobot en los puertos seriales adecuados y con el baudaje
# por defecto
try:
    mc2 = MyCobot("COM5", 115200)
except:
    print('MyCobot 2 is not connected.')

try:
    mc1 = MyCobot("COM3", 115200)
except:
    print('MyCobot 1 is not connected.')

wait_time = 0 # tiempo de espera entre movimientos

# Se definen los mutexes para el manejo compartido de los robots
lock1 = Lock()
lock2 = Lock()

# This is a callback function that gets connected to the NatNet client 
# and called once per mocap frame.
def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, 
                    rigidBodyCount, skeletonCount, labeledMarkerCount, 
                    timecode, timecodeSub, timestamp, isRecording, 
                    trackedModelsChanged ):
    #print("Receiving mocap data...")
    pass

# This is a callback function that gets connected to the NatNet client. 
# It is called once per rigid body per frame.
def receiveRigidBodyFrame( id, position, rotation ):
    #print("ID: " + str(id) + " position: [" + str(position[0]) + ", " + str(position[1]) + ", " + str(position[2]) + "]")
    mocap_data[id, 0] = position[0];
    mocap_data[id, 1] = position[1];
    mocap_data[id, 2] = position[2];
    mocap_data[id, 3] = rotation[3];
    mocap_data[id, 4] = rotation[0];
    mocap_data[id, 5] = rotation[1];
    mocap_data[id, 6] = rotation[2];
    #print(json.dumps(np.ndarray.tolist(mocap_data[id-1])).encode())
    
# TCP Server
def client_handler(connection):
    connection.send(str.encode('Connected to the Robotat server... Type EXIT to stop'))
    while True:
        data = connection.recv(2048)
        message = data.decode('utf-8')
        if message == 'EXIT':
            break

        pkt = json.loads(data)
        match pkt["dst"]:
            case 1: # DST_ROBOTAT
                match pkt["cmd"]:
                    case 1: # CMD_GET_POSE
                        reply = mocap_data[pkt["pld"]]
                        connection.sendall(json.dumps(reply.flatten().tolist()).encode())
                    case _:
                        pass
            
            case 2: # DST_MYCOBOT1
                with lock1:
                    match pkt["cmd"]:
                        case 1: # CMD_GET_POSE
                            while(len(mc1.get_coords()) == 0):
                                pass
                            connection.sendall(json.dumps(mc1.get_coords()).encode())

                        case 2: # CMD_SET_CONFIG
                            mc1.send_angles(pkt["pld"], 50)
                            time.sleep(wait_time)

                        case 3: # CMD_GET_CONFIG
                            while(len(mc1.get_angles()) == 0):
                                pass
                            connection.sendall(json.dumps(mc1.get_angles()).encode())

                        case 4: # CMD_OPEN_GRIPPER
                            mc1.set_gripper_state(0, 70) # se abre el gripper
                            time.sleep(wait_time)

                        case 5: #CMD_CLOSE_GRIPPER
                            mc1.set_gripper_state(1, 70) # se cierra el gripper
                            time.sleep(wait_time)

                        case _:
                            pass
            
            case 3: # DST_MYCOBOT2
                with lock2:
                    match pkt["cmd"]:
                        case 1: # CMD_GET_POSE
                            while(len(mc2.get_coords()) == 0):
                                pass
                            connection.sendall(json.dumps(mc2.get_coords()).encode())

                        case 2: # CMD_SET_CONFIG
                            mc2.send_angles(pkt["pld"], 50)
                            time.sleep(wait_time)

                        case 3: # CMD_GET_CONFIG
                            while(len(mc2.get_angles()) == 0):
                                pass
                            connection.sendall(json.dumps(mc2.get_angles()).encode())

                        case 4: # CMD_OPEN_GRIPPER
                            mc2.set_gripper_state(0, 70) # se abre el gripper
                            time.sleep(wait_time)

                        case 5: #CMD_CLOSE_GRIPPER
                            mc2.set_gripper_state(1, 70) # se cierra el gripper
                            time.sleep(wait_time)

                        case _:
                            pass

            case _:
                pass

        #reply = mocap_data[agents_ids]
        #connection.sendall(json.dumps(reply.flatten().tolist()).encode())
        #print(reply)
        #print(json.dumps(reply.flatten().tolist()))
        #reply = f'Server: {message}'
        #connection.sendall(str.encode(reply))
    connection.close()
    print('Disconnected from client.')

def accept_connections(ServerSocket):
    Client, address = ServerSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(client_handler, (Client, ))

def start_server(host, port):
    ServerSocket = socket.socket()
    try:
        ServerSocket.bind((host, port))
    except socket.error as e:
        print(str(e))
    print(f'Robotat server is listening on port {port}...')
    ServerSocket.listen()

    while True:
        accept_connections(ServerSocket)  
    

# Main Program  
streamingClient = NatNetClient()
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run() 
start_server(host, port)         
