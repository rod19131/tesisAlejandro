import socket
import json
import time
from scipy.spatial.transform import Rotation as R

def robotat_connect():
    ip = '192.168.50.200'
    port = 1883
    try:
        tcp_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_obj.connect((ip, port))
        return tcp_obj
    except Exception as e:
        print('ERROR: Could not connect to Robotat server.')
        return None

def robotat_disconnect(tcp_obj):
    try:
        tcp_obj.send(b'EXIT')
        tcp_obj.close()
        print('Disconnected from Robotat server.')
    except Exception as e:
        print('Error while disconnecting:', e)

def robotat_get_pose(tcp_obj, agent_id):
    try:
        # Clear any existing data
        tcp_obj.recv(1024)

        # Prepare the request payload
        request_payload = {
            "dst": 1,   # DST_ROBOTAT
            "cmd": 1,   # CMD_GET_POSE
            "pld": [agent_id]
        }

        # Send the request as JSON-encoded data
        tcp_obj.send(json.dumps(request_payload).encode())

        # Receive the response
        response_data = tcp_obj.recv(4096)  # Adjust buffer size as needed

        # Decode and process the response
        if response_data:
            pose_data = json.loads(response_data)
            return pose_data
        else:
            print("Received empty response from server.")
            return None
    except Exception as e:
        print("An error occurred:", e)
        return None