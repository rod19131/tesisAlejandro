import socket
import json
import time
from scipy.spatial.transform import Rotation as R
 
def robotat_get_pose(tcp_obj, agents_ids, rotrep):
    timeout_count = 0
    timeout_in100ms = 100

    tcp_obj.recv(1024)  # Clear any existing data

    if all(0 < agent_id <= 100 for agent_id in agents_ids):
        s = {
            "dst": 1,
            "cmd": 1,
            "pld": [int(agent_id) for agent_id in agents_ids]
        }
        tcp_obj.send(json.dumps(s).encode())

        while (tcp_obj.recv(1024).decode() == "" and timeout_count < timeout_in100ms):
            timeout_count += 1
            time.sleep(0.1)

        if timeout_count == timeout_in100ms:
            print('ERROR: Could not receive data from server.')
            return
        else:
            mocap_data = json.loads(tcp_obj.recv(1024).decode())
            mocap_data = [mocap_data[i:i + 7] for i in range(0, len(mocap_data), 7)]

            if rotrep == 'quat':
                # No additional conversion needed
                pass
            else:
                try:
                    for i in range(len(mocap_data)):
                        rotation = R.from_quat(mocap_data[i][3:])
                        eul_angles = rotation.as_euler(rotrep)
                        mocap_data[i][3:] = list(eul_angles)
                except:
                    raise ValueError('Invalid Euler angle sequence.')

            return mocap_data
    else:
        print('ERROR: Invalid ID(s).')
        return None
