import json
import time
from scipy.spatial.transform import Rotation as R

def robotat_get_pose(tcp_obj, agents_ids, rotrep):
    timeout_count = 0
    timeout_in100ms = 1 / 0.1
    tcp_obj.recv(1024)  # Clear buffer

    if min(agents_ids) > 0 and max(agents_ids) <= 100:
        s = {
            "dst": 1,  # DST_ROBOTAT
            "cmd": 1,  # CMD_GET_POSE
            "pld": [int(agent_id) for agent_id in agents_ids],
        }
        tcp_obj.sendall(json.dumps(s).encode())

        while tcp_obj.recv(1024) == b"" and timeout_count < timeout_in100ms:
            timeout_count += 1
            time.sleep(0.1)

        if timeout_count == timeout_in100ms:
            print("ERROR: Could not receive data from server.")
            return None
        else:
            mocap_data = json.loads(tcp_obj.recv(1024).decode())
            mocap_data = [mocap_data[i:i + 7] for i in range(0, len(mocap_data), 7)]
            mocap_data = [list(map(float, data)) for data in mocap_data]

            if rotrep == "quat":
                # No conversion needed for quaternion representation
                pass
            else:
                try:
                    rotation = R.from_quat([data[3:] for data in mocap_data])
                    euler_angles = rotation.as_euler(rotrep,degrees=True) # el angulo se da en mayúsculas i.e. "XYZ"
                    mocap_data = [
                        mocap_data[i][:3] + list(euler_angles[i]) for i in range(len(mocap_data))
                    ]
                except:
                    raise ValueError("Invalid Euler angle sequence.")

            return mocap_data
    else:
        print("ERROR: Invalid ID(s).")
        return None
