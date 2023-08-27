from robotat_connect import *
from robotat_disconnect import *
from robotat_get_pose import *
# Example Robotat server IP address
# Connect to the Robotat server
tcp_obj = robotat_connect()

# Check if the connection was successful
if tcp_obj is not None:
    print('Connected to Robotat server.')
    
    try:
        agents_ids = [1, 2, 3]
        rotrep = 'ZYX'

        pose_data = robotat_get_pose(tcp_obj, agents_ids, rotrep)
        print('Pose Data:', pose_data)

        robotat_disconnect(tcp_obj)
        pass# Perform operations using the tcp_obj
        
    
    except:
        print('error ocurrió')
        pass

    finally:
        # Disconnect from the Robotat server
        robotat_disconnect(tcp_obj)
else:
    print('Connection failed.')