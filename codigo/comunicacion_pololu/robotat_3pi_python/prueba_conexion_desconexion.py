from robotat_connect import *
from robotat_disconnect import *
from robotat_get_pose import *
# Example Robotat server IP address
robotat_ip = "192.168.50.200"

# Connect to the Robotat server
tcp_obj = robotat_connect(robotat_ip)

# Check if the connection was successful
if tcp_obj is not None:
    print("Connected to Robotat server.")
    
    try:
        #agents_ids = [1, 2, 3]
        agents_ids = 1
        rotrep = "XYZ"

        # Call the robotat_get_pose function
        pose_data = robotat_get_pose(tcp_obj, agents_ids, rotrep)
        print(pose_data)
        # Print the received pose data
        """
        if pose_data is not None:
            for agent_id, pose in zip(agents_ids, pose_data):
                print(f"Agent ID: {agent_id}, Pose Data: {pose}")
        pass# Perform operations using the tcp_obj
        """
    
    except:
        print("error ocurrió")
        pass

    finally:
        # Disconnect from the Robotat server
        robotat_disconnect(tcp_obj)
else:
    print("Connection failed.")