import socket
from robotat_connect import robotat_connect
from robotat_get_pose import robotat_get_pose
from robotat_disconnect import robotat_disconnect

robotat = robotat_connect()
if robotat:
    print("connected to robotat")
else:
    print("error")

try:
    print(robotat)
    pose = None
    timeout_seconds = 10

    # Set a timeout for the robotat_get_pose() function call
    with socket.timeout(timeout_seconds):
        pose = robotat_get_pose(robotat, [1], 'quat')

    if pose is not None:
        print("Received pose:", pose)
    else:
        print("Pose not received within the timeout.")
except socket.timeout:
    print(f"Timeout: robotat_get_pose() took more than {timeout_seconds} seconds.")
except Exception as e:
    print("An error occurred:", e)
finally:
    robotat_disconnect(robotat)






