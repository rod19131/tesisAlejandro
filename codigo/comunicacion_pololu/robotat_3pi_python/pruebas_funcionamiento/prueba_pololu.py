import time
from funciones_conjunto_3pi import *
try:
    #robot = robotat_3pi_connect(5)
    robot2 = robotat_3pi_connect(1)
    #print(f"Connected to robot ID: {robot['id']}, IP: {robot['ip']}, Port: {robot['port']}")
    print(f"Connected to robot ID: {robot2['id']}, IP: {robot2['ip']}, Port: {robot2['port']}")
    #robotat_3pi_set_wheel_velocities(robot, -50, 50)
    robotat_3pi_set_wheel_velocities(robot2, 50, -50)
    time.sleep(6)
    #robotat_3pi_force_stop(robot)
    robotat_3pi_force_stop(robot2)
except ValueError as ve:
    print(ve)
except Exception as e:
    print('An error occurred:', e)
finally:
    #robotat_3pi_disconnect(robot)
    robotat_3pi_disconnect(robot2)

    