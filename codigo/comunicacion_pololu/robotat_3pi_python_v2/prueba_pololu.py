import time
from robotat_3pi_connect import *
from robotat_3pi_disconnect import *
from robotat_3pi_set_wheel_velocities import *
from robotat_3pi_force_stop import *
try:
    robot = robotat_3pi_connect(7)
    print(f"Connected to robot ID: {robot['id']}, IP: {robot['ip']}, Port: {robot['port']}")
    robotat_3pi_set_wheel_velocities(robot, 20, -20)
    time.sleep(5)
    robotat_3pi_force_stop(robot)
except ValueError as ve:
    print(ve)
except Exception as e:
    print('An error occurred:', e)
finally:
    robotat_3pi_disconnect(robot)
    