import socket

def robotat_3pi_connect(agent_id):
    if not isinstance(agent_id, int) or agent_id < 0 or agent_id > 19:
        raise ValueError('Invalid agent ID. Allowed IDs: 0 - 19.')

    robot = {}

    if isinstance(agent_id, int):
        robot_id = round(agent_id)
    else:
        raise ValueError('Can only pair with a single 3Pi agent.')

    robot['id'] = robot_id

    if robot_id > 9:
        ip = '192.168.50.1'
    else:
        ip = '192.168.50.10'

    ip = ip + str(robot_id)
    robot['ip'] = ip
    robot['port'] = 8888

    try:
        tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcpsock.connect((robot['ip'], robot['port']))
        robot['tcpsock'] = tcpsock
    except Exception as e:
        print('ERROR: Could not connect to the robot.')
        print(e)

    return robot
"""
# Example usage:
try:
    robot = robotat_3pi_connect(5)
    print(f"Connected to robot ID: {robot['id']}, IP: {robot['ip']}, Port: {robot['port']}")
except ValueError as ve:
    print(ve)
except Exception as e:
    print('An error occurred:', e)
"""