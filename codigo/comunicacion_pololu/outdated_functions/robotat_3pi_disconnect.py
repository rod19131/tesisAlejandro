def robotat_3pi_disconnect(robot):
    try:
        del robot
        print('Disconnected from robot.')
    except Exception as e:
        print('An error occurred while disconnecting:', e)