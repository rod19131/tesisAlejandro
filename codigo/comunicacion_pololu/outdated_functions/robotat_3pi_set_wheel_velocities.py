import struct

def robotat_3pi_set_wheel_velocities(robot, dphiL, dphiR):
    wheel_maxvel_rpm = 850
    wheel_minvel_rpm = -850

    if dphiL > wheel_maxvel_rpm:
        print(f'Warning: Left wheel speed saturated to {wheel_maxvel_rpm} rpm')
        dphiL = wheel_maxvel_rpm

    if dphiR > wheel_maxvel_rpm:
        print(f'Warning: Right wheel speed saturated to {wheel_maxvel_rpm} rpm')
        dphiR = wheel_maxvel_rpm

    if dphiL < wheel_minvel_rpm:
        print(f'Warning: Left wheel speed saturated to {wheel_minvel_rpm} rpm')
        dphiL = wheel_minvel_rpm

    if dphiR < wheel_minvel_rpm:
        print(f'Warning: Right wheel speed saturated to {wheel_minvel_rpm} rpm')
        dphiR = wheel_minvel_rpm

    # Encode to a simple CBOR array
    cbormsg = struct.pack('BBBBfBf', 130, 250, 0, 0, dphiL, 250, 0, 0, dphiR)

    try:
        robot['tcpsock'].sendall(cbormsg)
    except Exception as e:
        print('An error occurred while sending the wheel velocities command:', e)
