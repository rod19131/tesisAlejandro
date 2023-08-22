import struct

def robotat_3pi_force_stop(robot):
    dphiL = 0
    dphiR = 0

    # Encode to a simple CBOR array
    cbormsg = struct.pack('BBBBfBf', 130, 250, 0, 0, dphiL, 250, 0, 0, dphiR)

    try:
        robot['tcpsock'].sendall(cbormsg)
    except Exception as e:
        print('An error occurred while sending the force stop command:', e)
