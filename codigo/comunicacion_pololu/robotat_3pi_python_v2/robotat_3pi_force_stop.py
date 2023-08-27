import struct
import socket
def robotat_3pi_force_stop(robot):
    dphiL = 0.0
    dphiR = 0.0

    # Encode to a simple CBOR array
    cbormsg = bytearray([130, 250]) + struct.pack('<f', dphiL) + bytearray([250]) + struct.pack('<f', dphiR)

    try:
        robot['tcpsock'].sendall(cbormsg)
    except Exception as e:
        print('An error occurred while sending the force stop command:', e)
