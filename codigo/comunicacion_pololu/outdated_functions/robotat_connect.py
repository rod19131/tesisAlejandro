import socket

def robotat_connect(ip):
    port = 1883
    try:
        tcp_obj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_obj.connect((ip, port))
        return tcp_obj
    except Exception as e:
        print('ERROR: Could not connect to Robotat server.')
        return None
