def robotat_disconnect(tcp_obj):
    try:
        tcp_obj.sendall(b'EXIT')
        tcp_obj.close()
        print('Disconnected from Robotat server.')
    except Exception as e:
        print('An error occurred while disconnecting:', e)
