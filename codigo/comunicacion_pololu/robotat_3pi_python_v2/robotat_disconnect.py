def robotat_disconnect(tcp_obj):
    try:
        tcp_obj.send(b'EXIT')
        tcp_obj.close()
        print('Disconnected from Robotat server.')
    except Exception as e:
        print('Error while disconnecting:', e)