import sys
class Connect:
    def __init__(self):
        pass
    def connect_udp(self):
        return "127.0.0.1:14551"
    def connect_tcp(self):
        return '127.0.0.1:5760'
    def connect_serial(self):
        return '/dev/serial0'
    def connect_usb(self):
        return '/dev/ttyUSB0'