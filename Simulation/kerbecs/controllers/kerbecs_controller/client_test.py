import socket
import math
import re


def convert_input(bytes):
    input_string = bytes.decode('utf-8')
    print(input_string)

    radian_strings = re.findall('\d[.]\d+', input_string)

    steps = list(int(math.degrees(float(radian)) / 1.8) for radian in radian_strings)

    return steps


with  socket.socket() as comm:

    port = 6666
    comm.connect(('127.0.0.1', port))

    while(True):
        # print((comm.recv(1024).decode('utf-8')))
        input_bytes = comm.recv(4096)
        
        print(convert_input(input_bytes))
