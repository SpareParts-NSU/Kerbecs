import socket
import math
from random import randrange
from time import sleep

#set up communication socket

with socket.socket() as comm:
    
    print('socket created')

    port = 6666
    comm.bind(('', port))
    comm.listen(5)

    client, address = comm.accept()

    client.send(b'connected to kerbecs\n')

    while(True):
        s = '{} {}  {}  {}  {} {}  {}  {}  {} {}  {}  {}  '.format(math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)), math.radians(randrange(60, 120)))
        # s = '1.28243899126  -1.28244447083 1.28243986043  -1.28244451604    1.28243899126  -1.28244447083 1.28243986043  -1.28244451604 1.28243899126  -1.28244447083 1.28243986043  -1.28244451604'
        client.send(bytes(s, 'utf-8'))
        sleep(0.5)
