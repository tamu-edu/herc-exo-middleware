"""
Objects for passing data between two real-time loops.
"""

import zmq
import numpy as np



class ZmqBinarySynchB:
    def __init__(self, bindport="tcp://*:5558", connectport="tcp://localhost:5557", dtype=float):
        self.context = zmq.Context()
        self.socketB = self.context.socket(zmq.PUB)
        self.socketB.bind(bindport)

        self.socketA = self.context.socket(zmq.SUB)
        self.socketA.setsockopt(zmq.SUBSCRIBE, b'A')
        self.socketA.connect(connectport)

        self.dtype = dtype

        self.my_count = 0
        self.data_out = None


    def update(self, data_in):
        """ read all messages, then send data."""
        while True:
            try: 
                message = self.socketA.recv(zmq.NOBLOCK)
            
                if int(message[1:4])==self.my_count:
                    self.my_count+=1
                    if self.my_count>=1000:
                        self.my_count=0
                self.data_out = np.frombuffer(message[4:], dtype=self.dtype)
            except zmq.error.Again:
                break
    
        self.socketB.send(b"B%03d%s"%(self.my_count, data_in.tobytes()))
        return self.data_out

class ZmqBinarySynchA:
    def __init__(self, bindport="tcp://*:5557", connectport="tcp://localhost:5558", dtype=float):
        self.context = zmq.Context()
        self.socketA = self.context.socket(zmq.PUB)
        self.socketA.bind(bindport)

        self.socketB = self.context.socket(zmq.SUB)
        self.socketB.setsockopt(zmq.SUBSCRIBE, b'B')
        self.socketB.connect(connectport)

        self.dtype = dtype

        self.my_count = -42
        self.data_out = None

    def update(self, data_in):
        """ read all messages, then send data. """
        while True:
            try: 
                message = self.socketB.recv(zmq.NOBLOCK)
                self.my_count=int(message[1:4])
                try:
                    self.data_out = np.frombuffer(message[4:], dtype=self.dtype)
                except ValueError as e:
                    print("halp", message[4:], "didn't fit as ", self.dtype)
                    raise e
            except zmq.error.Again:
                break
        self.socketA.send(b"A%03d%s"%(self.my_count, data_in.tobytes()))

        return self.data_out
    