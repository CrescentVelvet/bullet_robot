import matplotlib.pyplot as plt
import math
class CarMoveOne:
    def __init__(self):
        self.id = -1
        self.send_vel = -1
        self.raw_vel = []
    def assign(self, raw_vel):
        self.raw_vel.append(raw_vel)
    def clear(self):
        self.id = -1
        self.send_vel = -1
        self.raw_vel = []
class CarMoveAll:
    def __init__(self):
        self.id = []
        self.send_vel = []
        self.raw_vel = []
    def assign(self, id, send_vel, raw_vel):
        self.id.append(id)
        self.send_vel.append(send_vel)
        self.raw_vel.append(raw_vel)
    def clear(self):
        self.id = []
        self.send_vel = []
        self.raw_vel = []
class CarMoveTest:
    def read(in_address):
        car = CarMoveAll()
        with open(in_address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 3:
                    continue
                if line_data[1] == '0':
                    continue
                car.assign(line_data[0], line_data[1], line_data[2])
        print(car.id)
in_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/Speedconstant.txt"
CarMoveTest.read(in_address)