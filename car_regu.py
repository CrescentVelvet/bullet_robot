from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
class CarRegulation:
    def __init__(self):
        self.rotvel = []
        self.power = []
        self.ballx = []
        self.bally = []
    def assign(self, rotvel, power, ballx, bally):
        self.rotvel.append(rotvel)
        self.power.append(power)
        self.ballx.append(ballx)
        self.bally.append(bally)
    def clear(self):
        self.rotvel = []
        self.power = []
        self.ballx = []
        self.bally = []
    def draw(self):
        ax=plt.gca(projection="3d")
        ax.plot_surface(self.rotvel,self.power,self.bally,cmap="rainbow")
        plt.show()
class CarDataTest:
    def read(address):
        with open(in_address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 4:
                    continue
                