from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
class CarRegulation:
    def __init__(self):
        self.rotvel = []
        self.power = []
        self.ballx = []
        self.bally = []
        self.sum_bally = []
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
        self.sum_bally = []
    def summary(self): # 求rotvel平均值
        print("rotvel", len(set(self.rotvel)), "power", len(set(self.power)))
        num_bally = [[0 for cols in range(len(set(self.rotvel)))] for rows in range(len(set(self.power)))] # row行,col列
        self.sum_bally = [[0 for cols in range(len(set(self.rotvel)))] for rows in range(len(set(self.power)))]
        for i in range(len(self.rotvel)):
            num_power = list(set(self.power)).index(self.power[i]) # 当前行列号
            num_rotvel = list(set(self.rotvel)).index(self.rotvel[i])
            num_bally[num_power][num_rotvel] += 1 # 求个数
            self.sum_bally[num_power][num_rotvel] += self.bally[i] # 求和
        for i in range(len(set(self.power))):
            for j in range(len(set(self.rotvel))):
                if num_bally[i][j] == 0:
                    self.sum_bally[i][j] = 0
                else:
                    self.sum_bally[i][j] = self.sum_bally[i][j] / num_bally[i][j] # 求平均
    # def analy(self):
    #     old_power = -1
    #     for i in range(len(self.power)):
    #         if old_power != self.power[i]:

    def draw(self):
        ax = Axes3D(plt.figure())
        ax.scatter(self.power, self.rotvel, self.bally, c='g')
        # for i in range(len(set(self.power))):
            # for j in range(len(set(self.rotvel))):
                # ax.scatter(list(set(self.power))[i], list(set(self.rotvel))[j], self.sum_bally[i][j], c='r')
        plot_power, plot_carvel = np.meshgrid(sorted(list(set(self.power))), sorted(list(set(self.rotvel)))) # 数据网格化
        plot_bally = np.array(self.sum_bally)
        print(type(plot_bally))
        ax.plot_surface(plot_power, plot_carvel, plot_bally)
        # ax.plot_surface(plot_power, plot_carvel, self.sum_bally, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        # plot_rotvel = np.arange(1, 8, 0.1)
        # plot_power = [3000] * len(plot_rotvel)
        # raw_bally = np.polyfit(self.rotvel, self.bally, 1)
        # val_bally = np.poly1d(raw_bally)
        # plot_bally = val_bally(plot_rotvel)
        # ax.scatter(plot_rotvel, plot_bally, plot_power)
        ax.set_xlabel('rotvel', fontdict={'size': 15, 'color': 'black'})
        ax.set_ylabel('power', fontdict={'size': 15, 'color': 'black'})
        ax.set_zlabel('bally', fontdict={'size': 15, 'color': 'black'})
        plt.show()
class CarDataTest:
    def read(address):
        car_data = CarRegulation()
        with open(in_address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 4:
                    continue
                t_rotvel = float(line_data[0].strip())
                t_power = float(line_data[1].strip())
                t_ballx = float(line_data[2].strip())
                t_bally = float(line_data[3].strip())
                car_data.assign(t_rotvel, t_power, t_ballx, t_bally)
        # print(car_data.bally)
        car_data.summary()
        car_data.draw()
in_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguData.txt"
CarDataTest.read(in_address)