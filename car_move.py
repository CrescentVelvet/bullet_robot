import matplotlib.pyplot as plt
import math
import numpy as np
class CarMoveOne:
    def __init__(self):
        self.id = -1
        self.send_vel = []
        self.raw_vel = []
        self.avg_vel = []
        self.val_fit = []
    def assign(self, id, send_vel, raw_vel):
        self.id = id
        self.send_vel.append(send_vel)
        self.raw_vel.append(raw_vel)
    def clear(self):
        self.id = -1
        self.send_vel = []
        self.raw_vel = []
        self.avg_vel = []
    def draw(self): # 绘制一幅图像
        plot = plt.scatter(self.send_vel, self.raw_vel, s=0.1)
        plt.xticks(np.arange(100, 2100, 100))
        plt.yticks(np.arange(100, 2100, 100))
        plt.xlabel('send_vel')
        plt.ylabel('raw_vel')
        plt.show()
    def average(self): # 求send_vel对应的raw_vel中位数
        setdata = list(set(self.send_vel)) # 取不重复数列
        alldata = []
        onedata = []
        for i in range(len(setdata)): # 按照send_vel划分raw_vel
            for j in range(len(self.send_vel)):
                if setdata[i] == self.send_vel[j]:
                    onedata.append(self.raw_vel[j])
            alldata.append(onedata)
            onedata = []
        for i in range(len(setdata)):
            sortdata = sorted(alldata[i])
            size = len(sortdata)
            if size % 2 == 0: # 判断列表长度为偶数
                median = (sortdata[size//2]+sortdata[size//2-1])/2
            elif size % 2 == 1: # 判断列表长度为奇数
                median = sortdata[(size-1)//2]
            else:
                print("error in average")
            self.avg_vel.append(median)
    def calculate(self): # 计算拟合函数
        raw_fit = np.polyfit(list(set(self.send_vel)), self.avg_vel, 1)
        self.val_fit = np.poly1d(raw_fit)
class CarMoveAll:
    def __init__(self):
        self.id = []
        self.send_vel = []
        self.raw_vel = []
    def assign(self, id, send_vel, raw_vel): # 赋值
        self.id.append(id)
        self.send_vel.append(send_vel)
        self.raw_vel.append(raw_vel)
    def tolist(self): # 转化列表
        self.id = list(map(int, self.id))
        self.send_vel = list(map(float, self.send_vel))
        self.raw_vel = list(map(float, self.raw_vel))
    def clear(self): # 清除
        self.id = []
        self.send_vel = []
        self.raw_vel = []
    def getid(self): # 获取id个数
        return len(set(self.id))
class CarMoveTest:
    def read(in_address):
        carcar = CarMoveAll()
        with open(in_address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 3:
                    continue
                if line_data[1] == '0':
                    continue
                carcar.assign(line_data[0], line_data[1], line_data[2])
        carcar.tolist()
        car_list = []
        for i in range(static_car_num): # 初始化
            car_list.append(CarMoveOne())
        for i in range(len(carcar.id)): # 车号划分
            car_list[carcar.id[i]].assign(carcar.id[i], carcar.send_vel[i], carcar.raw_vel[i])
        # for i in range(static_car_num):
            # print(i, car_list[i].id)
        for i in range(static_car_num): # 计算中位数并拟合
            if car_list[i].id != -1:
                car_list[i].average()
                car_list[i].calculate()
        ax = [None] * carcar.getid()
        for i in range(carcar.getid()): # 绘制图像
            ax[i] = plt.subplot(2, 3, i+1)
            plot_send = np.arange(100, 2100, 1)
            plot_raw = car_list[carcar.id[i]].val_fit(plot_send)
            plot = plt.scatter(car_list[carcar.id[i]].send_vel, car_list[carcar.id[i]].raw_vel, s=1, color='limegreen')
            plot = plt.scatter(list(set(car_list[carcar.id[i]].send_vel)), car_list[carcar.id[i]].avg_vel, s=30, color='orange', alpha=0.8)
            plot = plt.plot(plot_send, plot_raw, 'red')
            # plt.xlabel('send_vel')
            plt.xlabel(str(car_list[carcar.id[i]].id) + 'car' + str(car_list[carcar.id[i]].val_fit))
            plt.ylabel('raw_vel')
        plt.show()
        # car_list[10].draw()
        # car_list[10].average()
# class MoveData:
static_car_num = 16
in_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/Speedconstant.txt"
CarMoveTest.read(in_address)