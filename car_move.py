import matplotlib.pyplot as plt
import configparser
import numpy as np
import math
class MyConfigParser(configparser.ConfigParser):
    def __init__(self, defaults=None):
        configparser.ConfigParser.__init__(self, defaults=defaults)
    def optionxform(self, optionstr): # 重载ConfigParser实现大小写区分
        return optionstr
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
    def getid(self): # 获取不同id个数
        return len(set(self.id))
class CarMoveTest:
    def read(in_address, ini_address):
        carcar = CarMoveAll() # 全部车数据
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
        car_list = [] # 单车数据列表
        for i in range(static_car_num): # 初始化
            car_list.append(CarMoveOne())
        for i in range(len(carcar.id)): # 车号划分
            car_list[carcar.id[i]].assign(carcar.id[i], carcar.send_vel[i], carcar.raw_vel[i])
        for i in range(static_car_num):
            print(i, car_list[i].id)
        for i in range(static_car_num): # 计算中位数并拟合
            if car_list[i].id != -1:
                car_list[i].average()
                car_list[i].calculate()
        ax = [None] * carcar.getid()
        # robot_conf = MyConfigParser() # 参数写入
        # robot_conf.read(ini_address, encoding="utf-8")
        # for i in range(carcar.getid()):
        #     now_id = list(set(carcar.id))[i]
        #     robot_conf.set("Robot"+str(now_id), "OPEN_SPEED", str(car_list[now_id].val_fit[1]))
        #     print("write "+str(now_id)+" "+str(car_list[now_id].val_fit[1]))
        # robot_conf.write(open(ini_address, "w", encoding="utf-8"))
        for i in range(carcar.getid()): # 绘制图像
            now_id = list(set(carcar.id))[i]
            ax[i] = plt.subplot(carcar.getid()//3+1, 3, i+1)
            plot_send = np.arange(100, 2100, 1)
            plot_raw = car_list[now_id].val_fit(plot_send)
            plot = plt.scatter(car_list[now_id].send_vel, car_list[now_id].raw_vel, s=1, color='limegreen')
            plot = plt.scatter(list(set(car_list[now_id].send_vel)), car_list[now_id].avg_vel, s=30, color='orange', alpha=0.8)
            plot = plt.plot(plot_send, plot_raw, 'red')
            plt.xlabel(str(car_list[now_id].id) + '-send_vel' + str(car_list[now_id].val_fit))
            plt.ylabel(str(car_list[now_id].id) + '-raw_vel' + str(car_list[now_id].val_fit))
        plt.show()
# class MoveData:
static_car_num = 16
in_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/Speedconstant.txt"
ini_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/kickparam.ini"
CarMoveTest.read(in_address, ini_address)