import matplotlib.pyplot as plt
import numpy as np

class car_data: # 单个车数据
    def __init__(self):
        self.id = -1
        self.maxvel = []
        self.power = []
        self.val_fit = []
    def assign(self, id, maxvel, power):
        self.id = id
        self.maxvel.append(maxvel)
        self.power.append(power)
    def calculate(self):
        if self.id == -1:
            self.val_fit = 0
        else:
            raw_fit = np.polyfit(self.maxvel, self.power, 2)
            self.val_fit = np.poly1d(raw_fit)
    def draw(self):
        plot_maxvel = np.arange(0, 6000, 1)
        plot_power = self.val_fit(plot_maxvel)
        plot1 = plt.plot(self.maxvel, self.power, '*', label='original values')
        plot2 = plt.plot(plot_maxvel, plot_power, 'r', label='polyfit values')
        plt.xlabel('maxvel')
        plt.ylabel('power')
        plt.show()

class car_cal: # 全部车数据
    def __init__(self):
        self.raw_id = []
        self.raw_vel = []
        self.raw_maxvel = []
        self.raw_power = []
        self.all_id = []
        self.all_vel = []
        self.all_maxvel = []
        self.all_power = []
    def read_data(self, address):
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if line_data[0] != '\n':
                    self.raw_id.append(line_data[0].strip())
                if line_data[1] != '\n' and line_data[1] != 0.0:
                    self.raw_vel.append(line_data[1].strip())
                if line_data[2] != '\n':
                    self.raw_maxvel.append(line_data[2].strip())
                if line_data[3] != '\n':
                    self.raw_power.append(line_data[3].strip())
        self.raw_id = list(map(float, self.raw_id))
        self.raw_vel = list(map(float, self.raw_vel))
        self.raw_maxvel = list(map(float, self.raw_maxvel))
        self.raw_power = list(map(float, self.raw_power))
        old_id = -1
        for i in range(len(self.raw_id)):
            if old_id != self.raw_id[i]:
                old_id = self.raw_id[i]
                if self.raw_power[i] > 0.0 and self.raw_power[i] < 130.0 and self.raw_maxvel[i] > 0.0 and self.raw_maxvel[i] < 7000.0:
                    self.all_id.append(self.raw_id[i])
                    self.all_vel.append(self.raw_vel[i])
                    self.all_maxvel.append(self.raw_maxvel[i])
                    self.all_power.append(self.raw_power[i])

address = "/home/zjunlict-vision-1/Desktop/bullet_robot/VelData.txt"
all_car = car_cal() # 数据读取
all_car.read_data(address)
car_list = []

for i in range(16): # 初始化
    car_list.append(car_data())
for i in range(len(all_car.all_id)): # 车号划分
    car_list[int(all_car.all_id[i])].assign(int(all_car.all_id[i]), all_car.all_maxvel[i], all_car.all_power[i])
for i in range(16):
    car_list[i].calculate()
    print(car_list[i].id, '---', car_list[i].val_fit)
car_list[0].draw()