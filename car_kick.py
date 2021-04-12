import matplotlib.pyplot as plt
import numpy as np
import configparser
class car_data: # 单个车数据
    def __init__(self):
        self.id = -1
        self.maxvel = []
        self.power = []
        self.val_fit = []
    def assign(self, id, maxvel, power): # 参数赋值
        self.id = id
        self.maxvel.append(maxvel)
        self.power.append(power)
    def calculate(self): # 计算拟合函数
        if self.id == -1:
            self.val_fit = 0
        else:
            raw_fit = np.polyfit(self.maxvel, self.power, 2)
            self.val_fit = np.poly1d(raw_fit)
    def draw(self): # 绘制拟合曲线
        if self.id == -1:
            plot_maxvel = np.arange(0, 7500, 1)
            plot_power = np.zeros(len(plot_maxvel))
            plot2 = plt.plot(plot_maxvel, plot_power, 'r', label='polyfit values')
        else:
            plot_maxvel = np.arange(0, 7500, 1)
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
    def read_data(self, address): # txt数据读取
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if line_data[1] == '0' or line_data[2] == '-1000': # 排除异常数据
                    continue
                if line_data[0] != '\n':
                    self.raw_id.append(line_data[0].strip())
                if line_data[1] != '\n':
                    self.raw_vel.append(line_data[1].strip())
                if line_data[2] != '\n':
                    self.raw_maxvel.append(line_data[2].strip())
                if line_data[3] != '\n':
                    self.raw_power.append(line_data[3].strip())
        self.raw_id = list(map(float, self.raw_id))
        self.raw_vel = list(map(float, self.raw_vel))
        self.raw_maxvel = list(map(float, self.raw_maxvel))
        self.raw_power = list(map(float, self.raw_power))
        old_id = self.raw_id[0]
        for i in range(len(self.raw_id)):
            if old_id != self.raw_id[i]: # id变化时记录数据
                old_id = self.raw_id[i]
                if self.raw_power[i-1] > 0.0 and self.raw_power[i-1] < 130.0 and self.raw_maxvel[i-1] > 0.0 and self.raw_maxvel[i-1] < 8000.0:
                    self.all_id.append(self.raw_id[i-1]) # 排除越界数据
                    self.all_vel.append(self.raw_vel[i-1])
                    self.all_maxvel.append(self.raw_maxvel[i-1])
                    self.all_power.append(self.raw_power[i-1])
static_car_num = 16
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData2_6_10.txt"
ini_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/kickparam.ini"
car_all = car_cal() # txt是小车踢球原始数据
car_all.read_data(txt_address) # txt数据读取
car_list = []
for i in range(static_car_num): # 初始化
    car_list.append(car_data())
for i in range(len(car_all.all_id)): # 车号划分
    car_list[int(car_all.all_id[i])].assign(int(car_all.all_id[i]), car_all.all_maxvel[i], car_all.all_power[i])
for i in range(static_car_num): # 计算拟合函数
    car_list[i].calculate()
car_list[6].draw()
robot_conf = configparser.ConfigParser() # ini是小车踢球拟合参数
robot_conf.read(ini_address, encoding="utf-8") # ini数据读取
for robot_id in range(static_car_num):
    robot_conf.set("Robot"+str(robot_id), "chip_min", str(30))
    robot_conf.set("Robot"+str(robot_id), "chip_max", str(120))
    robot_conf.set("Robot"+str(robot_id), "flat_min", str(30))
    robot_conf.set("Robot"+str(robot_id), "flat_max", str(120))
    if car_list[robot_id].id != -1: # ini参数更新
        robot_conf.set("Robot"+str(robot_id), "flat_a", str(car_list[robot_id].val_fit[2]))
        robot_conf.set("Robot"+str(robot_id), "flat_b", str(car_list[robot_id].val_fit[1]))
        robot_conf.set("Robot"+str(robot_id), "flat_c", str(car_list[robot_id].val_fit[0]))
robot_conf.write(open(ini_address, "w", encoding="utf-8")) # ini参数写入