import matplotlib.pyplot as plt
import numpy as np
import configparser
class Car_data: # 单个车数据
    def __init__(self):
        self.id = -1
        self.maxvel = []
        self.power = []
        self.val_fit = []
    def assign(self, id, maxvel, power): # 参数赋值
        self.id = id
        self.maxvel.append(maxvel)
        self.power.append(power)
    def calculate_1(self): # 计算一次拟合函数
        if self.id == -1:
            self.val_fit = 0
        else:
            raw_fit = np.polyfit(self.maxvel, self.power, 1)
            self.val_fit = np.poly1d(raw_fit)
    def calculate_2(self): # 计算二次拟合函数
        if self.id == -1:
            self.val_fit = 0
        else:
            raw_fit = np.polyfit(self.maxvel, self.power, 2)
            self.val_fit = np.poly1d(raw_fit)
    def draw_txt_one(self): # 绘制拟合曲线
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
class Car_cal: # 全部车数据
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
class Analy_car: # 操作数据
    def analy_txt(address): # 拟合计算
        car_all = Car_cal() # txt是小车踢球原始数据
        car_all.read_data(address) # txt数据读取
        out_car_list = []
        for i in range(static_car_num): # 初始化
            out_car_list.append(Car_data())
        for i in range(len(car_all.all_id)): # 车号划分
            out_car_list[int(car_all.all_id[i])].assign(int(car_all.all_id[i]), car_all.all_maxvel[i], car_all.all_power[i])
        for i in range(static_car_num): # 计算拟合函数
            out_car_list[i].calculate_1()
            print(i, '---', out_car_list[i].val_fit)
        return out_car_list
    def read_ini(address): # 读取配置信息
        robot_conf = configparser.ConfigParser() # ini是小车踢球拟合参数
        robot_conf.read(address, encoding="utf-8") # ini数据读取
        return robot_conf
    def write_ini(address, in_car_list): # 参数写入
        robot_conf = configparser.ConfigParser()
        robot_conf.read(address, encoding="utf-8")
        for temp_id in range(static_car_num):
            robot_conf.set("Robot"+str(temp_id), "CHIP_MIN", str(20))
            robot_conf.set("Robot"+str(temp_id), "CHIP_MAX", str(120))
            robot_conf.set("Robot"+str(temp_id), "FLAT_MIN", str(20))
            robot_conf.set("Robot"+str(temp_id), "FLAT_MAX", str(120))
            if in_car_list[temp_id].id != -1: # ini参数更新
                robot_conf.set("Robot"+str(temp_id), "FLAT_A", str(in_car_list[temp_id].val_fit[2]))
                robot_conf.set("Robot"+str(temp_id), "FLAT_B", str(in_car_list[temp_id].val_fit[1]))
                robot_conf.set("Robot"+str(temp_id), "FLAT_C", str(in_car_list[temp_id].val_fit[0]))
        robot_conf.write(open(address, "w", encoding="utf-8")) # ini参数写入
    def write_ini_one(address, in_car_list, in_id): # 参数写入
        robot_conf = configparser.ConfigParser()
        robot_conf.read(address, encoding="utf-8")
        robot_conf.set("Robot"+str(in_id), "FLAT_A", str(0))
        robot_conf.set("Robot"+str(in_id), "FLAT_B", str(in_car_list[in_id].val_fit[1]))
        robot_conf.set("Robot"+str(in_id), "FLAT_C", str(in_car_list[in_id].val_fit[0]))
        robot_conf.write(open(address, "w", encoding="utf-8"))
    def draw_txt(in_car_list): # 绘制全部txt图
        sum = 0 # 有效小车数
        for i in range(len(in_car_list)):
            if in_car_list[i].val_fit == 0:
                continue
            sum += 1
        ax = [None] * sum
        ax_num = 1
        for i in range(len(in_car_list)):
            if in_car_list[i].val_fit == 0:
                continue
            print(sum // 3 + 1, sum % 3, ax_num, i)
            ax[ax_num-1] = plt.subplot(sum // 3 + 1, 3, ax_num)
            plot_maxvel = np.arange(0, 7500, 1)
            plot_power = in_car_list[i].val_fit(plot_maxvel)
            plt.plot(in_car_list[i].maxvel, in_car_list[i].power, '*')
            plt.plot(plot_maxvel, plot_power, 'r')
            plt.xlabel('maxvel-'+str(i))
            plt.ylabel('power-'+str(i))
            ax_num += 1
        plt.show()
    def draw_ini(in_car_ini, mode): # 绘制全部ini图
        mode_str = ("FLAT_") if mode else ("CHIP_")
        ax = [None] * static_car_num
        for i in range(static_car_num):
            b = in_car_ini["Robot"+str(i)][mode_str+"B"]
            c = in_car_ini["Robot"+str(i)][mode_str+"C"]
            val_ini = np.poly1d([float(b), float(c)])
            ax[i] = plt.subplot(4, 4, i+1)
            plot_maxvel = np.arange(0, 7500, 1)
            plot_power = val_ini(plot_maxvel)
            plt.plot(plot_maxvel, plot_power, 'r')
            plt.xlabel('maxvel-'+str(i))
            plt.ylabel('power-'+str(i))
        plt.show()
    def draw_txt_ini():
        pass

static_car_num = 16

txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData_6_8_14_15.txt"
car_list = Analy_car.analy_txt(txt_address)
# Analy_car.draw_txt(car_list)
# car_list[0].draw_txt_one()

ini_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/kickparam.ini"
car_ini = Analy_car.read_ini(ini_address)
Analy_car.draw_ini(car_ini, 1)
# print(car_ini["Robot"+str(0)]["FLAT_"+"B"])
# Analy_car.write_ini(ini_address, car_list)
# Analy_car.write_ini_one(ini_address, car_list, 15)