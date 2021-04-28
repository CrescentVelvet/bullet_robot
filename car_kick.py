import matplotlib.pyplot as plt
import numpy as np
import configparser
import select
import time
from watchdog.observers import Observer
from watchdog.events import LoggingEventHandler
from watchdog.events import FileSystemEventHandler
class Flag:
    old_time = -1
class File_monitor(FileSystemEventHandler):
    def __init__(self, **kwargs):
        super(File_monitor, self).__init__(**kwargs)
        self._watch_path = './'
    def on_modified(self, event):
        if not event.is_directory:
            print("修改了文件", event.src_path)
    def on_created(self, event):
        print("创建了文件夹", event.src_path)
    def on_moved(self, event):
        print("移动了文件", event.src_path)
    def on_deleted(self, event):
        print("删除了文件", event.src_path)
    def on_any_event(self, event):
        print("------")
class Car_data_one: # 单个车数据类
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
class Feed_back: # 反馈数据类
    def __init__(self):
        self.raw_id = []
        self.raw_vel_hope = []
        self.raw_vel_real = []
        self.all_id = []
        self.all_vel_hope = []
        self.all_vel_real = []
    def read_data(self, address):
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 6:
                    continue
                if line_data[1] == '1' or line_data[2] == '0' or line_data[3] == '0': # 排除异常数据
                    continue
                if line_data[0] != '\n':
                    self.raw_id.append(line_data[0].strip())
                if line_data[2] != '\n':
                    self.raw_vel_hope.append(line_data[2].strip())
                if line_data[3] != '\n':
                    self.raw_vel_real.append(line_data[3].strip())
        self.raw_id = list(map(float, self.raw_id))
        self.raw_vel_hope = list(map(float, self.raw_vel_hope))
        self.raw_vel_real = list(map(float, self.raw_vel_real))
        old_id = self.raw_id[0]
        for i in range(len(self.raw_id)):
            if old_id != self.raw_id[i]: # id变化时记录数据
                old_id = self.raw_id[i]
                if self.raw_vel_hope[i-1] > 1000.0 and self.raw_vel_hope[i-1] < 7000.0 and self.raw_vel_real[i-1] > 1000.0 and self.raw_vel_real[i-1] < 7000.0:
                    self.all_id.append(self.raw_id[i-1]) # 排除越界数据
                    self.all_vel_hope.append(self.raw_vel_hope[i-1])
                    self.all_vel_real.append(self.raw_vel_real[i-1])
    def analy_data(in_car_feb):
        out_id = [-1, 0]
        out_id[0] = in_car_feb.all_id[-1]
        out_sum = in_car_feb.all_vel_hope[-1] - in_car_feb.all_vel_real[-1]
        if out_sum > 100.0:
            out_id[1] = 1
        elif out_sum < -100.0:
            out_id[1] = -1
        else:
            out_id[1] = 0
        return out_id
    def Watch_dog(address):
        observer = Observer() # 检测改动
        event_handler = File_monitor() # 事件处理
        observer.schedule(event_handler, path=address, recursive=True)
        observer.start()
        try:
            while True:
                time.sleep(100)
        except KeyboardInterrupt:
            observer.stop()
        observer.join()
class Car_data_all: # 全部车数据
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
                if len(line_data) != 4: # 排除位数错误数据
                    continue
                if line_data[1] == '0' or line_data[2] == '-1' or line_data[2] == '-1000': # 排除初始化数据
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
        print(self.all_maxvel)
class Analy_car: # 操作数据
    def analy_txt(address, flag): # 拟合计算
        car_all = Car_data_all() # txt是小车踢球原始数据
        car_all.read_data(address) # txt数据读取
        out_car_list = []
        for i in range(static_car_num): # 初始化
            out_car_list.append(Car_data_one())
        for i in range(len(car_all.all_id)): # 车号划分
            out_car_list[int(car_all.all_id[i])].assign(int(car_all.all_id[i]), car_all.all_maxvel[i], car_all.all_power[i])
        if flag == 1: # 平射用一次拟合
            for i in range(static_car_num): # 计算拟合函数
                out_car_list[i].calculate_1()
                print(i, '---', out_car_list[i].val_fit)
        elif flag == 0: # 挑射用二次拟合
            for i in range(static_car_num): # 计算拟合函数
                out_car_list[i].calculate_2()
                print(i, '---', out_car_list[i].val_fit)
        else:
            print("error in is_FlatChip")
        return out_car_list
    def read_ini(address): # ini数据读取
        robot_conf = configparser.ConfigParser() # ini是小车踢球拟合参数
        robot_conf.read(address, encoding="utf-8") # ini数据读取
        return robot_conf
    def write_ini(address, in_car_txt): # 参数写入
        robot_conf = configparser.ConfigParser()
        robot_conf.read(address, encoding="utf-8")
        for temp_id in range(static_car_num):
            robot_conf.set("Robot"+str(temp_id), "CHIP_MIN", str(0))
            robot_conf.set("Robot"+str(temp_id), "CHIP_MAX", str(130))
            robot_conf.set("Robot"+str(temp_id), "FLAT_MIN", str(0))
            robot_conf.set("Robot"+str(temp_id), "FLAT_MAX", str(130))
            if in_car_txt[temp_id].id != -1: # ini参数更新
                robot_conf.set("Robot"+str(temp_id), "FLAT_A", str(0))
                robot_conf.set("Robot"+str(temp_id), "FLAT_B", str(in_car_txt[temp_id].val_fit[1]))
                robot_conf.set("Robot"+str(temp_id), "FLAT_C", str(in_car_txt[temp_id].val_fit[0]))
        robot_conf.write(open(address, "w", encoding="utf-8")) # ini参数写入
    def write_ini_one(address, in_car_txt, in_id): # 参数写入
        robot_conf = configparser.ConfigParser()
        robot_conf.read(address, encoding="utf-8")
        robot_conf.set("Robot"+str(in_id), "FLAT_A", str(0))
        robot_conf.set("Robot"+str(in_id), "FLAT_B", str(in_car_txt[in_id].val_fit[1]))
        robot_conf.set("Robot"+str(in_id), "FLAT_C", str(in_car_txt[in_id].val_fit[0]))
        robot_conf.write(open(address, "w", encoding="utf-8"))
    def read_feb(address): # feb数据读取
        out_car_feb = Feed_back()
        out_car_feb.read_data(address)
        return out_car_feb
    def write_ini_feb(address, in_car_ini, in_id):
        robot_conf = configparser.ConfigParser()
        robot_conf.read(address, encoding="utf-8")
        mode_str = "FLAT_"
        a = 0
        b = in_car_ini["Robot"+str(in_id[0])][mode_str+"B"] + in_id[0] * 10
        c = in_car_ini["Robot"+str(in_id[0])][mode_str+"C"] + in_id[0] * 10
        robot_conf.set("Robot"+str(in_id[0]), "FLAT_A", str(a))
        robot_conf.set("Robot"+str(in_id[0]), "FLAT_B", str(b))
        robot_conf.set("Robot"+str(in_id[0]), "FLAT_C", str(c))
class Draw_car: # 绘制图像
    def draw_txt(in_car_txt): # 绘制全部txt图
        sum = 0 # 有效小车数
        for i in range(len(in_car_txt)):
            if in_car_txt[i].val_fit == 0:
                continue
            sum += 1
        ax = [None] * sum
        ax_num = 0
        for i in range(len(in_car_txt)):
            if in_car_txt[i].val_fit == 0:
                continue
            # print(sum // 3 + 1, sum % 3, ax_num, i)
            ax[ax_num] = plt.subplot(sum // 3 + 1, 3, ax_num + 1)
            plot_maxvel = np.arange(0, 7500, 1)
            plot_power = in_car_txt[i].val_fit(plot_maxvel)
            plt.plot(in_car_txt[i].maxvel, in_car_txt[i].power, '*')
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
    def draw_txt_ini(in_car_txt, in_car_ini, mode):
        mode_str = ("FLAT_") if mode else ("CHIP_")
        ax = [None] * static_car_num
        for i in range(static_car_num):
            b = in_car_ini["Robot"+str(i)][mode_str+"B"]
            c = in_car_ini["Robot"+str(i)][mode_str+"C"]
            val_ini = np.poly1d([float(b), float(c)])
            plot_maxvel_ini = np.arange(0, 7500, 1)
            plot_power_ini = val_ini(plot_maxvel_ini)
            ax[i] = plt.subplot(4, 4, i+1)
            plt.plot(plot_maxvel_ini, plot_power_ini, 'r')
            plt.xlabel('maxvel-'+str(i))
            plt.ylabel('power-'+str(i))
        for i in range(len(in_car_txt)):
            if in_car_txt[i].val_fit == 0:
                continue
            plot_maxvel_txt = np.arange(0, 7500, 1)
            plot_power_txt = in_car_txt[i].val_fit(plot_maxvel_txt)
            ax[i] = plt.subplot(4, 4, i+1)
            plt.plot(in_car_txt[i].maxvel, in_car_txt[i].power, '*')
            plt.plot(plot_maxvel_txt, plot_power_txt, 'g')
        plt.show()


static_car_num = 16
is_FlatChip = 1
if is_FlatChip:
    txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData_all.txt"
else:
    txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/ChipData.txt"
car_txt = Analy_car.analy_txt(txt_address, is_FlatChip) # 读取txt
# car_txt[0].draw_txt_one()                             # 绘制一张txt
Draw_car.draw_txt(car_txt)                            # 绘制全部txt
# ini_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/kickparam.ini"
# car_ini = Analy_car.read_ini(ini_address)               # 读取ini
# Draw_car.draw_ini(car_ini, 1)                         # 绘制全部ini
# Draw_car.draw_txt_ini(car_txt, car_ini, 1)              # 绘制全部txt和ini
# Analy_car.write_ini_one(ini_address, car_txt, 9)      # 写入一车ini
# Analy_car.write_ini(ini_address, car_txt)             # 写入全部ini
# feb_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/feedbackData1.txt"
# car_feb = Analy_car.read_feb(feb_address)               # 读取feb
# car_feb_id = Feed_back.analy_data(car_feb)              # 拿到偏差
# Analy_car.write_ini_feb(ini_address, car_ini, car_feb_id) # 修改ini
# Feed_back.Watch_dog(feb_address)                        # 检测feb改动
# while True:
#     new_time = int(time.time() % 1000)
#     if Flag.old_time == -1:
#         Flag.old_time = new_time
#     if new_time - Flag.old_time > 5:
#         Flag.old_time = new_time
#         print('Reading feedbackData ', new_time)
#         car_feb = Feed_back.read_data(feb_address)