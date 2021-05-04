import matplotlib.pyplot as plt
import math
class CarMoveOne:
    def __init__(self):
        self.id = -1
        self.send_vel = []
        self.raw_vel = []
    def assign(self, id, send_vel, raw_vel):
        self.id = id
        self.send_vel.append(send_vel)
        self.raw_vel.append(raw_vel)
    def clear(self):
        self.id = -1
        self.send_vel = []
        self.raw_vel = []
    def draw(self):
        plot = plt.scatter(self.send_vel, self.raw_vel, s=0.1)
        plt.xlabel('send_vel')
        plt.ylabel('raw_vel')
        plt.show()
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
        ax = [None] * carcar.getid()
        for i in range(carcar.getid()): # 绘制图像
            ax[i] = plt.subplot(3, 3, i+1)
            plot = plt.scatter(car_list[i].send_vel, car_list[i].raw_vel, s=0.1)
            plt.xlabel('send_vel')
            plt.ylabel('raw_vel')
        plt.show()
        # for i in range(static_car_num):
            # print(i, car_list[i].id)
# class MoveData:
static_car_num = 16
in_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/Speedconstant"
CarMoveTest.read(in_address)