import matplotlib.pyplot as plt
class BallDataOne: # 单次踢球的球数据
    def __init__(self):
        self.id = -1
        self.vel = []
        self.dist = []
        self.power = []
        self.avg_vel = []
        self.chip_point = -1
    def assign(self, id, vel, dist, power): # 数据赋值
        self.id = id
        self.vel.append(vel)
        self.dist.append(dist)
        self.power.append(power)
    def clear(self): # 数据清除
        self.id = -1
        self.vel = []
        self.dist = []
        self.power = []
        self.avg_vel = []
        self.chip_point = -1
    def analy(self, per):
        for i in range(0, len(self.vel)-per, per): # 均值滤波
            sum = 0
            for j in range(per):
                sum += self.vel[i+j]
            for j in range(per):
                self.avg_vel.append(sum/per)
        max_vel = max(self.avg_vel) # 求最值
        max_index = self.avg_vel.index(max_vel)
        self.chip_point = -1
        for i in range(max_index, len(self.avg_vel)-1): #　找落地点
            if self.avg_vel[i] < self.avg_vel[i+1]:
                self.chip_point = i
                break
    def draw(self):
        plt.plot(self.vel, color='limegreen')
        plt.scatter(self.chip_point, self.vel[self.chip_point], s=500, color='deeppink', alpha=0.2)
        plt.scatter(self.chip_point, self.vel[self.chip_point], s=200, color='deeppink', alpha=0.4)
        plt.scatter(self.chip_point, self.vel[self.chip_point], s=50, color='mediumvioletred', alpha=1.0)
        plt.plot(self.avg_vel, color='orangered')
        plt.show()
class BallTest: # 单次踢球数据测试
    def read(address):
        ball = BallDataOne()
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if line_data[1] != '\n' or line_data[1] != '0':
                    ball.vel.append(line_data[1].strip())
        ball.vel = list(map(float, ball.vel))
        ball.analy(5)
        ball.draw()
        ball.clear()
class BallDataAll: # 多次踢球的球数据
    def __init__(self):
        self.id = []
        self.raw_vel = []
        self.raw_dist = []
        self.power = []
        self.real_dist = []
    def read(self, address): # 数据读取
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if line_data[1] != '\n' or line_data[1] != '0':
                    self.id.append(line_data[0].strip())
                    self.raw_vel.append(line_data[1].strip())
                    self.raw_dist.append(line_data[2].strip())
                    self.power.append(line_data[3].strip())
        self.id = list(map(int, self.id))
        self.raw_vel = list(map(float, self.raw_vel))
        self.raw_dist = list(map(float, self.raw_dist))
        self.power = list(map(float, self.power))
        old_id = self.id[0]
        ball = BallDataOne()
        for i in range(self.id):
            if old_id == self.id[i]: # id不变时计算落地点
                ball.assign(self.id[i], self.vel[i], self.dist[i], self.power[i])
            else: # id变化时记录距离和力度
                old_id= self.id[i]
                ball.analy(5)
                if ball.chip_point != -1:
                    self.real_dist.append(ball.dist[ball.chip_point])
                ball.clear()
class CarDataOne:
    def __init__(self):
        self.id = -1
        self.dist = []
        self.power = []
        self.val_fit = []
    def assign(self, id, dist, power): # 参数赋值
        self.id = id
        self.dist.append(dist)
        self.power.append(power)
    def calculate(self, degree): # 计算拟合函数
        if self.id == -1:
            self.val_fit = 0
        else:
            raw_fit = np.polyfit(self.dist, self.power, degree)
            self.val_fit = np.poly1d(raw_fit)
    def draw(self): # 绘制拟合曲线
        plot_dist = np.arange(0, 7500, 1)
        if self.id == -1:
            plot_power = np.zeros(len(plot_dist))
        else:
            plot_power = self.val_fit(plot_dist)
            plot1 = plt.plot(self.dist, self.power, '*')
        plot2 = plt.plot(plot_dist, plot_power, 'r')
        plt.xlabel('dist')
        plt.ylabel('power')
        plt.show()
# class CarDataAll:
# class ChipData:
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/BallVel7.txt"
BallTest.read(txt_address)