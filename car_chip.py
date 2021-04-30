import matplotlib.pyplot as plt
class BallDataOne:
    def __init__(self):
        self.id = -1
        self.vel = []
        self.dist = []
        self.power = []
        self.avg_vel = []
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
    def mean_filter(self, per): # 均值滤波
        for i in range(0, len(self.vel)-per, per):
            sum = 0
            for j in range(per):
                sum += self.vel[i+j]
            for j in range(per):
                self.avg_vel.append(sum/per)
        max_vel = max(self.avg_vel) # 求最值
        max_index = self.avg_vel.index(max_vel)
        chip_point = -1 #　找落地点
        for i in range(max_index, len(self.avg_vel)-1):
            if self.avg_vel[i] < self.avg_vel[i+1]:
                chip_point = i
                break
        return chip_point
class BallDataAll:
    def __init__(self):
        self.id = []
        self.vel = []
        self.dist = []
        self.power = []
    def read_data(self, address): # 数据读取
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if line_data[1] != '\n' or line_data[1] != '0':
                    self.id.append(line_data[0].strip())
                    self.vel.append(line_data[1].strip())
                    self.dist.append(line_data[2].strip())
                    self.power.append(line_data[3].strip())
        self.id = list(map(int, self.id))
        self.vel = list(map(float, self.vel))
        self.dist = list(map(float, self.dist))
        self.power = list(map(float, self.power))
class CarData:
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
# class ChipData:
txt_address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/BallVel1.txt"
raw_id = []
raw_vel = []
raw_dist = []
avg_vel = []
with open(txt_address, "r") as f: # 读取数据
    raw_data = f.readlines()
    for line in raw_data:
        line_data = line.split()
        if line_data[1] != '\n' or line_data[1] != '0':
            raw_vel.append(line_data[1].strip())
raw_vel = list(map(float, raw_vel))
per = 5 # 均值滤波
for i in range(0, len(raw_vel)-per, per):
    sum = 0
    for j in range(per):
        sum += raw_vel[i+j]
    for j in range(per):
        avg_vel.append(sum/per)
max_vel = max(avg_vel) # 求最值
max_index = avg_vel.index(max_vel)
chip_point = -1 #　找落地点
for i in range(max_index, len(avg_vel)-1):
    if avg_vel[i] < avg_vel[i+1]:
        chip_point = i
        break
plt.plot(raw_vel, color='limegreen')
plt.scatter(chip_point, raw_vel[chip_point], s=500, color='deeppink', alpha=0.2)
plt.scatter(chip_point, raw_vel[chip_point], s=200, color='deeppink', alpha=0.4)
plt.scatter(chip_point, raw_vel[chip_point], s=50, color='mediumvioletred', alpha=1.0)
plt.plot(avg_vel, color='orangered')
plt.show()