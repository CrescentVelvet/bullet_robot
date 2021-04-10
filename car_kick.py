import matplotlib.pyplot as plt
import numpy as np

class car_data:
    def __init__(self):
        self.id = -1
        self.maxvel = []
        self.power = []
    def __init__(self, id, maxvel, power):
        self.id = id
        self.maxvel = maxvel
        self.power = power

class car_cal:
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
        old_id = -1
        for i in range(len(self.raw_id)):
            if old_id != self.raw_id[i]:
                old_id = self.raw_id[i]
                if self.raw_power[i] != -1.0:
                    self.all_id.append(self.raw_id[i])
                    self.all_vel.append(self.raw_vel[i])
                    self.all_maxvel.append(self.raw_maxvel[i])
                    self.all_power.append(self.raw_power[i])


address = "/home/zjunlict-vision-1/Desktop/dhz/Kun2/ZBin/data/VelData.txt"
car = car_cal()
car.read_data(address)
print(car.all_id)
# raw_fit = np.polyfit(all_power, all_vel, 2)
# val_fit = np.poly1d(raw_fit)
# print(val_fit)
# plot_power = np.arange(0, 120, 0.1)
# plot_fit = val_fit(plot_power)
# plot1 = plt.plot(all_power, all_vel, '*', label='original values')
# plot2 = plt.plot(plot_power, plot_fit, 'r', label='polyfit values')
# plt.show()
