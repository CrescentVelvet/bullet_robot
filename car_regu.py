from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
import FitModule
class CarRegulation:
    def __init__(self):
        self.rovel = []
        self.power = []
        self.cardir = []
        self.balldir = []
        self.sum_bally = []
    def assign(self, rovel, power, ballx, bally, cardir):
        self.rovel.append(rovel)
        self.power.append(power)
        self.cardir.append(cardir)
        self.balldir.append(bally/ballx)
    def clear(self):
        self.rovel = []
        self.power = []
        self.cardir = []
        self.balldir = []
        self.sum_balldir = []
    def summary(self): # 求balldir平均值
        # print("rovel", len(set(self.rovel)), "power", len(set(self.power)))
        num_balldir = [[0 for cols in range(len(set(self.rovel)))] for rows in range(len(set(self.power)))] # row行,col列
        self.sum_balldir = [[0 for cols in range(len(set(self.rovel)))] for rows in range(len(set(self.power)))]
        for i in range(len(self.balldir)):
            num_power = sorted(list(set(self.power)),reverse=False).index(self.power[i]) # 求行列号
            num_rovel = sorted(list(set(self.rovel)),reverse=False).index(self.rovel[i])
            num_balldir[num_power][num_rovel] += 1 # 求个数
            self.sum_balldir[num_power][num_rovel] += self.balldir[i] # 求和
        for i in range(len(set(self.power))):
            for j in range(len(set(self.rovel))):
                if num_balldir[i][j] == 0:
                    self.sum_balldir[i][j] = 0
                else:
                    self.sum_balldir[i][j] = self.sum_balldir[i][j] / num_balldir[i][j] # 求平均
    def draw3D(self): # 绘制三维图像
        ax = Axes3D(plt.figure())
        ax.scatter(self.power, self.rovel, self.balldir, c='g') # 绘制散点
        for i in range(len(set(self.power))):
            for j in range(len(set(self.rovel))):
                ax.scatter(sorted(list(set(self.power)),reverse=False)[i], sorted(list(set(self.rovel)),reverse=False)[j], self.sum_balldir[i][j], c='r')
        plot_power, plot_rovel = np.meshgrid(sorted(list(set(self.power)),reverse=True), sorted(list(set(self.rovel)),reverse=True)) # 数据网格化
        plot_balldir = np.array(self.sum_balldir)
        ax.plot_surface(plot_power, plot_rovel, plot_balldir, cmap='Greens') # 绘制曲面OrRd
        matrix_A = np.zeros((3,3)) # 创建系数矩阵A
        for i in range(len(set(self.power))):
            for j in range(len(set(self.rovel))):
                matrix_A[0,0] = matrix_A[0,0] + plot_power[i][j]**2
                matrix_A[0,1] = matrix_A[0,1] + plot_power[i][j] * plot_rovel[i][j]
                matrix_A[0,2] = matrix_A[0,2] + plot_power[i][j]
                matrix_A[1,0] = matrix_A[0,1]
                matrix_A[1,1] = matrix_A[1,1] + plot_rovel[i][j]**2
                matrix_A[1,2] = matrix_A[1,2] + plot_rovel[i][j]
                matrix_A[2,0] = matrix_A[0,2]
                matrix_A[2,1] = matrix_A[1,2]
                matrix_A[2,2] = 100
        matrix_b = np.zeros((3,1)) # 创建系数矩阵b
        for i in range(len(set(self.power))):
            for j in range(len(set(self.rovel))):
                matrix_b[0,0] = matrix_b[0,0] + plot_power[i][j] * plot_balldir[i][j]
                matrix_b[1,0] = matrix_b[1,0] + plot_rovel[i][j] * plot_balldir[i][j]
                matrix_b[2,0] = matrix_b[2,0] + plot_balldir[i][j]
        inv_A = np.linalg.inv(matrix_A)
        inv_X = np.dot(inv_A, matrix_b)
        print('平面拟合结果为：z = %.3f * x + %.3f * y + %.3f'%(inv_X[0,0], inv_X[1,0], inv_X[2,0]))
        t_power = np.linspace(sorted(list(set(self.power)),reverse=False)[0], sorted(list(set(self.power)),reverse=False)[-1], 100) # 构建等差数列
        t_rovel = np.linspace(sorted(list(set(self.rovel)),reverse=False)[0], sorted(list(set(self.rovel)),reverse=False)[-1], 100)
        t_power, t_rovel = np.meshgrid(t_power, t_rovel)
        t_balldir = inv_X[0, 0] * t_power + inv_X[1, 0] * t_rovel + inv_X[2, 0]
        ax.plot_wireframe(t_power, t_rovel, t_balldir, rstride=5, cstride=5)
        ax.set_xlabel('power', fontdict={'size': 15, 'color': 'black'})
        ax.set_ylabel('rovel', fontdict={'size': 15, 'color': 'black'})
        ax.set_zlabel('balldir', fontdict={'size': 15, 'color': 'black'})
        plt.show()
    def draw2D(self): # 绘制二维图像
        t_x = []
        t_y = []
        for i in range(len(self.rovel)):
            t_x.append( math.atan(self.rovel[i] / self.power[i])) # rovel和power相除
            t_y.append( math.atan(self.balldir[i]) - self.cardir[i]) # balldir和cardir相减
        raw_fit = np.polyfit(t_x, t_y, 1)
        val_fit = np.poly1d(raw_fit)
        print('直线拟合结果为：', val_fit)
        plot_x = np.arange(sorted(t_x)[0], sorted(t_x)[-1], 0.1)
        plot_y = val_fit(plot_x)
        plt.scatter(t_x, t_y, color='green')
        plt.plot(plot_x, plot_y, color='red')
        plt.xlabel('rovel/power')
        plt.ylabel('balldir-cardir')
        plt.show()
    def ransacTest(self): # RANSAC随机采样一致算法
        A_exact = []
        B_exact = []
        for i in range(len(self.rovel)):
            A_exact.append( math.atan(self.rovel[i] / self.power[i])) # rovel和power相除
            B_exact.append( math.atan(self.balldir[i]) - self.cardir[i]) # balldir和cardir相减
        # FitModule.Poly1d(A_exact, B_exact, 0.01, 10, True, True)
        FitModule.Poly2d(A_exact, B_exact, 0.01, 10, True)
class RotateTest:
    def read(address):
        car_data = CarRegulation()
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 5:
                    continue
                t_rovel = float(line_data[0].strip()) * 98
                t_power = float(line_data[1].strip())
                t_ballx = float(line_data[2].strip())
                t_bally = float(line_data[3].strip())
                t_dir = float(line_data[4].strip())
                car_data.assign(t_rovel, t_power, t_ballx, t_bally, t_dir)
        # car_data.summary()
        # car_data.draw3D()
        # car_data.draw2D()
        car_data.ransacTest()
class SlideTest:
    def read(address):
        car_data = CarRegulation()
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) != 5:
                    continue
                t_vel = float(line_data[0].strip())
                t_power = float(line_data[1].strip())
                t_ballx = float(line_data[2].strip())
                t_bally = float(line_data[3].strip())
                t_dir = float(line_data[4].strip())
                car_data.assign(t_vel, t_power, t_ballx, t_bally, t_dir)
        # car_data.summary()
        # car_data.draw3D()
        car_data.draw2D()
R_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataRotate.txt"
S_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataSlide.txt"
RotateTest.read(R_address)
# SlideTest.read(S_address)