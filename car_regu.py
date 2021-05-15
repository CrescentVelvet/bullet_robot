from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import math
import FitModule
class CarRegulation:
    def __init__(self):
        self.id = []
        self.vel = []
        self.dir = []
        self.sum_bally = []
    def assign(self, iid, ivel, idir):
        self.id.append(iid)
        self.vel.append(ivel)
        self.dir.append(idir)
    def clear(self):
        self.id = []
        self.vel = []
        self.dir = []
        self.sum_balldir = []
    def old_summary(self): # 求balldir平均值
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
    def old_draw3D(self): # 绘制三维图像
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
    def old_draw2D(self): # 绘制二维图像
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
    def draw2D(self, flag): # 绘制二维图像
        X_exact = self.vel
        Y_exact = self.dir
        raw_fit = np.polyfit(X_exact, Y_exact, flag)
        val_fit = np.poly1d(raw_fit)
        print('拟合结果为：', val_fit)
        plot_X = np.arange(min(X_exact), max(X_exact), 0.01)
        plot_y = val_fit(plot_x)
        plt.plot(X_exact, Y_exact, 'o', color='green')
        plt.plot(plot_x, plot_y, color='blue')
        # for i_x, i_y in zip(X_exact, Y_exact):
        #     plt.text(i_x, i_y, '({:.6f}, {:.6f})'.format(i_x, i_y))
        plt.xlabel('rovel/power')
        plt.ylabel('balldir-cardir')
        plt.show()
    def ransac(self, flag): # RANSAC随机采样一致算法
        X_exact = self.vel
        Y_exact = self.dir
        if flag == 1:
            ransac_fit = FitModule.Poly1d(X_exact, Y_exact, 0.01, 10, True, True)
        elif flag == 2:
            ransac_fit = FitModule.Poly2d(X_exact, Y_exact, 0.005, 10, True)
        elif flag == 3:
            ransac_fit = FitModule.Poly3d(X_exact, Y_exact, 0.005, 10, True)
            print(("double fit_ball2car = {}*act_rot2vell*act_rot2vell*act_rot2vell+{}*act_rot2vell*act_rot2vell+{}*act_rot2vell+{};").format(ransac_fit.T[0][3],ransac_fit.T[0][2],ransac_fit.T[0][1],ransac_fit.T[0][0]))
        else:
            print("error in ransac flag")
        plot_X = np.arange(min(X_exact), max(X_exact), 0.01)
        plot_fit = np.poly1d(ransac_fit.reshape(1,-1)[0][::-1])
        plot_Y = plot_fit(plot_X)
        plt.plot(plot_X, plot_Y, color='red', alpha=1.0, linewidth=3, label='RANSAC fit')
        line_x = np.arange(-0.2, 0.2, 0.01)
        tan_y = np.zeros(len(line_x))
        lin_y = np.zeros(len(line_x))
        for i in range(len(line_x)):
            tan_y[i] = -math.tanh(line_x[i]*20)/6
            lin_y[i] = 1 * line_x[i]
        plt.plot(line_x, tan_y, color='blue', alpha=1.0, linewidth=3, label='tan fit')
        plt.plot(line_x, lin_y, color='blue', alpha=1.0, linewidth=3, label='lin fit')
        plt.legend() # 添加图例
        plt.xlabel('rovel/power')
        plt.ylabel('balldir-cardir')
        plt.show()
class RegulationTest:
    def read(address):
        car_data = CarRegulation()
        with open(address, "r") as f:
            raw_data = f.readlines()
            for line in raw_data:
                line_data = line.split()
                if len(line_data) < 3: # 位数可以大,不可以小
                    continue
                t_id = float(line_data[0].strip())
                t_vel = (float(line_data[1].strip()))
                t_dir = (float(line_data[2].strip()))
                car_data.assign(t_id, t_vel, t_dir)
        # car_data.draw2D(3)
        car_data.ransac(3)
R_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataRotate_reset.txt"
S_address = "/home/zjunlict-vision-1/Desktop/czk/Kun2/ZBin/data/ReguDataSlide.txt"
RegulationTest.read(R_address)