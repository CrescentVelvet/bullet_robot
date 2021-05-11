from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.linalg
import math
class LinearLeastSquareModel: # 最小二乘求线性解,用于RANSAC的输入模型      
    def __init__(self, input_columns, output_columns, debug = False):
        self.input_columns = input_columns
        self.output_columns = output_columns
        self.debug = debug
    def fit(self, data):
        A = np.vstack( [data[:,i] for i in self.input_columns] ).T # 第一列Xi-->行Xi
        B = np.vstack( [data[:,i] for i in self.output_columns] ).T # 第二列Yi-->行Yi
        x, resids, rank, s = scipy.linalg.lstsq(A, B) # residues:残差和
        return x # 返回最小平方和向量   
    def get_error(self, data, model):
        A = np.vstack( [data[:,i] for i in self.input_columns] ).T # 第一列Xi-->行Xi
        B = np.vstack( [data[:,i] for i in self.output_columns] ).T # 第二列Yi-->行Yi
        B_fit = scipy.dot(A, model) # 计算的y值,B_fit = model.k*A + model.b
        err_per_point = np.sum( (B - B_fit) ** 2, axis = 1 ) # sum squared error per row
        return err_per_point
    def random_partition(n,n_data):
        """return n random rows of data (and also the other len(data)-n rows)"""
        all_idxs = np.arange( n_data )
        np.random.shuffle(all_idxs)
        idxs1 = all_idxs[:n]
        idxs2 = all_idxs[n:]
        return idxs1, idxs2
    def ransac(data, model, n, k, t, d, debug=False, return_all=False):
        # 输入:all_data, model, 20, 1000, 7e3, 40, debug=False, return_all=True
        #     data  - 样本点
        #     model - 假设模型:事先自己确定
        #     n     - 生成模型所需的最少样本点
        #     k     - 最大迭代次数
        #     t     - 阈值:作为判断点满足模型的条件
        #     d     - 拟合较好时,需要的样本点最少的个数,当做阈值看待
        # 输出:
        #     bestfit - 最优拟合解(如果未找到返回nil)
        iterations = 0
        bestfit = None
        besterr = np.inf # 设置默认值
        best_inlier_idxs = None
        while iterations < k:
            maybe_idxs, test_idxs = LinearLeastSquareModel.random_partition(n,data.shape[0])
            maybeinliers = data[maybe_idxs,:]
            test_points = data[test_idxs]
            maybemodel = model.fit(maybeinliers)
            test_err = model.get_error( test_points, maybemodel)
            also_idxs = test_idxs[test_err < t] # select indices of rows with accepted points
            alsoinliers = data[also_idxs,:]
            if len(alsoinliers) > d:
                betterdata = np.concatenate( (maybeinliers, alsoinliers) )
                bettermodel = model.fit(betterdata)
                better_errs = model.get_error( betterdata, bettermodel)
                thiserr = np.mean( better_errs )
                if thiserr < besterr:
                    bestfit = bettermodel
                    besterr = thiserr
                    best_inlier_idxs = np.concatenate( (maybe_idxs, also_idxs) )
            iterations+=1
        if bestfit is None:
            raise ValueError("did not meet fit acceptance criteria")
        if return_all:
            return bestfit, {'inliers':best_inlier_idxs}
        else:
            return bestfit
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
        n_samples = len(self.rovel) # 样本个数
        n_inputs = 1 # 输入变量个数
        n_outputs = 1 # 输出变量个数  
        A_exact = []
        B_exact = []
        for i in range(len(self.rovel)):
            A_exact.append( math.atan(self.rovel[i] / self.power[i])) # rovel和power相除
            B_exact.append( math.atan(self.balldir[i]) - self.cardir[i]) # balldir和cardir相减
        A_exact = np.array([A_exact]).T
        B_exact = np.array([B_exact]).T
        all_data = np.hstack((A_exact,B_exact)) # 在水平方向上平铺拼接数组
        input_columns = range(n_inputs) # 数组的第一列x:0
        output_columns = [n_inputs + i for i in range(n_outputs)] # 数组最后一列y:1
        model = LinearLeastSquareModel(input_columns, output_columns, debug = False) # 类的实例化:用最小二乘生成已知模型
        linear_fit, resids, rank, s = scipy.linalg.lstsq(all_data[:,input_columns], all_data[:,output_columns])
        ransac_fit, ransac_data = LinearLeastSquareModel.ransac(all_data, model, 20, 1000, 7e3, 40, debug=False, return_all=True) # RANSAC模型
        sort_idxs = np.argsort(A_exact[:,0])
        print('linear_fit', linear_fit)
        print('ransac_fit', ransac_fit)
        A_col0_sorted = A_exact[sort_idxs] # 秩为2的数组
        plt.plot(A_exact[:,0], B_exact[:,0], 'k.', label='data')
        plt.plot(A_exact[ransac_data['inliers'],0], B_exact[ransac_data['inliers'],0], 'bx', label='RANSAC data')
        plt.plot(A_col0_sorted[:,0], np.dot(A_col0_sorted,ransac_fit)[:,0], label='RANSAC fit' )
        plt.plot(A_col0_sorted[:,0], np.dot(A_col0_sorted,linear_fit)[:,0], label='linear fit' )
        plt.legend() # 添加图例
        plt.show()
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