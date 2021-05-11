import matplotlib.pyplot as plt
import numpy as np
import scipy
import scipy.linalg
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

class FitModule:
    @staticmethod
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
        def random_partition(n,n_data):
            """return n random rows of data (and also the other len(data)-n rows)"""
            all_idxs = np.arange( n_data )
            np.random.shuffle(all_idxs)
            idxs1 = all_idxs[:n]
            idxs2 = all_idxs[n:]
            return idxs1, idxs2
        iterations = 0
        bestfit = None
        besterr = np.inf # 设置默认值
        best_inlier_idxs = None
        while iterations < k:
            maybe_idxs, test_idxs = random_partition(n,data.shape[0])
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

def Poly1d(x, y, needB=True):
    n_samples = len(x) # 样本个数
    x = np.array([x]).T
    y = np.array([y]).T
    n_inputs = 1 # 输入变量个数
    n_outputs = 1 # 输出变量个数  
    if needB:
        x = np.hstack((np.ones(x.shape),x))
        n_inputs = 2
    all_data = np.hstack((x,y)) # 在水平方向上平铺拼接数组
    input_columns = range(n_inputs) # 数组的第一列x:0
    output_columns = [n_inputs + i for i in range(n_outputs)] # 数组最后一列y:1
    # print(input_columns, output_columns)
    model = LinearLeastSquareModel(input_columns, output_columns, debug = False) # 类的实例化:用最小二乘生成已知模型
    linear_fit, resids, rank, s = scipy.linalg.lstsq(all_data[:,input_columns], all_data[:,output_columns])
    ransac_fit, ransac_data = FitModule.ransac(all_data, model, int(2*n_samples/3), 1000, 0.01, 0.1, debug=False, return_all=True) # RANSAC模型
    sort_idxs = np.argsort(x[:,0])
    print('linear_fit', linear_fit)
    print('ransac_fit', ransac_fit)
    A_col0_sorted = x[sort_idxs] # 秩为2的数组
    plt.plot(x[:,1], y[:,0], 'k.', label='data')
    plt.plot(x[ransac_data['inliers'],1], y[ransac_data['inliers'],0], 'bx', label='RANSAC data')
    plt.plot(A_col0_sorted[:,1], np.dot(A_col0_sorted,ransac_fit)[:,0], label='RANSAC fit' )
    plt.plot(A_col0_sorted[:,1], np.dot(A_col0_sorted,linear_fit)[:,0], label='linear fit' )
    plt.legend() # 添加图例
    plt.show()
