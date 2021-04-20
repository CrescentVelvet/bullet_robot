import math
import numpy as np
# import scipy.sparse.linalg

class AA: # 垃圾命名的变量
    # 以下参数的含义参见仿人机器人的书和MATLAB程序pendulum_walk.m
    tick_num = 0
    x0 = 0
    xt = 0
    vx = 0
    com_h = 0
    Tc = 1
    tao = 0
    # 这里固定y0为3cm,和理论不符,实际把这个轨迹进行了平移到了6.5（ANKLE_DIS/2）处,真实的相对于的支撑脚的y方向起点是y00
    y00 = 0
    ytt = 0
    y0 = 0
    yt = 0
    m = 0
    vy = 0
    x = 0
    y = 0
    # 行走插值序列
    comYaw = []
    comY = []
    accX = []
    accY = []
    akX = []
    akY = []
    akZ = []
    akYaw = []
class PendulumWalkParam: # 步态参数
    ANKLE_DIS = 15 * 0.01 # 通过观察,机器人模型踝间距差不多是11到12左右,单位为cm(bullet中单位应该是m)
    TAO = 0.30 # 通过观察,肉包的步态单元的时长是0.35s
    TICK_NUM = 30.0 # 每个步态周期发35个值
    COM_H = 37 * 0.01 # 机器人倒立摆长度37cm
    ACC_COEF_X = 0.15 * 0.01 # 把本次质心前进dx和上一回dx进行做差对比,乘以系数的数字作为质心的位置移动,插值后在本次中叠加在倒立摆x轨迹上
    ACC_COEF_Y = 0.3 * 0.01
    COM_HEIGHT = 30.8637 * 0.01 # 0.28 # 默认规划质心高度
    Y_HALF_AMPLITUDE = 3 * 0.01 # y方向倒立摆起点坐标长度
    COM_X_OFFSET = 1 * 0.01 # 在理想质心规划基础上视走路情况而定的x方向偏移
    TURNING_ERROR = 4.0 # 旋转时每步真实角度和理论角度的差异
    # 抬脚曲线参数,调参得出的,不会改变
    foot_z_t = [0, 0.04439, 0.16583, 0.33] # 抬脚高度曲线的时间向量
    foot_z_p = [0.1, 0.06215, 4.67307, 0.1] # 抬脚高度曲线的z轴向量
    foot_z_s = [-3.0, 0, 0, 3.0] # 抬脚高度曲线的tp曲线斜率向量
class ElementGait: # 单个步态指令类
    def __init__(self):
        self.X = 0 # 该步的x方向位移
        self.Y = 0 # 该步的y方向位移
        self.YAW = 0 # 该步的旋转角度
        self.IS_RIGHT = 1 # 该步由右脚迈出
    def __init__(self, x, y, yaw, is_right):
        self.X = x
        self.Y = y
        self.YAW = yaw
        self.IS_RIGHT = is_right
class BB: # 步态参数初始化    
    com_ac_x = 0
    com_ac_y = 0
    support_is_right = 0
    hang_foot = [0, PendulumWalkParam.ANKLE_DIS, 0] # 悬荡腿质心坐标
    com_pos = [0, PendulumWalkParam.ANKLE_DIS/2.0, 0] # 上半身质心坐标
    com_x_changed = 0
    com_y_changed = 0
class stp:
    gait_queue = [] # 步态指令序列
    tmp_gait = ElementGait(0, 0, 0, 1) # 单个步态指令
    last_gait = ElementGait(0, 0, 0, 1) # 上一个步态指令
class threeInterPolation:
    # 注意,python不能对构造函数重载,这里需要修改！
    # def __init__(self): # 默认构造函数
    #     # 默认构造函数
    #     self.x_array_ = []
    #     self.y_array_ = []
    #     self.s_angle_ = []
    #     self.time_interval_ = ThreeInterpolationParam.DEFAULT_POINT_INTERVAL # 默认的发值时间间隔
    #     self.is_order = 0 # x_array序列是否合格
    #     self.piece_num_ = 0
    #     self.poly_ = []
    #     self.x_samples_ = []
    #     self.y_samples_ = []
    # def __init__(self, x_array, y_array): # 缺省构造函数
    #     # 缺省构造函数,没有斜率输入,默认斜率都是0
    #     # @param x_array  样本点的x坐标序列,dmoiton工程中是时间,单位为s,通常从0开始
    #     # @param y_array  样本点的y坐标序列,dmotion工程中是肢端坐标值
    #     self.x_array_ = x_array
    #     self.y_array_ = y_array
    #     self.s_angle_ = [] # 在append前需要先设置为空
    #     self.time_interval_ = ThreeInterpolationParam.DEFAULT_POINT_INTERVAL
    #     if self.y_array_[0] > self.y_array_[1]:
    #         self.s_angle_.append(-ThreeInterpolationParam.DEFAULT_BOUNDARY_SLOPE)
    #     elif self.y_array_[0] < self.y_array_[1]:
    #         self.s_angle_.append(ThreeInterpolationParam.DEFAULT_BOUNDARY_SLOPE)
    #     else:
    #         self.s_angle_.append(0.0)
    #     for i in range(len(self.x_array_) - 1):
    #         if ( self.y_array_[i - 1] <= self.y_array_[i] and self.y_array_[i] <= y_array_[i + 1] ) or ( self.y_array_[i - 1] >= self.y_array_[i] and self.y_array_[i] >= y_array_[i + 1] ):
    #             self.s_angle_.append( (self.y_array_[i + 1] - self.y_array_[i - 1]) / (self.x_array_[i + 1] - self.x_array_[i - 1]) )
    #         else:
    #             self.s_angle_.append(0.0)
    #     if self.y_array_[len(self.y_array_) - 2] > self.y_array_[len(self.y_array_) - 1]:
    #         self.s_angle_.append(-ThreeInterpolationParam.DEFAULT_BOUNDARY_SLOPE)
    #     elif self.y_array_[len(self.y_array_) - 2] < self.y_array_[len(self.y_array_) - 1]:
    #         self.s_angle_.append(ThreeInterpolationParam.DEFAULT_BOUNDARY_SLOPE)
    #     else:
    #         self.s_angle_.append(0.0)
    #     if self.isInOrder(self.x_array_):
    #         self.piece_num_ = int(len(self.x_array_) - 1)
    #         self.Calculate()
    #         self.CalculatePoints(self.time_interval_)
    #     else:
    #         print('error : The x_array is not available!')            
    #     self.poly_ = []
    #     self.x_samples_ = []
    #     self.y_samples_ = []
    def __init__(self, x_array, y_array, s_angle): # 标准构造函数
        # 标准构造函数
        # @param x_array  样本点的x坐标序列,dmoiton工程中是时间,单位为s,通常从0开始
        # @param y_array  样本点的y坐标序列,dmotion工程中是肢端坐标值
        # @param s_angle  样本点的倾斜角度,dmotion工程中是肢端坐标值的变化速度
        self.x_array_ = x_array
        self.y_array_ = y_array
        self.s_angle_ = s_angle
        self.time_interval_ = ThreeInterpolationParam.DEFAULT_POINT_INTERVAL
        if self.isInOrder(self.x_array_):
            self.piece_num_ = int(len(self.x_array_) - 1)
            self.Calculate()
            self.CalculatePoints(self.time_interval_)
        else:
            print('error : The x_array is not available!')
    def isInOrder(self, x_ar): # 检测x_array合格函数
        # 判定输入的x_array序列是否合格
        # @param x_ar 输入的x_array序列
        # return 返回是否合格的bool值
        for i in range(len(x_ar) - 1):
            if x_ar[i] > x_ar[i + 1]: # 该帧时间序列倒置,不合格
                self.is_order = 0
                return 0
            elif x_ar[i] == x_ar[i + 1]: # 该帧时间序列相等,删除该帧
                self.x_array_.pop(i)
                self.y_array_.pop(i)
                self.s_angle_.pop(i)
            else:
                pass
        self.is_order = 1
        return 1
    def Calculate(self): # 计算poly_多项式函数
        # 计算分段多项式,无返回值
        self.poly_ = []
        for i in range(self.piece_num_):
            self.poly_.append(self.onePiece(self.x_array_[i], self.y_array_[i], self.s_angle_[i], self.x_array_[i + 1], self.y_array_[i + 1], self.s_angle_[i + 1]))
    def CalculatePoints(self, t0_in): # 计算三次曲线值函数
        # 用于获得固定时间间隔的分段三次曲线的值,无返回值
        # @param t0_in 时间序列的间隔,单位为ns
        self.x_samples_ = []
        self.y_samples_ = []
        t_tmp = self.x_array_[0]
        t0 = float(t0_in / 1000) # 时间间隔的单位从ns转换为ms
        self.time_interval_ = t0
        while (self.x_array_[self.piece_num_] - t_tmp) > 0.000001:
            self.x_samples_.append(t_tmp) # t_tmp时间值
            self.y_samples_.append(self.EvalHere(t_tmp)) # t_tmp时刻曲线上的点值
            t_tmp += t0 # 时刻增加一个时间间隔
        if (self.x_array_[self.piece_num_] - t_tmp + t0) > 0.000001:
            self.x_samples_.append(self.x_array_[self.piece_num_])
            self.y_samples_.append(self.EvalHere(self.x_array_[self.piece_num_]))
    def onePiece(self, x_r, y_r, s_r, x_l, y_l, s_l): # 计算单个piece函数
        # 计算单独一个piece
        # @param x_r
        # @param y_r
        # @param s_r
        # @param x_l
        # @param y_l
        # @param s_l
        # return coef_tmp 返回
        B = np.array([y_r, s_r, y_l, s_l])
        A1 = np.array([math.pow(x_r, 3), math.pow(x_r, 2), x_r, 1])
        A2 = np.array([3 * math.pow(x_r, 2), 2 * x_r, 1, 0])
        A3 = np.array([math.pow(x_l, 3), math.pow(x_l, 2), x_l, 1])
        A4 = np.array([3 * math.pow(x_l, 2), 2 * x_l, 1, 0])
        A = np.array([A1, A2, A3, A4])
        coef_tmp = np.linalg.solve(A, B) # LU三角分解
        # coef_tmp = scipy.sparse.linalg.spilu(A).solve(B) # LU三角分解
        return coef_tmp
    def getPoints(self): # 获取坐标函数
        # 在CalculatePoints之后获得计算出来的点的坐标值
        # return 返回这些坐标点的vector
        return self.y_samples_
    def getTimes(self): # 获取时间函数
        # 在CalculatePoints之后获得计算出来的时间点,作为舵机发值的时间戳
        # return 返回这些时间点的vector
        return self.x_samples_
    def EvalHere(self, point_x0): # 计算插值曲线函数
        # 获得分段三次插值曲线某一点的值
        # @param point_x0 输入欲获得坐标点的横坐标
        # @return result_x0 返回point_x0代入多项式poly_求得的坐标值
        for i in range(self.piece_num_):
            if point_x0 >= self.x_array_[i] and point_x0 <= self.x_array_[i + 1]:
                break
        if i == self.piece_num_:
            print('error : x is beyond the domain of definition!')
            print(point_x0, ' is not between ', self.x_array_[0], ' and ', self.x_array_[self.piece_num_])
            return 0
        result_x0 = 0 # point_x0代入多项式poly_求得的坐标值
        for j in range(len(self.poly_)):
            result_x0 += self.poly_[0][j] * math.pow(point_x0, len(self.poly_) - j - 1) # 这里的poly是个array,需要取[0]来获得一维数组
        return result_x0
class ThreeInterpolationParam: # 三次曲线参数
    DEFAULT_POINT_INTERVAL = 10.0 # 默认的发值时间间隔
    DEFAULT_BOUNDARY_SLOPE = 1.0
class MotionTick: # 发值瞬间机器人参数类
    def __init__(self):
        self.hang_foot = [] # 悬荡腿质心坐标与方向
        self.whole_com = [] # 全身质心坐标
        self.upbody_pose = [] # 上半身角度
class UpbodyMode: # 上半身状态参数
    upbody_still = 1 # 上半身rpy(0,0,0)
    self_adaption = 2 # 上半身根据对角线准则自适应
    constant_value = 3 # 上半身设置yaw,固定roll和pitch
    customize_value = 4 # 上半身设置yaw,roll和pitch
class OneFootLandingParam: # OneFootLanding参数
    UPBODY_CONSTANT_ROLL = 0
    UPBODY_CONSTANT_PITCH = 0.2
    IS_RIGHT = 1
    ANKLE_OFFSET_X = 1
    ANKLE_OFFSET_Y = 0
    ANKLE_OFFSET_Z = 10
    BODY_CENTER_X = 2.5
    BODY_CENTER_Y = 0
    BODY_CENTER_Z = -2.4
    UPBODY_MASS = 3158
    FOOT_MASS = 877
    # 下面是逆运动学的参数
    UPPER_LEG_LENGTH = 12.3
    LOWER_LEG_LENGTH = 13.0
    ANKLE_FROM_GROUND = 6.0
    HALF_HIP_WIDTH = 4.5
    HIP_X_FROM_ORIGIN = 0
    HIP_Z_FROM_ORIGIN = 8.1
class OneFootLanding: # 单步计算类
    def GetOneStep(hang_foot, whole_body_com, upbody_pose): # 单帧计算函数
        # 单帧计算函数,用于计算接下来一帧中各个舵机的值
        # @param hang_foot 输入上一帧悬荡腿质心坐标与方向
        # @param whole_body_com 输入上一帧全身质心坐标
        # @param upbody_pose 输入上一帧上半身角度
        # @return one_foot_return 返回计算出这一帧12个舵机值的序列
        hang_foot[3] = math.radians(hang_foot[3]) # 角度转弧度
        hang_foot[4] = math.radians(hang_foot[4])
        hang_foot[5] = math.radians(hang_foot[5])
        upbody_yaw = math.radians(upbody_pose[2])
        is_right = OneFootLandingParam.IS_RIGHT # 设置参数
        ankle_offset_x = OneFootLandingParam.ANKLE_OFFSET_X
        ankle_offset_y = OneFootLandingParam.ANKLE_OFFSET_Y
        ankle_offset_z = OneFootLandingParam.ANKLE_OFFSET_Z
        body_center_x = OneFootLandingParam.BODY_CENTER_X
        body_center_y = OneFootLandingParam.BODY_CENTER_Y
        body_center_z = OneFootLandingParam.BODY_CENTER_Z
        upbody_mass = OneFootLandingParam.UPBODY_MASS
        foot_mass = OneFootLandingParam.FOOT_MASS
        hangfoot_com = [] # 计算悬荡腿的质心
        landfoot_com = [] # 计算立足腿的质心
        upbody_com = [] # 计算上半身的质心
        v = [] # 逆运动学计算临时变量
        upbody_mode = UpbodyMode.upbody_still # 上半身状态
        body_centre = [] # 计算身体中心的位置和姿态
        hanging_invkin = [] # 计算悬荡腿的逆运动学向量
        landing_invkin = [] # 计算立足腿的逆运动学向量
        one_foot_result = [] # 单步逆运动学最终结果,这一帧12个舵机的值(先6个right,后6个left)
        # 求上半身的质心位置,已考虑脚底中心点和脚质心的区别
        hangfoot_com.append(
            - ( ankle_offset_y if is_right else (-ankle_offset_y) )
            * ( math.cos(hang_foot[3]) * math.sin(hang_foot[5]) - math.cos(hang_foot[5]) * math.sin(hang_foot[4]) * math.sin(hang_foot[3]) )
            + ankle_offset_z
            * ( math.sin(hang_foot[3]) * math.sin(hang_foot[5]) + math.cos(hang_foot[3]) * math.cos(hang_foot[5]) * math.sin(hang_foot[4]) )
            + ankle_offset_x
            * math.cos(hang_foot[4]) * math.cos(hang_foot[5])
            + hang_foot[0]
        )
        hangfoot_com.append(
            + ( ankle_offset_y if is_right else (-ankle_offset_y) )
            * ( math.cos(hang_foot[3]) * math.cos(hang_foot[5]) + math.sin(hang_foot[3]) * math.sin(hang_foot[4]) * math.sin(hang_foot[5]) )
            - ankle_offset_z
            * ( math.cos(hang_foot[5]) * math.sin(hang_foot[3]) - math.cos(hang_foot[3]) * math.sin(hang_foot[4]) * math.sin(hang_foot[5]) )
            + ankle_offset_x
            * math.cos(hang_foot[4]) * math.sin(hang_foot[5])
            + hang_foot[1]
        )
        hangfoot_com.append(
            + ( ankle_offset_y if is_right else (-ankle_offset_y) )
            * math.cos(hang_foot[4]) * math.sin(hang_foot[3])
            + ankle_offset_z
            * math.cos(hang_foot[4]) * math.cos(hang_foot[3])
            - ankle_offset_x
            * math.sin(hang_foot[4])
            + hang_foot[2]
        )
        landfoot_com.append(ankle_offset_x)
        landfoot_com.append(ankle_offset_y if not is_right else (-ankle_offset_y))
        landfoot_com.append(ankle_offset_z)
        # 考虑到了支撑脚作为质心的一部分
        upbody_com.append( ( (upbody_mass + 2 * foot_mass) * whole_body_com[0] - foot_mass * hangfoot_com[0] - foot_mass * landfoot_com[0] ) / upbody_mass )
        upbody_com.append( ( (upbody_mass + 2 * foot_mass) * whole_body_com[1] - foot_mass * hangfoot_com[1] - foot_mass * landfoot_com[1] ) / upbody_mass )
        upbody_com.append( ( (upbody_mass + 2 * foot_mass) * whole_body_com[2] - foot_mass * hangfoot_com[2] - foot_mass * landfoot_com[2] ) / upbody_mass )
        # 确定一下使用什么类型的上半身姿态
        for i in range(len(upbody_com)):
            v.append( (upbody_com[i] - landfoot_com[i]) + (upbody_com[i] - hangfoot_com[i]) )
        v = OneFootLanding.unit_arrow(v)
        if upbody_mode == UpbodyMode.upbody_still: # 上半身rpy(0,0,0)
            upbody_roll = 0
            upbody_pitch = 0
        elif upbody_mode == UpbodyMode.constant_value: # 上半身根据对角线准则自适应
            upbody_roll = OneFootLandingParam.UPBODY_CONSTANT_ROLL
            upbody_pitch = OneFootLandingParam.UPBODY_CONSTANT_PITCH
        elif upbody_mode == UpbodyMode.self_adaption: # 上半身设置yaw,固定roll和pitch
            upbody_roll = OneFootLanding.gait_atan(-v[1], v[2]) * 4.0 / 5.0
            upbody_pitch = OneFootLanding.gait_atan(v[0], v[2] / math.cos(upbody_roll)) * 4.0 / 5.0
        elif upbody_mode == UpbodyMode.customized_value: # 上半身设置yaw,roll和pitch
            upbody_roll = math.radians(upbody_pose[0])
            upbody_pitch = math.radians(upbody_pose[1])
        body_centre.append( # 计算身体中心的位置和姿态
            + body_center_z * math.sin(upbody_pitch) + body_center_x * math.cos(upbody_pitch) * math.cos(upbody_yaw)
            - body_center_y * math.cos(upbody_pitch) * math.sin(upbody_yaw)
            + upbody_com[0]
        )
        body_centre.append(
            + body_center_x
            * ( math.cos(upbody_roll) * math.sin(upbody_yaw) + math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(upbody_roll) )
            + body_center_y
            * ( math.cos(upbody_roll) * math.cos(upbody_yaw) - math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(upbody_yaw) )
            - body_center_z
            * math.cos(upbody_pitch) * math.sin(upbody_roll)
            + upbody_com[1]
        )
        body_centre.append(
            + body_center_x
            * ( math.sin(upbody_roll) * math.sin(upbody_yaw) - math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(upbody_pitch) )
            + body_center_y
            * ( math.cos(upbody_yaw) * math.sin(upbody_roll) + math.cos(upbody_roll) * math.sin(upbody_pitch) * math.sin(upbody_yaw) )
            + body_center_z
            * math.cos(upbody_pitch) * math.cos(upbody_roll)
            + upbody_com[2]
        )
        hanging_invkin.append( # 给悬荡脚的逆运动学向量加入值
            + hang_foot[0] * math.cos(upbody_pitch) * math.cos(upbody_yaw)
            - body_centre[0] * math.cos(upbody_pitch) * math.cos(upbody_yaw)
            + hang_foot[1] * math.cos(upbody_roll) * math.sin(upbody_yaw)
            - body_centre[1] * math.cos(upbody_roll) * math.sin(upbody_yaw)
            + hang_foot[2] * math.sin(upbody_roll) * math.sin(upbody_yaw)
            - body_centre[2] * math.sin(upbody_roll) * math.sin(upbody_yaw)
            - hang_foot[2] * math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(upbody_pitch)
            + body_centre[2] * math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(upbody_pitch)
            + hang_foot[1] * math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(upbody_roll)
            - body_centre[1] * math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(upbody_roll)
        )
        hanging_invkin.append(
            + hang_foot[1] * math.cos(upbody_roll) * math.cos(upbody_yaw)
            - body_centre[1] * math.cos(upbody_roll) * math.cos(upbody_yaw)
            - hang_foot[0] * math.cos(upbody_pitch) * math.sin(upbody_yaw)
            + hang_foot[2] * math.cos(upbody_yaw) * math.sin(upbody_roll)
            + body_centre[0] * math.cos(upbody_pitch) * math.sin(upbody_yaw)
            - body_centre[2] * math.cos(upbody_yaw) * math.sin(upbody_roll)
            + hang_foot[2] * math.cos(upbody_roll) * math.sin(upbody_pitch) * math.sin(upbody_yaw)
            - body_centre[2] * math.cos(upbody_roll) * math.sin(upbody_pitch) * math.sin(upbody_yaw)
            - hang_foot[1] * math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(upbody_yaw)
            + body_centre[1] * math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(upbody_yaw)
        )
        hanging_invkin.append(
            + hang_foot[0] * math.sin(upbody_pitch) - body_centre[0] * math.sin(upbody_pitch)
            + hang_foot[2] * math.cos(upbody_pitch) * math.cos(upbody_roll)
            - body_centre[2] * math.cos(upbody_pitch) * math.cos(upbody_roll)
            - hang_foot[1] * math.cos(upbody_pitch) * math.sin(upbody_roll)
            + body_centre[1] * math.cos(upbody_pitch) * math.sin(upbody_roll)
        )
        T1_1 = (
            + math.cos(upbody_pitch) * math.cos(hang_foot[4]) * math.cos(hang_foot[5]) * math.cos(upbody_yaw)
            - math.sin(hang_foot[4]) * math.sin(upbody_roll) * math.sin(upbody_yaw)
            + math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(hang_foot[4])
            + math.cos(hang_foot[4]) * math.cos(upbody_roll) * math.sin(hang_foot[5]) * math.sin(upbody_yaw)
            + math.cos(hang_foot[4]) * math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(hang_foot[5])
        )
        T2_1 = (
            + math.cos(hang_foot[4]) * math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(hang_foot[5])
            - math.cos(upbody_pitch) * math.cos(hang_foot[4]) * math.cos(hang_foot[5]) * math.sin(upbody_yaw)
            - math.cos(upbody_yaw) * math.sin(hang_foot[4]) * math.sin(upbody_roll)
            - math.cos(upbody_roll) * math.sin(upbody_pitch) * math.sin(hang_foot[4]) * math.sin(upbody_yaw)
            - math.cos(hang_foot[4]) * math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(hang_foot[5]) * math.sin(upbody_yaw)
        )
        T3_1 = (
            + math.cos(hang_foot[4]) * math.cos(hang_foot[5]) * math.sin(upbody_pitch)
            - math.cos(upbody_pitch) * math.cos(upbody_roll) * math.sin(hang_foot[4])
            - math.cos(upbody_pitch) * math.cos(hang_foot[4]) * math.sin(upbody_roll) * math.sin(hang_foot[5])
        )
        T3_2 = (
            + math.cos(upbody_pitch) * math.cos(hang_foot[4]) * math.cos(upbody_roll) * math.sin(hang_foot[3])
            - math.cos(hang_foot[3]) * math.sin(upbody_pitch) * math.sin(hang_foot[5])
            - math.cos(upbody_pitch) * math.cos(hang_foot[3]) * math.cos(hang_foot[5]) * math.sin(upbody_roll)
            + math.cos(hang_foot[5]) * math.sin(upbody_pitch) * math.sin(hang_foot[4]) * math.sin(hang_foot[3])
            - math.cos(upbody_pitch) * math.sin(hang_foot[4]) * math.sin(hang_foot[3]) * math.sin(upbody_roll) * math.sin(hang_foot[5])
        )
        T3_3 = (
            + math.sin(upbody_pitch) * math.sin(hang_foot[3]) * math.sin(hang_foot[5])
            + math.cos(upbody_pitch) * math.cos(hang_foot[4]) * math.cos(hang_foot[3]) * math.cos(upbody_roll)
            + math.cos(hang_foot[3]) * math.cos(hang_foot[5]) * math.sin(upbody_pitch) * math.sin(hang_foot[4])
            + math.cos(upbody_pitch) * math.cos(hang_foot[5]) * math.sin(hang_foot[3]) * math.sin(upbody_roll)
            - math.cos(upbody_pitch) * math.cos(hang_foot[3]) * math.sin(hang_foot[4]) * math.sin(upbody_roll) * math.sin(hang_foot[5])
        )
        hanging_invkin.append(math.degrees(OneFootLanding.gait_atan(T3_2, T3_3)))
        hanging_invkin.append(math.degrees(math.asin(-T3_1)))
        hanging_invkin.append(math.degrees(OneFootLanding.gait_atan(T2_1, T1_1)))
        landing_invkin.append( # 给立足脚的逆运动学向量加入值
            + body_centre[2] * math.cos(upbody_roll) * math.cos(upbody_yaw) * math.sin(upbody_pitch)
            - body_centre[1] * math.cos(upbody_roll) * math.sin(upbody_yaw)
            - body_centre[2] * math.sin(upbody_roll) * math.sin(upbody_yaw)
            - body_centre[0] * math.cos(upbody_pitch) * math.cos(upbody_yaw)
            - body_centre[1] * math.cos(upbody_yaw) * math.sin(upbody_pitch) * math.sin(upbody_roll)
        )
        landing_invkin.append(
            + body_centre[0] * math.cos(upbody_pitch) * math.sin(upbody_yaw)
            - body_centre[1] * math.cos(upbody_roll) * math.cos(upbody_yaw)
            - body_centre[2] * math.cos(upbody_yaw) * math.sin(upbody_roll)
            - body_centre[2] * math.cos(upbody_roll) * math.sin(upbody_pitch) * math.sin(upbody_yaw)
            + body_centre[1] * math.sin(upbody_pitch) * math.sin(upbody_roll) * math.sin(upbody_yaw)
        )
        landing_invkin.append(
            + body_centre[1] * math.cos(upbody_pitch) * math.sin(upbody_roll)
            - body_centre[2] * math.cos(upbody_pitch) * math.cos(upbody_roll)
            - body_centre[0] * math.sin(upbody_pitch)
        )
        landing_invkin.append(-math.degrees(upbody_roll))
        landing_invkin.append(-math.degrees(upbody_pitch))
        landing_invkin.append(-math.degrees(upbody_yaw))
        if is_right: # 右脚立足
            one_foot_result.append(landing_invkin) # one_foot_result[0]到[5]是右脚
            one_foot_result.append(hanging_invkin) # one_foot_result[6]到[11]是左脚
        else: # 左脚立足
            one_foot_result.append(hanging_invkin)
            one_foot_result.append(landing_invkin)
        one_foot_return = []
        for i in range(len(one_foot_result)): # 二维数组转换一维数组
            for j in range(len(one_foot_result[i])):
                one_foot_return.append(math.radians(one_foot_result[i][j])) # 角度制还得转回弧度制
        return one_foot_return
    def unit_arrow(arrow): # 向量归一化函数
        # 向量归一化函数,将向量的长度归一
        # @param arrow 输入任意维度的向量
        # @return 输出长度为一的向量
        len_arrow = 0
        for i in range(len(arrow)):
            len_arrow = len_arrow + arrow[i] * arrow[i]
        len_arrow = math.sqrt(len_arrow)
        for j in range(len(arrow)):
            arrow[j] = arrow[j] / len_arrow
        return arrow
    def gait_atan(opposite, neighbor): # 双变量反正切函数
        # 双变量反正切函数,用于准确求取一个角度,值域是[-pi,pi)
        # @param opposite 对边长度（直角边）
        # @param neighbor 临边长度 (直角边)
        # @return 所求的角度,弧度制
        if 0 < neighbor:
            return math.atan(opposite / neighbor)
        elif 0 == neighbor:
            if 0 > opposite:
                return -math.pi / 2 
            elif 0 < opposite:
                return math.pi / 2
            else:
                return 0
        else:
            if 0 <= opposite:
                return math.atan(opposite / neighbor) + math.pi
            else:
                return math.atan(opposite / neighbor) - math.pi
# OneFootLanding.GetOneStep(hang_foot=[1,1,1,1,1,1], whole_body_com=[1,1,1], upbody_pose=[1,1,1,1,1,1])