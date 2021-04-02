import math

class PendulumWalkParam: # 步态参数
    ANKLE_DIS = 15.0 # 通过观察,机器人模型踝间距差不多是11到12左右,单位为cm
    TAO = 0.30 # 通过观察,肉包的步态单元的时长是0.35s
    TICK_NUM = 30.0 # 每个步态周期发35个值
    COM_H = 37.0 # 机器人倒立摆长度37cm
    ACC_COEF_X = 0.15 # 把本次质心前进dx和上一回dx进行做差对比，乘以系数的数字作为质心的位置移动，插值后在本次中叠加在倒立摆x轨迹上
    ACC_COEF_Y = 0.3
    COM_HEIGHT = 27 # 默认规划重心高度
    Y_HALF_AMPLITUDE = 3.0 # y方向倒立摆起点坐标长度
    COM_X_OFFSET = 1 # 在理想重心规划基础上视走路情况而定的x方向偏移
    TURNING_ERROR = 4.0 # 旋转时每步真实角度和理论角度的差异

class MotionTick: # 发值瞬间的机器人参数
    def __init__(self):
        upbody_pose = [] # 双足位置与角度
        whole_com = [] # 全身重心位置
        upbody_pose = [] # 上半身角度

class UpbodyMode: # 上半身状态
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

class ElementGait: # 单个步态
    def __init__(self):
        self.X = 0 # 该步的x方向位移
        self.Y = 0 # 该步的y方向位移
        self.YAW = 0 # 该步的旋转角度
        self.IS_RIGHT = 1 # 该步由右脚迈出
        self.IS_LEFT = 0 # 该步由左脚迈出
    def __init__(self, x, y, yaw, is_right, is_left):
        self.X = x
        self.Y = y
        self.YAW = yaw
        self.IS_RIGHT = is_right
        self.IS_LEFT = is_left

class OneFootLanding:
    def GetOneStep(hang_foot, whole_body_com, upbody_pose):
        # 单步计算函数，用于计算接下来一步中各个舵机的值
        # @param hang_foot 双足位置与角度
        # @param whole_body_com 全身重心位置
        # @param upbody_pose 上半身角度
        # @return 计算出12个舵机值的序列
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
        hangfoot_com = [] # 计算悬荡腿的重心
        landfoot_com = [] # 计算立足腿的重心
        upbody_com = [] # 计算上半身的重心
        v = [] # 计算逆运动学临时变量
        upbody_mode = UpbodyMode.upbody_still # 上半身状态
        body_centre = [] # 计算身体中心的位置和姿态
        hanging_invkin = [] # 计算悬荡腿的逆运动学向量
        landing_invkin = [] # 计算立足腿的逆运动学向量
        one_foot_result = [] # 单步逆运动学最终结果,12个舵机的值(先6个right,后6个left)
        # 求上半身的重心位置,已考虑脚底中心点和脚重心的区别
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
        # 考虑到了支撑脚作为重心的一部分
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
            one_foot_result.append(landing_invkin)
            one_foot_result.append(hanging_invkin)
        else: # 左脚立足
            one_foot_result.append(hanging_invkin)
            one_foot_result.append(landing_invkin)
        # print(one_foot_result)
        return one_foot_result
    def unit_arrow(arrow):
        # 向量归一化函数，将向量的长度归一
        # @param arrow 输入任意维度的向量
        # @return 输出长度为一的向量
        len_arrow = 0
        for i in range(len(arrow)):
            len_arrow = len_arrow + arrow[i] * arrow[i]
        len_arrow = math.sqrt(len_arrow)
        for j in range(len(arrow)):
            arrow[j] = arrow[j] / len_arrow
        return arrow
    def gait_atan(opposite, neighbor):
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