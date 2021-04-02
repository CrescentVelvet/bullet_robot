import math

class ParamGait: # 步态参数
    ANKLE_DIS = 0.15 # 机器人模型踝间距
    COM_H = 0.37 # 机器人倒立摆长度37cm
    Y_HALF_AMPLITUDE = 3.0 # y方向倒立摆起点坐标长度

class MotionTick: # 机器人参数
    HANG_FOOT = 0 # 双足位置与角度
    WHOLE_BODY_COM = 0 # 全身重心位置
    UPBODY_POSE = 0 # 上半身角度

class OneFootLandingParam: # OneFootLanding参数
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
    def GetOneStep(hang_foot, whole_body_com, upbody_pose): # 输入双足位置与角度，全身重心位置，上半身角度，输出12个舵机的值
        hang_foot[3] = math.radians(hang_foot[3]) # 角度转弧度
        hang_foot[4] = math.radians(hang_foot[4])
        hang_foot[5] = math.radians(hang_foot[5])
        upbody_yaw = math.radians(upbody_pose[2])
        is_right = OneFootLandingParam.IS_RIGHT
        ankle_offset_x = OneFootLandingParam.ANKLE_OFFSET_X
        ankle_offset_y = OneFootLandingParam.ANKLE_OFFSET_Y
        ankle_offset_z = OneFootLandingParam.ANKLE_OFFSET_Z
        hangfoot_com = []
        landingfoot_com = []
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
            + (ankle_offset_y if is_right else (-ankle_offset_y))
            * ( math.cos(hang_foot[3]) * math.cos(hang_foot[5]) + math.sin(hang_foot[3]) * math.sin(hang_foot[4]) * math.sin(hang_foot[5]) )
            - ankle_offset_z
            * ( math.cos(hang_foot[5]) * math.sin(hang_foot[3]) - math.cos(hang_foot[3]) * math.sin(hang_foot[4]) * math.sin(hang_foot[5]) )
            + ankle_offset_x
            * math.cos(hang_foot[4]) * math.sin(hang_foot[5])
            + hang_foot[1]
        )
        hangfoot_com.append(
            + (ankle_offset_y if is_right else (-ankle_offset_y))
            * math.cos(hang_foot[4]) * math.sin(hang_foot[3])
            + ankle_offset_z
            * math.cos(hang_foot[4]) * math.cos(hang_foot[3])
            - ankle_offset_x
            * math.sin(hang_foot[4])
            + hang_foot[2]
        )
        landingfoot_com.append(ankle_offset_x)
        landingfoot_com.append(ankle_offset_y if not is_right else (-ankle_offset_y))
        landingfoot_com.append(ankle_offset_z)