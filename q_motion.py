'''
Author       : velvet
Date         : 2021-03-19 21:57:57
LastEditTime : 2021-03-19 22:29:50
LastEditors  : velvet
Description  : 
'''
import time
import math
import numpy as np
import pybullet as p
import pybullet_data
import q_param

physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -98)
useRealTimeSim = 0
p.setRealTimeSimulation(useRealTimeSim)
planeId = p.loadSDF("stadium.sdf")
p.changeDynamics(planeId[0], -1, lateralFriction = 0.8, spinningFriction = 0.3, rollingFriction = 0.0001)
p.resetDebugVisualizerCamera(cameraDistance=4.0, cameraYaw=75.0, cameraPitch=-25.0, cameraTargetPosition=[-1.0, 1.0, -0.5], physicsClientId=physicsClient)
# 各种参数
use_robot = 1 # 显示机器人模型
use_camera = 0 # 开启摄像头
draw_message = 0 # 显示关节名称
draw_interia = 0 # 绘制转动惯量
draw_sphere  = 0 # 绘制质心小球
init_robot_pos = [0, 0, 0.28] # 机器人坐标
init_robot_ori = [-0.4, 0.9, 0.0, 0.0] # 机器人方向
joint_name_robot = {} # 机器人关节名
link_name_robot = {} # 机器人部件名

def useRobot(): # 仿真机器人函数
    robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                            basePosition = init_robot_pos,
                            baseOrientation = init_robot_ori,
                            flags = p.URDF_USE_SELF_COLLISION or p.URDF_USE_INERTIA_FROM_FILE,
                            useFixedBase = 0,
                            )
    for i in range(p.getNumJoints(robot_urdf)): # 获取机器人关节信息
        info = p.getJointInfo(robot_urdf, i)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        if jointType == p.JOINT_REVOLUTE:
            joint_name_robot[jointName] = info[0]
            link_name_robot[info[12].decode('UTF-8')] = info[0]
            if draw_message: # 显示关节名称
                p.addUserDebugText(str(jointName), [0, 0, 0],
                                    parentObjectUniqueId = robot_urdf,
                                    parentLinkIndex = i,
                                    textColorRGB = [1, 0, 0])
    if draw_interia: # 绘制机器人模型部件link转动惯量
        drawInertiaBox(robot_urdf, -1, [1, 0, 0])
        for i in range(p.getNumJoints(robot_urdf)):
            drawInertiaBox(robot_urdf, i, [0, 1, 0])
    if draw_sphere: # 绘制机器人模型部件link质心小球
        drawLinkSphere(-1, p.getBasePositionAndOrientation(robot_urdf)[0][:3])
        for i in range(p.getNumJoints(robot_urdf)):
            drawLinkSphere(i, p.getLinkState(robot_urdf, i)[0][:3])
    pos_joint_robot = np.zeros(16) # 关节角度控制信息
    f = open("/home/zjunlict-vision-1/Desktop/bullet_robot/my_motion/back_climb.txt","r") # 读取后躺爬起数据
    init_flag = 1 # 竖直躺平sleep2s
    endi_flag = 1 # 竖直站立sleep2s
    gait_flag = 0 # 步态函数运行间隔帧数
    line_string = f.readline()
    while True:
        walkForward(pos_joint_robot, q_param.BB.now_gait)
        # if line_string: # 从后躺状态爬起站立
        #     line_data = climbUp(line_string) # 分析motion爬起数据得到关节控制信号
        #     pos_joint_robot = transControl(line_data) # 转换bullet仿真和motion控制中的关节控制信号正负区别
        #     if init_flag: # 爬起开始前,竖直躺平sleep2s
        #         time.sleep(2)
        #         init_flag = 0
        #     line_string = f.readline()
        # else:
        #     if endi_flag: # 爬起结束后,竖直站立sleep2s
        #         time.sleep(2)
        #         endi_flag = 0
        #         f.close()
        if not endi_flag: # 开始向前行走
            if gait_flag > 10: # 每隔10帧执行一次步态函数
                # pos_joint_robot = walkForward(pos_joint_robot, q_param.BB.now_gait)
                gait_flag = 0
            gait_flag += 1
        setControl(robot_urdf, joint_name_robot, pos_joint_robot) # 设置关节控制器
        if useRealTimeSim == 0: # 非实时仿真
            p.stepSimulation() # 在单个正向动力学模拟步骤中执行所有操作,例如碰撞检测,约束求解和积分
            if use_camera: # 开启摄像头
                useCamera(robot_urdf)
            time.sleep(0.01)
def drawInertiaBox(parentUid, parentLinkIndex, color): # 绘制转动惯量函数
    dyn = p.getDynamicsInfo(parentUid, parentLinkIndex)
    mass = dyn[0]
    frictionCoeff = dyn[1]
    inertia = dyn[2]
    if (mass > 0):
        Ixx = inertia[0]
        Iyy = inertia[1]
        Izz = inertia[2]
        boxScaleX = 0.5 * math.sqrt(6 * (Izz + Iyy - Ixx) / mass)
        boxScaleY = 0.5 * math.sqrt(6 * (Izz + Ixx - Iyy) / mass)
        boxScaleZ = 0.5 * math.sqrt(6 * (Ixx + Iyy - Izz) / mass)
        halfExtents = [boxScaleX, boxScaleY, boxScaleZ]
        pts = [[halfExtents[0], halfExtents[1], halfExtents[2]],
               [-halfExtents[0], halfExtents[1], halfExtents[2]],
               [halfExtents[0], -halfExtents[1], halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], halfExtents[2]],
               [halfExtents[0], halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], halfExtents[1], -halfExtents[2]],
               [halfExtents[0], -halfExtents[1], -halfExtents[2]],
               [-halfExtents[0], -halfExtents[1], -halfExtents[2]]]
        p.addUserDebugLine(pts[0],
                           pts[1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[0],
                           pts[4],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[1],
                           pts[5],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[2],
                           pts[6],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[3],
                           pts[7],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)

        p.addUserDebugLine(pts[4 + 0],
                           pts[4 + 1],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 1],
                           pts[4 + 3],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 3],
                           pts[4 + 2],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
        p.addUserDebugLine(pts[4 + 2],
                           pts[4 + 0],
                           color,
                           1,
                           parentObjectUniqueId=parentUid,
                           parentLinkIndex=parentLinkIndex)
def drawLinkSphere(i, link_pos): # 绘制质心小球函数
    p.loadURDF(r'dancer_urdf_model/model/sphere_1cm.urdf',
                link_pos,
                useMaximalCoordinates=True,
                useFixedBase=1)
def useCamera(id): # 开启摄像头函数
    pos, ori = [list(l) for l in p.getBasePositionAndOrientation(id)]
    pos[2] = 0.3
    pos[0] += 0.25
    rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
    camera_vec = np.matmul(rot_mat, [1, 0, 0])
    up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
    view_matrix = p.computeViewMatrix(cameraEyePosition=pos, cameraTargetPosition=pos+camera_vec, cameraUpVector=up_vec)
    proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1, nearVal=0.01, farVal=100)
    w, h, rgb, depth, segmask = p.getCameraImage(width=84, height=84, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    rgb = rgb[:, :, :-1] / 255.
    rgbd = np.concatenate([rgb, np.expand_dims(depth, 2)], axis=2)
def setControl(robot_name, joint_name, joint_pos): # 设置关节控制器函数
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_body_hip_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[0],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_body_hip2_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[1],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_leg_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[2],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_leg2_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[3],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_foot1_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[4],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_foot2_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[5],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_body_hip_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[6],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_body_hip2_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[7],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_leg_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[8],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_leg2_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[9],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_foot1_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[10],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_foot2_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[11],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_arm_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[12],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_hand_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[13],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_arm_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[14],
                            force=10
                            )
    p.setJointMotorControl2(bodyUniqueId=robot_name,
                            jointIndex=joint_name['joint_hand_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=joint_pos[15],
                            force=10
                            )
def transControl(line_data, joint_num = 16): # 关节控制正负转换函数
    # 关节控制正负转换函数，用于转换bullet仿真和motion控制中的关节控制信号正负区别
    # @param line_data 输入motion关节控制信号
    # @param joint_num 输入关节个数，默认16个
    # @return pos_joint 返回bullet关节控制信号
    pos_joint = np.zeros(16) # 关节角度控制信息
    for i in range(joint_num):
        pos_joint[0]  =  line_data[0] # body_hip_right
        pos_joint[1]  =  line_data[1] # body_hip2_right
        pos_joint[2]  = -line_data[2] # leg_right
        pos_joint[3]  =  line_data[3] # leg2_right
        pos_joint[4]  = -line_data[4] # foot1_right
        pos_joint[5]  =  line_data[5] # foot2_right
        pos_joint[6]  =  line_data[6] # body_hip_left
        pos_joint[7]  =  line_data[7] # body_hip2_left
        pos_joint[8]  = -line_data[8] # leg_left
        pos_joint[9]  =  line_data[9] # leg2_left
        pos_joint[10] = -line_data[10] # foot1_left
        pos_joint[11] =  line_data[11] # foot2_left
        pos_joint[12] = -line_data[12] # arm_right
        pos_joint[13] = -line_data[13] # hand_right
        pos_joint[14] = -line_data[14] # arm_left
        pos_joint[15] = -line_data[15] # hand_left
    return pos_joint
def climbUp(line_string): # 平躺爬起函数
    # climb函数，用于分析爬起数据得到关节控制信号
    # @param line_string 输入读取当前行数据
    # @return line_data 返回关节控制信号
    line_str = line_string.split(' ')
    while '' in line_str:
        line_str.remove('')
    line_str.pop(-1) # 删除时间戳
    line_data = list(map(float,line_str))
    for index in range(len(line_data)): # 角度制转换弧度制
        line_data[index] = math.radians(line_data[index])
    return line_data
def walkForward(joint_pos, walk_gait): # 向前行走函数
    # walkp函数，用于生成下一帧的动作数据
    # @param joint_pos 
    # @param walk_gait 
    # @return new_joint_pos
    new_joint_pos = np.zeros(len(joint_pos))

    base_upbody = [0, 0, 0]
    base_com = [q_param.PendulumWalkParam.COM_X_OFFSET, -q_param.PendulumWalkParam.ANKLE_DIS / 2.0, q_param.PendulumWalkParam.COM_HEIGHT]
    base_ankle = [0, -q_param.PendulumWalkParam.ANKLE_DIS, 0, 0, 0, 0]
    base_angles = q_param.OneFootLanding.GetOneStep(base_ankle, base_com, base_upbody)
    # new_joint_pos = base_angles
    # print('base_com', base_ankle)
    # new_joint_pos.append(q_param.)




    giveAStepTick(walk_gait)
    return new_joint_pos
def giveAStep(dx_input, dy_input, dyaw_input): # 下一步动作数据函数
    # step函数，用于生成下一步的动作数据
    # @param dx_input 下一步质心的x变化，相对于上半身，单位是cm
    # @param dy_input 下一步执行的y变化，相对于上半身，单位是cm
    # @param dyaw_input 下一步执行的yaw变化，相对于上半身，角度制
    # @return
    q_param.AA.x0 = q_param.BB.com_pos[0] * math.cos(q_param.BB.hang_foot[2]) - q_param.BB.hang_foot[0] * math.cos(q_param.BB.hang_foot[2]) + q_param.BB.com_pos[1] * math.sin(q_param.BB.hang_foot[2]) - q_param.BB.hang_foot[1] * math.sin(q_param.BB.hang_foot[2])
    q_param.AA.xt = (dx_input - ( (q_param.PendulumWalkParam.ANKLE_DIS / 2.0) if q_param.BB.support_is_right else (-q_param.PendulumWalkParam.ANKLE_DIS / 2.0) ) * math.sin(math.radians(dyaw_input))) / 2.0
    q_param.AA.tao = q_param.PendulumWalkParam.TAO
    # 算出摆的周期常数，这里的com_h暂时是由机器人crouch姿态下倒挂着摆动测量得出的
    q_param.AA.com_h = q_param.PendulumWalkParam.COM_H
    q_param.AA.Tc = math.sqrt(q_param.AA.com_h / 980)
    # 算出来这个步态单元的初速度vx
    q_param.AA.vx = (q_param.AA.xt - q_param.AA.x0 * math.cosh(q_param.AA.tao / q_param.AA.Tc)) / (q_param.AA.Tc * math.sinh(q_param.AA.tao / q_param.AA.Tc))
    # y方向的研究
    q_param.AA.y00 = q_param.BB.com_pos[1] * math.cos(q_param.BB.hang_foot[2]) - q_param.BB.hang_foot[1] * math.cos(q_param.BB.hang_foot[2]) - q_param.BB.com_pos[0] * math.sin(q_param.BB.hang_foot[2]) + q_param.BB.hang_foot[0] * math.sin(q_param.BB.hang_foot[2])
    q_param.AA.ytt = (dy_input + ( (q_param.PendulumWalkParam.ANKLE_DIS / 2.0) if q_param.BB.support_is_right else (-q_param.PendulumWalkParam.ANKLE_DIS / 2.0) ) + ( (q_param.PendulumWalkParam.ANKLE_DIS / 2.0) if q_param.BB.support_is_right else (-q_param.PendulumWalkParam.ANKLE_DIS / 2.0)) * math.cos(math.radians(dyaw_input)) ) / 2
    # 这里实际计算质心轨迹的是y0和yt，防止∆y太大
    q_param.AA.y0 = (q_param.PendulumWalkParam.Y_HALF_AMPLITUDE) if q_param.BB.support_is_right else (-q_param.PendulumWalkParam.Y_HALF_AMPLITUDE)
    q_param.AA.yt = (q_param.PendulumWalkParam.Y_HALF_AMPLITUDE) if q_param.BB.support_is_right else (-q_param.PendulumWalkParam.Y_HALF_AMPLITUDE)
    # 计算过程中换元得到的一个临时变量m
    q_param.AA.m = math.exp(q_param.AA.tao / q_param.AA.Tc)
    # 步行单元的周期定了后y方向的最大速度是确定的
    q_param.AA.vy = (q_param.AA.yt - q_param.AA.m * q_param.AA.y0) / ( (q_param.AA.m + 1) * q_param.AA.Tc)
    # 使用插值算法定义抬脚的时间位移曲线
    akZ_t = q_param.PendulumWalkParam.foot_z_t
    akZ_p = q_param.PendulumWalkParam.foot_z_p
    akZ_s = q_param.PendulumWalkParam.foot_z_s
    ankle_z = q_param.threeInterPolation(akZ_t, akZ_p, akZ_s)
    q_param.AA.akZ = ankle_z.getPoints()
    # 线性规划上半身yaw
    comYaw_t = [0, q_param.PendulumWalkParam.TAO]
    comYaw_p = [math.degrees(q_param.BB.com_pos[2] - q_param.BB.hang_foot[2]), dyaw_input / 2.0]
    slope_comYaw = (dyaw_input / 2.0 - math.degrees(q_param.BB.com_pos[2] - q_param.BB.hang_foot[2])) / q_param.PendulumWalkParam.TAO
    comYaw_s = [slope_comYaw, slope_comYaw]
    com_yaw = q_param.threeInterPolation(comYaw_t, comYaw_p, comYaw_s)
    q_param.AA.comYaw = com_yaw.getPoints()
    # 线性规划质心的y基础位置
    comY_t = [0, q_param.PendulumWalkParam.TAO]
    comY_p = [q_param.AA.y00, q_param.AA.ytt]
    slope_comY = (q_param.AA.ytt - q_param.AA.y00) / q_param.PendulumWalkParam.TAO
    comY_s = [slope_comY, slope_comY]
    com_y = q_param.threeInterPolation(comY_t, comY_p, comY_s)
    q_param.AA.comY = com_y.getPoints()
    # 线性规划质心加速度x偏移
    ac_x = (dx_input - q_param.BB.com_x_changed) * q_param.PendulumWalkParam.ACC_COEF_X
    accX_t = [0, q_param.PendulumWalkParam.TAO]
    accX_p = [q_param.BB.com_ac_x, ac_x]
    slope_accX = (ac_x - q_param.BB.com_ac_x) / q_param.PendulumWalkParam.TAO
    accX_s = [slope_accX, slope_accX]
    acc_x = q_param.threeInterPolation(accX_t, accX_p, accX_s)
    q_param.AA.accX = acc_x.getPoints()
    # 线性规划质心加速度y偏移
    ac_y = (dy_input - q_param.BB.com_y_changed) * q_param.PendulumWalkParam.ACC_COEF_Y
    accY_t = [0, q_param.PendulumWalkParam.TAO]
    accY_p = [q_param.BB.com_ac_y, ac_y]
    slope_accY = (ac_y - q_param.BB.com_ac_y) / q_param.PendulumWalkParam.TAO
    accY_s = [slope_accY, slope_accY]
    acc_y = q_param.threeInterPolation(accY_t, accY_p, accY_s)
    q_param.AA.accY = acc_y.getPoints()
    # 线性规划脚踝的x方向起点终点的插值
    ak_x_0 = -q_param.BB.hang_foot[0] * math.cos(q_param.BB.hang_foot[2]) - q_param.BB.hang_foot[1] * math.sin(q_param.BB.hang_foot[2])
    ak_x_t = 2 * q_param.AA.xt
    akX_t = [0, q_param.PendulumWalkParam.TAO]
    akX_p = [ak_x_0, ak_x_t]
    slope_akX = (ak_x_t - ak_x_0) / q_param.PendulumWalkParam.TAO
    akX_s = [slope_akX, slope_akX]
    ak_x = q_param.threeInterPolation(akX_t, akX_p, akX_s)
    q_param.AA.akX = ak_x.getPoints()
    # 线性规划脚踝的y方向起点终点的插值
    ak_y_0 = q_param.BB.hang_foot[0] * math.sin(q_param.BB.hang_foot[2]) - q_param.BB.hang_foot[1] * math.cos(q_param.BB.hang_foot[2])
    ak_y_t = 2 * q_param.AA.ytt
    akY_t = [0, q_param.PendulumWalkParam.TAO]
    akY_p = [ak_y_0, ak_y_t]
    slope_akY = (ak_y_t - ak_y_0) / q_param.PendulumWalkParam.TAO
    akY_s = [slope_akY, slope_akY]
    ak_y = q_param.threeInterPolation(akY_t, akY_p, akY_s)
    q_param.AA.akY = ak_y.getPoints()
    # 线性规划脚踝的yaw起点终点的插值
    akYaw_t = [0, q_param.PendulumWalkParam.TAO]
    akYaw_p = [math.degrees(-q_param.BB.hang_foot[2]), dyaw_input]
    slope_akYaw = (akYaw_p[1] - akYaw_p[0]) / q_param.PendulumWalkParam.TAO / 2
    akYaw_s = [slope_akYaw, slope_akYaw]
    ak_yaw = q_param.threeInterPolation(akYaw_t, akYaw_p, akYaw_s)
    q_param.AA.akYaw = ak_yaw.getPoints()
    # 为下一步的数据做准备
    q_param.BB.support_is_right = 1 - q_param.BB.support_is_right
    q_param.BB.com_x_changed = dx_input
    q_param.BB.com_y_changed = dy_input
    q_param.BB.com_pos = []
    q_param.BB.com_pos.append(q_param.AA.xt)
    q_param.BB.com_pos.append(q_param.AA.ytt)
    q_param.BB.com_pos.append(math.radians(dyaw_input) / 2.0)
    q_param.BB.hang_foot = []
    q_param.BB.hang_foot.append(ak_x_t)
    q_param.BB.hang_foot.append(ak_y_t)
    q_param.BB.hang_foot.append(2.0 * q_param.BB.com_pos[2]) # 这里使得落脚点的yaw和质心一致了
    q_param.BB.com_ac_x = ac_x
    q_param.BB.com_ac_y = ac_y
    tick_num = 0
def giveATick(): # 下一帧动作数据函数
    # tick函数，用于用于生成下一帧的动作数据
    tmptick =  q_param.MotionTick()
    q_param.AA.x = q_param.AA.x0 * math.cosh(0.01 * q_param.AA.tick_num / q_param.AA.Tc) + q_param.AA.Tc * q_param.AA.vx * math.sinh(0.01 * (q_param.AA.tick_num) / q_param.AA.Tc)
    q_param.AA.y = q_param.AA.y0 * math.cosh(0.01 * q_param.AA.tick_num / q_param.AA.Tc) + q_param.AA.Tc * q_param.AA.vy * math.sinh(0.01 * (q_param.AA.tick_num) / q_param.AA.Tc)
    tmptick.upbody_pose.append(0)
    tmptick.upbody_pose.append(0)
    tmptick.upbody_pose.append(q_param.AA.comYaw[q_param.AA.tick_num])
    tmptick.whole_com.append(q_param.AA.accX[q_param.AA.tick_num] + q_param.AA.x + q_param.PendulumWalkParam.COM_X_OFFSET) # (x * 100 +1.5)
    tmptick.whole_com.append(q_param.AA.y - q_param.AA.y0 + q_param.AA.comY[q_param.AA.tick_num] + q_param.AA.accY[q_param.AA.tick_num])
    tmptick.whole_com.append(q_param.PendulumWalkParam.COM_HEIGHT) # 0.308637
    tmptick.hang_foot.append(q_param.AA.akX[q_param.AA.tick_num])
    tmptick.hang_foot.append(q_param.AA.akY[q_param.AA.tick_num])
    tmptick.hang_foot.append(q_param.AA.akZ[q_param.AA.tick_num])
    tmptick.hang_foot.append(0)
    tmptick.hang_foot.append(0)
    tmptick.hang_foot.append(q_param.AA.akYaw[q_param.AA.tick_num])
    q_param.AA.tick_num += 1
def giveAStepTick(give_gait): # 步态规划函数
    # steptick函数，调用step函数和tick函数
    # @param give_gait 单个步态的实例
    giveAStep(give_gait.X, give_gait.Y, give_gait.YAW) # 根据单个步态，计算运动参数
    q_param.AA.tick_num = 0
    for i in range(q_param.PendulumWalkParam.TICK_NUM):
        giveATick() # 根据运动参数，计算每一帧的运动指令




















# 机器人仿真
if use_robot:
    useRobot()
