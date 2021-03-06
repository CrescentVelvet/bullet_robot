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

physicsClient = p.connect(p.GUI)
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
    f = open("/home/zjunlict-vision-1/Desktop/bullet_robot/my_motion/back_climb.txt","r") # 读取数据
    init_flag = 1 # 竖直躺平sleep2s
    endi_flag = 1 # 竖直站立sleep2s
    line_string = f.readline()
    while True:
        if line_string: # 从后躺状态爬起站立
            line_str = line_string.split(' ')
            while '' in line_str:
                line_str.remove('')
            line_str.pop(-1) # 删除时间戳
            line_data = list(map(float,line_str))
            for index in range(len(line_data)): # 角度制转换弧度制
                line_data[index] = math.radians(line_data[index])
            pos_joint_robot[0] = line_data[0] # body_hip_right
            pos_joint_robot[1] = line_data[1] # body_hip2_right
            pos_joint_robot[2] = -line_data[2] # leg_right
            pos_joint_robot[3] = line_data[3] # leg2_right
            pos_joint_robot[4] = -line_data[4] # foot1_right
            pos_joint_robot[5] = line_data[5] # foot2_right
            pos_joint_robot[6] = line_data[6] # body_hip_left
            pos_joint_robot[7] = line_data[7] # body_hip2_left
            pos_joint_robot[8] = -line_data[8] # leg_left
            pos_joint_robot[9] = line_data[9] # leg2_left
            pos_joint_robot[10] = -line_data[10] # foot1_left
            pos_joint_robot[11] = line_data[11] # foot2_left
            pos_joint_robot[12] = -line_data[12] # arm_right
            pos_joint_robot[13] = -line_data[13] # hand_right
            pos_joint_robot[14] = -line_data[14] # arm_left
            pos_joint_robot[15] = -line_data[15] # hand_left
            if init_flag: # 爬起开始前,竖直躺平sleep2s
                time.sleep(2)
                init_flag = 0
            line_string = f.readline()
        else:
            if endi_flag: # 爬起结束后,竖直站立sleep2s
                time.sleep(2)
                endi_flag = 0
                f.close()
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
# 机器人仿真
if use_robot:
    useRobot()
