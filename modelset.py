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

# 连接物理引擎
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 设置重力
p.setGravity(0, 0, -98)
# 实时仿真
useRealTimeSim = 0
p.setRealTimeSimulation(useRealTimeSim)
# 设置地面
# plane = p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0, plane)
# 加载默认地面
planeId = p.loadURDF("plane.urdf")
# 加载足球场地面
# planeId = p.loadSDF("stadium.sdf")
# 设置地面摩擦力
p.changeDynamics(planeId,-1,lateralFriction = 0.6,spinningFriction = 0.3,rollingFriction = 0.0001)
# 设置相机
p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=50.0, cameraPitch=-23.80,
                                cameraTargetPosition=[-1.0, 1.0, -0.5], physicsClientId=physicsClient)
# 各种参数
use_robot = 1                       # 显示机器人模型
use_car = 0                         # 显示小车模型
use_rozen = 1 - use_robot           # 显示我的模型
use_camera = 0                      # 开启摄像头
draw_interia = 1                    # 绘制转动惯量
draw_sphere  = 0                    # 绘制质心小球
robotPos = [0, 0, 0.28]             # 机器人坐标
robotOri = [0.4, -0.6, 0.6, -0.4]   # 机器人方向(站立)
# robotOri = [-0.4, 0.9, 0.0, 0.0]    # 机器人方向(后躺)
jointNameToID_robot = {}            # 机器人关节名
linkNameToID_robot = {}             # 机器人部件名

# 仿真机器人函数
def useRobot():
    # 加载机器人模型(相对路径)(转动惯量URDF_USE_INERTIA_FROM_FILE,碰撞URDF_USE_SELF_COLLISION)
    robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                            basePosition = robotPos,
                            baseOrientation = robotOri,
                            # flags = p.URDF_USE_INERTIA_FROM_FILE,
                            flags = p.URDF_USE_SELF_COLLISION or p.URDF_USE_INERTIA_FROM_FILE,
                            useFixedBase = 0,
                            )
    # 获取机器人关节信息
    for i in range(p.getNumJoints(robot_urdf)):
        info = p.getJointInfo(robot_urdf, i)
        # print(info)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        if jointType == p.JOINT_REVOLUTE:
            jointNameToID_robot[jointName] = info[0]
            linkNameToID_robot[info[12].decode('UTF-8')] = info[0]
            # 显示关节名称
            p.addUserDebugText(str(jointName),
                                [0, 0, 0],
                                parentObjectUniqueId = robot_urdf,
                                parentLinkIndex = i,
                                textColorRGB = [1, 0, 0])
    # 查看世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
    cube_pos, cube_orn = p.getBasePositionAndOrientation(robot_urdf)
    # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
    cube_euler = p.getEulerFromQuaternion(cube_orn)
    # 显示debug信息
    p.addUserDebugText(str(cube_orn),
                        [0, 0, 0],
                        textSize = 1,
                        parentObjectUniqueId = robot_urdf,
                        parentLinkIndex = -1,
                        textColorRGB = [0.3, 0.3, 0.3])
    # 绘制机器人模型部件link转动惯量
    if draw_interia:
        drawInertiaBox(robot_urdf, -1, [1, 0, 0])
        for i in range(p.getNumJoints(robot_urdf)):
            drawInertiaBox(robot_urdf, i, [0, 1, 0])
    # 绘制机器人模型部件link质心小球
    if draw_sphere:
        drawLinkSphere(-1, p.getBasePositionAndOrientation(robot_urdf)[0][:3])
        for i in range(p.getNumJoints(robot_urdf)):
            drawLinkSphere(i, p.getLinkState(robot_urdf, i)[0][:3])
    # 设置机器人直立的关节参数
    # ini_body_head = 0
    # ini_body_head2 = 0
    # ini_arm_left = 1.7
    # ini_hand_left = 0.2
    # ini_arm_right = 0.2
    # ini_hand_right = 0.2
    # ini_body_hip = 0
    # ini_body_hip_left = 0
    # ini_body_hip2_left = 0
    # ini_body_hip_right = 0
    # ini_body_hip2_right = -0.7
    # ini_leg_left = -1.7
    # ini_leg2_left = 1
    # ini_foot1_left = -0.2
    # ini_foot2_left = 0
    # ini_leg_right = 0.5
    # ini_leg2_right = 1
    # ini_foot1_right = -1
    # ini_foot2_right = -1
    ini_body_head = 0
    ini_body_head2 = 0
    ini_arm_left = 0
    ini_hand_left = 0
    ini_arm_right = 0
    ini_hand_right = 0
    # ini_body_hip = 0
    # ini_body_hip_left = 0
    # ini_body_hip2_left = 0
    # ini_body_hip_right = 0
    # ini_body_hip2_right = 0
    ini_leg_left = 0
    ini_leg2_left = 0
    ini_foot1_left = 0
    # ini_foot2_left = 0
    ini_leg_right = 0
    ini_leg2_right = 0
    ini_foot1_right = 0
    # ini_foot2_right = 0
    # 设置控制滑块，参数分别是最小值，最大值，当前值
    pos_body_head_slider    = p.addUserDebugParameter("pos_body_head", -10, 10, ini_body_head)
    pos_body_head2_slider   = p.addUserDebugParameter("pos_body_head2", -10, 10, ini_body_head2)
    vel_arm_left_slider     = p.addUserDebugParameter("vel_arm_left", -10, 10, ini_arm_left)
    vel_hand_left_slider    = p.addUserDebugParameter("vel_hand_left", -10, 10, ini_hand_left)
    vel_arm_right_slider    = p.addUserDebugParameter("vel_arm_right", -10, 10, ini_arm_right)
    vel_hand_right_slider   = p.addUserDebugParameter("vel_hand_right", -10, 10, ini_hand_right)
    # vel_body_hip_slider     = p.addUserDebugParameter("vel_body_hip", -10, 10, ini_body_hip)
    # vel_body_hip_left_slider    = p.addUserDebugParameter("vel_body_hip_left", -10, 10, ini_body_hip_left)
    # vel_body_hip2_left_slider   = p.addUserDebugParameter("vel_body_hip2_left", -10, 10, ini_body_hip2_left)
    # vel_body_hip_right_slider   = p.addUserDebugParameter("vel_body_hip_right", -10, 10, ini_body_hip_right)
    # vel_body_hip2_right_slider  = p.addUserDebugParameter("vel_body_hip2_right", -10, 10, ini_body_hip2_right)
    vel_leg_left_slider     = p.addUserDebugParameter("vel_leg_left", -10, 10, ini_leg_left)
    vel_leg2_left_slider    = p.addUserDebugParameter("vel_leg2_left", -10, 10, ini_leg2_left)
    vel_foot1_left_slider    = p.addUserDebugParameter("vel_foot1_left", -10, 10, ini_foot1_left)
    # vel_foot2_left_slider    = p.addUserDebugParameter("vel_foot2_left", -10, 10, ini_foot2_left)
    vel_leg_right_slider    = p.addUserDebugParameter("vel_leg_right", -10, 10, ini_leg_right)
    vel_leg2_right_slider   = p.addUserDebugParameter("vel_leg2_right", -10, 10, ini_leg2_right)
    vel_foot1_right_slider   = p.addUserDebugParameter("vel_foot1_right", -10, 10, ini_foot1_right)
    # vel_foot2_right_slider   = p.addUserDebugParameter("vel_foot2_right", -10, 10, ini_foot2_right)
    reset_all_slider        = p.addUserDebugParameter("reset_all", -10, 10, 0)
    while True:
        # 查看世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        # cube_pos, cube_orn = p.getBasePositionAndOrientation(robot_urdf)
        # 输出debug信息
        # print('\r', cube_orn, end='', flush=True)
        # 读取控制滑块数据
        pos_body_head   = p.readUserDebugParameter(pos_body_head_slider)
        pos_body_head2  = p.readUserDebugParameter(pos_body_head2_slider)
        vel_arm_left    = p.readUserDebugParameter(vel_arm_left_slider)
        vel_hand_left   = p.readUserDebugParameter(vel_hand_left_slider)
        vel_arm_right   = p.readUserDebugParameter(vel_arm_right_slider)
        vel_hand_right  = p.readUserDebugParameter(vel_hand_right_slider)
        # vel_body_hip    = p.readUserDebugParameter(vel_body_hip_slider)
        # vel_body_hip_left   = p.readUserDebugParameter(vel_body_hip_left_slider)
        # vel_body_hip2_left  = p.readUserDebugParameter(vel_body_hip2_left_slider)
        # vel_body_hip_right  = p.readUserDebugParameter(vel_body_hip_right_slider)
        # vel_body_hip2_right = p.readUserDebugParameter(vel_body_hip2_right_slider)
        vel_leg_left    = p.readUserDebugParameter(vel_leg_left_slider)
        vel_leg2_left   = p.readUserDebugParameter(vel_leg2_left_slider)
        vel_foot1_left   = p.readUserDebugParameter(vel_foot1_left_slider)
        # vel_foot2_left   = p.readUserDebugParameter(vel_foot2_left_slider)
        vel_leg_right   = p.readUserDebugParameter(vel_leg_right_slider)
        vel_leg2_right  = p.readUserDebugParameter(vel_leg2_right_slider)
        vel_foot1_right  = p.readUserDebugParameter(vel_foot1_right_slider)
        # vel_foot2_right  = p.readUserDebugParameter(vel_foot2_right_slider)
        reset_all       = p.readUserDebugParameter(reset_all_slider)
        # 重置每一个关节的位置
        if reset_all < 0:
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_head'], ini_body_head)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_head2'], ini_body_head2)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_arm_left'], ini_arm_left)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_hand_left'], ini_hand_left)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_arm_right'], ini_arm_right)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_hand_right'], ini_hand_right)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_hip'], ini_body_hip)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_hip_left'], ini_body_hip_left)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_hip2_left'], ini_body_hip2_left)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_hip_right'], ini_body_hip_right)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_body_hip2_right'], ini_body_hip2_right)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_leg_left'], ini_leg_left)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_leg2_left'], ini_leg2_left)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_foot1_left'], ini_foot1_left)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_foot2_left'], ini_foot2_left)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_leg_right'], ini_leg_right)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_leg2_right'], ini_leg2_right)
            p.resetJointState(robot_urdf, jointNameToID_robot['joint_foot1_right'], ini_foot1_right)
            # p.resetJointState(robot_urdf, jointNameToID_robot['joint_foot2_right'], ini_foot2_right)
            # for joint_name in jointNameToID_robot:
            #     print(joint_name)
            #     p.resetJointState(robot_urdf, jointNameToID_robot[joint_name], 0)
        # 设置关节控制器
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_body_head'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=pos_body_head,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_body_head2'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=pos_body_head2,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_arm_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_arm_left,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_hand_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_hand_left,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_arm_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_arm_right,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_hand_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_hand_right,
                            force=10
                            )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_body_hip'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_body_hip,
        #                     force=10
        #                     )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_body_hip_left'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_body_hip_left,
        #                     force=10
        #                     )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_body_hip2_left'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_body_hip2_left,
        #                     force=10
        #                     )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_body_hip_right'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_body_hip_right,
        #                     force=10
        #                     )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_body_hip2_right'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_body_hip2_right,
        #                     force=10
        #                     )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_leg_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_leg_left,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_leg2_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_leg2_left,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_foot1_left'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_foot1_left,
                            force=10
                            )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_foot2_left'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_foot2_left,
        #                     force=10
        #                     )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_leg_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_leg_right,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_leg2_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_leg2_right,
                            force=10
                            )
        p.setJointMotorControl2(bodyUniqueId=robot_urdf,
                            jointIndex=jointNameToID_robot['joint_foot1_right'],
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=vel_foot1_right,
                            force=10
                            )
        # p.setJointMotorControl2(bodyUniqueId=robot_urdf,
        #                     jointIndex=jointNameToID_robot['joint_foot2_right'],
        #                     controlMode=p.POSITION_CONTROL,
        #                     targetPosition=vel_foot2_right,
        #                     force=10
        #                     )
        # 非实时仿真
        if useRealTimeSim == 0:
            # 在单个正向动力学模拟步骤中执行所有操作，例如碰撞检测，约束求解和积分
            p.stepSimulation()
            # 查看机器人位姿,state = 
            # robot_state = p.getBasePositionAndOrientation(robot_urdf)
            # 查看关节参数,state = (关节角度vec1,关节角速度vec1,关节力与力矩vec6,关节电机转矩vec1)
            # joint_state = p.getJointState(robot_urdf, jointNameToID_robot['joint_arm_left'])
            # 查看部件参数,state = (质心坐标vec3,质心朝向vec4,,,部件坐标vec3,部件坐标vec4,部件速度vec3,部件角速度vec3)
            # link_state = p.getLinkState(robot_urdf, linkNameToID_robot['arm_left'])
            # print('\r', ?_state[4][2], end='', flush=True)
            # 开启摄像头
            if use_camera:
                useCamera(robot_urdf)
            time.sleep(0.01)

# 仿真小车函数
def useCar():
    # 加载默认小车(相对路径)
    car = p.loadURDF("racecar/racecar.urdf")
    # 设置小车从动轮
    inactive_wheels = [3, 5, 7]
    for wheel in inactive_wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=0,
                                force=0)
    # 设置小车主动轮
    wheels = [2]
    # 设置小车转向轮
    steering = [4, 6]
    # 自定义参数滑块，分别为速度，转向角度，驱动力参数
    targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0)
    steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0)
    maxForceSlider = p.addUserDebugParameter("maxForce", 0, 10, 10)
    # 开始仿真
    while 1:
        # 读取速度，转向角度，驱动力参数
        maxForce = p.readUserDebugParameter(maxForceSlider)
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        steeringAngle = p.readUserDebugParameter(steeringSlider)
        # 根据上面读取到的值对小车主动轮进行设置
        for wheel in wheels:
            p.setJointMotorControl2(car,
                                    wheel,
                                    p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocity,
                                    force=maxForce)
        # 根据上面读取到的值对小车转向轮进行设置
        for steer in steering:
            p.setJointMotorControl2(car,
                                    steer,
                                    p.POSITION_CONTROL,
                                    targetPosition=steeringAngle)
        # 非实时仿真
        if useRealTimeSim == 0:
            # 在单个正向动力学模拟步骤中执行所有操作，例如碰撞检测，约束求解和积分
            p.stepSimulation()

# 仿真模型函数
def useRozen():
    rozen_urdf = p.loadURDF(r'dancer_urdf_model/model/rozen_urdf_model.URDF',
                            basePosition = robotPos,
                            baseOrientation = robotOri,
                            flags = p.URDF_USE_INERTIA_FROM_FILE,
                            useFixedBase = 0)
    while True:
        if useRealTimeSim == 0:
            p.stepSimulation()
            time.sleep(0.01)

# 绘制转动惯量函数
def drawInertiaBox(parentUid, parentLinkIndex, color):
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

# 绘制质心小球函数
def drawLinkSphere(i, link_pos):
    p.loadURDF(r'dancer_urdf_model/model/sphere_1cm.urdf',
                link_pos,
                useMaximalCoordinates=True,
                useFixedBase=1)

# 开启摄像头函数
def useCamera(id):
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

# 仿真机器人
if use_robot:
    useRobot()

# 仿真小车
if use_car:
    useCar()

# 仿真模型
if use_rozen:
    useRozen()