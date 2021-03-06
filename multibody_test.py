import time
import pybullet as p
import pybullet_data

# 连接物理引擎
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# 设置重力
p.setGravity(0, 0, -10)
# 实时仿真
useRealTimeSim = 0
p.setRealTimeSimulation(useRealTimeSim)
# 设置地面
# plane = p.createCollisionShape(p.GEOM_PLANE)
# p.createMultiBody(0, plane)
# 加载默认地面
# p.loadURDF("plane.urdf")
# 加载足球场地面
p.loadSDF("stadium.sdf")
# 加载默认小车
car = p.loadURDF("racecar/racecar.urdf")
# 设置相机
p.resetDebugVisualizerCamera(cameraDistance=1.30, cameraYaw=50.0, cameraPitch=-23.80,
                             cameraTargetPosition=[-0.65, 0.49, -0.25], physicsClientId=physicsClient)
# 设置小车从动轮
inactive_wheels = [3, 5, 7]
# 设置小车主动轮
wheels = [2]
for wheel in inactive_wheels:
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
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
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

    if useRealTimeSim == 0:
        p.stepSimulation()



# use_1 = 0
# use_2 = 1
# use_robot = 1
# useConstraint = 1
# robotPos_1 = [0, 0.00302, 0.37853]
# robotPos_2 = [0, 0, 0]
# pos_robot = [0, 0, 1]
# jointNameToID_1 = {}
# linkNameToID_1 = {}
# revoluteID_1 = []
# jointNameToID_2 = {}
# linkNameToID_2 = {}
# revoluteID_2 = []
# jointNameToID_robot = {}
# linkNameToID_robot = {}
# revoluteID_robot = []

# if use_1:
#     robot_1 = p.loadURDF(r'leg-1/urdf/leg-1.urdf',
#                          robotPos_1,
#                          useFixedBase=0,
#                          )
#     for j in range(p.getNumJoints(robot_1)):
#         info = p.getJointInfo(robot_1, j)
#         print(info)
#         jointID = info[0]
#         jointName = info[1].decode('UTF-8')
#         jointType = info[2]
#         if jointType == p.JOINT_REVOLUTE:
#             jointNameToID_1[jointName] = info[0]
#             linkNameToID_1[info[12].decode('UTF-8')] = info[0]
#             revoluteID_1.append(j)
#             p.addUserDebugText(str(jointName),
#                                [0, 0, 0],
#                                parentObjectUniqueId=robot_1,
#                                parentLinkIndex=j,
#                                textColorRGB=[1, 0, 0])
#     p.addUserDebugText("base_1",
#                        [0, 0, 0],
#                        parentObjectUniqueId=robot_1,
#                        parentLinkIndex=-1,
#                        textColorRGB=[1, 0, 0])
#     for i in range(len(linkNameToID_1)):
#         p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
#                            parentObjectUniqueId=robot_1, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
#                            parentObjectUniqueId=robot_1, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
#                            parentObjectUniqueId=robot_1, parentLinkIndex=i)
# if use_2:
#     robot_2 = p.loadURDF(r'leg-2/urdf/leg-2.urdf',
#                          robotPos_2,
#                          useFixedBase=1,
#                          )

#     for j in range(p.getNumJoints(robot_2)):
#         info = p.getJointInfo(robot_2, j)
#         print(info)
#         jointID = info[0]
#         jointName = info[1].decode('UTF-8')
#         jointType = info[2]
#         if jointType == p.JOINT_REVOLUTE:
#             jointNameToID_2[jointName] = info[0]
#             linkNameToID_2[info[12].decode('UTF-8')] = info[0]
#             revoluteID_2.append(j)
#             p.addUserDebugText(str(jointName),
#                                [0, 0, 0],
#                                parentObjectUniqueId=robot_2,
#                                parentLinkIndex=j,
#                                textColorRGB=[1, 0, 0])

#     p.addUserDebugText("base_2",
#                        [0, 0, 0],
#                        parentObjectUniqueId=robot_2,
#                        parentLinkIndex=-1,
#                        textColorRGB=[1, 0, 0])

#     for i in range(len(linkNameToID_2)):
#         p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
#                            parentObjectUniqueId=robot_2, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
#                            parentObjectUniqueId=robot_2, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
#                            parentObjectUniqueId=robot_2, parentLinkIndex=i)

# if use_robot:
#     # 导入模型
#     robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
#                          pos_robot,
#                          useFixedBase=1,
#                          )
#     for i in range(p.getNumJoints(robot_urdf)):
#         info = p.getJointInfo(robot_urdf, i)
#         print(info)
#         jointID = info[0]
#         jointName = info[1].decode('UTF-8')
#         jointType = info[2]
#         if jointType == p.JOINT_REVOLUTE:
#             jointNameToID_robot[jointName] = info[0]
#             linkNameToID_robot[info[12].decode('UTF-8')] = info[0]
#             revoluteID_robot.append(i)
#             p.addUserDebugText(str(jointName),
#                                [0, 0, 0],
#                                parentObjectUniqueId = robot_urdf,
#                                parentLinkIndex = i,
#                                textColorRGB = [1, 0, 0])
#     p.addUserDebugText('base_robot',
#                        [0, 0, 0],
#                        parentObjectUniqueId = robot_urdf,
#                        parentLinkIndex = i,
#                        textColorRGB = [1, 0, 0])
#     for j in range(len(linkNameToID_robot)):
#         p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
#                            parentObjectUniqueId=robot_urdf, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
#                            parentObjectUniqueId=robot_urdf, parentLinkIndex=i)
#         p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
#                            parentObjectUniqueId=robot_urdf, parentLinkIndex=i)

# if use_1 and use_2:
#     for i in range(len(linkNameToID_1)):
#         for j in range(len(linkNameToID_2)):
#             p.setCollisionFilterPair(robot_1, robot_2, i, j, 0)
#     pass

# 将机械腿的两部分连接起来
# if use_1 and use_2 and useConstraint:
#     constraintNum = 10
#     for i in range(constraintNum):
#         p.createConstraint(parentBodyUniqueId=robot_1,
#                            parentLinkIndex=linkNameToID_1["L_L_1"],
#                            childBodyUniqueId=robot_2,
#                            childLinkIndex=linkNameToID_2['L_L_2'],
#                            jointType=p.JOINT_POINT2POINT,
#                            jointAxis=[0, 0, 0],
#                            parentFramePosition=[
#                                0.055, 0, -0.005 + (0.01 / constraintNum) * i],
#                            childFramePosition=[
#                                0.03706, 0.00578, -0.005 + (0.01 / constraintNum) * i]
#                            )
#     for i in range(constraintNum):
#         p.createConstraint(parentBodyUniqueId=robot_1,
#                            parentLinkIndex=linkNameToID_1["L_L_3"],
#                            childBodyUniqueId=robot_2,
#                            childLinkIndex=linkNameToID_2["L_L_2"],
#                            jointType=p.JOINT_POINT2POINT,
#                            jointAxis=[0, 0, 0],
#                            parentFramePosition=[
#                                0.06, 0, -0.005 + (0.01 / constraintNum) * i],
#                            childFramePosition=[
#                                0.07595, -0.03311, -0.005 + (0.01 / constraintNum) * i, ]
#                            )
#     for i in range(constraintNum):
#         p.createConstraint(parentBodyUniqueId=robot_1,
#                            parentLinkIndex=linkNameToID_1["L_R_1"],
#                            childBodyUniqueId=robot_2,
#                            childLinkIndex=linkNameToID_2['L_R_2'],
#                            jointType=p.JOINT_POINT2POINT,
#                            jointAxis=[0, 0, 0],
#                            parentFramePosition=[
#                                0.055, 0, -0.005 + (0.01 / constraintNum) * i],
#                            childFramePosition=[
#                                0.03706, -0.00578, -0.005 + (0.01 / constraintNum) * i]
#                            )
#     for i in range(constraintNum):
#         p.createConstraint(parentBodyUniqueId=robot_1,
#                            parentLinkIndex=linkNameToID_1["L_R_3"],
#                            childBodyUniqueId=robot_2,
#                            childLinkIndex=linkNameToID_2["L_R_2"],
#                            jointType=p.JOINT_POINT2POINT,
#                            jointAxis=[0, 0, 0],
#                            parentFramePosition=[
#                                0.06, 0, 0.005 - (0.01 / constraintNum) * i],
#                            childFramePosition=[
#                                0.07595, 0.03311, -0.005 + (0.01 / constraintNum) * i]
#                            )


# step_time = 0
# motorDebugParam = []
# motorDebugParam.append(p.addUserDebugParameter('motor_0', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('motor_1', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('motor_2', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('motor_3', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('motor_4', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('motor_5', -3.14, 3.14, 0))
# motorDebugParam.append(p.addUserDebugParameter('controller', -1, 1, 0))
# while True:
#     motorsParamRead = []
#     for i in range(len(motorDebugParam)):
#         motorsParamRead.append(p.readUserDebugParameter(motorDebugParam[i]))
#     # print(motorsParamRead)
#     if motorsParamRead[6] < 0:
#         for joint_name in jointNameToID_1:
#             p.resetJointState(robot_1, jointNameToID_1[joint_name], 0)
#         for joint_name in jointNameToID_2:
#             p.resetJointState(robot_2, jointNameToID_2[joint_name], 0)
#     p.setJointMotorControl2(robot_2,
#                             jointNameToID_2['J_L_0'],
#                             p.VELOCITY_CONTROL,
#                             targetVelocity=3
#                             # motorsParamRead[0]
#                             )

#     p.stepSimulation()
#     time.sleep(0.01)
