import time

import pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

p.resetDebugVisualizerCamera(cameraDistance=1.30, cameraYaw=50.0, cameraPitch=-23.80,
                             cameraTargetPosition=[-0.65, 0.49, -0.25], physicsClientId=physicsClient)

use_2 = 1
useConstraint = 1

robotPos_2 = [0, 0, 0]
jointNameToID_1 = {}
linkNameToID_1 = {}
revoluteID_1 = []
jointNameToID_2 = {}
linkNameToID_2 = {}
revoluteID_2 = []

if use_1:
    robot_1 = p.loadURDF(r'leg-1/urdf/leg-1.urdf',
                         robotPos_1,
                         useFixedBase=1,
                         )
    for j in range(p.getNumJoints(robot_1)):
        info = p.getJointInfo(robot_1, j)
        print(info)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        if jointType == p.JOINT_REVOLUTE:
            jointNameToID_1[jointName] = info[0]
            linkNameToID_1[info[12].decode('UTF-8')] = info[0]
            revoluteID_1.append(j)
            p.addUserDebugText(str(jointName),
                               [0, 0, 0],
                               parentObjectUniqueId=robot_1,
                               parentLinkIndex=j,
                               textColorRGB=[1, 0, 0])
    p.addUserDebugText("base_1",
                       [0, 0, 0],
                       parentObjectUniqueId=robot_1,
                       parentLinkIndex=-1,
                       textColorRGB=[1, 0, 0])
    for i in range(len(linkNameToID_1)):
        p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
                           parentObjectUniqueId=robot_1, parentLinkIndex=i)
        p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
                           parentObjectUniqueId=robot_1, parentLinkIndex=i)
        p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
                           parentObjectUniqueId=robot_1, parentLinkIndex=i)
if use_2:
    robot_2 = p.loadURDF(r'leg-2/urdf/leg-2.urdf',
                         robotPos_2,
                         useFixedBase=0,
                         )

    for j in range(p.getNumJoints(robot_2)):
        info = p.getJointInfo(robot_2, j)
        print(info)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        if jointType == p.JOINT_REVOLUTE:
            jointNameToID_2[jointName] = info[0]
            linkNameToID_2[info[12].decode('UTF-8')] = info[0]
            revoluteID_2.append(j)
            p.addUserDebugText(str(jointName),
                               [0, 0, 0],
                               parentObjectUniqueId=robot_2,
                               parentLinkIndex=j,
                               textColorRGB=[1, 0, 0])

    p.addUserDebugText("base_2",
                       [0, 0, 0],
                       parentObjectUniqueId=robot_2,
                       parentLinkIndex=-1,
                       textColorRGB=[1, 0, 0])

    for i in range(len(linkNameToID_2)):
        p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
                           parentObjectUniqueId=robot_2, parentLinkIndex=i)
        p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
                           parentObjectUniqueId=robot_2, parentLinkIndex=i)
        p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
                           parentObjectUniqueId=robot_2, parentLinkIndex=i)

if use_1 and use_2:
    for i in range(len(linkNameToID_1)):
        for j in range(len(linkNameToID_2)):
            p.setCollisionFilterPair(robot_1, robot_2, i, j, 0)
    pass

if use_1 and use_2 and useConstraint:
    constraintNum = 10
    for i in range(constraintNum):
        p.createConstraint(parentBodyUniqueId=robot_1,
                           parentLinkIndex=linkNameToID_1["L_L_1"],
                           childBodyUniqueId=robot_2,
                           childLinkIndex=linkNameToID_2['L_L_2'],
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[
                               0.055, 0, -0.005 + (0.01 / constraintNum) * i],
                           childFramePosition=[
                               0.03706, 0.00578, -0.005 + (0.01 / constraintNum) * i]
                           )
    for i in range(constraintNum):
        p.createConstraint(parentBodyUniqueId=robot_1,
                           parentLinkIndex=linkNameToID_1["L_L_3"],
                           childBodyUniqueId=robot_2,
                           childLinkIndex=linkNameToID_2["L_L_2"],
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[
                               0.06, 0, -0.005 + (0.01 / constraintNum) * i],
                           childFramePosition=[
                               0.07595, -0.03311, -0.005 + (0.01 / constraintNum) * i, ]
                           )
    for i in range(constraintNum):
        p.createConstraint(parentBodyUniqueId=robot_1,
                           parentLinkIndex=linkNameToID_1["L_R_1"],
                           childBodyUniqueId=robot_2,
                           childLinkIndex=linkNameToID_2['L_R_2'],
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[
                               0.055, 0, -0.005 + (0.01 / constraintNum) * i],
                           childFramePosition=[
                               0.03706, -0.00578, -0.005 + (0.01 / constraintNum) * i]
                           )
    for i in range(constraintNum):
        p.createConstraint(parentBodyUniqueId=robot_1,
                           parentLinkIndex=linkNameToID_1["L_R_3"],
                           childBodyUniqueId=robot_2,
                           childLinkIndex=linkNameToID_2["L_R_2"],
                           jointType=p.JOINT_POINT2POINT,
                           jointAxis=[0, 0, 0],
                           parentFramePosition=[
                               0.06, 0, 0.005 - (0.01 / constraintNum) * i],
                           childFramePosition=[
                               0.07595, 0.03311, -0.005 + (0.01 / constraintNum) * i]
                           )

p.setGravity(0, 0, -10)

step_time = 0
motorDebugParam = []
motorDebugParam.append(p.addUserDebugParameter('motor_0', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('motor_1', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('motor_2', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('motor_3', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('motor_4', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('motor_5', -3.14, 3.14, 0))
motorDebugParam.append(p.addUserDebugParameter('controller', -1, 1, 0))
while True:
    motorsParamRead = []
    for i in range(len(motorDebugParam)):
        motorsParamRead.append(p.readUserDebugParameter(motorDebugParam[i]))
    # print(motorsParamRead)
    if motorsParamRead[6] < 0:
        for joint_name in jointNameToID_1:
            p.resetJointState(robot_1, jointNameToID_1[joint_name], 0)
        for joint_name in jointNameToID_2:
            p.resetJointState(robot_2, jointNameToID_2[joint_name], 0)
    p.setJointMotorControl2(robot_2,
                            jointNameToID_2['J_L_0'],
                            p.VELOCITY_CONTROL,
                            targetVelocity=3
                            # motorsParamRead[0]
                            )

    p.stepSimulation()
    time.sleep(0.01)
