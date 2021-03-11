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
# 设置相机
p.resetDebugVisualizerCamera(cameraDistance=3.0, cameraYaw=50.0, cameraPitch=-23.80,
                                cameraTargetPosition=[-1.0, 1.0, -0.5], physicsClientId=physicsClient)
# 各种参数
use_car = 0
use_robot = 1
pos_robot = [0, 0, 1]
jointNameToID_robot = {}
linkNameToID_robot = {}
revoluteID_robot = []

if use_car:
    # 加载默认小车
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
        # 实时仿真
        if useRealTimeSim == 0:
            p.stepSimulation()

if use_robot:
    # 加载机器人模型
    robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                            pos_robot,
                            useFixedBase=0,
                            )
    # 获取机器人关节信息
    for i in range(p.getNumJoints(robot_urdf)):
        info = p.getJointInfo(robot_urdf, i)
        print(info)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        if jointType == p.JOINT_REVOLUTE:
            jointNameToID_robot[jointName] = info[0]
            linkNameToID_robot[info[12].decode('UTF-8')] = info[0]
            revoluteID_robot.append(i)
            # 显示关节名称
            p.addUserDebugText(str(jointName),
                                [0, 0, 0],
                                parentObjectUniqueId = robot_urdf,
                                parentLinkIndex = i,
                                textColorRGB = [1, 0, 0])
    # 显示基本关节名称
    p.addUserDebugText('base_robot',
                        [0, 0, 0],
                        parentObjectUniqueId = robot_urdf,
                        parentLinkIndex = i,
                        textColorRGB = [1, 0, 0])
    # 设置控制滑块，参数分别是最小值，最大值，当前值
    pos_body_head_slider    = p.addUserDebugParameter("pos_body_head", -10, 10, 0)
    pos_body_head2_slider   = p.addUserDebugParameter("pos_body_head2", -10, 10, 0)
    vel_arm_left_slider     = p.addUserDebugParameter("vel_arm_left", -10, 10, 0)
    vel_hand_left_slider    = p.addUserDebugParameter("vel_hand_left", -10, 10, 0)
    vel_arm_right_slider    = p.addUserDebugParameter("vel_arm_right", -10, 10, 0)
    vel_hand_right_slider   = p.addUserDebugParameter("vel_hand_right", -10, 10, 0)
    # vel_body_hip_slider     = p.addUserDebugParameter("vel_body_hip", -10, 10, 0)
    # vel_body_hip_left_slider    = p.addUserDebugParameter("vel_body_hip_left", -10, 10, 0)
    # vel_body_hip2_left_slider   = p.addUserDebugParameter("vel_body_hip2_left", -10, 10, 0)
    # vel_body_hip_right_slider   = p.addUserDebugParameter("vel_body_hip_right", -10, 10, 0)
    # vel_body_hip2_right_slider  = p.addUserDebugParameter("vel_body_hip2_right", -10, 10, 0)
    vel_leg_left_slider     = p.addUserDebugParameter("vel_leg_left", -10, 10, 0)
    vel_leg2_left_slider    = p.addUserDebugParameter("vel_leg2_left", -10, 10, 0)
    vel_leg3_left_slider    = p.addUserDebugParameter("vel_leg3_left", -10, 10, 0)
    vel_leg4_left_slider    = p.addUserDebugParameter("vel_leg4_left", -10, 10, 0)
    vel_leg_right_slider    = p.addUserDebugParameter("vel_leg_right", -10, 10, 0)
    vel_leg2_right_slider   = p.addUserDebugParameter("vel_leg2_right", -10, 10, 0)
    vel_leg3_right_slider   = p.addUserDebugParameter("vel_leg3_right", -10, 10, 0)
    vel_leg4_right_slider   = p.addUserDebugParameter("vel_leg4_right", -10, 10, 0)
    reset_all_slider        = p.addUserDebugParameter("reset_all", -10, 10, 0)
    while True:
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
        vel_leg3_left   = p.readUserDebugParameter(vel_leg3_left_slider)
        vel_leg4_left   = p.readUserDebugParameter(vel_leg4_left_slider)
        vel_leg_right   = p.readUserDebugParameter(vel_leg_right_slider)
        vel_leg2_right  = p.readUserDebugParameter(vel_leg2_right_slider)
        vel_leg3_right  = p.readUserDebugParameter(vel_leg3_right_slider)
        vel_leg4_right  = p.readUserDebugParameter(vel_leg4_right_slider)
        reset_all       = p.readUserDebugParameter(reset_all_slider)
        # 全部复位
        if reset_all < 0:
            for joint_name in jointNameToID_robot:
                p.resetJointState(robot_urdf, jointNameToID_robot[joint_name], 0)
        # 控制机器人关节位置
        p.setJointMotorControl2(robot_urdf,
                            jointNameToID_robot['joint_body_head'],
                            p.POSITION_CONTROL,
                            targetPosition=pos_body_head,
                            force=10
                            )
        p.setJointMotorControl2(robot_urdf,
                            jointNameToID_robot['joint_body_head2'],
                            p.POSITION_CONTROL,
                            targetPosition=pos_body_head2,
                            force=10
                            )
        # 控制机器人关节速度
        p.setJointMotorControl2(robot_urdf,
                            jointNameToID_robot['joint_arm_left'],
                            p.VELOCITY_CONTROL,
                            targetVelocity=vel_arm_left,
                            force=10
                            )
        # 实时仿真
        if useRealTimeSim == 0:
            p.stepSimulation()
            time.sleep(0.01)