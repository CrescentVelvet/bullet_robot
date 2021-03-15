import math
import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from gym.utils import seeding

class RobotEnv(gym.Env):
    # 初始化函数
    def __init__(self, render=False):
        # 机器人初始位置
        self.robotPos = [0, 0, 0.2]
        self.jointNameToID_robot = {}
        self.linkNameToID_robot = {}
        self.revoluteID_robot = []
        self._observation = []
        self._envStepCounter = 0
        # 设置单步时间sec
        self.time_step = 0.01
        action_dim = 12
        self._action_bound = 1
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        observation_dim = 36
        self._observation_bound = math.pi
        observation_high = np.array([self._observation_bound] * observation_dim)
        self.observation_space = spaces.Box(-observation_high, observation_high)
        if render:
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)  # non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # 步骤执行函数
    def _step(self, action):
        # 设置关节控制器
        self._set_controler(action)
        # 在单个正向动力学模拟步骤中执行所有操作，例如碰撞检测，约束求解和积分
        p.stepSimulation()
        # 计算环境观测值(object)
        self._observation = self._compute_observation()
        # 计算动作奖励量(float)
        reward = self._compute_reward()
        # 计算事件完成情况(bool)
        done = self._compute_done()
        # 时间步数
        self._envStepCounter += 1
        return np.array(self._observation), reward, done, {}

    # 重置环境函数
    def _reset(self):
        p.resetSimulation()
        # 设置重力m/s^2
        p.setGravity(0, 0, -10)
        # 设置单步时间sec
        p.setTimeStep(self.time_step)
        # 加载默认地面
        planeId = p.loadURDF("plane.urdf")
        # 加载足球场地面
        # planeId = p.loadSDF("stadium.sdf")
        # 设置地面摩擦力(旋转摩擦,横向摩擦,滚动摩擦)
        p.changeDynamics(planeId,-1,lateralFriction = 0.5,spinningFriction = 0.5,rollingFriction = 0.1)
        # 加载机器人模型
        self.robot_urdf = self._load_robot()
        # 计算环境观测值(object)
        self._observation = self._compute_observation()
        # 重置时间步数
        self._envStepCounter = 0
        return np.array(self._observation)

    # 计算环境观测值(object)
    def _compute_observation(self):
        # 获取当前机器人关节信息
        states = [p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg3_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg4_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg3_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg4_right'])]
        obs = []
        for state in states:
            # 0是位置信息,1是速度信息
            obs.append(state[0])
            obs.append(state[1])
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.robot_urdf)
        # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        cube_euler = p.getEulerFromQuaternion(cube_orn)
        linear, angular = p.getBaseVelocity(self.robot_urdf)
        # print('cube_euler', cube_euler)
        # print('cube_pos', cube_pos)
        # print('linear', linear)
        # print('angular', angular)
        obs.append(cube_pos[0])
        obs.append(cube_pos[1])
        obs.append(cube_pos[2])
        obs.append(cube_euler[0])
        obs.append(cube_euler[1])
        obs.append(cube_euler[2])
        obs.append(linear[0])
        obs.append(linear[1])
        obs.append(linear[2])
        obs.append(angular[0])
        obs.append(angular[1])
        obs.append(angular[2])
        return obs

    # 计算动作奖励量(float)
    def _compute_reward(self):
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_urdf)
        # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        robot_euler = p.getEulerFromQuaternion(robot_orn)
        # 设置reward为机器人的z坐标
        # reward = robot_pos[2] * 10 + self._envStepCounter * self.time_step
        reward = robot_pos[2]
        print('------reward', reward)
        return reward

    # 计算事件完成情况(bool)
    def _compute_done(self):
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_urdf)
        return self._envStepCounter >= 10000

    # 图形化显示
    def _render(self, mode='human', close=False):
        pass

    # 加载机器人模型
    def _load_robot(self):
        robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                                  self.robotPos,
                                  useFixedBase=0,
                                  )
        # 获取机器人关节信息
        for i in range(p.getNumJoints(robot_urdf)):
            info = p.getJointInfo(robot_urdf, i)
            # print(info)
            jointID = info[0]
            jointName = info[1].decode('UTF-8')
            jointType = info[2]
            if jointType == p.JOINT_REVOLUTE:
                self.jointNameToID_robot[jointName] = info[0]
                self.linkNameToID_robot[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_robot.append(i)
        # 设置机器人直立的关节参数
        ini_body_head = 0
        ini_body_head2 = 0
        ini_arm_left = 1.7
        ini_hand_left = 0.2
        ini_arm_right = 0.2
        ini_hand_right = 0.2
        ini_body_hip = 0
        ini_body_hip_left = 0
        ini_body_hip2_left = 0
        ini_body_hip_right = 0
        ini_body_hip2_right = -0.7
        ini_leg_left = -1.7
        ini_leg2_left = 1
        ini_leg3_left = -0.2
        ini_leg4_left = 0
        ini_leg_right = 0.5
        ini_leg2_right = 1
        ini_leg3_right = -1
        ini_leg4_right = -1
        # 重置每一个关节的位置
        reset_all = -1
        if reset_all < 0:
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head'], ini_body_head)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head2'], ini_body_head2)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_left'], ini_arm_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_left'], ini_hand_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_right'], ini_arm_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_right'], ini_hand_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_hip'], ini_body_hip)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_hip_left'], ini_body_hip_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_hip2_left'], ini_body_hip2_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_hip_right'], ini_body_hip_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_hip2_right'], ini_body_hip2_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_left'], ini_leg_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_left'], ini_leg2_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg3_left'], ini_leg3_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg4_left'], ini_leg4_left)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_right'], ini_leg_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_right'], ini_leg2_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg3_right'], ini_leg3_right)
            p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg4_right'], ini_leg4_right)
        return robot_urdf

    # 设置关节控制器
    def _set_controler(self, action):
        action = action * math.pi
        # print('action', action)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg3_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg4_left'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg3_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg4_right'],
                                controlMode=p.VELOCITY_CONTROL,
                                targetVelocity=action[0])