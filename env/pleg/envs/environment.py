'''
Author       : velvet
Date         : 2021-03-19 21:57:57
LastEditTime : 2021-03-19 22:19:21
LastEditors  : velvet
Description  : 
'''
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
        self.robotPos = [0, 0, 0.28]            # 机器人坐标
        self.robotOri = [0.4, -0.6, 0.6, -0.4]  # 机器人方向
        self.jointNameToID_robot = {}           # 机器人关节名
        self.linkNameToID_robot = {}            # 机器人部件名
        self._observation = []
        # 设置步数计数器
        self._envStepCounter = 0
        # 设置单步时间sec
        self.time_step = 0.01
        # 动作空间
        action_dim = 10
        self._action_bound = 1
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(np.float32(-action_high), np.float32(action_high))
        # 观测空间
        observation_dim = 32
        self._observation_bound = math.pi
        observation_high = np.array([self._observation_bound] * observation_dim)
        self.observation_space = spaces.Box(np.float32(-observation_high), np.float32(observation_high))
        # 连接物理引擎
        if render:
            # 包含可视化的引擎
            self.physicsClient = p.connect(p.GUI)
        else:
            # 不含可视化的引擎
            self.physicsClient = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._seed()

    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # 步骤执行函数
    def step(self, action):
        # 设置关节控制器(action发送控制参数)
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
    def reset(self):
        p.resetSimulation()
        # 设置重力m/s^2
        p.setGravity(0, 0, -9.8)
        # 设置单步时间sec
        p.setTimeStep(self.time_step)
        # 加载默认地面
        planeId = p.loadURDF("plane.urdf")
        # 加载足球场地面
        # planeId = p.loadSDF("stadium.sdf")
        # 设置地面摩擦力(旋转摩擦,横向摩擦,滚动摩擦)
        p.changeDynamics(planeId,-1,lateralFriction = 0.6,spinningFriction = 0.3,rollingFriction = 0.0001)
        # 加载机器人模型
        self.robot_urdf = self._load_robot()
        # 计算环境观测值(object)
        self._observation = self._compute_observation()
        # 重置时间步数
        self._envStepCounter = 0
        return np.array(self._observation)

    # 计算环境观测值(object)
    def _compute_observation(self):
        # 获取当前机器人关节角度与角速度信息
        states = [p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_foot1_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_foot1_right'])]
        obs = []
        for state in states:
            # 0是关节角度信息,1是关节角速度信息
            obs.append(state[0])
            obs.append(state[1])
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        cube_pos, cube_orn = p.getBasePositionAndOrientation(self.robot_urdf)
        # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        cube_euler = p.getEulerFromQuaternion(cube_orn)
        # 返回世界坐标系中的速度[x,y,z]和角速度[yaw,pitch,roll]
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
        # print('------obs : ', cube_pos[0])
        return obs

    # 计算动作奖励量(float)
    def _compute_reward(self):
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_urdf)
        # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        robot_euler = p.getEulerFromQuaternion(robot_orn)
        # 设置reward:头部最高,脚部最低,质心越高越好
        # reward = robot_pos[2] * 10 + self._envStepCounter * self.time_step
        # reward = robot_pos[2]
        # reward = p.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2]
        reward = p.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2]*2 - p.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_left'])[4][2] - p.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_right'])[4][2]
        # reward = p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left'])[0][2]
        print('------reward : ', reward)
        # print(0)
        return reward

    # 计算事件完成情况(bool)
    def _compute_done(self):
        is_done = p.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2]*2 - p.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_left'])[4][2] - p.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_right'])[4][2]
        # if self._envStepCounter >= 10000 or is_done >= 0.5:
        if self._envStepCounter >= 10000:
            return True

    # 图形化显示
    def _render(self, mode='human', close=False):
        pass

    # 加载机器人模型
    def _load_robot(self):
        robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                                basePosition=self.robotPos,
                                baseOrientation=self.robotOri,
                                flags=p.URDF_USE_SELF_COLLISION or p.URDF_USE_INERTIA_FROM_FILE,
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
        # # 设置机器人直立的关节参数
        # ini_body_head = 0
        # ini_body_head2 = 0
        # ini_arm_left = 0
        # ini_hand_left = 0
        # ini_arm_right = 0
        # ini_hand_right = 0
        # ini_leg_left = 0
        # ini_leg2_left = 0
        # ini_foot1_left = 0
        # ini_leg_right = 0
        # ini_leg2_right = 0
        # ini_foot1_right = 0
        # # 重置每一个关节的位置
        # reset_all = -1
        # if reset_all < 0:
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head'], ini_body_head)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head2'], ini_body_head2)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_left'], ini_arm_left)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_left'], ini_hand_left)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_right'], ini_arm_right)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_right'], ini_hand_right)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_left'], ini_leg_left)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_left'], ini_leg2_left)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_foot1_left'], ini_foot1_left)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_right'], ini_leg_right)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_right'], ini_leg2_right)
        #     p.resetJointState(robot_urdf, self.jointNameToID_robot['joint_foot1_right'], ini_foot1_right)
        return robot_urdf

    # 设置关节控制器
    def _set_controler(self, action):
        # 最大力矩
        max_force = 500
        # 最大角速度
        max_velocity = 0.02
        # 控制模式
        control_mode = p.VELOCITY_CONTROL
        # 控制参数
        action = action
        # action = action * math.pi
        # print('action', action)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_left'],
                                controlMode=control_mode,
                                targetVelocity=action[0],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_left'],
                                controlMode=control_mode,
                                targetVelocity=action[1],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_right'],
                                controlMode=control_mode,
                                targetVelocity=action[2],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_right'],
                                controlMode=control_mode,
                                targetVelocity=action[3],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_left'],
                                controlMode=control_mode,
                                targetVelocity=action[4],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_left'],
                                controlMode=control_mode,
                                targetVelocity=action[5],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_foot1_left'],
                                controlMode=control_mode,
                                targetVelocity=action[6],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_right'],
                                controlMode=control_mode,
                                targetVelocity=action[7],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_right'],
                                controlMode=control_mode,
                                targetVelocity=action[8],
                                force=max_force,
                                maxVelocity=max_velocity)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_foot1_right'],
                                controlMode=control_mode,
                                targetVelocity=action[9],
                                force=max_force,
                                maxVelocity=max_velocity)