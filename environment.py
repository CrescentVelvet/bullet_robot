import math
import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym import spaces
from gym.utils import seeding

class ParallelEnv(gym.Env):

    def __init__(self, render=False):
        self.robotPos = [0, 0, 0]
        self.jointNameToID_robot = {}
        self.linkNameToID = {}
        self.revoluteID = []
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

    def _step(self, action):
        # 设置关节控制器
        self._set_controler(action)
        p.stepSimulation()
        self._observation = self._compute_observation()
        reward = self._compute_reward()
        done = self._compute_done()
        self._envStepCounter += 1
        return np.array(self._observation), reward, done, {}

    def _reset(self):
        p.resetSimulation()
        # 设置重力m/s^2
        p.setGravity(0, 0, -10)
        # 设置单步时间sec
        p.setTimeStep(self.time_step)
        # 加载默认地面
        # p.loadURDF("plane.urdf")
        # 加载足球场地面
        p.loadSDF("stadium.sdf")
        # 加载机器人模型
        self.robot_urdf = self._load_robot()
        self._observation = self._compute_observation()
        return np.array(self._observation)

    # 设置关节控制器
    def _set_controler(self, action):
        action = action * math.pi
        print('action', action)
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_left'],
                                controlMode=p.POSITION_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_left'],
                                controlMode=p.POSITION_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_right'],
                                controlMode=p.POSITION_CONTROL,
                                targetVelocity=action[0])
        p.setJointMotorControl2(bodyUniqueId=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_right'],
                                controlMode=p.POSITION_CONTROL,
                                targetVelocity=action[0])

    # 计算observation
    def _compute_observation(self):
        states = [p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_left']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_right']),
                  p.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_right'])]
        obs = []
        for state in states:
            obs.append(state[0])
            obs.append(state[1])  # 0 for pos, 1 for vel
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

    # 计算reward
    def _compute_reward(self):
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_urdf)
        # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        robot_euler = p.getEulerFromQuaternion(robot_orn)
        # 设置reward为机器人的z坐标
        # reward = robot_pos[2] * 10 + self._envStepCounter * self.time_step
        reward = robot_pos[2] * 10
        print('reward', reward)
        return reward

    # 计算done
    def _compute_done(self):
        # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_urdf)
        return self._envStepCounter >= 1500

    def _render(self, mode='human', close=False):
        pass

    # 加载机器人模型
    def _load_robot(self):
        robot_urdf = p.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                                  self.pos_robot,
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
        return robot_urdf
