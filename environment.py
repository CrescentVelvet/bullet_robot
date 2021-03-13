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
        self._add_constraint()
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

    def _compute_observation(self):
        pass

    def _compute_reward(self):
        pass

    def _compute_done(self):
        pass

    def _render(self, mode='human', close=False):
        pass

    def _load_robot_1(self):

        robot_1 = p.loadURDF(r'leg-1/urdf/leg-1.urdf',
                                  self.robotPos_1,
                                  useFixedBase=0,
                                  )
        for j in range(p.getNumJoints(self.robot_1)):
            info = p.getJointInfo(self.robot_1, j)
            print(info)
            joint_name = info[1].decode('utf8')
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.jointNameToID_1[joint_name] = info[0]
                self.linkNameToID_1[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_1.append(j)
                p.addUserDebugText(str(joint_name),
                                   [0, 0, 0],
                                   parentObjectUniqueId=self.robot_1,
                                   parentLinkIndex=j,
                                   textColorRGB=[1, 0, 0])
        p.addUserDebugText("base_1",
                           [0, 0, 0],
                           parentObjectUniqueId=self.robot_1,
                           parentLinkIndex=-1,
                           textColorRGB=[1, 0, 0])
        for i in range(len(self.linkNameToID_1)):
            p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
                               parentObjectUniqueId=self.robot_1, parentLinkIndex=i)
            p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
                               parentObjectUniqueId=self.robot_1, parentLinkIndex=i)
            p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
                               parentObjectUniqueId=self.robot_1, parentLinkIndex=i)
        return robot_1

    def _load_robot_2(self):
        robot_2 = p.loadURDF(r'leg-2/urdf/leg-2.urdf',
                                  self.robotPos_2,
                                  useFixedBase=1,
                                  )

        for j in range(p.getNumJoints(self.robot_2)):
            info = p.getJointInfo(self.robot_2, j)
            joint_name = info[1].decode('utf8')
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                self.jointNameToID_2[joint_name] = info[0]
                self.linkNameToID_2[info[12].decode('UTF-8')] = info[0]
                self.revoluteID_2.append(j)
                p.addUserDebugText(str(joint_name),
                                   [0, 0, 0],
                                   parentObjectUniqueId=self.robot_2,
                                   parentLinkIndex=j,
                                   textColorRGB=[1, 0, 0])

        p.addUserDebugText("base_2",
                           [0, 0, 0],
                           parentObjectUniqueId=self.robot_2,
                           parentLinkIndex=-1,
                           textColorRGB=[1, 0, 0])

        for i in range(len(self.linkNameToID_2)):
            p.addUserDebugLine([0., 0, 0], [0.1, 0, 0], [1, 0, 0],
                               parentObjectUniqueId=self.robot_2, parentLinkIndex=i)
            p.addUserDebugLine([0., 0, 0], [0, 0.1, 0], [0, 1, 0],
                               parentObjectUniqueId=self.robot_2, parentLinkIndex=i)
            p.addUserDebugLine([0., 0, 0], [0, 0, 0.1], [0, 0, 1],
                               parentObjectUniqueId=self.robot_2, parentLinkIndex=i)

        return robot_2

    def _add_constraint(self):

        # collisions
        for i in range(len(self.linkNameToID_1)):
            for j in range(len(self.linkNameToID_2)):
                p.setCollisionFilterPair(self.robot_1, self.robot_2, i, j, 0)
        pass

        constraintNum = 10
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_L_1"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2['L_L_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   0.055, 0, -0.005 + (0.01 / constraintNum) * i],
                               childFramePosition=[
                                   0.03706, 0.00578, -0.005 + (0.01 / constraintNum) * i]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_L_3"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2["L_L_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   0.06, 0, -0.005 + (0.01 / constraintNum) * i],
                               childFramePosition=[
                                   0.07595, -0.03311, -0.005 + (0.01 / constraintNum) * i, ]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_R_1"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2['L_R_2'],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   0.055, 0, -0.005 + (0.01 / constraintNum) * i],
                               childFramePosition=[
                                   0.03706, -0.00578, -0.005 + (0.01 / constraintNum) * i]
                               )
        for i in range(constraintNum):
            p.createConstraint(parentBodyUniqueId=self.robot_1,
                               parentLinkIndex=self.linkNameToID_1["L_R_3"],
                               childBodyUniqueId=self.robot_2,
                               childLinkIndex=self.linkNameToID_2["L_R_2"],
                               jointType=p.JOINT_POINT2POINT,
                               jointAxis=[0, 0, 0],
                               parentFramePosition=[
                                   0.06, 0, 0.005 - (0.01 / constraintNum) * i],
                               childFramePosition=[
                                   0.07595, 0.03311, -0.005 + (0.01 / constraintNum) * i]
                               )


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
