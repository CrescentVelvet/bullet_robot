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
import pybullet
import pybullet_data

class RobotEnv(gym.Env):    
    def __init__(self, render=False): # 初始化函数
        self.robotPos = [0, 0, 0.28] # 机器人坐标
        self.robotOri = [0.4, -0.6, 0.6, -0.4] # 机器人方向
        self.jointNameToID_robot = {} # 机器人关节名
        self.linkNameToID_robot = {} # 机器人部件名
        self._observation = [] # 观测空间
        self._envStepCounter = 0 # 设置步数计数器        
        self.time_step = 0.01 # 设置单步时间sec        
        action_dim = 10 # 动作空间
        self._action_bound = 1
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = gym.spaces.Box(np.float32(-action_high), np.float32(action_high))
        observation_dim = 32 # 观测空间
        self._observation_bound = math.pi
        observation_high = np.array([self._observation_bound] * observation_dim)
        self.observation_space = gym.spaces.Box(np.float32(-observation_high), np.float32(observation_high))
        self.command_vel = np.zeros(action_dim) # 机器人关节速度指令
        self.friction = np.zeros(len(self.command_vel)) # 摩擦力
        self.acceleration = np.zeros(len(self.command_vel)) # 加速度
        if render: # 连接物理引擎
            self.physicsClient = pybullet.connect(pybullet.GUI) # 仿真可视化
        else:
            self.physicsClient = pybullet.connect(pybullet.DIRECT) # 仿真不可视化
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        self._seed()
    def _seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]
    def step(self, action): # 步骤执行函数       
        self._set_controler(action) # 设置关节控制器(action发送控制参数)        
        pybullet.stepSimulation() # 在单个正向动力学模拟步骤中执行所有操作，例如碰撞检测，约束求解和积分        
        self._observation = self._compute_observation() # 计算环境观测值(object)        
        reward = self._compute_reward() # 计算动作奖励量(float)        
        done = self._compute_done() # 计算事件完成情况(bool)        
        self._envStepCounter += 1 # 时间步数
        return np.array(self._observation), reward, done, {}
    def reset(self): # 重置环境函数
        pybullet.resetSimulation() # 开始仿真        
        pybullet.setGravity(0, 0, -9.8) # 设置重力m/s^2        
        pybullet.setTimeStep(self.time_step) # 设置单步时间sec        
        planeId = pybullet.loadURDF("plane.urdf") # 加载默认地面        
        # planeId = pybullet.loadSDF("stadium.sdf") # 加载足球场地面        
        pybullet.changeDynamics(planeId,-1,lateralFriction = 0.6,spinningFriction = 0.3,rollingFriction = 0.0001) # 设置地面摩擦力(旋转摩擦,横向摩擦,滚动摩擦)        
        self.robot_urdf = self._load_robot() # 加载机器人模型        
        self._observation = self._compute_observation() # 计算环境观测值(object)        
        self._envStepCounter = 0 # 重置时间步数
        return np.array(self._observation) # 返回观测值    
    def _compute_observation(self): # 计算环境观测值(object)
        states = [pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left']), # 获取当前机器人关节角度与角速度信息
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_left']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_right']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_hand_right']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_left']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_left']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_foot1_left']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg_right']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_leg2_right']),
                  pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_foot1_right'])]
        obs = []
        for state in states:
            obs.append(state[0]) # 获取关节角度信息
            obs.append(state[1]) # 获取关节角速度信息
        cube_pos, cube_orn = pybullet.getBasePositionAndOrientation(self.robot_urdf) # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        cube_euler = pybullet.getEulerFromQuaternion(cube_orn) # 将姿态四元数转换为欧拉角[yaw,pitch,roll]
        linear, angular = pybullet.getBaseVelocity(self.robot_urdf) # 返回世界坐标系中的速度[x,y,z]和角速度[yaw,pitch,roll]
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
    def _compute_reward(self): # 计算动作奖励量(float)
        # robot_pos, robot_orn = pybullet.getBasePositionAndOrientation(self.robot_urdf) # 返回世界坐标系中的位置[x,y,z]和姿态[x,y,z,w]
        robot_vel, robot_wvel = pybullet.getBaseVelocity(self.robot_urdf) # 返回世界坐标系中的速度[x,y,z]和角速度[wx,wy,wz]
        reward = robot_vel[0] # 奖励 = 全身x速度
        # reward = robot_pos[0] # 奖励 = 全身x坐标
        # reward = pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2] # 奖励 = 头部z坐标
        # 奖励 = 头部z坐标*2 - 左脚z坐标 - 右脚z坐标
        # reward = pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2]*2 - pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_left'])[4][2] - pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_right'])[4][2]
        # reward = pybullet.getJointState(self.robot_urdf, self.jointNameToID_robot['joint_arm_left'])[0][2]
        # 惩罚 = 各关节扭矩 * 角速度
        punish = 0
        for i in range(len(self.command_vel)):
            punish += self.acceleration[i] * self.command_vel[i] * 0.01
        # print('------reward : ', reward)
        # print('------punish : ', punish)
        return reward-punish
    def _compute_done(self): # 计算事件完成情况(bool)
        # is_done = pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['body_head'])[4][2]*2 - pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_left'])[4][2] - pybullet.getLinkState(self.robot_urdf, self.linkNameToID_robot['foot1_right'])[4][2]
        # if self._envStepCounter >= 10000 or is_done >= 0.5:
        is_done = False
        if self._envStepCounter >= 100000:
            is_done = True
        return is_done
    def _render(self, mode='human', close=False): # 图形化显示
        pass
    def _load_robot(self): # 加载机器人模型
        robot_urdf = pybullet.loadURDF(r'dancer_urdf_model/model/dancer_urdf_model.URDF',
                                basePosition=self.robotPos,
                                baseOrientation=self.robotOri,
                                flags=pybullet.URDF_USE_SELF_COLLISION or pybullet.URDF_USE_INERTIA_FROM_FILE,
                                useFixedBase=0,
                                )
        for i in range(pybullet.getNumJoints(robot_urdf)): # 获取机器人关节信息
            info = pybullet.getJointInfo(robot_urdf, i)
            # print(info)
            jointID = info[0]
            jointName = info[1].decode('UTF-8')
            jointType = info[2]
            if jointType == pybullet.JOINT_REVOLUTE:
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
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head'], ini_body_head)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_body_head2'], ini_body_head2)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_left'], ini_arm_left)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_left'], ini_hand_left)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_arm_right'], ini_arm_right)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_hand_right'], ini_hand_right)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_left'], ini_leg_left)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_left'], ini_leg2_left)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_foot1_left'], ini_foot1_left)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg_right'], ini_leg_right)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_leg2_right'], ini_leg2_right)
        #     pybullet.resetJointState(robot_urdf, self.jointNameToID_robot['joint_foot1_right'], ini_foot1_right)
        return robot_urdf    
    def _set_controler(self, input_action): # 设置关节控制器        
        max_force = 500 # 最大力矩
        max_velocity = 0.02 # 最大角速度
        # control_mode = pybullet.POSITION_CONTROL
        control_mode = pybullet.VELOCITY_CONTROL # 控制模式
        action = input_action # 控制参数
        # action = action * math.pi
        # print('action', action)
        c_drag = 0.01 # 滑动阻力系数
        c_rolling = 0.2 # 滚动阻力系数
        c_throttle = 20 # 控制系数
        for i in range(len(self.command_vel)): # 遍历机器人关节速度指令
            self.friction[i] = - self.command_vel[i] * (self.command_vel[i] * c_drag + c_rolling) # 根据速度计算摩擦力
            self.acceleration[i] = action[i] * c_throttle + self.friction[i] # 根据摩擦力计算加速度
            self.command_vel[i] = self.command_vel[i] + 1/30 * self.acceleration[i] # 根据加速度计算速度
            self.command_vel[i] = max(min(self.command_vel[i], 1), -1) # 速度取上下边界(-1,1)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_left'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[0],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_left'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[1],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_arm_right'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[2],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_hand_right'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[3],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_left'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[4],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_left'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[5],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_foot1_left'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[6],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg_right'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[7],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_leg2_right'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[8],
                                force=max_force,
                                maxVelocity=max_velocity)
        pybullet.setJointMotorControl2(bodyIndex=self.robot_urdf,
                                jointIndex=self.jointNameToID_robot['joint_foot1_right'],
                                controlMode=control_mode,
                                targetVelocity=self.command_vel[9],
                                force=max_force,
                                maxVelocity=max_velocity)