import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
import time
import torch
import numpy as np
import tensorboardX
import pybullet_envs
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import pleg.envs.environment
use_my_robot = 1
if use_my_robot:
    env = gym.make('MyEnv-v0', render=False) # 创建环境，读取我的环境MyEnv-v0
else:
    env = DummyVecEnv([lambda: gym.make('HumanoidBulletEnv-v0', render=True)]) # 创建环境
s_tensor_board = tensorboardX.SummaryWriter(comment='_s_main') # 创建监视器
print('------环境构建完成---开始构建模型------')
model = PPO('MlpPolicy', env, verbose=1) # 建立模型
model.learn(total_timesteps=2000, log_interval=10) # 训练2000次
model.save('my_ppo/s-wyf-pleg') # 保存模型
print('------模型构建完成---开始仿真训练------')
obs, state, dones, done = env.reset(), None, [False], False # 重设环境
while True: # 循环训练
    action = model.predict(obs, state=state, mask=dones)
    obs, reward, done, info = env.step(action[0])
    s_tensor_board.add_scalar('reward', reward)
    if done:
        obs = env.reset()
        time.sleep(1/30)