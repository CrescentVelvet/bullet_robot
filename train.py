'''
Author       : velvet
Date         : 2021-03-19 21:57:57
LastEditTime : 2021-03-19 22:16:32
LastEditors  : velvet
Description  : 
'''
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
# import torch
# torch.cuda.set_device(0)
import gym
# import helper
import numpy as np
import pybullet_envs
from stable_baselines3 import PPO
# from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
# from gym.envs.classic_control.pendulum import PendulumEnv
# from stable_baselines3.common.env_util import make_vec_env
# from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise
# from wrappers import Reset, Monitor
import pleg.envs.environment
# 定义超参数
# params = {'learning_rate': 1e-5,    # 学习率
#           'gamma': 0.99,            # reward的衰减因子
#           'batch_size': 1024,       # 一次训练选取的样本数
#           'verbose': 1,
#           'tensorboard_log': './log/',
#           'policy_kwargs': dict(net_arch=[256, 256, 256])}
# 创建环境(名为MyEnv-v0,render绘制bullet仿真界面,unwrapped解除环境限制)
# env = make_vec_env('MyEnv-v0', n_envs = 4)
# env = gym.make('MyEnv-v0', render=True)
# env = DummyVecEnv([lambda: gym.make('MyEnv-v0', render=True)]) # 创建环境，读取我的环境MyEnv-v0
env = DummyVecEnv([lambda: gym.make('HumanoidBulletEnv-v0', render=True)]) # 创建环境
# env = env.unwrapped
# env = Monitor(env, helper.ensure_dir('./monitor'), allow_early_resets=True) # 添加监视器
# env = Reset(env)
# env.seed(1)
# env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0) # 环境标准化
# 添加噪声
# n_actions = env.action_space.shape[-1]
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
# 创建模型(DDPG/PPO算法,MlpPolicy类型)
model = PPO("MlpPolicy", env, verbose=1)
# model = DDPG("MlpPolicy", env, verbose=1)
# model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1)
# model = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)
# 构建模型(代理执行最大步数total_timesteps,采集数据样本时间间隔log_interval)
# total_timesteps = 10000 # 代理执行最大步数
# log_interval = 10 # 采集数据样本时间间隔
model.learn(total_timesteps=10000, log_interval=10)
# save_dir='/home/zjunlict-vision-1/Desktop/bullet_robot/model' # 保存地址
# os.makedirs(save_dir, exist_ok=True) # 没有model文件夹就新建一个
# model_path = os.path.join(save_dir, 'PPO_tutorial') # 模型保存地址
# 保存模型,获取环境,移除模型,读取模型
model.save('wyf-pleg')
# env_path = os.path.join(save_dir, 'vec_normalize.pkl') # 环境保存地址
# env.save(env_path) # 保存环境
# env = model.get_env()
del model
model = PPO.load(r'wyf-pleg.zip')
# model = DDPG.load(r'wyf-pleg.zip')
# if use_my_robot:
#     loaded_env = DummyVecEnv([lambda: gym.make("MyEnv-v0")]) # 加载环境
# else:
#     loaded_env = DummyVecEnv([lambda: gym.make("HumanoidBulletEnv-v0")]) # 加载机器人环境
# loaded_env = VecNormalize.load(env_path, env) # 加载环境，环境标准化
# loaded_env.training = False # 更新
# loaded_env.norm_reward = False # reward归一化
# 初始化环境(获取obs反馈)
obs, state, dones, done = env.reset(), None, [False], False
# 循环训练
while not done:
    action, state = model.predict(obs, state=state, mask=dones)
    # print('action = ', action)
    # print('state = ', state)
    # print('dones = ', dones)
    # 环境演化一步
    obs, reward, done, info = env.step(action)
    # 绘制当前环境
    # env.render()
    # if done:
    #     obs = env.reset()
