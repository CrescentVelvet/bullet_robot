import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
import torch
import numpy as np
import pybullet_envs
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
from gym.envs.classic_control.pendulum import PendulumEnv
import pleg.envs.environment
env = DummyVecEnv([lambda: gym.make('HalfCheetahBulletEnv-v0')]) # 创建环境
env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)
model = PPO('MlpPolicy', env) # 建立模型
model.learn(total_timesteps=2000) # 训练2000次
save_dir='/home/zjunlict-vision-1/Desktop/bullet_robot/model' # 保存地址
os.makedirs(save_dir, exist_ok=True) # 没有model文件夹就新建一个
model_path = os.path.join(save_dir, 'PPO_tutorial') # 模型保存地址
model.save(model_path) # 保存模型
env_path = os.path.join(save_dir, 'vec_normalize.pkl') # 环境保存地址
env.save(env_path) # 保存环境
del model, env # 删除模型和环境
loaded_model = PPO.load(model_path, 'ppo_halfcheetah') # 加载模型
env = DummyVecEnv([lambda: gym.make("HalfCheetahBulletEnv-v0")]) # 加载环境
env = VecNormalize.load(env_path, env) # 加载环境，环境标准化
env.training = False