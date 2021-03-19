'''
Author       : velvet
Date         : 2021-03-19 21:57:57
LastEditTime : 2021-03-19 22:16:32
LastEditors  : velvet
Description  : 
'''
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import pleg.envs.environment
import numpy as np
from stable_baselines3 import DDPG
# from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

# 定义超参数
# params = {'learning_rate': 1e-5,    # 学习率
#           'gamma': 0.99,            # reward的衰减因子
#           'batch_size': 1024,       # 一次训练选取的样本数
#           'verbose': 1,
#           'tensorboard_log': './log/',
#           'policy_kwargs': dict(net_arch=[256, 256, 256])}
# 创建环境(名为MyEnv-v0,可被render绘制)
env = gym.make('MyEnv-v0', render=True)
# 添加噪声
n_actions = env.action_space.shape[-1]
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
# 创建模型(DDPG算法,MlpPolicy类型)
model = DDPG("MlpPolicy", env, action_noise=action_noise, verbose=1)
# model = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)
# 构建模型(代理执行最大步数total_timesteps,采集数据样本时间间隔log_interval)
total_timesteps = 10000
log_interval = 10
model.learn(total_timesteps=total_timesteps, log_interval=log_interval)
# 保存模型,获取环境,移除模型,读取模型
model.save('wyf-pleg')
env = model.get_env()
del model
model = DDPG.load(r'wyf-pleg.zip')
# 初始化环境(获取obs反馈)
obs, state, dones = env.reset(), None, [False]
# 循环训练
while True:
    action, state = model.predict(obs, state=state, mask=dones)
    # 环境演化一步
    obs, reward, done, info = env.step(action)
    # 绘制当前环境
    env.render()
    if done:
        obs = env.reset()
