import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import pleg.envs.environment
import gym
from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env import DummyVecEnv

# 定义超参数
params = {'learning_rate': 1e-5,
          'gamma': 0.99,
          'batch_size': 1024,
          'verbose': 1,
          'tensorboard_log': './log/',
          'policy_kwargs': dict(net_arch=[256, 256, 256])}

# 环境初始化
env = gym.make('MyEnv-v0', render=True)

# 创建模型
model = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)

# 重置环境
obs, state, dones = env.reset(), None, [False]

# 构建环境并保存为wyf-pleg
# 为代理训练10000个时间步
# model.learn(total_timesteps=10000)
# model.save('wyf-pleg')
# env.close()

# 读取环境,进入训练循环
model = DDPG.load(r'wyf-pleg.zip')
while True:
    actions, state = model.predict(obs, state=state, mask=dones)
    obs, reward, done, info = env.step(actions)
    if done:
        obs = env.reset()
        # break
env.close()
