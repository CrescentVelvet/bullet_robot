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

# 创建环境
env = gym.make('MyEnv-v0', render=True)

# 创建模型(DDPG算法,MlpPolicy类型)
model = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)

# 重置环境
obs, state, dones = env.reset(), None, [False]

# 构建模型
model.learn(total_timesteps=10000) # 为代理训练10000个时间步

# 保存模型
model.save('wyf-pleg')
env = model.get_env()

# 移除模型
del model

# 读取模型
model = DDPG.load(r'wyf-pleg.zip')
obs = env.reset()

# 循环训练
while True:
    actions, state = model.predict(obs, state=state, mask=dones)
    obs, reward, done, info = env.step(actions)
    env.render()
    if done:
        obs = env.reset()
