import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import environment
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
env = gym.make('pleg-v0', render=True)

# 创建代理
agent = DDPG('MlpPolicy', DummyVecEnv([lambda: env]), **params)

# 环境重置
obs, state, dones = env.reset(), None, [False]

# 为代理训练200000个时间步
agent.learn(total_timesteps=200000)
agent.save('wyf-pleg')
env.close()

# once the environment is trained, run a while loop
# agent = DDPG.load(r'wyf-pleg.zip')
# while True:
#     actions, state = agent.predict(obs, state=state, mask=dones)
#     obs, rew, done, info = env.step(actions)
#     if done:
#         env.reset()
#         # break
# env.close()
