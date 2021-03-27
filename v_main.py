# 可执行文件,调用v_argument.py解析命令行参数,初始化环境和PPO模型
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
from v_ppo import PPO
import pleg.envs.environment
env = gym.make('MyEnv-v0', render=True) # 建立可视化环境
env = env.unwrapped # 解除环境限制
model = PPO(env, verbose=1) # 建立PPO模型
total_timesteps = 10000 # 循环设定总步数
model.learn(total_timesteps=total_timesteps) # 模型训练
model.save('v-wyf-pleg') # 模型保存
env = model.get_env()
del model
model = PPO.load(r'v-wyf-pleg.zip')
obs, state, dones = env.reset(), None, [False]
while True:
    action, state = model.predict(obs, state=state, mask=dones)
    obs, rew, done, info = env.step(action)
    if done:
        obs = env.reset()
