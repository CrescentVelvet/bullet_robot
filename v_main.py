# 可执行文件,调用v_argument.py解析命令行参数,初始化环境和PPO模型
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
from v_ppo import PPO
import pleg.envs.environment
env = gym.make('MyEnv-v0', render=True)
env = env.unwrapped
env.seed(1)
model = PPO(env, verbose=1)
total_timesteps = 10000
log_interval = 10
model.learn(total_timesteps=total_timesteps)
model.save('wyf-pleg')
env = model.get_env()
del model
model = PPO.load(r'wyf-pleg.zip')
obs, state, dones = env.reset(), None, [False]
while True:
    action, state = model.predict(obs, state=state, mask=dones)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
