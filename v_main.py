# 可执行文件,调用v_argument.py解析命令行参数,初始化环境和PPO模型
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
# import torch
# torch.cuda.set_device(0)
import gym
from v_ppo import PPO
import pleg.envs.environment
env = gym.make('MyEnv-v0', render=True) # 建立可视化环境
env = env.unwrapped # 解除环境限制
save_dir='/home/zjunlict-vision-1/Desktop/bullet_robot/model' # 设置模型保存地址
print('------环境构建完成---开始构建模型------')
model = PPO(env, save_dir) # 建立PPO模型
total_timesteps = 10000 # 循环设定总步数
model.learn(total_timesteps=total_timesteps) # 模型训练
model.save('wyf-pleg') # 保存模型
print('------模型构建完成---开始仿真训练------')
# del model # 删除模型
# model = PPO.load(save_dir) # 读取模型
obs, state, dones = env.reset(), None, [False]
while True:
    action, _ = model.get_action(obs)
    obs, rew, done, info = env.step(action)
    if done:
        obs = env.reset()
        time.sleep(1/30)