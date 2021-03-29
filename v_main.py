# 可执行文件,调用v_argument.py解析命令行参数,初始化环境和PPO模型
import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
import time
import tensorboardX
from v_ppo import PPO
import pleg.envs.environment
v_tensor_board = tensorboardX.SummaryWriter(comment='_v_main') # 创建监视器
env = gym.make('MyEnv-v0', render=False) # 构建环境
env = env.unwrapped # 解除环境限制
print('------环境构建完成---开始构建模型------')
save_dir='/home/zjunlict-vision-1/Desktop/bullet_robot/model' # 设置模型保存地址
model = PPO(env, save_dir) # 建立PPO模型
total_timesteps = 10000 # 循环设定总步数
model.learn(total_timesteps=total_timesteps) # 模型训练
model.save('v-wyf-pleg') # 保存模型
print('------模型构建完成---开始仿真训练------')
obs, state, dones = env.reset(), None, [False] # 重设环境
while True: # 循环训练
    action, _ = model.get_action(obs)
    obs, reward, done, info = env.step(action)
    v_tensor_board.add_scalar('reward', reward)
    if done:
        obs = env.reset()
        time.sleep(1/30)