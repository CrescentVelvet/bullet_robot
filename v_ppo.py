# 保存PPO模型
from v_network import FeedForwardNN
class PPO:
    def __init__(self, env):
        self.env = env # 环境设置
        self.obs_dim = env.observation_space.shape[0] # 观测空间的大小
        self.act_dim = env.action_space.shape[0] # 动作空间的大小
        self.actor  = FeedForwardNN(self.obs_dim, self.act_dim) # actor网络初始化
        self.critic = FeedForwardNN(self.obs_dim, 1) # critic网络初始化
        self._init_hyperparameters(hyperparameters) # 使用默认的超参数
    def learn(self, total_timesteps):
        t_so_far = 0 # 到目前为止已经训练了多少步
        while t_so_far < total_timesteps: # 循环设定总步数
            return True
    def _init_hyperparameters(self, hyperparameters): # 定义默认的超参数
	    self.timesteps_per_batch = 4800 # 运行一个batch的步数,batch是一次批处理的数据集样本数
	    self.max_timesteps_per_episode = 1600 # 运行一个episode的最大步数,episode是agent在环境中根据某个策略执行一系列action到结束的过程
	    self.n_updates_per_iteration = 5 # 每次迭代更新actor/critic的次数
	    self.lr = 0.005 # actor优化器的学习率
	    self.gamma = 0.95 # 计算reward时的折扣系数
	    self.clip = 0.2 # 建议为0.2
	    self.render = False # 是否在rollout期间可视化
	    self.save_freq = 10 # 迭代多少次保存一下
	    self.seed = None # 设置程序种子,用于结果再现
	    for param, val in hyperparameters.items(): # 将超参数的默认值修改为自定义值
	        exec('self.' + param + ' = ' + str(val))
	    if self.seed != None: # 设置程序种子
	        assert (type(self.seed) == int) # 首先检查程序种子是否有效(int)
	        torch.manual_seed(self.seed) # 然后设置程序种子
    def rollout(self): # rollout就是运行当前actor策略从一组episode中收集数据batch
	    batch_obs = [] # batch观测空间,维度(一个batch步数,观测空间维度)
	    batch_acts = [] # batch动作空间,维度(一个batch步数,动作空间维度)
	    batch_log_probs = [] # batch的log可能性,维度(一个batch步数)
	    batch_rews = [] # batch奖励,维度(一个batch步数)
	    batch_rtgs = [] # batch奖励,维度(一个batch步数)
	    batch_lens = [] # batch长度,维度(episode数)
        # ep_rews = [] # episode奖励，会被清除
        t = 0 # 记录到目前为止这个batch已经运行了多少步
        while t < self.timesteps_per_batch: # 循环一个batch最大步数
            ep_rews = [] # 一个episode的奖励
            obs = self.env.reset() # 重置环境,得到初始的观测空间obs
            done = False # 一个episode是否已结束
            for ep_t in range(self.max_timesteps_per_episode): # 循环一个episode最大步数
                if self.render: # 如果render=True就把环境可视化
                    self.env.render() # 环境可视化
                t += 1 # batch运行步数加一
                batch_obs.append(obs) # 记录batch中的观测空间obs
                action, log_prob = self.get_action(obs) # 计算动作空间act和log可能性
                obs, rew, done, _ = self.env.step(action) # 在环境中执行一步step，得到奖励rew
                ep_rews.append(rew) # 记录episode的奖励rew
                batch_acts.append(action) # 记录batch的动作空间act
                batch_log_probs.append(log_prob) # 记录batch的log可能性
                if done: # 如果episode结束了就退出循环
                    break
            batch_lens.append(ep_t + 1) # 记录batch长度
            batch_rews.append(ep_rews) # 记录batch奖励