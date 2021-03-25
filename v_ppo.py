# 建立PPO模型
import torch
import pleg.envs.environment
from v_network import FeedForwardNN
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
class PPO:
    def __init__(self, env, **hyperparameters):
        self.env = env # 环境设置
        self.obs_dim = env.observation_space.shape[0] # 观测空间的大小
        self.act_dim = env.action_space.shape[0] # 动作空间的大小
        self.act_std = 1 # 之后计算cov_mat协方差矩阵时需要用到的对角阵的主对角线值
        self.actor  = FeedForwardNN(self.obs_dim, self.act_dim) # actor网络初始化
        self.critic = FeedForwardNN(self.obs_dim, 1) # critic网络初始化
        self._init_hyperparameters(hyperparameters) # 使用默认的超参数
        self.actor_optim = torch.optim.Adam(self.actor.parameters(), lr=self.lr) # 定义Adam优化器
        self.action_var = torch.full((self.act_dim,), self.act_std*self.act_std, dtype=torch.float).to(device)
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
        self.lr = 0.005 # 设置Adam参数
        for param, val in hyperparameters.items(): # 将超参数的默认值修改为自定义值
            exec('self.' + param + ' = ' + str(val))
        if self.seed != None: # 设置程序种子
            assert (type(self.seed) == int) # 首先检查程序种子是否有效(int)
            torch.manual_seed(self.seed) # 然后设置程序种子
    def learn(self, total_timesteps):
        t_so_far = 0 # 到目前为止已经训练了多少步
        while t_so_far < total_timesteps: # 循环设定总步数
            batch_obs, batch_acts, batch_log_probs, batch_rtgs, batch_lens = self.rollout() # 运行策略收集数据batch
            print('----------------', batch_obs)
            V, _ = self.evaluate(batch_obs, batch_acts) # 计算obs的预测值（V-value）
            A_k = batch_rtgs - V.detach() # 计算优势函数
            A_k = (A_k - A_k.mean()) / (A_k.std() + 1e-10) # 对优势函数进行规范化
            ratios = torch.exp(curr_log_probs - batch_log_probs) # 两个log概率相减再取e的幂
            surr1 = ratios * A_k # 计算代替损失
            surr2 = torch.clamp(ratios, 1 - self.clip, 1 + self.clip) * A_k
            actor_loss = (-torch.min(surr1, surr2)).mean() # 计算loss
            self.actor_optim.zero_grad() # 计算梯度
            actor_loss.backward() # 反向传播
            self.actor_optim.step()
            self.critic_optim = Adam(self.critic.parameters(), lr=self.lr) # 定义critic的Adam优化器
            critic_loss = nn.MSELoss()(V, batch_rtgs) # 计算loss
            self.critic_optim.zero_grad() # 计算梯度
            critic_loss.backward() # 反向传播
            self.critic_optim.step()
    def rollout(self): # rollout就是运行当前actor策略并从一组episode中收集数据batch
        batch_obs = [] # batch观测空间,维度(一个batch步数,观测空间维度)
        batch_acts = [] # batch动作空间,维度(一个batch步数,动作空间维度)
        batch_log_probs = [] # batch的log概率,维度(一个batch步数)
        batch_rews = [] # batch奖励rews,维度(episode数，一个episode步数)
        batch_rtgs = [] # batch后续奖励rtgs,维度(一个batch步数)
        batch_lens = [] # batch长度,维度(episode数)
        t = 0 # 记录到目前为止这个batch已经运行了多少步
        while t < self.timesteps_per_batch: # 循环一个batch最大步数
            ep_rews = [] # 一个episode的奖励rews
            obs = self.env.reset() # 重置环境,得到初始的观测空间obs
            done = False # 一个episode是否已结束
            for ep_t in range(self.max_timesteps_per_episode): # 循环一个episode最大步数
                if self.render: # 如果render=True就把环境可视化
                    self.env.render() # 环境可视化
                t += 1 # batch运行步数加一
                batch_obs.append(obs) # 记录batch中的观测空间obs
                action, log_prob = self.get_action(obs) # 计算动作空间act和log概率
                obs, rew, done, _ = self.env.step(action) # 在环境中执行一步step，得到奖励rew
                ep_rews.append(rew) # 记录episode的奖励rews
                batch_acts.append(action) # 记录batch的动作空间act
                batch_log_probs.append(log_prob) # 记录batch的log概率
                if done: # 如果episode结束了就退出循环
                    break
            batch_lens.append(ep_t + 1) # 记录batch长度
            batch_rews.append(ep_rews) # 记录batch奖励rews
        batch_obs = torch.tensor(batch_obs, dtype=torch.float) # 转换为张量
        batch_acts = torch.tensor(batch_acts, dtype=torch.float) # 转换为张量
        batch_log_probs = torch.tensor(batch_log_probs, dtype=torch.float) # 转换为张量
        batch_rtgs = self.compute_rtgs(batch_rews) # 计算batch奖励rews的后续奖励rtgs
    def get_action(self, obs): # 根据一个观测空间obs来得到动作空间act，输入obs，输出act和log概率
        mean = self.actor(obs) # 使用actor网络在前向输出mean动作act
        action_var = self.action_var.expand_as(mean)
        cov_mat = torch.diag_embed(action_var).to(device)
        dist = torch.distributions.multivariate_normal.MultivariateNormal(mean, cov_mat) # 使用act和std创建多元正态分布
        action = dist.sample() # 对接近均值的act进行采样，随机选取一个act
        log_prob = dist.log_prob(action) # 计算该act在分布中的log概率
        return action.detach().numpy(), log_prob.detach() # 返回采样的act和该act在分布中的log概率
    def compute_rtgs(self, batch_rews): # 计算给定batch奖励rews的后续奖励rtgs，输入batch奖励rews，输出后续奖励rtgs（Q-value）
        batch_rtgs = [] # batch后续奖励rtgs
        for ep_rews in reversed(batch_rews): # 循环每一个episode
            discounted_reward = 0 # 到目前为止的奖励
            for rew in reversed(ep_rews): # 循环episode中的奖励rews
                discounted_reward = rew + discounted_reward * self.gamma # 奖励乘上折扣系数加上循环次数，在一开始比较低
                batch_rtgs.insert(0, discounted_reward) # 把奖励插入后续奖励rtgs的第一个位置
        batch_rtgs = torch.tensor(batch_rtgs, dtype=torch.float) # 将后续奖励rtgs转换为张量tensor
        return batch_rtgs # 返回后续奖励rtgs
    def evaluate(self, batch_obs, batch_acts): # 计算log概率和obs的value，输入obs和act，输出log概率和obs的预测值（V-value）
        V = self.critic(batch_obs).squeeze() # 查询critic网络中对于obs的V的值，V的维度应该与batch_rtgs相同
        mean = self.actor(batch_obs) # 这三句代码
        action_var = self.action_var.expand_as(mean)
        cov_mat = torch.diag_embed(action_var).to(device)
        dist = torch.distributions.multivariate_normal.MultivariateNormal(mean, cov_mat) # 类似于get_action函数
        log_probs = dist.log_prob(batch_acts) # 使用actor网络计算log概率
        return V, log_probs # 返回V-value和log概率

