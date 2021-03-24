import os
os.environ['CUDA_VISIBLE_DEVICES'] = "2"
import gym
import torch
import helper
import argparse
import pleg.envs.environment
import numpy as np
from my_ppo2 import PPO2
from collections import deque
from wrappers import Reset, Monitor
from memory_collector import MemoryCollector

# 参数设置函数
def __pars_args__():
    parser = argparse.ArgumentParser(description='PPO')
    parser.add_argument('-gam', '--gamma', type=float, default=0.95,
                        help='discounting factor')
    parser.add_argument('-lam', '--lam', type=float, default=0.99,
                        help='discounting factor')
    parser.add_argument('-m_grad', '--max_grad_norm', type=float, default=0.5, # 梯度的最大范数
                        help='max norm of the gradients')
    parser.add_argument('-ent_coef', '--ent_coef', type=float, default=0., # 优化目标中的策略熵系数
                        help='policy entropy coefficient in the optimization objective')
    parser.add_argument('-vf_coef', '--vf_coef', type=float, default=0.5, # 优化目标中的值函数损失系数
                        help='value function loss coefficient in the optimization objective')
    parser.add_argument("-tot_t", "--total_timesteps", type=int, default=10000000, # 时间步数（即在环境中执行的操作数）
                        help="number of timesteps (i.e. number of actions taken in the environment)")
    parser.add_argument('-n_train_ep', '--number_train_epoch', type=int, default=4, # 每次更新的训练次数
                        help='number of training epochs per update')
    parser.add_argument('-clp_rng', '--clip_range', type=float, default=0.2, # 削波范围、常数或调度函数[0,1]->R+，其中1表示训练开始，0表示训练结束
                        help='clipping range, constant or schedule function [0,1] -> R+ where 1 is beginning of the training and 0 is the end of the training')
    parser.add_argument('--seed', type=int, default=1, # 随机数种子
                        help='random seed (default: 1)')
    parser.add_argument('-lr', '--learning_rate', type=float, default=3e-4, # 学习率
                        help='learning rate (default: 0.001)')
    parser.add_argument('-bs', '--batch_size', type=int, default=512, # 学习过程使用的batch size
                        help='batch size used during learning')
    parser.add_argument('-mini_bs', '--mini_batchs', type=int, default=4, # 每次更新的最小batch size。对于经常性策略，应小于或等于并行运行的环境数
                        help='number of training minibatches per update. For recurrent policies, should be smaller or equal than number of environments run in parallel.')
    parser.add_argument('-n_step', '--n_step', type=int, default=1024, # 每次更新矢量化环境的步数
                        help='number of steps of the vectorized environment per update')
    parser.add_argument('--hidden_dim', type=int, default=64,
                        help='hidden dimension')
    parser.add_argument('-m_path', '--model_path', default='./trained_model', # 模型存储路径
                        help='Path to save the model')
    parser.add_argument('-v_path', '--monitor_path', default='./monitor',
                        help='Path to save monitor of agent')
    parser.add_argument('-v', '--version', default='0',
                        help='version')
    parser.add_argument('-save_every', '--save_every', type=int, default=10, # 保存事件之间的时间步数
                        help='number of timesteps between saving events')
    parser.add_argument('-log_every', '--log_every', type=int, default=10, # 日志事件之间的时间步数
                        help='number of timesteps between logs events')
    parser.add_argument("--max_steps", type=int, default=100,
                        help="Max step for an episode")
    parser.add_argument("--state_size", type=list, default=[56, 84],
                        help="Frame size")
    parser.add_argument("-uc", "--use_cuda", type=bool, default=False,
                        help="Use cuda")
    return parser.parse_args()

# 单步设置函数
def step_setup(args, train_model, device):
    optimizer = torch.optim.Adam(train_model.parameters(), lr=args.learning_rate, eps=1e-5) # 构建优化器，用来保存当前的状态，并根据计算得到的梯度来更新参数
    def train_step_fn(obs, returns, dones, old_actions, old_values, old_neg_log_prbs): # 单步训练函数
        assert old_neg_log_prbs.min() > 0
        obs = torch.tensor(obs).float().to(device)
        returns = torch.tensor(returns).float().to(device)
        old_actions = torch.tensor(old_actions).to(device)
        old_values = torch.tensor(old_values).float().to(device)
        old_neg_log_prbs = torch.tensor(old_neg_log_prbs).float().to(device)
        with torch.set_grad_enabled(False):
            advantages = returns - old_values            
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8) # 归一化
        train_model.train()
        with torch.set_grad_enabled(True):
            train_model.zero_grad()
            value_f, actions, neg_log_probs, entropy = train_model(obs, action=old_actions)
            assert(actions.sum().item() == old_actions.sum().item())
            loss, pg_loss, value_loss, entropy_mean, approx_kl = train_model.loss(returns, value_f, neg_log_probs, entropy, advantages,
                                                                               old_values, old_neg_log_prbs,
                                                                               args.clip_range, args.ent_coef, args.vf_coef)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(train_model.parameters(), args.max_grad_norm)
            optimizer.step()
        return list(map(lambda x: x.detach().item(), [loss, pg_loss, value_loss, entropy_mean, approx_kl]))
    return train_step_fn, optimizer

if __name__ == '__main__':
    args = __pars_args__() # 参数设置
    device = torch.device("cuda:0" if args.use_cuda else "cpu")
    env = gym.make('MyEnv-v0', render=False) # 环境搭建
    env = env.unwrapped # 解除环境限制
    env = Monitor(env, helper.ensure_dir('./monitor'), allow_early_resets=True) # 添加监视器
    env = Reset(env)
    obs_size = env.observation_space.shape[0] # 观测空间
    act_space = env.action_space.shape[0] # 动作空间
    print('obs_size = ', obs_size)
    print('act_space = ', act_space)
    model = PPO2(input_dim=obs_size, hidden_dim=args.hidden_dim, action_space=act_space, dropout=0) # 输入层观测,隐层,输出层动作
    model.to(device)
    train_fn, optm = step_setup(args, model, device) # 训练函数配置
    memory_collector = MemoryCollector(env, model, args.n_step, args.gamma, args.lam, device) # 创建存储器以便训练batch
    ep_info_buf = deque(maxlen=100)
    n_env = 1
    n_batch = n_env * args.n_step
    n_updates = args.total_timesteps // args.batch_size
    n_batch_train = n_batch // args.mini_batchs
    for update in range(1, n_updates+1):
        assert n_batch % args.mini_batchs == 0
        # Start timer
        frac = 1.0 - (update - 1.0) / n_updates
        if update % args.log_every == 0:
            print('Stepping environment...')
        # Get minibatch
        obs, returns, dones, actions, values, neg_log_prb, ep_infos = memory_collector.run()
        print('main actions = ', actions)
        if update % args.log_every == 0:
            print('Done.')
        ep_info_buf.extend(ep_infos)
        mb_loss_vals = [] # 为每个小batch计算loss并加起来
        inds = np.arange(n_batch) # 对batch_size的每个元素创建索引数组
        for _ in range(args.number_train_epoch):
            np.random.shuffle(inds) # 将索引随机打散
            for start in range(0, n_batch, n_batch_train): # start从0到batch_size，以batch_train_size作为步长
                end = start + n_batch_train # end为start后一个步长
                mbinds = inds[start:end] # 小batch索引为batch索引的start到end
                slices = (arr[mbinds] for arr in (obs, returns, dones, actions, values, neg_log_prb))
                loss, pg_loss, value_loss, entropy, approx_kl = train_fn(*slices)
                mb_loss_vals.append([loss, pg_loss, value_loss, entropy, approx_kl])
        # Feedforward --> get losses --> update
        loss_vals = np.mean(mb_loss_vals, axis=0)
        loss_names = ['loss', 'policy_loss', 'value_loss', 'policy_entropy', 'approxkl']
        if update % args.log_every == 0 or update == 1:
            # Calculates if value function is a good predicator of the returns (ev > 1)
            # or if it's just worse than predicting nothing (ev =< 0)
            ev = helper.explained_variance(values, returns)
            print("misc/serial_timesteps", update * args.n_step)
            print("misc/n_updates", update)
            print("misc/total_timesteps", update * n_batch)
            print("misc/explained_variance", float(ev))
            ep_rew_mean = helper.safemean([ep_info['r'] for ep_info in ep_info_buf])
            ep_len_mean = helper.safemean([ep_info['l'] for ep_info in ep_info_buf])
            print('ep_rew_mean', ep_rew_mean)
            print('ep_len_mean', ep_len_mean)
            for (loss_val, loss_name) in zip(loss_vals, loss_names):
                print('loss/' + loss_name, loss_val)
            plot_lines(loss_vals,  loss_names, update, 'losses')
            plot_lines([ep_rew_mean, ep_len_mean], ['reward', 'length'], update, 'rewards')
        if update % args.save_every == 0 or update == 1:
            # save model checkpoint
            helper.save_checkpoint({
                'update': update,
                'state_dict': model.state_dict(),
                'optimizer': optm.state_dict()
            },
                path=args.model_path,
                filename='train_net.cptk',
                version=args.version
            )
    env.close()