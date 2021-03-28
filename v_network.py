# 用于在PPO模型中定义Actor-Critic网络的神经网络模块,包含一个前馈神经网络
# Actor-Critic模型会定期保存到二进制文件ppo_actor.pth和ppo_critic.pth中,可以在测试或继续训练时加载它们
import torch
import torch.nn as nn # 包含了卷积,池化,RNN,Loss等计算
import torch.nn.functional as F  # 包含了激活函数,规范化函数,Dropout等函数
import numpy as np
class FeedForwardNN(nn.Module):
    def __init__(self, in_dim, out_dim): # 初始化函数
        super(FeedForwardNN, self).__init__()
        self.layer1 = nn.Linear(in_dim, 64) # 三层全连接层,in_dim->64->64->out_dim
        self.layer2 = nn.Linear(64, 64)
        self.layer3 = nn.Linear(64, out_dim)
        # self.layer = nn.Linear(in_dim, out_dim) # 只用一层全连接层
    def forward(self, obs): # 前向传播函数,用来定义actor和critic,输入obs输出action
        if isinstance(obs, np.ndarray): # 将obs从数组array转换为张量tensor
            obs = torch.tensor(obs, dtype=torch.float)
        activation1 = F.relu(self.layer1(obs)) # 使用ReLU函数激活,经过三层全连接层后输出
        activation2 = F.relu(self.layer2(activation1))
        output = self.layer3(activation2)
        # output = F.relu(self.layer(obs)) # 只用一层全连接层
        return output