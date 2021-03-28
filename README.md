# bullet_robot

## 构建环境
```ssh
./build_environment.sh
```
由尺寸计算转动惯量
```python
python inertia_calculator.py
```
由转动惯量计算尺寸
```python
python length_calculator.py
```
[Gym安装教程](https://blog.csdn.net/ms961516792/article/details/79122914)
```ssh
git clone https://github.com/openai/gym.git
cd gym
pip install -e .
```
[mujoco安装教程](https://blog.csdn.net/jianghao_ava/article/details/81062337)
[与问题解决](https://blog.csdn.net/jianghao_ava/article/details/80874254)
```
Full name	velvet
Email address	crescentvelvet3@gmail.com
Computer id	LINUX_SBEP2I_25HK8HP2W6A35SNN6LG6EDHKT093340
IP address	210.32.151.94
```
## DDPG算法

DDPG用来解决连续动作问题

[baselines3知乎教程](https://zhuanlan.zhihu.com/p/149771220)
[baselines3代码](https://github.com/DLR-RM/stable-baselines3)
[baselines3官方文档](https://stable-baselines3.readthedocs.io/en/master/)

[DDPG参数调节](https://www.zhihu.com/question/309162916)
[DDPG调参理论](https://zhuanlan.zhihu.com/p/345353294)
[DDPG代码](https://zhuanlan.zhihu.com/p/47873624)

[pytorch实现DDPG](https://github.com/ghliu/pytorch-ddpg)
[pytorch实现PPO2](https://github.com/andompesta/ppo2)

[PPO算法理论](https://zhuanlan.zhihu.com/p/111068310)
[PPO算法实战](https://zhuanlan.zhihu.com/p/111049450)

[PPO机器人代码](https://github.com/openai/baselines/tree/master/baselines/ppo2)
[PPO机器人演示](https://openai.com/blog/openai-baselines-ppo/)

[pytorch的PPO思路](https://blog.csdn.net/melody_cjw/article/details/112851552)

## baselines3最强pytorch下的PPO

[简介](https://araffin.github.io/post/sb3/)
[代码](https://github.com/DLR-RM/stable-baselines3)
[文档](https://stable-baselines3.readthedocs.io/en/master/)
[框架](https://github.com/DLR-RM/rl-baselines3-zoo)
[教程](https://github.com/araffin/rl-tutorial-jnrr19)

## debug记录
报错unindent does not match any outer indentation level：

tab和空格缩进混用

安装box2d失败：
```
sudo apt-get install swig
pip install box2d
pip install box2d-kengz

```

train.py是最早的运行代码，大杂烩

v_main.py是运行我自己写的PPO的代码

s_main.py是运行baseline的PPO的代码
