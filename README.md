# bullet_robot

## 构建环境
```ssh
./build_environment.sh
```
## 由尺寸计算转动惯量
```python
python inertia_calculator.py
```
## 由转动惯量计算尺寸
```python
python length_calculator.py
```
## Ubuntu18安装Gym

[安装须知](https://blog.csdn.net/ms961516792/article/details/79122914)
```ssh
pip install gym
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