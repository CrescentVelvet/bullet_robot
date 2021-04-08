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

查看baseline练的机器人走路
```
python -m pybullet_envs.examples.enjoy_TF_HumanoidBulletEnv_v0_2017may
```
## debug记录
报错unindent does not match any outer indentation level：

tab和空格缩进混用

安装box2d失败：
```
sudo apt-get install swig
pip install box2d
pip install box2d-kengz

```
## 使用tensorboardX监视器
```
pip install tensorboardX
```
在命令行里输入
```
tensorboard --logdir=/home/zjunlict-vision-1/Desktop/bullet_robot/runs/
```
或者进入目录下
```
tensorboard --logdir=runs/
```
如果报错Not a TBLoader or TBPlugin subclass，重装一下
```
pip uninstall tensorboard
pip uninstall tensorboard-plugin-wit
pip install tensorboard
```
会生成网址http://localhost:6006/，进入即可查看，30s刷新一次
## 文件说明
```
modelset.py
```
显示bullet模型信息
```
train.py
```
强化学习代码，大杂烩
```
v_main.py
```
运行自己写的v_ppo代码
```
s_main.py
```
运行baselines3库PPO代码
```
d_motion.py
```
仿真模型后躺爬起站立

## 效果图片
<img width=850 src="https://img-blog.csdnimg.cn/20210401215002255.png" alt="模型爬起站立"/>

模型爬起站立

## motion代码
python

小括号( )：代表tuple元组数据类型，元组是一种不可变序列

中括号[ ]，代表list列表数据类型

大括号{ }花括号：代表dict字典数据类型，字典是由键对值组组成

角度转弧度：math.radians---Deg2Rad

弧度转角度：math.degrees---Rad2Deg

把/usr/include/eigen3/Eigen文件夹复制到bullet_robot下，编译c文件：
```
gcc ./d_test.c -o ./d_test.out
./d_test.out
```
### motion思路
motion_hub.cpp里527行WALK_TO_BALL调用GenerateNewGait函数，对步态单元tmp_gait和其序列gait_queue进行赋值

motion_hub.cpp是整体运动控制,包含了很多状态与条件.walk_test.cpp是测试步行的简化版.

motion_hub.cpp里1302行GUARD_LEFT调用ServoPublish函数，传入单步函数GetOneStep计算结果，发送控制信号给关节舵机

dmotion_math.hpp里327行的ServoPublish函数，对关节舵机序列cur_servo_angles进行赋值

walk_test.cpp里225行调用PendulumWalk.cpp里的步态函数GiveAStepTick，传入gait_queue的首位，再删除gait_queue首位

GiveAStepTick调用GiveAStep函数和GiveATick函数

GiveAStep调用OneFootLanding.cpp里的单步函数GetOneStep

GiveAStep调用ThreeInterpolation.cpp里的三次插值曲线函数GetPoints和EvalHere和Calculate

EvalHere调用Polynomial.hpp里的计算多项式函数compute

compute调用PolynomialInternal.hpp里的迭代函数compute

Calculate调用Eigen库的矩阵LU分解函数lu().solve

