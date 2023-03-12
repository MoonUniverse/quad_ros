# MoonUniverse  QuadCopter

## Hardware
1.穿越机机架（轴距250）目前使用的是px4 vision kit 中的机架

2.飞控（pixhawk 6c或6x）

3.板载pc（NUCI7 8代 或 RB5 高通865）

4.动力系统（T-motor 穿越机电机电调 5045桨叶）

5.相机（realsense D435i 或 D455）

6.遥控器（DJI LB2 或 任意支持dbus的遥控器）

7.电池（格式 4s 2200mA）

## Software

### 1.px4 mavros 

### 2.视觉惯性里程计
##### 目前使用的是港科大的vins，后续可能改进其中的featureTrack，用一些深度学习的特征点提取和追踪。

### 3.control控制目前使用的是RPG Quadrotor Control 移植了其中的position control部分。
代码位于如下位置
```C++
quad_ros/navigation/happymoon_quad_control
```
原始控制仓库如下：
[RPG Quadrotor Control](https://github.com/uzh-rpg/rpg_quadrotor_control)


## 仿真


## 实机测试