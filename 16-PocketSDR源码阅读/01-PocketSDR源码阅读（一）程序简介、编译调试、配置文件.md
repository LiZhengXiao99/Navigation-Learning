> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、程序简介

### 1、概述

PocketSDR 是 RTKLIB 作者写的一款 GNSS 软件接收机，包含一个射频前端和一套后处理 GNSS 接收机程序（只支持后处理），实现了一整套完整的 GNSS 接收机功能，采用 C、Python 编写，支持几乎所有的 GNSS 信号（比商业接收机支持的还要多），目前 0.8 版本的程序支持的信号如下：

* **GPS**: L1C/A, L1CP, L1CD, L2CM, L5I, L5Q, 
* **GLONASS**: L1C/A, L2C/A, L3OCD, L3OCP,
* **Galileo**: E1B, E1C, E5aI, E5aQ, E5bI, E5bQ, E6B, E6C,
* **QZSS**: L1C/A, L1C/B, L1CP,L1CD, L1S, L2CM, L5I, L5Q, L5SI, L5SQ, L6D, L6E, 
* **BeiDou**: B1I, B1CP, B1CD, B2I,B2aD, B2aP, B2bI, B3I, 
* **NavIC**: L5-SPS, SBAS: L1C/A, L5I, L5Q

不同于 RTKLIB 等 GNSS 数据处理软件，PocketSDR 直接对 GNSS 信号进行处理，



### 2、文件结构



![image-20240105200733817](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240105200733817.png)

### 3、射频前端





![image-20240105200757404](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240105200757404.png)





## 二、编译调试



























