> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、softGNSS 简介

### 1、概述





卫星信号由导航电文、测距码、载波三个层次组成，





GNSS 原始的信号频率高达 1.5 Ghz，且信号过于微弱，









### 2、相关工作

* CSDN 博客：[基于MATLAB编写的GNSS_SDR（GNSS软件接收机）——自学笔记（1）](https://jldxwsj.blog.csdn.net/article/details/116794856)

* 基于 softGNSS 实现的深组合：[kongtian-SiBu / ZCJ_GNSSINS_DeepIntegration](https://github.com/kongtian-SiBu/ZCJ_GNSSINS_DeepIntegration)

  > 作者是空天院的硕士，他还上传的他和他同学的一些硕士论文开题报告，相关论文。





### 3、我用 softGNSS 做的事

* 我的本科大创课题是《面向城市复杂环境车载INSGNSS深组合软件接收机研发》

* 基于 softGNSS 学习 GNSS 基带信号处理的原理

* 老师在淘宝上买的射频前端可以采集数据
* 但是买回来的射频前端不好调参数，也没给通信说明，不好自己写采集数据的程序，所以我打算自己实现射频前端，考虑了如下几种射频前端方案：
  * **GNSS 射频芯片**：要自己画开发板，参数难以自定义，没有高频信号处理的经验做不来。
  * **USRP**：太贵，问了国内的代理，最便宜的要一万多。
  * **ZYNQ + AD9361**：FPGA学习难度大，而且深组合算法我打算直接在开源 GNSS 软件接收机上改，用不上 FPGA 部分。
  * **Hack rf one**：

* 







### 4、文件结构





### 5、程序执行流程图







## 二、程序使用

### 1、射频前端





### 2、参数设置

程序的根目录里：







必须要设置好的配置有：





### 3、处理开源数据









### 4、处理自己采集的数据







### 5、用 Hack RF one 采集数据进行处理

> 暂时还没买 Hack RF，等做成了再更新





## 三、程序执行流程

### 1、init.m：程序入口

* `clear; close all; clc;`清理工作空间，关闭打开的窗口。
* format 
* addpath 添加 include、geoFunctions 文件夹到工作空间。
* 调用 initSettings() 生成选项结构体 settings。

* 尝试打开设置的文件路径，打开失败输出错误信息，程序终止。
* 打开成功就调用 probeData() 绘制原始 IF 数据的时域采样波、直方图统计、功率谱密度，以对采样数据进行时域和频域分析。
* 





### 2、probeData()：绘制原始数据的时域采样波、直方图统计、功率谱密度









### 3、postProcessing：开始后处理

包括解算初始化、信号捕获、通道初始化、信号跟踪、导航定位结算、结果绘制



























