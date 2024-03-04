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



<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240105200733817.png" alt="image-20240105200733817" style="zoom:50%;" />

### 3、射频前端

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240105200757404.png" alt="image-20240105200757404" style="zoom: 33%;" />





MAX2771是美信（亚德诺的子公司）开发的一款专用 GNSS 射频芯片，集成了芯片上集成了完整的接收器链，包括双输入低噪声放大器和混频器，然后是滤波器、PGA 和多位 ADC，以及分数 N 频率合成器和晶体振荡器，该接收器的总级联该接收器的总级联噪声系数低至 1.4dB。采用片上单片式滤波器，完全不需要外部中频滤波器，只需少量外部元件即可构成一个完整、低成本的 GNSS 接收机。特性如下：

* **多星座支持**：GPS、Galileo、GLONASS、BeiDou、IRNSS、QZSS、 SBAS
* **多波段支持**：L1、L2、L5、E1、E5、E6、B1、B2、B3
* **可编程中频带宽**：2.5MHz、4.2MHz、 8.7MHz、16.4MHz、23.4MHz、36MHz
  * 支持宽带载波，适用于精密定位应用，例如 GPS L5、伽利略 E5可在低中频或零中频模式下运行。
* 片上 LNA 支持多个频带
* 级联噪声系数为 1.4dB，级联增益为 110dB 级联增益，从 PGA，增益控制范围为 59dB
* 可编程中频中心频率集成 VCO 的分数 N 合成器支持广泛的基准频率
* 集成晶体振荡器
* 电源电压范围：2.7V 至 3.3V
* 符合 RoHS 规范的 28 引脚薄型 QFN 无引线封装（5 毫米 x 5 毫米）




 



#### 1. 低噪声放大器 LNA

MAX2771 集成了两个低噪声放大器，一个用于 L1 频段（高频段），另一个用于 L2/L5 频段 (低频段）。两个输入端都需要交流耦合电容器。
配置 1 寄存器中的 LNAMODE 位可控制 
两个 LNA 的模式。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240229090931518.png" alt="image-20240229090931518" style="zoom:67%;" />

高带 LNA 输入阻抗在频率为 1575MHz 时匹配为 50Ω。
在 1575MHz 频率下，高带 LNA 输入阻抗与 50Ω 匹配。
匹配电路。在频率为 1227MHz 时，低频 LNA 输入阻抗与 50Ω 匹配、 
只要使用指定的低频段外部匹配电路 
使用指定的低频外部匹配电路。

每个 LNA 的输出都接到一个单独的引脚上，高频段 LNA 的输出阻抗与 1575Ω 的频率相匹配，低频 LNA 的频率为 1227MHz，输入阻抗都与 50Ω 匹配。

#### 2. 混频器

















## 二、编译调试



























