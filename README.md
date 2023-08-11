![cover](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/cover.png)

[TOC]



## 导航定位开源程序介绍

### RTKLIB：最知名的GNSS数据处理开源软件



### GAMP：基于RTKLIB的后处理PPP

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的，是用纯 C 语言编写，由于做了简化，代码比 RTKLIB 原版还要简单；使用也非常简单，软件包里直接有 VS 工程，和组织好的配置、数据文件，简单改改路径就能算出结果。

### PSINS：MATLAB-C++ 捷联惯导工具箱

PSINS（Precise Strapdown Inertial Navigation System 高精度捷联惯导系统算法）工具箱由西北工业大学自动化学院惯性技术教研室严恭敏老师开发和维护。工具箱分为Matlab和C++两部分。主要应用于**捷联惯导**系统的数据处理和算法验证开发，它包括**惯性传感器数据分析**、**惯组标定**、**初始对准**、**惯导AVP**（姿态-速度-位置）更新解算、**组合导航**Kalman滤波等功能。C++部分采用VC6编写，可以用于嵌入式开发。

### PPPLib：



### goGPS：MATLAB-GNSS数据处理



### GICI-LIB：GNSS+INS+Camera 图优化融合定位

GICI-LIB 全称 **G**NSS/**I**NS/**C**amera **I**ntegrated Navigation Library，是上海交大最新开源的一套基于图优化的 GNSS+INS+Camera 集成导航定位库。基于 RTKLIB 处理 I/O 流、编解码；基于 OKVIS 因子图优化类型封装；基于 SVO 做特征提取。以 GNSS 为主，再加入 INS、Camera 做组合，支持相当多的数据格式、定位模式，包含很多 GNSS 因子、惯导因子、视觉因子及运动约束。以处理实时数据为主，后处理也采用模拟实时数据处理的方式进行。

### Ginan：



### ORB-SLAM：



### VINS：





### GraphGNSSLib：基于 RTKLIB 的图优化 GNSS 定位



### BNC：Qt-GNSS 实时数据流处理

BNC（The BKG Ntrip Client）是由德国 BKG 开发的一款软件，它的功能丰富，可以选择不同分析中心以及挂载点，接收实时数据，并实现对其编码、解码、转换、处理和分析等功能。

### GNSS-SDR：GNSS 软件接收机

GNSS-SDR（**GNSS:** **G**lobal **N**avigation **S**atellite **S**ystems、**SDR:** **S**oftware **D**efined **R**eceiver），在 GitHub 上搜索 “GNSS” 排第一的仓库，收藏量也高达 1.3k。与上面列举的 RTKLIB、GAMP 等 GNSS 数据处理软件不同，GNSS-SDR 直接对信号进行处理，是一个用 C++ 实现的 GNSS 软件接收机开源项目。有了 GNSS-SDR，用户可以通过创建一个图来构建 GNSS 软件接收器，图中的节点是信号处理块，线条代表它们之间的数据流。该软件为不同的合适射频前端提供接口，并实现从接收器一直到 PVT 解算的所有功能。它的设计允许任何形式的定制，包括信号源、信号处理算法、与其他系统的互操作性、输出格式的互换，并为所有中间信号、参数和变量提供接口。

软件旨在促进新信号处理技术的发展，提供一种简便的方法来衡量这些技术对接收机整体性能的影响。通过对每个软件模块进行系统功能验证，以及使用真实和合成信号对整个接收机进行实验验证，对所有流程进行测试。

目前的技术仍无法以卫星发射频率（约 1.5 GHz）对信号进行数字处理，因此我们仍需要一个射频前端，将信号降频到较低频率，在此过程中进行一些滤波和放大，并以一定的速率进行采样，将量化的数字原始采样流传输到计算平台（通过 USB、以太网等）。

软件接收机可在普通的 PC 中运行，并通过 USB 和以太网总线为各种市售或定制的射频前端提供接口，使处理算法适应不同的采样频率、中间频率和采样分辨率。它还可以处理存储在文件中的原始数据样本。软件对可用的卫星信号进行信号采集和跟踪，对导航信息进行解码，并计算定位算法所需的观测值，最终实现完整导航解决方案。处理输出可存储在 RINEX 文件中，或通过 TCP/IP 服务器以 RTCM 3.2 消息形式实时传输。导航结果以 KML 和 GeoJSON 格式存储。









































