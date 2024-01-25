![cover](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/cover.png)

[TOC]

---

* 本仓库会长期更新，做为我学习的记录，分享出来，一方面是希望有人用得上，另一方面也是激励着自己坚持学下去。
* 我还有一个仓库【[Navigation-Hardware](https://github.com/LiZhengXiao99/Navigation-Hardware)】，两者相结合，希望打下一个扎实的软硬件基础。

- 有些程序会详细写（比如 RTKLIB、KF-GINS），另外一些可能不会写的面面俱到（比如 TGINS、PPPLIB），就只画个流程图、列举重点的函数。

- 水平不高，理解尚浅，列举的理论公式无法保证准确性，只能作为读代码时候的一个参考，千万不要照着我的笔记来写代码写论文，希望小伙伴们注意。

- 不建议直接下载整个仓库，内容太多可能大部分都用不着，Markdown 文件和 PDF 论文可以在线看，想要的文件也可以[单独下载](https://zhuanlan.zhihu.com/p/578116206)。Github 上的 Markdown 公式可能显示不完全，可以下载下来用 [Typora](https://typoraio.cn/) 看，强烈建议用纯白色做背景，否则显示出的图片会很奇怪。

- 整理差不多的笔记，会发到[知乎](https://www.zhihu.com/people/dao-ge-92-60)、[CSDN](https://blog.csdn.net/daoge2666)上，既看着方便，也是做个推广，辛苦写出来东西肯定希望能让更多人看到。

- 本仓库所有内容都可以随意转载，可以用于任何目的，不必征求我的意见。

- 如果您认可我的整理，并且想在该仓库更新时在 Github All activity 页面和 Github 通知页面看到本仓库更新，欢迎您 watch 本仓库而不是 fork 本仓库，fork 到自己名下的仓库只会保持您 fork 时的状态，不会自动更新。如果您愿意 star 一下那当然更好不过啦~

  <a href="https://star-history.com/?utm_source=bestxtools.com#LiZhengXiao99/Navigation-Learning&Date">

    <picture>
      <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/svg?repos=LiZhengXiao99/Navigation-Learning&type=Date&theme=dark" />
      <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/svg?repos=LiZhengXiao99/Navigation-Learning&type=Date" />
      <img alt="Star History Chart" src="https://api.star-history.com/svg?repos=LiZhengXiao99/Navigation-Learning&type=Date" width=80% />
    </picture>

  </a>

---

有问题欢迎随时联系我：

* **电话**：15255291038
* **微信**：lizhengxiao99
* **QQ**：1482275402
* **Email**：dauger@126.com

---

## 01-RTKLIB：GNSS 数据处理

RTKLIB 是全球导航卫星系统 GNSS 开源定位解算程序包，由日本东京海洋大学的高须知二（Tomoji Takasu）开发，由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好，许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来，适合作为 GNSS 入门学习的代码。

![RTKLIB](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB.png)

* **支持多个 GNSS 系统的标准和精密定位算法**，包括 GPS，GLONASS，Beidou，Galileo，QZSS 和 SBAS。

* **支持 9 种GNSS实时和后处理定位模式**：
  * **single**：伪距单点定位
  * **DGPS/DGNSS**：伪距差分
  * **kinematic**：载波动态相对定位，动态RTK，假设流动站是移动的，可以做车载定位
  * **Static**：载波静态相对定位，静态RTK，两站都是静止的，可以得到很高的精度
  * **Moving-Baseline**：两站都动，主要用来定姿
  * **Fixed**：固定坐标，解算模糊度、对流层、电离层等参数
  * **PPP-Kinematic**：动态精密单点定位
  * **PPP-Static**：静态精密单点定位
  * **PPP-Fixed**：PPP 固定坐标，解算模糊度、对流层、电离层等参数。

* **支持多种GNSS标准格式和协议**：RINEX2.10、RINEX2.11、RINEX2.12、RINEX3.00、RINEX3.01、RINEX3.02、RTCM2.3、RTCM3.1、RTCM3.2、BINEX、NTRIP、NMEA0183、SP3、ANTEX1.4、IONEX1.0、NGS PCV、EMS 2.0。

* **支持多种GNSS接收机专有数据协议格式**：NovAtel:OEM4/V/6，OEM3, OEMStar、Superstar II、 Hemisphere、Crescent、u‐blox:LEA-4T/5T/6T、SkyTraq、JAVAD 、GW10-II/III 和 NVS。

* **支持外部通信**：Serial、TCP/IP、NTRIP、本地日志文件、FTP 和 HTTP。

* **提供许多代码库和API**：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

---

## 02-GAMP：基于 RTKLIB 的后处理双频 PPP

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的，是用纯 C 语言编写，由于做了简化，代码比 RTKLIB 原版还要简单；使用也非常简单，软件包里直接有 VS 工程，和组织好的配置、数据文件，简单改改路径就能算出结果。

![GAMP](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GAMP.png)





---

## 03-PSINS：MATLAB-C++ 捷联惯导工具箱

PSINS（Precise Strapdown Inertial Navigation System 高精度捷联惯导系统算法）工具箱由西北工业大学自动化学院惯性技术教研室严恭敏老师开发和维护。工具箱分为Matlab和C++两部分。主要应用于**捷联惯导**系统的数据处理和算法验证开发，它包括**惯性传感器数据分析**、**惯组标定**、**初始对准**、**惯导AVP**（姿态-速度-位置）更新解算、**组合导航**Kalman滤波等功能。C++部分采用VC6编写，可以用于嵌入式开发。

![image-20231224221345981](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231224221345981.png)

---

## 04-PPPLib

**PPPLib** 全称 (**P**recise **P**oint **P**ositioning **Lib**rary) 是我的老师写的一套 C++ 版的 GNSS 数据处理软件，改写自 RTKLIB，支持单频、双频、三频的消电离层和非差非组合定位，支持与惯导的松紧组合。

![image-20231224221608754](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231224221608754.png)

---

## 05-goGPS：MATLAB-GNSS 数据处理

**goGPS**是一个处理GNSS原始数据的软件，最初支持单频低成本GPS接收机数据，但现在也可以用来处理多频多系统GNSS数据。它实现了多种算法来解算，目前包括两个主要的最小二乘法（LS）引擎：一个基于于组合观测数据（例如无电离层观测）；另一个能够使用所有的频率和记录的信号数据，而不进行任何组合（电离层延迟是正常方程的参数）。组合和不组合的引擎都支持PPP、NET解算。目前只支持静态测站的解算，还不能动态解算。

![goGPS Main Window](https://gogps-project.github.io/wiki/images/goGPS_MainWindow.png?raw=true)

---

## 06-GICI-LIB：GNSS+INS+Camera 图优化融合定位

GICI-LIB 全称 **G**NSS/**I**NS/**C**amera **I**ntegrated Navigation Library，是上海交大最新开源的一套基于图优化的 GNSS+INS+Camera 集成导航定位库。基于 RTKLIB 处理 I/O 流、编解码；基于 OKVIS 因子图优化类型封装；基于 SVO 做特征提取。以 GNSS 为主，再加入 INS、Camera 做组合，支持相当多的数据格式、定位模式，包含很多 GNSS 因子、惯导因子、视觉因子及运动约束。以处理实时数据为主，后处理也采用模拟实时数据处理的方式进行。

- **论文**：[GICI-LIB: A GNSS/INS/Camera Integrated Navigation Library](https://arxiv.org/abs/2306.13268)，可以[在这](https://arxiv.org/pdf/2306.13268.pdf)下载
- **源码**：[https://github.com/chichengcn/gici-open](https://github.com/chichengcn/gici-open)
- **数据**：[https://github.com/chichengcn/gici-open-dataset](https://github.com/chichengcn/gici-open-dataset)

典型应用方式如下图：

![1689512108793](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689512108793.png)

包含以下估计器：

![1690024555067](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690024555067.png)

---

## 07-SoftGSS：MATLAB-GPS 软件接收机

SoftGNSS 是《软件定义的GPS和伽利略接收机》附带的程序，MATLAB 编写，实现了一套最简单的 GNSS 软件接收机功；输入经过天线接收，射频前端滤波下变频后的数字中频信号文件，进行 GPS L1 C/A 码的捕获跟踪，生成伪距观测值，解译导航电文，最小二乘定位解算；代码量很小也很简单，适合作为 GNSS 基带数字信号处理的入门阅读程序。主要执行流程如下：

![SoftGNSS流程图](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/SoftGNSS%25E6%25B5%2581%25E7%25A8%258B%25E5%259B%25BE.png)

---

## 08-VINS：

与双目相机和 RGB-D 相机相比，单目相机具有结构简单、成本低和处理速度快的优点。然而，单目 VSLAM 存在尺度不确定性、无法对齐位姿和重力方向的自身缺点和快速运动导致的运动模糊的环境下容易跟踪丢失等不足。为弥补此问题，可将单目相机和 IMU 相结合的传感器融合，这种融合方案被称为单目视觉惯性里程计（Visual Inertial Odometry，VIO）或单目视觉惯性 SLAM（Visual-inertial SLAM，VINS）。

* IMU 也可以弥补视觉 SLAM 在短时间、快速运动上的不足，另外由于 IMU 不依赖外界环境信息，对环境变化不敏感，也可以在少纹理、明暗变化较大或者光线较弱场景内提供短期的定位方案以及位姿估计方案。

* 较之惯性信息，视觉里程计可以提供丰富的外界信息，在低速平稳的运动中位姿估计稳定，而且视觉里程计在长时间运行后的漂移较小，并且可以通过回环检测修正自身位置以减小累积误差。

香港科技大学沈劭劼团队开发的 VINS 系统，用了一种紧耦合的非线性优化方法。该团队在 2017 年发布的 VINS-Mono 通过在四元数上进行 IMU 的预积分，并且采用滑动窗口法融合 IMU 信息和相机观测到的特征数据，实现了数据的紧耦合。并且采用四自由度的图优化方法实现了回环检测模块，来得到全局约束。在 2019 年，该团队又发布了 VINS-Fusion，在 VINS-Mono 的基础上又加入了双目、双目+IMU 等更多的传感器类型，以及支持了 VINS 和 GPS 的融合。它支持在线标定相机及 IMU 参数及鱼眼相机模型，并且支持保存当前地图和加载过往地图。在与 IMU 的结合上，它采用了四元数积分方案，与视觉信息进行紧耦合，具有很强的鲁棒性和定位精度。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-1939bbe6498166275bf55eec6b68542f_r.jpg)

---

## 09-ORB-SLAM3：

ORB 指 **O**riented FAST and **r**otated **B**RIEF，是一种结合 FAST 和 BRIEF，并引入旋转不变性的一种特征点和描述子；SLAM 指 **S**imultaneous **L**ocalization **a**nd **M**apping，指的是同时进行实时定位和地图构建。

ORB-SLAM3 是**迄今为止，最完整的视觉惯性 SLAM 系统系统**，它是第一个集成了单目相机、双目相机、RGB-D相机，以及单目相机结合 IMU、双目相机结合 IMU 的 SLAM 系统。并且在 ORB-SLAM2 的基础上，改进了相机模型，使其不再局限于传统的小孔成像模型，而是可以**扩展到鱼眼模型**。在与 IMU 的结合上，它根据运动模型在流形上进行 **IMU 的预积分**的方式，然后采用非线性优化的思想，**将 IMU 的预积分结果和视觉 SLAM 的重投影模型一同进行图优化，使得预积分残差以及重投影误差共同达到最小**，以此来完成视觉信息和惯导系统的**紧耦合**。并且它采用了更为快速的**初始化**方法，以及丢失跟踪后利用惯导系统快速**重定位**方法。此外，它还采用**地图集**的方式，实现了对大场景的定位建图。这也是如今众多开源方案中，功能最强大、最精准的方法。系统框图如下：

![image-20230815102741960](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815102741960.png)

---

## 10-GNSS-SDR：GNSS 软件接收机

GNSS-SDR（**GNSS:** **G**lobal **N**avigation **S**atellite **S**ystems、**SDR:** **S**oftware **D**efined **R**eceiver），在 GitHub 上搜索 “GNSS” 排第一的仓库，收藏量也高达 1.3k。与上面列举的 RTKLIB、GAMP 等 GNSS 数据处理软件不同，GNSS-SDR 直接对信号进行处理，是一个用 C++ 实现的 GNSS 软件接收机开源项目。有了 GNSS-SDR，用户可以通过创建一个图来构建 GNSS 软件接收器，图中的节点是信号处理块，线条代表它们之间的数据流。该软件为不同的合适射频前端提供接口，并实现从接收器一直到 PVT 解算的所有功能。它的设计允许任何形式的定制，包括信号源、信号处理算法、与其他系统的互操作性、输出格式的互换，并为所有中间信号、参数和变量提供接口。

软件旨在促进新信号处理技术的发展，提供一种简便的方法来衡量这些技术对接收机整体性能的影响。通过对每个软件模块进行系统功能验证，以及使用真实和合成信号对整个接收机进行实验验证，对所有流程进行测试。

目前的技术仍无法以卫星发射频率（约 1.5 GHz）对信号进行数字处理，因此我们仍需要一个射频前端，将信号降频到较低频率，在此过程中进行一些滤波和放大，并以一定的速率进行采样，将量化的数字原始采样流传输到计算平台（通过 USB、以太网等）。

软件接收机可在普通的 PC 中运行，并通过 USB 和以太网总线为各种市售或定制的射频前端提供接口，使处理算法适应不同的采样频率、中间频率和采样分辨率。它还可以处理存储在文件中的原始数据样本。软件对可用的卫星信号进行信号采集和跟踪，对导航信息进行解码，并计算定位算法所需的观测值，最终实现完整导航解决方案。处理输出可存储在 RINEX 文件中，或通过 TCP/IP 服务器以 RTCM 3.2 消息形式实时传输。导航结果以 KML 和 GeoJSON 格式存储。

![GeneralBlockDiagram](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GeneralBlockDiagram.png)

---

## 11-TGINS：







---

## 12-KF-GINS：

KF-GINS 是武大 i2Nav 实验室开源的一套松组合导航程序；可以读取 IMU 数据文件、GNSS 结果文件，进行松组合解算，计算位置、速度、姿态、陀螺仪零偏、加速度计零偏、陀螺仪比例、加速度计比力，共 21 维状态向量。代码量小，有详细的文档、注释和讲解，代码结构很好理解，有一些可以学习的工程技巧。

![image-20230928094541518](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230928094541518.png)

![image-20230925181044694](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181044694.png)

* **项目开源地址**：https://github.com/i2Nav-WHU
* **i2NAV组合导航讲义、数据集**：http://www.i2nav.cn/index/newList_zw?newskind_id=13a8654e060c40c69e5f3d4c13069078
* **介绍视频**：https://www.bilibili.com/video/BV1Zs4y1B7m2/

---

## 13-OB-GINS：

武大 I2NAV 开源，基于图优化的 IMU/GNSS 松组合解算，IMU 预积分算法相比以视觉为主的 ORB-SLAM3、VINS 要精细一些。

![OB-GINS 执行流程](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/OB-GINS%2520%25E6%2589%25A7%25E8%25A1%258C%25E6%25B5%2581%25E7%25A8%258B.png)

相关链接：

* 开源地址：https://github.com/i2Nav-WHU/OB_GINS，
* 相关论文：
  * Hailiang Tang, Tisheng Zhang, Xiaoji Niu, Jing Fan, and Jingnan Liu, “Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization,” *IEEE Sensors Journal*, 2022. [下载](http://www.i2nav.com/ueditor/jsp/upload/file/20220801/1659348408510061111.pdf)
  * Junxiang Jiang, Xiaoji Niu, and Jingnan Liu, “Improved IMU Preintegration with Gravity Change and Earth Rotation for Optimization-Based GNSS/VINS,” *Remote Sensing*, vol. 12, no. 18, p. 3048, Sep. 2020, doi: [10.3390/rs12183048](https://doi.org/10.3390/rs12183048). [下载](https://sci-hub.se/10.3390/rs12183048)
  * Le Chang, Xiaoji Niu, and Tianyi Liu, “GNSS/IMU/ODO/LiDAR-SLAM Integrated Navigation System Using IMU/ODO Pre-Integration,” *Sensors*, vol. 20, no. 17, p. 4702, Aug. 2020, doi: [10.3390/s20174702](https://doi.org/10.3390/s20174702). [下载](https://www.mdpi.com/1424-8220/20/17/4702/pdf)

---

## 14-ROS导航功能包

功能简单来说，就是根据输入的里程计等传感器的信息流和机器人的全局位置，通过导航算法，计算得出安全可靠的机器人速度控制指令。广泛用在一些对可靠性要求没那么高的自主导航机器人场景中，比如扫地机器人、物流机器人等。

![image-20231111160631063](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231111160631063.png)

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1-13.png)

---

## 15-北斗GPS双模软件接收机

《北斗GPS双模软件接收机》书配套程序，MATLAB 编写，程序运行相当耗时，70s 的示例程序要算几个小时。

![GnssRcvr_V14程序文件结构](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GnssRcvr_V14%E7%A8%8B%E5%BA%8F%E6%96%87%E4%BB%B6%E7%BB%93%E6%9E%84.png)

* 网址：http://www.gnssbook.cn/book2/index.html

* 程序下载：http://www.gnssbook.cn/book2/GnssRcvr_V14.rar

* 实例北斗GPS双模中频数据文件下载：[UTREK210_16369000_70s.DAT](https://pan.baidu.com/s/1EWB0oQxDneNDk9iqExLuqQ)，提取码: 829c 

