> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

![5c056d5b27f86b81bc1fcfbd884b865e](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5c056d5b27f86b81bc1fcfbd884b865e.png)

最初写

本系列的第一篇文章，

* 先对 RTKLIB 做介绍。
* VS2022下编译调试步骤
* 介绍 VScode + WSL 下编译成动态库，同时编译 5 个命令行程序，在自己项目中引入核心代码库的方法。
* 介绍 Qt 下 RTKLIB 的编译
* 介绍如何用 RTKLIB 做后处理解算，由 RINEX 文件得出定位解，rnx2rtkp 主函数，自己写后处理主函数的方法。
* 介绍界面程序 RTKGET、RTKCONV、RTKPLOT、RTKPOST、STRSVR 的使用
* 最后介绍核心代码库，主要是头文件 rtklib.h 的内容，包括宏定义、结构体定义、函数定义

[TOC]

## 一、RTKLIB 介绍

### 1、概述

#### 1. 简介

RTKLIB 是全球导航卫星系统 GNSS 开源定位解算程序包，由日本东京海洋大学的高须知二（Tomoji Takasu）开发，由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好，许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来，适合作为 GNSS 入门学习的代码。

![RTKLIB](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB.png)

#### 2. 主要功能

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

RTKLIB 可以初步实现以下功能，相对于商业软件，可靠性没那么高，精度没那么高，但对于部分科研已经能够满足：

* **静态短基线解算**：相对定位，比如把一个测站安装在比较稳定的地区，把另一个测站安装在比较容易形变的地区做变形监测。
* **动态后处理差分 PPK**：比如无人机遥感、倾斜摄影测量等，需要高精度的位置和姿态解算精度。
* **实时动态差分 RTK**：导航定位。
* **精密单点定位 PPP**：可以用来算基准站坐标，地震监测、精密定轨、电离层对流层建模、时间传递。
* **实时精密单点定位 RT-PPP**： 比如接收实时的精密卫星的改正数，靠本地接收机的数据进行实时单点定位。用途比较广泛在海洋上，海啸的监测预警、海平面变化的监测、船只定位、海上石油平台作业等。

#### 3. 开源协议

基于的 BDS2-Clause 开源协议，用户能够自由地使用，修改源代码，也可以将修改后的代码选择继续开源或者闭源，须遵守如下两项要求：

* 如果分发的软件包含**源代码**，需在源代码中**保留原始的 BSD 许可证声明**；

* 如果分发的软件仅**包含二进制程序**，需在文档或版权说明中**保留原始的 BSD 许可证声明**。

#### 4. 延伸程序

各种各样的都有，有对定位解算算法做增强的、有做组合导航的、有做服务端程序的、有做软件接收机的、做应用的。下面介绍几个我了解的：

* [RTKLIB-demo5](https://github.com/rtklibexplorer/RTKLIB)：针对低成本接收机做了算法增强，下面的部分程序是基于 demo5 开发的。
* [rtklib-py](https://github.com/rtklibexplorer/rtklib-py)：
* [GPSTK](https://github.com/SGL-UT/GPSTk)：
* GAMP：山科大周峰写的双频非差非组合浮点解 PPP，在 RTKLIB 基础上做精简和算法的增强，比原版 RTKLIB 简单，是入门学习 PPP 不错的选择。曾经老师给我一套他师弟在 GAMP 基础上改的三频 PPP 让我看。
* [Ginan](https://github.com/GeoscienceAustralia/ginan)：澳大利亚，包括精密定位程序 PEA、定轨程序 POD，文档很详细，老师让我看，但我没看下去，代码比较难懂，而且它的代码风格很奇怪。
* [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)：港理工，支持图优化 SPP、RTK。作者在知乎很活跃，时常发一些科普文章。
* PPPLIB：我老师在矿大读研的时候写的，我看的第一个 C++ 程序，
* [TGINS](https://github.com/heiwa0519/TGINS)：我老师去年刚来安理的时候写的，文档和注释很少，紧组合看着比较费劲
* GINAV：MATLAB 紧组合，文件名起的和 RTKLIB 函数名一模一样
* [GICI-LIB](https://github.com/chichengcn/gici-open)：上海交大，
* [PPP-AR](https://github.com/PrideLab/PRIDE-PPPAR)：武大GNSS中心，使用了 rnx2rtkp 可执行程序计算测站初值坐标，网址
* [IGNAV](https://github.com/Erensu/ignav)：武大GNSS中心，
* [pppwizard](http://www.ppp-wizard.net/)：
* [GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)：GNSS 软件接收机，与上面列举的数据处理软件不同，GNSS-SDR 实现基带算法直接对接收机输出的数字中频信号处理，PVT 部分用了 RTKLIB。
* [APOLLO](https://github.com/ApolloAuto/apollo)：百度的开源无人驾驶系统，

#### 5. 我用 RTKLIB 做的事

RTKLIB 自带的程序除了 rtkplot 之外我都没咋用过，我主要是用 RTKLIB 的代码库，比如正在写的多源融合定位解算程序，GNSS + INS + Camera + Lidar 组合定位，引入了 RTKLIB 的代码库，主要用下面几部分内容：

* **时间系统**：用 `gtime_t` 作为量测数据的时间戳，时间转换都用 RTKLIB 提供的接口。
* **坐标转换**：我程序的坐标都用 Eigen 库的 Vector3d 向量表示；为方便调用，我对 ENU、ECEF、BLH 坐标之间的转换函数做了一层封装，接口为 Eigen 形式。
* **结果输出**：为了能输出姿态，扩展了 sol_t 结构体，加上三个欧拉角，输出结果的语句上加上欧拉角；然后拓展 rtkplot，把姿态角结果也画出来。
* **GNSS相关的类型定义**：卫星系统、卫星、观测值定义
* **配置选项**：GNSS 相关的 
* **数据读取**：RINEX、RTCM、NMEA
* **数据流**：
* 



项目文件结构和 CMakeLists.txt 文件内容如下：

![26b7f5cba7ef6929f88b4cc7bb85a6b3](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/26b7f5cba7ef6929f88b4cc7bb85a6b3.png)

### 2、下载 RTKLIB

在[RTKLIB官网](https://www.rtklib.com/)选最新版 **2.4.3 b34**，点 **Source Programs and Data** 和 **Binary APs for Windows** 下面的**GitHub**进入**GitHub页面**：

![image-20231015225119927](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231015225119927.png)

点开绿色的 **Code** 下拉菜单，再点 **Download ZIP**：

![image-20231016092003733](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016092003733.png)

解压两个压缩文件，得到的文件目录如下：

![image-20231026103732552](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231026103732552.png)

**Source Programs and Data** 是程序的源文件，**Binary APs for Windows** 是编译好的可执行文件，建议移到 **Source Programs and Data** 的 bin 目录下。rtkplot 用于原始数据和结果数据绘图，可以分析数据质量、解算精度，使用很频繁，建议放一个快捷方式在桌面，把数据文件、结果文件拖到图标上直接就画图。

### 3、界面程序简介

![image-20231016114907087](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016114907087.png)

* **rtklaunch**：界面程序启动器，界面如上，用来启动另外的界面程序

* **rtkget**：下载 GNSS 数据，包括 OBS、EPH、ATX、CLK、DCB 等多种文件，可同时下载起止时间内多个机构、多个测站的数据，但可能下载速度很慢。

* **rtkcov**：GNSS 数据转换，把采集的接收机原始数据转成 RINEX。

* **rtkplot**：原始数据绘图、结果绘图，可以用来做原始数据质量分析、结果精度分析、结果轨迹绘图。

* **rtkpost**：后处理定位解算，传入观测文件、星历文件和其它改正信息文件，设置好解算选项进行后处理解算。

* **rtknavi**：实时定位解算，接通导航数据流，实时定位解算绘图。

* **strsvr**：数据流转换播发。

* **srctblbrows**：NTRIP  资源列表浏览器。

### 4、命令行程序简介

命令行功能的程序和界面程序功能基本对应。界面程序好用，命令行程序代码好读；可以通过界面程序学软件的用法，理解程序运行逻辑；然后再通过阅读命令行程序的源码，来更深入的理解。当然，命令行程序还有一大作用就是写用来脚本进行批处理。

* **rnx2rtkp**：后处理定位解算，功能类似 rtkpos。
* **rtkrcv**：实时定位解算，功能类似 rtknavi。
* **str2str**：数据流转换播发，功能类似 strsvr。
* **convbin**：数据转换，功能类似 rtkcov。
* **pos2kml**：定位结果转谷歌地图数据格式。

### 5、源码学习建议

#### 1. 编程基础

* 有一点点 C 语言基础就可以了，之前上课学过那一点点 C 语言就足够了，不需要再特意的学语法，直接看代码，没见过的语法查一下，以后就会了；学编程光看书看网课是远远不够的，得多练，先看别人写的代码，然后才能自己写。
* 如果语法基础不好，一开始看的可能会比较艰难；可以先不想那么多，就从最基础的矩阵计算开始看，这几个小函数总能看下去吧；看完矩阵运算就继续看时间系统、坐标系统，RINEX文件读取......，一点点看，拼命的看，坚持下去；刚开始看的慢没关系，过了最初的坎，熟悉起来之后，后面就会慢慢顺起来，之后无论是再继续看别的程序还是自己写都能得心应手。
* 有了一点点编程基础之后，不要去刷什么 LeetCode，咱们的核心是算法，编程只是个工具，找工作的时候笔试不一定会考 LeetCode 题，至少比重不会到纯程序员那么大；再者说，你会看我一个大三学生写的的 RTKLIB 入门，应该离工作还较为遥远，当务之急还是要是先把专业能力提上来。
* 学完 RTKLIB，最好再看点 C++ 程序，比如松组合 [KF-GINS](https://blog.csdn.net/daoge2666/article/details/133376618)，代码量小只有大约 1500 行，学一学 C++ 的写法。算法工作大部分都要求会 C++、纯 GNSS 工作机会也没那么多。

#### 2. 代码看不懂怎么办

* 现在人工智能越来越强，把 RTKLIB 的代码段扔给 AI，基本都能给你解释解释。
* 网上 RTKLIB 的资料很丰富，基本上能把每一行代码的意思都给你讲明白了；可以先照着博客，把代码快速的过一遍，把博客上的注释、讲解复制到你手头的代码里，自己再看能顺很多。当然，博客大多写的很随意，不严谨，但有个参考总比没有好。
* 学 RTK 可以看我的老师杨旭的硕士论文，算法和 RTKLIB 几乎一模一样，学短基线算法、模糊度固定。
* 学 PPP 算法推荐看吉林大学周昌杰的硕士论文：《基于RTKLIB的GNSS精密单点定位研究》。
* RTKLIB 算法除了模糊度固定，理解起来都没啥难度，不懂的算法找论文看看就会了。
* 代码量很大，直接看可能会一头雾水，很难一下记住那么复杂的代码逻辑；可以通过流程图、函数调用关系图、思维导图，来辅助理解；通过画图来理清思路，画出的图也可以用来复习。

#### 3. 看 manual

* RTKLIB 的 manual 有 181 面，比较详细，先介绍附带工具包的使用，然后介绍核心代码库定义的 API，最后介绍算法模型。
* 工具包的使用可以找中文版的看，比如我的[仓库](https://github.com/LiZhengXiao99/Navigation-Learning)里就有。
* 附录 E 介绍 RTKLIB 的模型和算法，要对着公式重点看；不要依赖翻译，直接看英文的，不认识的单词查一下，下次就会了。
* 常用的英文表述最好记下来，不止后面的模型和算法有，还会出现在代码的注释里，看英文论文也经常见。

#### 4. 学习顺序

* 命令行功能的程序和界面程序功能基本对应。界面程序好用，命令行程序代码好读。可以通过界面程序学软件的用法，理解程序运行逻辑；然后再通过阅读命令行程序的源码，来更深入的理解。

* 学的时候先从后处理开始，先看 [postpos 的用法](https://www.bilibili.com/video/BV1m5411Y7xV)，然后顺着 rnx2rtkp 的源码，把从读取 RINEX 文件到算出定位结果整个过程看明白。


#### 5. 算法学习顺序

* **矩阵运算**：矩阵都是用一维 double 数组表示、列优先，要熟练掌握矩阵的加减乘除转置求逆，还要会 `matprint()` 输出矩阵用于 debug。
* **参数估计**：最小二乘、卡尔曼滤波、前向滤波、后向滤波、正反向结合。
* **时间系统**：知道基本概念（GPS 时、UTC、周内秒、跳秒、儒略日），理解 `gtime_t` 类型，会用操作 `gtime_t` 的函数，比如算时间差、比较时间先后、输出时间字符串、输出当前北京时间字符串、转周内秒。
* **坐标系统**：矩阵用三维向量表示，要掌握 ECEF（XYZ）、LLH（经纬高）、ENU（东北天）的用途、转换函数。
* **卫星系统定义**：有两套表示方法：表示卫星系统的字母：GRECJIS；或者 7 位二进制码，对应位写 1 表示有对应的系统，做与运算可加系统。
* **卫星定义**：可以表示为各系统的卫星 ID（系统缩写+PRN）：B02、C21；也可表示为连续的整型数字 satellite number。
* **观测量定义**：**C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比；`CODE_XXX`：观测值类型定义，用一串连续的数字表示。
* **配置选项**：主要是三个结构体：`prcopt_t` 存处理选项、`filopt_t` 存文件路径、`solopt_t` 存结果输出格式；默认处理选项、结果选项要理解，常用的处理选项要记住。
* **后处理解算大致流程**：结合流程图把 rnx2rtkp、postpos、procpos、rtkpos 看明白，知道配置存到哪、数据存到哪、结果存到哪、哪个函数把数据读进来、SPP/RTK/PPP 分别在哪些函数进行、前向滤波/后向滤波区别。
* **RINEX读取**：不用太细看，对数据格式有个基本的认识，知道读进来的数据以什么形式，存到什么变量里就 OK。
* **Trace输出**：知道怎么打开和关闭 Trace 输出、设置 Trace 等级，出了问题能根据 Trace 输出定位到出错位置、看明白出错原因。
* **结果输出**：有两套，一套是输出定位结果，包括位置速度钟差以及它们的协方差等，存在 `sol_t`、`solbuf_t` 中，由 `outsol()` 函数输出；一套输出解算状态，包括高度角方位角残差等，存在 `solstat_t`、`solstatbuf_t` 中，由 `outsolstat()` 输出。
* **卫星位置**：就是读文件套公式计算，没啥难度，过一遍文件格式和公式，有点点印象，知道 BDS、GLONASS 和其它系统计算的区别就可以。
* **卫星钟差计算**：用  ，迭代三次，要做群波延迟校正、相对论效应改正
* **克罗布歇模型电离层改正**：
* **对流层改正**：GPT 模型的气压温度算的准一点
* **天线相位改正**：包括卫星端和接收机端、PCO 和 PCV，GNSS 观测量是卫星和接收机天线相位之间的，而不是几何中心，需要转到几何中心，常通过 igs14.atx 文件来改正。
* **天线相位缠绕**：
* **地球自转改正**：也称 Sagnac 效应改正，卫星信号到达地球时 ECEF 坐标系会绕地球时 转动 $\omega r$，计算卫星与接收机间的几何距离时需要套公式改正。
* **潮汐改正**：地球并非刚体，会在日月引力、地球负荷作用下产生周期性形变，分为固体潮、极潮、海洋潮，改正的时候先算日月坐标，然后套公式计算。
* **观测值排除**：星历缺失、高度角、信噪比、人为排除卫星、URA。
* **差分码偏差 DCB**：GPS 广播星历是相对 P 码而言，而我们普通用户定位解算的时候用 C/A 码，需要通过 DCB 文件中的参数或者广播星历中的 TGD 来把测量的伪距归化到 P 码。
* **单频单系统伪距单点定位**：高度角方位角、卫地视线向量、近似距离计算，设计矩阵 H、新息向量 V 的构建，量测协方差阵 var
* **多系统**：多系统涉及到系统间偏差 ISB，以系统间时间偏差为主，还包括硬件延迟，每多一个系统，就要多估计一个相对于 GPS 的 ISB，增广状态向量和。
* **DOP 值计算**：
* **RAIM-FDE**：定位解算迭代若干次之后，残差仍然过大，认为定位解算发射，剔除残差最大的卫星观测值再进行解算，不断重复这个过程，知道解算成功，或者卫星数量过少不足以解算。
* **多频**：多频涉及到频间偏差 IFB；由于 GLONASS  信号频分多址调制，同频还存在频间偏差。
* **周跳检测**：RTKLIB 实现了两套周跳检测 LLI 和 GF，检测到周跳要重置模糊度估计参数，没做周跳修复。
* **模糊度固定**：
* **差分定位**：
* **电离层 INOEX 文件**：
* **估计电离层 STEC**：
* **估计对流层 ZTD**：
* **浮点解 PPP**：理清楚改正了哪些误差，用了哪些文件，估计了哪些参数，参数的排列顺序，每种参数建立什么随机模型，初始噪声过程噪声怎么设置，出现什么情况要重置参数。
* **实时解算流程**：从 rtkrcv  
* **数据流**：
* **RTCM、RAW 读取**：要简单了解数据格式，知道每种语句都有什么数据，用什么函数能读取到什么类型的哪个变量中。
* **SBAS 改正**：
* **SSR 改正**：



### 6、manual

RTKLIB 的 manual 有 181 面，先介绍附带工具包的使用，然后介绍核心代码库定义的 API，最后介绍算法模型。

![image-20231012203219194](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012203219194.png)

常用的英文表述最好记下来，不止后面的模型和算法有，还会出现在代码的注释里，看英文论文也经常见。

![image-20231025152050507](C:/Users/李郑骁的spin5/AppData/Roaming/Typora/typora-user-images/image-20231025152613781.png)![image-20231025152714889](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025152714889.png)![image-20231025152127766](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025152127766.png)![image-20231025152852639](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025152852639.png)



### 7、后处理程序执行流程（以 RNX2RTKP 为例）

![image-20231025155540386](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025155540386.png)

### 8、后处理函数调用关系（以 postpos() 函数为例）

![image-20231012212121216](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012212121216.png)

### 9、实时处理程序执行流程（以 RTKRCV 为例）

![image-20231024203128852](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024203128852.png)

### 10、实时处理函数调用关系（以 RTKRCV 为例）





## 二、VS2022 + Windows 编译调试

### 1、下载 RTKLIB

在[RTKLIB官网](https://www.rtklib.com/)选最新版 **2.4.3 b34**，点**Source Programs and Data**下面的**GitHub**进入**GitHub页面**，点开绿色的**Code**下拉菜单，再点**Download ZIP**，下载解压即可。

![1689207739592](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689207739592.png)

### 2、在 VS2022 中创建空 C++ 项目、导入源码文件

1. 创建**C++空项目**，可以勾选“解决方案和项目放在统一目录中”，记住创建的项目目录。

2. 把 RTKLIB 源码文件中**整个src文件夹**复制到创建的项目文件目录中。

3. 把 RTKLIB 源码文件中 **\app\consapp** 中的 **rnx2rtkp.c** 放到刚刚复制过去的 **src文件夹**。

4. 在解决源文件中添加名为 “src” 的筛选器，再在 src 筛选器下面添加名为 “rcv” 的筛选器 。

   右键添加现有项目把 **src/rcv文件夹** 中的所有文件加到 **src/rcv筛选器** 中，src 中所有代码文件加到 src 筛选器中。

5. 把主函数 **rnx2rtkp.c** 文件中的 **#include "rtklib.h"** 修改为 **#include "./rtklib.h“** 

   把在 **src/rcv文件夹几个的.c文件** 中的 **#include "rtklib.h"** 修改为 **#include "../rtklib.h”** 

### 3、项目属性设置

1. 打开项目属性，在**链接器—输入—附加依赖项**中添加依赖库**winmm.lib**和**ws2_32.lib**。 

2. **配置属性—高级—字符集**中设置为用**使用多字节字符集** 。

3. **C/C++**中的**SDL检查设置为否**，**附加包含目录**添加**.\src** 、预编译头为**不使用预编译头** 。

   预处理器中添加如下内容： 

   ```
   _LIB
   _CRT_SECURE_NO_WARNINGS
   _WINSOCK_DEPRECATED_NO_WARNINGS             
   ENAGLO
   ENACMP
   DENAGAL
   DLL
   WIN32
   ```

   > 尤其主要加 WIN32，好多博客都没加这一项，加了这一项后 RTKLIB 就不会用 Linux 下的 <pthread.h> 和 <sys/select.h>，咱们项目要在 Windows 下编译运行的，不加会报 ”找不到 <pthread.h> 和 <sys/select.h>“ 的错。

4. 将常规中的目标文件名改为 rnx2rtkp 。

   > 改不改都行，默认目标文件名是项目名。

### 4、改代码的 BUG

可能会报“使用了可能未初始化的本地指针变量 “sbs” 的错误，解决方式是对指针变量进行初始化，将 ephemeris.c 文件中的第 579 行改为 “const sbssatp_t *sbs=NULL;” ，还有些未初始化只报了警告，可以不用管。



## 三、VScode + WSL 下编译调试

开发中经常要用到 Linux，

WSL 

### 1、WSL 环境配置

* **安装 WSL**：建议安装 WSL2，先在系统设置里启用虚拟机，然后在微软商城安装 Ubuntu，



* **更新软件列表**

  ```bash
  sudo apt update
  ```

* **安装编译器和调试器** GCC、G++、GDB

  ```bash
  sudo apt install build-essential gdb
  ```

* **安装 CMake**

  ```bash
  sudo apt install cmake
  ```

* **检测安装是否成功**

  ```bash
  cmake -version
  ```

* **安装 ssh**

  ```bash
  sudo apt install ssh
  ```

* 其它推荐软件的安装：安装方式都是 `sudo apt install 软件名`

  - **vim**：LInux 系统必备的文件编辑器，
  - **git**：
  - **ranger**：资源管理器
  - **mc**：
  - **exa**：代替 ls
  - **tree**：用于查看树状目录结构
  - **cloc**：统计代码行数
  - **ack**：替代grep
  - **glances**：系统监控工具
  - **colordiff**：替代bat
  - **bat**：替代cat
  - **dstat**：

### 2、VScode 插件安装



插件下载，可能会比较慢，可以官网下载，然后导入VScode，

注意插件要装在 WSL 中，而不是本地(Local)。



### 3、把 RTKLIB 编译成第三方库



* 指定最小 CMake 版本、子项目名

  ```cmake
  cmake_minimum_required(VERSION 3.0)
  project(rtklib)
  ```

* 设置编译时 gcc 参数：

  ```cmake
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -O3 -ansi -pedantic")
  set(CMAKE_C_FLAGS "-Wno-unused-but-set-variable -Wno-format-overflow -Wno-unused-result -Wpointer-to-int-cast")
  ```

* 指定头文件目录：

  ```cmake
  include_directories(include)
  ```

* 指定可执行文件的输出路径为：rtklib/bin、库文件的输出路径为：rtklib/lib：

  ```cmake
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
  ```

* 将 src、src/rcv 目录下源文件加到 DIR_SRCS 列表：

  ```cmake
  aux_source_directory(src DIR_SRCS_RTKLIB)
  aux_source_directory(src/rcv DIR_SRCS_RTKLIB_RCV)
  list(APPEND DIR_SRCS ${DIR_SRCS_RTKLIB} ${DIR_SRCS_RTKLIB_RCV})
  ```

* 把代码编译成动态库，链接上 pthread m 库：

  ```cmake
  add_library(${PROJECT_NAME} SHARED ${DIR_SRCS})
  target_link_libraries(${PROJECT_NAME} pthread m)
  target_include_directories(${PROJECT_NAME}
      PUBLIC ${PROJECT_SOURCE_DIR}/include
  )
  ```

* 如果是 WIN32 还有链接上 wsock32 ws2_32 winmm 库，加上宏定义 -DWIN_DLL：

  ```cmake
  if(WIN32)
    target_link_libraries(${PROJECT_NAME} wsock32 ws2_32 winmm)
    add_definitions(-DWIN_DLL)
  endif()
  ```





### 4、编译调试命令行程序、链接 RTKLIB

RTKLIB APP 目录下有 5 个命令行程序

* **rnx2rtkp**：后处理定位解算
* **rtkrcv**：实时定位解算
* **str2str**：数据流转换播发
* **convbin**：数据转换
* **pos2kml**：定位结果转谷歌地图数据格式

用一个 CMakeList.txt 



### 5、编译调试自己写的程序、链接 RTKLIB

比如我用 C++ 语法，通过调用 RTKLIB 的函数，实现获取当前系统时间，输出年月日时分秒、GPS周 + 周内秒，主函数文件如下：

```C++
```



### 6、VS2022 + WSL 编译调试







### 7、CLion + Windows 编译调试







### 8、CLion + WSL 编译调试







## 四、Qt 编译调试











## 五、后处理解算

### 1、使用 rnx2rtkp 后处理解算

rnx2rtkp 全称 RINEX to RTK pos，通过原始 RINEX 文件，输出 RTKLIB 的定位坐标，下图可以很好的表示整个程序的逻辑：

![image-20231016123911526](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016123911526.png)

#### 1. 使用方式

* 使用方式：`rnx2rtkp [option]... file file [...] `

* 读取 RINEX：OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。

* 对于相对定位，第一个 OBS 观测值文件需含接收机、流动站观测值，第二个 OBS 文件需含基准站观测值。

* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。

* 想用 SP3 精密星历文件，需提供 .sp3/.eph 文件的路径。

* 输入文件路径可包含通配符 *，为了防止与命令行命令冲突，要用 `"..."`  括起带通配符符路径。

* 输命令行参数的几种方式：
  * **VS**：
  * **VScode**：
  * **Clion**：
  * **Windows 终端**：
  * **Linux 终端**：

#### 2. 命令行参数

1. **-？**：打印 help
2. **-k** file：配置文件的输入选项，默认值是 [off]
3. **-o** file：输出文件选项，默认值是 [stdout]
4. **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是 [obs start time] 
5. **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是 [obs end time] 
6. **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
7. **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是 [2]
8. **-m** mask：设置截止高度角，`(deg) `,默认值是 [15]
9. **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是 [G|R] ，想用除 GPS 以外的系统，还得加宏定义 ENAGLO、ENACMP、ENAGAL
10. **-f** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是 [2]
11. **-v** thres：设置整周模糊度 Ratio 值，写 0.0 为不固定整周模糊度，默认值是 [3.0] 
12. **-b**：后向滤波
13. **-c**：前后向滤波组合
14. **-i**：单历元模糊度固定 instantaneous 
15. **-h**：fix and hold 模糊度固定
16. **-e**：输出 XYZ-ecef 坐标
17. **-a**：输出 ENU-baseline
18. **-n**：输出 NMEA-0183 GGA
19. **-g**：输出经纬度格式为 ddd mm ss.ss ，默认为 [ddd.ddd] 
20. **-t**：输出时间格式为 yyyy/mm/dd hh:mm:ss.ss ，默认为 [sssss.ss] 
21. **-u**：输出为 UTC 时间，默认为 [gpst] 
22. **-d** col：设置时间的小数位数，默认为 [3] 
23. **-s** sep：设置文件分隔符，要写在单引号中，默认为 [' '] 
24. **-r** x y z：基站位置 ECEF-XYZ (m)，默认 [average of single pos] ，流动站位置用于 fixed 模式
25. **-l** lat lon hgt：基站位置 LLH (deg/m)，默认 [average of single pos]，流动站位置用于 fixed模式
26. **-y** level：输出结果信息 (**0**:off,**1**:states,**2**:residuals) ，默认为 [0] 
27. **-x** level：输出 debug trace 等级，默认为 [0] 

#### 3. 数据下载

我没咋下载过，推荐两篇博客吧：

* IGS官方产品及数据下载地址汇总

* 

把



RTKGET 可以批量下载数据，本文下一章会介绍。

#### 4. 错误解决

rnx2rtkp 的命令行参数很复杂，一不下心就会出错，这时候可以去看 trace 文件，重点看出现 error 的部分，复制 error 信息的前半部分，去程序里搜索，定位到出现错误的地方，在附近设几个断点，看看到的是咋错的。

比如我的朋友按照吴桐的[博客](https://zhuanlan.zhihu.com/p/528855325)运行 rnx2rtkp，出错了来问我：

![image-20231015211515575](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231015211515575.png)

* 首先看左边的 trace 输出，有两行里显示 error，说是打开观测文件和星历文件时候出错了。

* 复制前面的 ”rinex file error“，在代码中全局搜索，发现只有在 `readrnxfile()` 函数里有可能输出这段信息。

* 可以明显看出来是 `fopen()` 根据路径打开文件的时刻出错了，那就在 `fopen()` 的那一行设断点，看看打开文件路径到的是什么。

* 最后调试发现，路径中 `\` 都是连续出现四个。推测分析是因为朋友的 VS 版本，输完的命令行参数，会自动把其中的 `\` 换成 `\\`，自己输文件路径的时候只要一个 `\` 就行了。

  ```
  -x 5 -p 0 -m 15 -n -o D:\\source\\RTKLIB-rtklib_2.4.3\\rtklib\\out.pos D:\\source\\RTKLIB-rtklib_2.4.3\\test\\data\\rinex\\07590920.05o D:\\source\\RTKLIB-rtklib_2.4.3\\test\\data\\rinex\\07590920.05n
  ```

  要改成：

  ```
  -x 5 -p 0 -m 15 -n -o D:\source\RTKLIB-rtklib_2.4.3\rtklib\out.pos D:\source\RTKLIB-rtklib_2.4.3\test\data\rinex\07590920.05o D:\source\RTKLIB-rtklib_2.4.3\test\data\rinex\07590920.05n
  ```

再比如，程序运行结束看不到结果输出，可以在 `outhead()` 下面位置设断点，看看 `outfile` 变量里存没存文件路径，那个位置有没有输出了文件头的结果文件。

![image-20231028150122875](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028150122875.png)

### 2、rnx2rtkp 主函数

在文章的最后，对 rnx2rtkp 的主函数做一个介绍：

* 读取配置文件过程：
  * 循环判断参数是否有 `-k`，如果有就代表传入了配置文件，需要读取进来
  * 创建 `porcopt_t`、`solopt_t`、`filopt_t` 变量用于接受读取到的配置
  * 调用 `resetsysopts()` ，重置配置为默认值。
  * 调用 `loadopts()`，从文件中读取配置，存到 `opt_t` 类型的 `sysopt` 中。
  * 调用 `getsysopts()`，把 `opt_t` 类型的 `sysopt` 转到 `porcopt_t`、`solopt_t`、`filopt_t` 变量中，会调用 `buff2sysopts()`。
* 读其它参数：
  * 循环内，`if-else if`，判断参数，根据参数赋值
  * 若都不是参数，最后一个到 `else if`，认为是文件路径，用 `infile` 数组接收
* 最后调用 `postpos()` 后处理解算

我根据自己的理解给代码加了中文注释，代码如下：

```c
int main(int argc, char **argv)
{
    prcopt_t prcopt=prcopt_default;     // 定位处理模式
    solopt_t solopt=solopt_default;     // 结果输出形式
    filopt_t filopt={""};               // 文件路径选项
    gtime_t ts={0},te={0};              // ts开始时间、te结束时间
    double tint=0.0,
            es[]={2000,1,1,0,0,0},
            ee[]={2000,12,31,23,59,59},
            pos[3];
    int i,      // for循环的计数
        j,      // 嵌套的for循环计数
        n,      // 记录读入文件数
        ret;    // 接受postpos的返回值
    char *infile[MAXFILE],  // 读入文件，默认最多16个，可改MAXFILE定义
         *outfile="",       // 输出文件
         *p;                // 指向字符串的指针，用于循环指向各main函数参数
    
    prcopt.mode  =PMODE_KINEMA;     // 定位模式默认动态相对定位Kinematic
    prcopt.navsys=0;                // 卫星系统，先设置无
    prcopt.refpos=1;                // 基准站坐标，先设为由SPP平均解得到
    prcopt.glomodear=1;             // GLONASS AR mode,先设on
    solopt.timef=0;                 // 输出时间格式，先设sssss.s
    sprintf(solopt.prog ,"%s ver.%s %s",PROGNAME,VER_RTKLIB,PATCH_LEVEL);   // 项目名称
    sprintf(filopt.trace,"%s.trace",PROGNAME);
    
    /* load options from configuration file */
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-k")&&i+1<argc) {          // 如果有-k和配置文件输入
            resetsysopts();                             // 先重置所有配置
            if (!loadopts(argv[++i],sysopts)) return -1;// 再读取配置文件内容，存入opt_t的sysopt中
            getsysopts(&prcopt,&solopt,&filopt);        // opt_t转到porcopt_t/solopt_t/filopt_t，
        }   
    }          
    // for 循环判断 main 函数参数
    for (i=1,n=0;i<argc;i++) {  
        if      (!strcmp(argv[i],"-o")&&i+1<argc) outfile=argv[++i];//读取输出文件路径，赋值给outfile
        else if (!strcmp(argv[i],"-ts")&&i+2<argc) {    // 读取开始解算时间   
            sscanf(argv[++i],"%lf/%lf/%lf",es,es+1,es+2);
            sscanf(argv[++i],"%lf:%lf:%lf",es+3,es+4,es+5);
            ts=epoch2time(es);      // 转为gtime_t
        }
        else if (!strcmp(argv[i],"-te")&&i+2<argc) {    // 读取结束解算时间
            sscanf(argv[++i],"%lf/%lf/%lf",ee,ee+1,ee+2);   
            sscanf(argv[++i],"%lf:%lf:%lf",ee+3,ee+4,ee+5);
            te=epoch2time(ee);  // 转为gtime_t
        }
        else if (!strcmp(argv[i],"-ti")&&i+1<argc) tint=atof(argv[++i]);        // 读取解算时间间隔频率
        else if (!strcmp(argv[i],"-k")&&i+1<argc) {++i; continue;}              // 有-k，跳过
        else if (!strcmp(argv[i],"-p")&&i+1<argc) prcopt.mode=atoi(argv[++i]);  // 读取解算模式
        else if (!strcmp(argv[i],"-f")&&i+1<argc) prcopt.nf=atoi(argv[++i]);    // 读取用于计算的频率
        else if (!strcmp(argv[i],"-sys")&&i+1<argc) {       // 读取用于计算的导航系统
            for (p=argv[++i];*p;p++) {      
                switch (*p) {                           //有对应导航系统，就把它的码做与运算加上                 
                    case 'G': prcopt.navsys|=SYS_GPS;
                    case 'R': prcopt.navsys|=SYS_GLO;
                    case 'E': prcopt.navsys|=SYS_GAL;
                    case 'J': prcopt.navsys|=SYS_QZS;
                    case 'C': prcopt.navsys|=SYS_CMP;
                    case 'I': prcopt.navsys|=SYS_IRN;
                }
                if (!(p=strchr(p,','))) break;  
            }
        }
        else if (!strcmp(argv[i],"-m")&&i+1<argc) prcopt.elmin=atof(argv[++i])*D2R;     // 设置截止高度角     
        else if (!strcmp(argv[i],"-v")&&i+1<argc) prcopt.thresar[0]=atof(argv[++i]);    // 设置整周模糊度Ratio值
        else if (!strcmp(argv[i],"-s")&&i+1<argc) strcpy(solopt.sep,argv[++i]);         // 设置文件路径分隔符
        else if (!strcmp(argv[i],"-d")&&i+1<argc) solopt.timeu=atoi(argv[++i]);         // 设置时间小数位数
        else if (!strcmp(argv[i],"-b")) prcopt.soltype=1;           // 后向滤波
        else if (!strcmp(argv[i],"-c")) prcopt.soltype=2;           // 前后向滤波组合
        else if (!strcmp(argv[i],"-i")) prcopt.modear=2;            // 单历元模糊度固定
        else if (!strcmp(argv[i],"-h")) prcopt.modear=3;            // fix and hold 模糊度固定
        else if (!strcmp(argv[i],"-t")) solopt.timef=1;             // 输出时间格式为 yyyy/mm/dd hh:mm:ss.ss
        else if (!strcmp(argv[i],"-u")) solopt.times=TIMES_UTC;     // 输出为 UTC 时间
        else if (!strcmp(argv[i],"-e")) solopt.posf=SOLF_XYZ;       // 输出 XYZ-ecef 坐标
        else if (!strcmp(argv[i],"-a")) solopt.posf=SOLF_ENU;       // 输出 ENU-baseline
        else if (!strcmp(argv[i],"-n")) solopt.posf=SOLF_NMEA;      // 输出 NMEA-0183 GGA
        else if (!strcmp(argv[i],"-g")) solopt.degf=1;              // 输出经纬度格式为 ddd mm ss.ss
        else if (!strcmp(argv[i],"-r")&&i+3<argc) {                 // 基站位置E CEF-XYZ (m)              
            prcopt.refpos=prcopt.rovpos=0;                  // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) prcopt.rb[j]=atof(argv[++i]); // 循环存入基准站坐标
            matcpy(prcopt.ru,prcopt.rb,3,1);    
        }
        else if (!strcmp(argv[i],"-l")&&i+3<argc) {     // 循环存入基站位置基站位置LLH (deg/m)
            prcopt.refpos=prcopt.rovpos=0;              // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) pos[j]=atof(argv[++i]);   
            for (j=0;j<2;j++) pos[j]*=D2R;              // 角度转弧度   
            pos2ecef(pos,prcopt.rb);                    // LLH 转 XYZ
            matcpy(prcopt.ru,prcopt.rb,3,1);
        }
        else if (!strcmp(argv[i],"-y")&&i+1<argc) solopt.sstat=atoi(argv[++i]); //输出结果信息
        else if (!strcmp(argv[i],"-x")&&i+1<argc) solopt.trace=atoi(argv[++i]); //输出debug trace等级
        else if (*argv[i]=='-') printhelp();        //输入-，打印帮助
        else if (n<MAXFILE) infile[n++]=argv[i];    //循环判断完一遍参数之后，认为参数是文件路径，用infile数组接收
    }
    if (!prcopt.navsys) {               //如果没设卫星系统，默认为GPS、GLONASS
        prcopt.navsys=SYS_GPS|SYS_GLO;
    }
    if (n<=0) {         //如果读入文件数为0,报错，-2退出
        showmsg("error : no input file");
        return -2;
    }

    
    //   gtime_t ts       I   processing start time (ts.time==0: no limit)
    //   gtime_t te       I   processing end time   (te.time==0: no limit)
    //   double ti        I   processing interval  (s) (0:all)
    //   double tu        I   processing unit time (s) (0:all)
    //   prcopt_t *popt   I   processing options
    //   solopt_t *sopt   I   solution options
    //   filopt_t *fopt   I   file options
    //   char   **infile  I   input files (see below)
    //   int    n         I   number of input files
    //   char   *outfile  I   output file ("":stdout, see below)
    //   char   *rov      I   rover id list        (separated by " ")
    //   char   *base     I   base station id list (separated by " ")
    //后处理定位解算
    ret=postpos(ts,te,tint,0.0,&prcopt,&solopt,&filopt,infile,n,outfile,"",""); 
    
    if (!ret) fprintf(stderr,"%40s\r","");
    return ret;
}
```

### 3、自己写后处理主函数

> 参考：https://www.cnblogs.com/ahulh/p/15584998.html

rnx2rtkp 程序比较复杂，要传入很多命令行参数，可以自己写主函数。再比如说我松组合程序，可以先调用 `postpos()` 通过GNSS原始原始数据算出定位解，然后与 INS 组合。

rtklib 后处理主函数需要做的主要就是：读取配置文件（可省）、设置处理选项、传入文件路径、调用 postpos()；下面我将分别对这几部分的写法做介绍：

#### 1. 实现用户自定义函数 showmsg、settspan、settime

* **showmsg**：
* **settspan**：
* **settime**：

#### 2. 配置文件读取



读取配置文件流程：

* 先调用 `resetsysopts()` 重置所有配置为默认。
* 调用 `loadopts()` 读取配置文件内容，存入 `opt_t` 的 `sysopt` 中。
* 最后调用 `getsysopts()` 将 `opt_t` 转到 `porcopt_t`/`solopt_t`/`filopt_t`。



#### 3. postpos() 参数

先来看看需要传入的参数：

```c
gtime_t ts       I   处理的起始时间，写0表示不限制
gtime_t te       I   处理的起始时间，写0表示不限制
double ti        I   处理的间隔时间 (s)，写0表示不限制，全处理
double tu        I   处理的单元时间（s)，写0表示全部做一个单元处理
prcopt_t *popt   I   处理选项结构体
solopt_t *sopt   I   结果选项结构体
filopt_t *fopt   I   文件选项结构体
char   **infile  I   传入文件路径数组首地址
int    n         I   传入文件数量
char   *outfile  I   输出文件的路径，写0表示stdout终端
char   *rov      I   流动站ID列表，空格隔开
char   *base     I   基准站ID列表，空格隔开
```

* `ts`、`te`、`ti`、`tu`：这几个参数用于设置解算时间，可以都填 0，让程序把传入的观测文件全解算了。
* `infile`、`n`：
* `outfile`：
* `rov`、`base`：
* `popt`、`sopt`、`fopt`：这几个选项结构体是设置的重点
  * `filopt_t`：**文件选项**，存结果输出、Trace、各种改正文件路径，不包括星历文件和观测文件。
  * `solopt_t`：**结果选项**，可以设置结果输出形式（ENU、ECEF、NMEA、BLH），小数位数，是否输出文件头，是否输出速度等。
  * `prcopt_t`：**处理选项**，是配置的重头戏，可以先看 [postpos 的用法](https://www.bilibili.com/video/BV1m5411Y7xV) 学习界面程序的配置方式。写代码配置和界面程序需要配置的东西是一样的，只是从在界面上点，换成在了代码里给对应字段赋值。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB%25E9%2585%258D%25E7%25BD%25AE%25E9%2580%2589%25E9%25A1%25B9.png)

## 六、界面程序的使用

> 再次推荐一下 B 站赵老师的[视频讲解](https://space.bilibili.com/479790048?spm_id_from=333.337.search-card.all.click)，看视频学软件操作更直观。我主要是用 RTKLIB 的代码库，自带的程序除了 rtkplot 之外我都没咋用过，下面写的内容是看赵老师视频的时候做的笔记。

### 1、RTKGET 数据下载

> 视频里的下载地址有些不能用了，可以参考[GNSS观测数据及各种产品下载网址分享](https://blog.csdn.net/qq_38607471/article/details/129952202)

1. 观测值下载
   * 选择下载数据的时间：起始时间，结束时间

   * options 设置 URL_LIST，可以用 RTKLIB 默认配置，选择 rtklib 中 data 文件夹下 URL_LIST.txt 文件（我在 bin 版的 rtklib 里没找到，用的源码版的 rtklib 里的文件），加载进来，左边就有了两列内容。

   * OBS 为观测值文件，NAV 为导航电文，EPH 为精密轨道，CLK 精密钟差、ATX 天线文件

   * 做相对定位要下观测值，
     * 先选择分析中心，IGS、MGES 等。

     * 后在右边选测站，点...把测站加上去，ALIC、KARR

     * 把测站名点选，再点 Download，理论上就可把数据下到指定目录，但会比较慢

       * 可复制 FTP 路径直接进网页下载

       * 要在 Linux 下大量处理，可写脚本

   ![bf7b9a35f8b2c5f149215023bfd25831](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/bf7b9a35f8b2c5f149215023bfd25831.png)
   
2. 用 RTKGET 做时间转换：输入年月日时分秒，点问号 ？，就可看各种时间，下载观测值需要年积日 DOY，改链接的日期就可下载对应的观测值文件

   ![image-20231025204151284](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025204151284.png)

3. 数据命名格式：**测站名（4位）+机构信息+年+年积日+采样间隔**，crx 是压缩格式，gz 也是压缩格式，还有一种是 o.z 结尾只进行一次压缩

4. 广播星历文件和精密星历文件：也可用 rtkget 和 ftp下载

### 2、RTKCONV 数据转换

1. 为啥要介绍此模块：老师刚刚拿到了ublox 接收机连天宝天线采集的数据，想分析一下数据的质量。
2. ublox 通过串口导出的二进制文件，COM3 开头，.ublox 结尾，除了原始观测数据之外还有 NMEA 数据。通过 notepad++ 打开查看，开头乱码是二进制数据，后面是 NMEA 文本格式。
3. ublox 数据还可用 ucenter 接收机配置软件查看
4. RTKCONV 使用
   * 先选择需要转换数据的起止时间，采样率 Interval。
   * 输入原始数据地址。
   * 选格式，u-blox、RINEX、RTCM3...，不知道格式可选自动 Auto。
   * 勾选选输出数据，一般得要obs观测数据，如果实时数据从网上不能在网上下导航电文，需要转换出的 nav 文件。
   * 配置信息：RINEX 版本号、测站 ID 可以不写、RunBy 可以写自己、天线类型接收机类型有需要可以写，近视坐标，加哪些改正信息，输出哪些系统、观测值类型可都勾上，观测频率，信号通道。
   * 点 Convert 转换。 
   
   ![19792c3d48403c733efcf69677e4863b](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/19792c3d48403c733efcf69677e4863b.png)
   
   * 点 Plot 可以直观展示卫星数据质量
     * Sat Vis：卫星可见性，选频率，颜色代表信噪比 SNR。
     * Skyplot：卫星天空视图，站心地平级坐标系，可看出低高度角卫星信号差
     * DOP：上面是可视卫星数，下面是 DOP 值
     * SNR：载噪比、多路径，可选某一颗卫星指定频率，横坐标可选时间、高度角

### 3、RTKPOST 数据后处理

1. 主界面
   * 设置解算起止时间，解算间隔
   * 加载 RINEX OBS 数据：Rover 流动站、Base 基准站，右上角点天空图标开 RTKPLOT 看数据状态。基准站整天的数据非常大，截取流动站对应部分即可。
   * 加载其它数据：NAV、CLK、SP3 等。每个接收机输出的 NAV 只有它能观测到的卫星星历，从网上可下全部所有卫星所有系统的导航电文。
   * 输出默认在流动站文件路径，后缀为 .pos。 
   
   ![image-20231026163551141](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231026163551141.png)
2. Options设置
   * **定位模式**
     * Single：伪距单点定位
     * DGPS/DGNSS：伪距差分
     * Kinematic：载波动态相对定位，动态RTK，假设流动站是移动的，可以做车载定位
     * Static：载波静态相对定位，静态RTK，两站都是静止的，可以得到很高的精度
     * Static-Start：（demo5 才有）冷启动：先在比较开阔的地方，进行短时期的静态定位，模糊度固定，再动起来
     * Moving-Base：两站都动，主要用来定姿
     * Fixed：固定坐标，解算模糊度、对流层、电离层等参数
     * PPP Kinematic、PPP-Static、PPP Fixed
     
   * **频率**：可选不同频率组合，如L1+L2
   
   * **滤波**：前向（后面结果更可靠）、后向（可使刚开始的时候有高的精度）、Combind（正向一个结果，反向一个结果，根据方差加权平均），RTKLIB里除了SPP都用卡尔曼滤波，滤波有一个收敛的过程，后面更准。
   
   * **设置截止高度角**：可以看天空视图，如果低高度角数据很差，可设置更高的截止高度角。质量好可以不管，有残差检验也可剔除一些数据。
   
   * **设置截止信噪比**：做工程一般环境都不会很好，想做的序列稳定，要设置截止信噪比。RTK有很多算法，但其实传统算法效果已经很好了，算法不用做的太复杂，把数据质量控制做好就行，RTK就不会有太大的问题。
   
   * **Rec Dynamics**：动力学模式，选 ON 会估计速度加速度参数，，选 OFF 就只估算动态坐标参数
   
     * Kinematic 动态模式：把位置参数当白噪声估计。
     * Dynamics 动力学模型：建立 CA 模型，同时估计位置、速度、加速度。
   
   * **RCV、潮汐改正等**：PPP 才用的到
   
   * **电离层、对流层改正**：双差已经可以消除部分电离层对流层误差，可以关闭此改正，也可以直接采用广播星历的模型改正。RTKLIB 做 RTK 最好用非组合模式，短基线电离层可以关闭，对流层可以用 saastamoinen 模型直接修正。
   
   * **卫星星历**：RTK 相对定位距离近可以直接用广播星历，长距离相对定位可选精密星历。
   
     * SSR APC：参考天线相位中心
     * SSR CoM：参考质心，还需要天线相位中心改正
   
   * **剔除卫星**：写卫星号，空格隔开，如：C01 C02
   
   * **RAIM FDE完好性检验**：算法不是很稳健，不选
   
   * **模糊度固定模式 ARMODE**
   
     * **OFF**：浮点解，不固定
   
     * **Continues**：认为模糊度是连续解，通过前面历元的解算结果滤波提高后续历元模糊度固定精度。
   
     * **Instantaneous**：瞬时模糊度固定，单历元模糊度固定，每个历元都初始化一个参数，这个历元和上个历元模糊度不相关。
   
     * **Fix and Hold**：先 Continues，在不发生周跳情况下都采用之前模糊度固定的结果作为约束，也有问题：固定错了，时间序列会一直飘，到一定程度变成浮点解，会重置模糊度重新算。
   
       > 做工程可做两套，Instantaneous 和 Fix and Hold，发现 Fix and Hold 错了，就用 Instantaneous 的解把它替换掉，相当于把模糊度和方差初始化了一次，避免漂移和模糊度重新收敛的过程。
   
     * **PPP-AR**：PPP 时固定模糊度，不支持，需要额外产品。
   
   * **Ratio值**：用于检验模糊度是否固定成功，设为 3 即可。
   
   * **最小LOCK**：连续锁定这颗卫星几次，才用于计算模糊度固定。
   
   * **用于模糊度固定的最低高度角设置**：可设 15°
   
   * **最小Fix**：这个历元最少固定多少个模糊度才认为模糊度是固定的，可设 10，现在卫星系统多了，而且组合模式，双频一颗卫星就 2 个模糊度，5 颗卫星固定就能凑 10 个。
   
   * **Fix hold**：选择哪些模糊度固定结果用于约束后续。
   
   * **输出结果**：可选  LLH、XYZ、ENU、NMEA
   
   * **输出解算状态**：可选 OFF、Residuals 残差、State
   
   * **Debug Trace等级**：1-5级，level 越高输出越多
   
   * **基准站坐标**：可输入、也可选伪距单点定位
   
   * **天线类型**：选*，自动获取 O 文件里的



3. 算完之后
   * **Plot**：对解算结果可视化分析，黄色没固定，绿色固定
   * **view**：查看解算结果，类似记事本
   * **KML**：转为 GoogleXML 可把地图展示到地图上

> 建议：下静态数据，找动态车载数据，分别处理静态相对定位和处理动态相对定位，设置不同处理模式，分析定位结果的差异。

4. PPP 数据处理
   * 实时PPP：IGS/MGEX 分析中心播发的实时卫星轨道和钟差产品，结合广播星历
   * 事后或近实时：下载精密星历、钟差产品，结合其它精密改正信息实现定位
   * RTKLIB 使用必须给广播星历，因为解算前都会先进行一次伪距单点定位

### 4、SRTSVR 数据流收发

1. 功能概述

   * **TCP Server**：等待来自客户端的连接请求，处理请求并返回结果。

   * **TCP Client**：主动角色，发送连接请求，等待服务器响应。

   * **Ntrip Server**：将本地接收机的 RTCM 数据推送到 Ntrip Caster。

   * **Ntrip Caster**：用户管理和播发 RTCM 数据。

   * **Ntrip Client**：登录 Ntrip Caster 获取 RTCM 数据。


2. 界面

   * 一个输入，多个输出

   * 输入输出可以是：Serial，TCP Client、TCP Server、Ntrip Client、Ntrip Server、UDP Server、File、FTP、HTTP


3. RTK2GO

   * 相当于免费的 Ntrip Caster，所有的用户都可把自己的数据源上传到 Caster 中，其它的用户都可以用 Caster 接受数据

   * 连接：输入模式选 Ntrip Client，通过网址和端口，点 Ntrip 就会弹出弹出数据源


#### 4. 输出

* 点左下角 □ 框，开 Input Stream Monitor 查看数据流状态，可选很多种格式
* 输出也可选很多种，比如 Ntrip Server 可把自己的数据作为 Caster，别人可以通过网络接受你的数据，选 File 把数据存成文件



## 七、核心代码库

代码库是 RTKLIB 的核心，要不咋能叫"LIB"呢？

RTKLIB 提供许多代码库和 API，包括：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM 函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

头文件 rtklib.h 是库的核心 ，rtklib.h 主要有三大部分：**宏定义**、**结构体定义**、**全局变量**、**函数定义**

>  并不是所有函数都可以直接调用，只有加了 EXPORT 前缀，而且在 RTKLIB.h 中声明了才行。想用 static 前缀的函数也很简单，只需要把前缀改成 EXPORT，然后在 rtklib.h 中加上声明

### 1、宏定义

![rtklib.h 宏定义](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%20%E5%AE%8F%E5%AE%9A%E4%B9%89.png)

### 2、结构体定义

![rtklib.h结构体](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%E7%BB%93%E6%9E%84%E4%BD%93.png)

### 3、全局变量

* `extern const double chisqr[];`：**卡方检验表**
* `extern const prcopt_t prcopt_default;`：**默认处理选项**
* `extern const solopt_t solopt_default;`：**默认结果选项**
* `extern const sbsigpband_t igpband1[9][8];`：**SBAS IGP 波段 0-8**
* `extern const sbsigpband_t igpband2[2][5];`：**SBAS IGP 波段 9-10**
* `extern const char *formatstrs[];`：**数据流格式字符串**
* `extern opt_t sysopts[];`：**系统选项表**

### 4、基础函数定义

#### 1. 矩阵、向量、最小二乘、卡尔曼滤波

* RTKLIB 中用 double 类型一维数组表示矩阵，不能自动识别矩阵的行列数，每次传矩阵的时候都要传入行数 n、列数 m。
* 用矩阵的时候要先 malloc 开辟空间，用完记得 free 释放空间。
* 要能熟练计算矩阵加减乘除转置。
* 矩阵求逆用的 LU 分解法，时间复杂度 $O^3$ ，对于大规模的矩阵，如果利用矩阵的稀疏性和对称性等特性，而且当使用不完全分解方法（例如，只计算到一定程度或使用截断技术）时，LU 分解的效率会更高。
* matprint() 很常用，调试的时候不好直接看的矩阵元素的值，得输出到终端或者文件再看。

![image-20231021091639437](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091639437.png)

#### 2. 时间和字符串

* RTKLIB 中时间一般都以 `gtime_t` 类型存储，为了提高时间表示的精度，分开存 GPST 时间的整秒数和不足一秒的部分。
* 经常需要做年月日时分秒、周+周内秒、GPST 三种时间之间的转换；想输出北京时间的时候要加 8 小时。
* BDT、GLONASST 不怎么用，读完文件就转为 GPS 时间了。

![image-20231021090651319](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021090651319.png)

#### 3. 坐标转换

* ECI 用的很少，只在 `sunmoonpos()` 函数中计算太阳月亮时候用到了，不用太关注。
* ENU、ECEF、LLH 三套坐标系都频繁使用，要熟练掌握他们直接的转换，包括协方差的转换
* ENU 是局部相对坐标系，以某一个 LLH 坐标为原点，坐标转换的时候要传入这个 LLH 坐标。
* ENU 常用 `e`表示、ECEF 常用 `r` 表示、LLH 常用 `pos` 表示。

![image-20231021091756265](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091756265.png)

#### 4. 卫星系统、观测值

* **卫星系统表示**：
  * 表示卫星系统的字母：GRECJIS。
  * 7 位二进制码表示，对应位写 1 表示有对应的系统，做与运算可加系统。
* **卫星的表示**：
  * 可以表示为各系统的卫星ID（系统缩写+PRN）：B02、C21。
  * 也可表示为连续的 satellite number。
* **观测值类型**：
  * **C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比。
  * `CODE_XXX`：观测值类型定义，用一串连续的数字表示。
  * `sigind_t`：表示每种卫星系统的载波类型和观测值类型 ，每种类型的系统其实对应的就是一个 `sigind_t` 结构体。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024191221181.png)

### 5、选项调试输出

#### 1. 配置选项读取

* 选择主要存在 `prcopt_t`、`solopt_t`、`filopt_t` 三个结构体中。
* 读取结果文件流程：
  * 先调用 `resetsysopts()` 重置所有配置为默认。
  * 调用 `loadopts()` 读取配置文件内容，存入 `opt_t` 的 `sysopt` 中。
  * 最后调用 `getsysopts()` 将 `opt_t` 转到 `porcopt_t`/`solopt_t`/`filopt_t`。

![image-20231025205106288](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205106288.png)

#### 2. Trace 调试

* 在 rtklib.h 中加入 #define TRACE，启用 trace ，不定义则将 trace 函数全赋空值。
* 想输出北京时间，可以先用 timeset() 函数加 8 小时。

![image-20231025205158762](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205158762.png)

#### 3. 结果输入输出、NMEA

* 输出的结果有两套：
  * 定位结果：坐标、协方差、有效果卫星数、差分龄期
  * 解算状态：
* NMEA 读取：

![image-20231025205301277](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205301277.png)

### 5、量测数据

#### 1. 导航数据输入

* 用完数据记得释放内存。

![image-20231025205602582](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205602582.png)

#### 2. RINEX 文件读写



![image-20231025205701553](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205701553.png)

#### 3. 二进制数据读写



![image-20231025205800413](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205800413.png)

#### 4. 星历数据解析





#### 5. 接收机自定义数据读写





#### 6. RTCM 读写



![image-20231025205952192](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205952192.png)

### 6、解算相关

#### 1. 定位解算入口



![image-20231025210531296](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210531296.png)

#### 2. 实时解算



![image-20231025210624595](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210624595.png)

### 7、数据流相关

#### 1. 数据流

* 数据流函数用的 C 语言语法比较杂，好在现在 AI 发达，可以用来辅助理解。
* 每种数据流关注四个函数：打开、关闭、写数据、读数据。

![image-20231025210846191](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210846191.png)

#### 2. 数据流线程管理



![image-20231025210938225](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210938225.png)

### 8、模型改正

#### 1. 星历、钟差、DCB、FCB



![image-20231025223050670](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223050670.png)

#### 2. SBAS



![image-20231025223134482](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223134482.png)

#### 3. 定位模型



![image-20231025223220415](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223220415.png)

#### 4. 对流层、电离层模型



![image-20231025223257219](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223257219.png)

#### 5. 天线改正



![image-20231025223330203](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223330203.png)

#### 6. 潮汐改正



![image-20231025223344099](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223344099.png)

#### 7. 水准面模型



![image-20231025223357868](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223357868.png)

#### 8. 高程转换



![image-20231025223409859](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223409859.png)

### 9、其它杂项函数

#### 1. 下载函数

![image-20231025212102845](C:/Users/李郑骁的spin5/AppData/Roaming/Typora/typora-user-images/image-20231025212102845.png)

#### 2. 结果格式转换

![image-20231025212144253](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212144253.png)

#### 3. GIS 数据读取

可以读取 shapfile 矢量数据

![image-20231025212339125](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212339125.png)

#### 4. 平台相关函数

在 Windows 和 Linux 有完全不同的两套实现

![image-20231025212504258](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212504258.png)

#### 5. 用户自定义函数

在 rtklib.h 中声明了，在代码库中使用了，但没有实现，用代码库的时候需要我们自己实现，一般写个空函数就行。编译的时候经常会在这报错，如果说未定义就写三个空实现，如果重定义就把写的实现注释掉。







### 10、代码库的使用

总结一些命名和函数定义习惯：

* 矩阵做参数时一点要带上维度，矩阵 m 为行、n 为列，
* 
* 带 const 的指针一定是输入参数，不带 const 的指针一定是输出参数或者既是输入也是输出。
* 类型命名结尾都带 `_t`，
* 读文件函数：结尾带 t，
* 





