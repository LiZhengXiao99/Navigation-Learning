> 原始Markdown文档、Visio流程图、XMind思维导图：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、RTKLIB 介绍

### 1、概述

#### 1. 简介

RTKLIB 是知名的全球导航卫星系统 GNSS 开源定位解算程序包，日本东京海洋大学的高须知二（Tomoji Takasu）开发，由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好。RTKLIB 功能很齐全，GNSS 数据处理所需的基本功能都有，支持的数据格式很多，既可以实时解算也可以后处理，既可以接自己的 GNSS 模块也可以连 IGS 的数据流，既可以解算自己采集的数据也可以算 IGS 测站的数据，既可以 RTK 也可以 PPP；许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来，适合作为 GNSS 入门学习的项目。它的项目结构如下所示：

![RTKLIB](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB.png)

RTKLIB 可以初步实现以下功能，相对于商业软件，可靠性没那么高，精度没那么高，但对于部分科研已经能够满足：

* **静态短基线解算**：相对定位，比如把一个测站安装在比较稳定的地区，把另一个测站安装在比较容易形变的地区做变形监测。
* **动态后处理差分 PPK**：比如无人机遥感、倾斜摄影测量等，需要高精度的位置和姿态解算精度。
* **实时动态差分 RTK**：导航定位。
* **精密单点定位 PPP**：可以用来算基准站坐标，地震监测、精密定轨、电离层对流层建模、时间传递。
* **实时精密单点定位 RT-PPP**： 比如接收实时的精密卫星的改正数，靠本地接收机的数据进行实时单点定位。用途比较广泛在海洋上，海啸的监测预警、海平面变化的监测、船只定位、海上石油平台作业等。

---

**RTKLIB 界面程序包括以下这些，可以直接使用编译好的程序，也可以用作者提供的 Qt 和 C++ builder 两套界面程序的源码自己编译。**

![image-20231016114907087](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016114907087.png)

* **rtklaunch**：界面程序启动器，界面如上，用来启动另外的界面程序
* **rtkget**：下载 GNSS 数据，包括 OBS、EPH、ATX、CLK、DCB 等多种文件，可同时下载起止时间内多个机构、多个测站的数据，但可能下载速度很慢。
* **rtkcov**：GNSS 数据转换，把采集的接收机原始数据转成 RINEX。
* **rtkplot**：原始数据绘图、结果绘图，可以用来做原始数据质量分析、结果精度分析、结果轨迹绘图。
* **rtkpost**：后处理定位解算，传入观测文件、星历文件和其它改正信息文件，设置好解算选项进行后处理解算。
* **rtknavi**：实时定位解算，接通导航数据流，实时定位解算绘图。
* **strsvr**：数据流转换播发。
* **srctblbrows**：NTRIP  资源列表浏览器。

---

**命令行程序的功能和界面程序功能基本对应。界面程序好用，命令行程序代码好读；可以通过界面程序学软件的用法，理解程序运行逻辑；然后再通过阅读命令行程序的源码，来更深入的理解。当然，命令行程序还有一大作用就是写用来脚本进行批处理。**

* **rnx2rtkp**：后处理定位解算，功能类似 rtkpos。
* **rtkrcv**：实时定位解算，功能类似 rtknavi。
* **str2str**：数据流转换播发，功能类似 strsvr。
* **convbin**：数据转换，功能类似 rtkcov。
* **pos2kml**：定位结果转谷歌地图数据格式。

---

**如果有 GNSS 模块和天线，通过串口或网口连接电脑、树莓派，可以用 RTKLIB 程序包实现以下操作：**

* 你可以用 RTKNAVI 或 RTKRCV 进行实时定位解算，单 GNSS 一般只能进行 SPP 解算，米级精度；想提高精度：可以接入差分数据做 RTK（自己搭基准站或者买 CORS 账号）；或者提前下载一些精密改正文件，并申请 IGS 账号，接入 SSR 改正数据，做实时 PPP。在进行实时解算的过程中，可以通过 LOG 把数据流的数据都都存下来。
* 你可以通过 STRSVR 或 STR2STR 把数据存到文件里；或者通过 Ntrip、TCP 等协议把数据播发出去，进行远程解算；或者作为 Ntrip 数据源把数据挂载 NtripCaster 搭基准站。
* 如果你只有观测数据（伪距载波多普勒）需要自己下载星历文件，或者你要做 PPP 需要下载改正文件；可以用 RTKGET 或者通过对应的网址手动下载（比如去武大 IGS 中心：）。
* 想要更高的精度，或者实时解算的效果不好，你可以把数据采集下来进行后处理分析。
  * 后处理一般都用 RINEX 文件，而接收机采集下到的数据大多是 RTCM 或各种接收机的原始数据，这些数据大多都是二进制的（相比文本文件，二进制文件小，程序读读写快，但是不方便看里面存的内容），可以通过 CONVBIN 或 RTKCONV 将数据转为 RINEX。
  * 解算前先用 RTKPLOT 作图分析原始数据质量，看卫星数、观测值连续性，画天空图看卫星几何分布，画信噪比图看信号质量等。最新的 b34 版本直接下载的 RTKPLOT 好像有 bug，可以自己编译或者下个别的版本的，比如[这个]()。
  * 使用 RTKPOST 或 RNX2RTKP 都可以进行后处理解算：
    * 界面程序 RTKPOST 可以很方便的设置解算选项，还能导出设置的处理选项文件，适合处理单个数据。
    * 命令行程序 RNX2RTKP 需要输一串命令行参数，想改处理选项得用配置文件（可以用 RTKPOST 进行设置，导出配置文件），处理单个数据时略显麻烦，它的优势是可以写脚本进行批处理。
    * 也可以参照着 RNX2RTKP 自己写后处理主文件。
  * 算完之后可以输出三种文件：结果文件、解算中间结果文件、Trace 文件；通过 RTKPLOT 可以对结果文件、解算中间结果文件绘图，进行可视化分析，Trace 文件只能手动打开看。
    * **结果文件（solution）**：定位定速结果，可以输出 NMEA 格式也可以输出 RTKLIB 自定义的格式。
    * **解算中间结果文件（solution status）**：解算的中间结果，包括每颗卫星的残差、高度角方位角、模糊度、对流层延迟、电离层延迟、钟差等；
    * **调试文件（Trace）**：看 Trace 文件可以辅助断点调试，甚至替代断点调试。程序执行出错，开 2/3 级 Trace，看 Trace 文件里的 error、warring 就能，知道大致出了啥问题，定位出问题的函数，断点调试的时候你就知道该在哪设置断点了。Trace 信息分五个等级，从 1-5 重要性逐渐降低，通过 tracelevel() 函数可以设置输出的最高等级，设置 2 级意味着只输出 1/2 级信息。
      * **一级 Trace 是致命错误**，出现一级错误基本上意味着程序无法继续执行，比如观测星历文件读取错误、内存分配错误。
      * **二级 Trace 是警告**，出现二级警告程序可能依然能继续执行，但也可能无法进行解算，比如改正文件读取失败，数据解析出错，二进制数据校验出错，某一历元解算失败，缺失解算所需的星历或改正参数等。
      * **三级  Trace 是程序主要执行流程**，主要在函数的开头，告诉我们执行到了这个函数。
      * **四级 Trace 是比三级更深入的程序执行流程**，主要在三级  Trace 函数的中间或者调用的子函数开头，告诉我们执行到了这个操作。
      * **五级 Trace 是解算的中间过程**，具体到每颗卫星，每个频点，每次循环。
  * 如果解算出不了结果，可以把 Trace 调试等级设成 2 看看有哪些错误或警告。如果是打开什么文件失败了，说明文件路径没设置好；如果是文件解析出错，可能也是文件类型选错或文件有问题；如果总是报残差过大解算失败，那是解算选项设置的不对。
  * 如果解算结果达不到要求，可以输出解算中间结果 (solution status 文件)，通过 RTKPLOT 作图分析，看看能不能改进，比如有的卫星残差总是大，可以将其排除再解算。
  * 看 Trace 文件和解算中间结果文件还解决不了问题，那就只能断点调试了。
* 后处理比实时处理的有以下优势：
  * 实时处理必须要同时有观测数据（伪距载波多普勒）和星历数据流；而后处理只要有观测数据就行了，星历文件可以去 IGS 网站下载。
  * 后处理可以尝试不同的解算参数，可以对解算中间结果分析，根据解算情况调整参数、排除卫星。
  * 后处理可以对数据文件质量进行分析。
  * 后处理可以用正反向滤波平滑计算。
  * 后处理可以断点调试。

---

#### 2. 支持功能

* **支持六大 GNSS 系统**，包括 GPS，GLONASS，Beidou，Galileo，QZSS 和 SBAS。

  > 但是不支持全频点，不支持印度 IRNSS 系统，多频算法不完善，对北斗的支持不好。

* **支持 9 种 GNSS 实时和后处理定位模式**：
  
  * **single**：伪距单点定位；
  * **DGPS/DGNSS**：伪距差分；
  * **kinematic**：载波动态相对定位，动态RTK，假设流动站是移动的，可以做车载定位；
  * **Static**：载波静态相对定位，静态RTK，两站都是静止的，可以得到很高的精度；
  * **Moving-Baseline**：双天线，两站都动，主要用来定姿；
  * **Fixed**：固定坐标，解算模糊度、对流层、电离层等参数；
  * **PPP-Kinematic**：动态精密单点定位；
  * **PPP-Static**：静态精密单点定位；
  * **PPP-Fixed**：PPP 固定坐标，解算模糊度、对流层、电离层等参数。
  
* **支持多种 GNSS 标准格式和协议**：RINEX2.10、RINEX2.11、RINEX2.12、RINEX3.00、RINEX3.01、RINEX3.02、RTCM2.3、RTCM3.1、RTCM3.2、BINEX、NTRIP、NMEA0183、SP3、ANTEX1.4、IONEX1.0、NGS PCV、EMS 2.0。

* **支持多种 GNSS 接收机专有数据协议格式**：NovAtel:OEM4/5/6/7，OEM3, OEMStar、Superstar II、 Hemisphere、Crescent、u‐blox:LEA-4T/5T/6T、SkyTraq、JAVAD 、GW10-II/III 和 NVS。

  > 我手头 GNSS 接收机的原始数据都可以用 RTKLIB 简单解析（频点支持不全），部分国产接收机虽然没列举在上面，但用的也是上面这些数据格式，我用过的几款国产接收机（华测、和芯星通、北云科技）直接输出的原始数据都是诺瓦泰OEM格式，用 RTKLIB 可以解析，但频点不全。

* **支持外部通信**：Serial、TCP/IP、NTRIP、本地日志文件、FTP 和 HTTP。

  > 接收机一般通过**串口**或**网口**可以直接连电脑传输观测数据和星历数据；差分定位用的基准站数据和实时PPP用的SSR数据一般通过**NTRIP**接入；实时定位的时候可以保存原始数据流到**日志文件**，可以通过日志文件来模拟实时数据流解算来进行调试，也可以转为 RINEX 后处理。

* **提供许多代码库和API**：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

  > 在本文的最后会详细介绍这些 API。

#### 3. 开源协议

基于的 BDS2-Clause 开源协议，用户能够自由地使用，修改源代码，将修改后的代码选择继续开源或者闭源都可，须遵守如下两项要求：

* 如果分发的软件包含**源代码**，需在源代码中**保留原始的 BSD 许可证声明**；

* 如果分发的软件仅**包含二进制程序**，需在文档或版权说明中**保留原始的 BSD 许可证声明**。

#### 4. 延伸程序

各种各样的都有，有对定位解算算法做增强的、有做组合导航的、有做服务端程序的、有做软件接收机的、做应用的。下面介绍几个我了解的：

* [RTKLIB-demo5](https://github.com/rtklibexplorer/RTKLIB)：针对低成本接收机做了算法增强，下面的部分程序是基于 demo5 开发的。
* [rtklib-py](https://github.com/rtklibexplorer/rtklib-py)：
* [GPSTK](https://github.com/SGL-UT/GPSTk)：
* [GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)：山科大周峰写的双频浮点解 PPP，在 RTKLIB 基础上做精简和算法的增强，比原版 RTKLIB 简单，是入门学习 PPP 不错的选择。
* [Ginan](https://github.com/GeoscienceAustralia/ginan)：澳大利亚，包括精密定位程序 PEA、定轨程序 POD，文档很详细，老师让我看，但我没看下去，代码比较难懂，
* [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)：港理工，支持图优化 SPP、RTK，作者在知乎很活跃，发过一些科普文章。
* [GLIO](https://github.com/XikunLiu-huskit/GLIO)：在 GraphGNSSLib 基础上做的 GNSS-IMU-Lidar 图优化紧组合；
* [PPPLIB](https://geodesy.noaa.gov/gps-toolbox/PPPLib.htm)：我老师在矿大读研的时候写的，支持三频 SPP、PPK、PPP 和松紧组合。
* [GINAV](https://github.com/kaichen686/GINav)：MATLAB 紧组合，文件名起的和 RTKLIB 函数名一模一样，虽说是组合导航，但也可以只用其中的 GNSS 部分，相比 goGPS 简单不少。
* [GICI-LIB](https://github.com/chichengcn/gici-open)：上海交大池澄博士开源的 GNSS-IMU-Camera 图优化多源融合程序，以 GNSS 为主，实现了 RTK、PPP 的模糊度固定 
* [PPP-AR](https://github.com/PrideLab/PRIDE-PPPAR)：武大 GNSS 中心开源的后处理 PPP，使用配套的产品可以实现 PPP 模糊度固定，支持五频数据处理，使用了 rnx2rtkp 可执行程序计算测站初值坐标。
* [IGNAV](https://github.com/Erensu/ignav)：武大 GNSS 中心，图优化紧组合
* [pppwizard](http://www.ppp-wizard.net/)：
* [GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)：GNSS 软件接收机，与上面列举的数据处理软件不同，GNSS-SDR 实现基带算法直接对接收机输出的数字中频信号处理，PVT 部分用了 RTKLIB。
* [PocketSDR](https://github.com/tomojitakasu/PocketSDR)：RTKLIB 作者新开源的软件接收机，包含一个射频前端和一套后处理 GNSS 接收机程序（只支持后处理），实现了一整套完整的 GNSS 接收机功能，采用 C、Python 编写，支持几乎所有的 GNSS 信号（比商业接收机支持的还要多），引入 RTKLIB 做库，用到了 RTKLIB 的一些结构体。
* [APOLLO](https://github.com/ApolloAuto/apollo)：百度的开源无人驾驶系统，用到了 RTKLIB 的 NMEA 结构体。

---

### 2、下载 RTKLIB

在[RTKLIB官网](https://www.rtklib.com/)选最新版 **2.4.3 b34**，点 **Source Programs and Data** 和 **Binary APs for Windows** 下面的**GitHub**进入**GitHub页面**：

![image-20231015225119927](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231015225119927.png)

点开绿色的 **Code** 下拉菜单，再点 **Download ZIP**：

![image-20231016092003733](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016092003733.png)

解压两个压缩文件，得到的文件目录如下：

![image-20231026103732552](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231026103732552.png)

**Source Programs and Data** 是程序的源文件，**Binary APs for Windows** 是编译好的可执行文件，建议移到 **Source Programs and Data** 的 bin 目录下。rtkplot 用于原始数据和结果数据绘图，可以分析数据质量、解算精度，使用很频繁，建议放一个快捷方式在桌面，把数据文件、结果文件拖到图标上直接就画图。

---

### 3、源码学习建议

#### 1. 学会软件包的使用

* 看源码之前先会用软件
* 申请一个 IGS 账号
* 有 GNSS 接收机更好，没有也能学
* 做 RTK，
* 采集数据后处理

#### 2. 编程基础

* 有一点点 C 语言基础就可以了，之前上课学过那一点点 C 语言就足够了，不需要再特意的学语法，直接看代码，没见过的语法查一下，以后就会了；学编程光看书看网课是远远不够的，得多练，先看别人写的代码，然后才能自己写。
* 如果语法基础不好，一开始看的可能会比较艰难；可以先不想那么多，就从最基础的矩阵计算开始看，这几个小函数总能看下去吧；看完矩阵运算就继续看时间系统、坐标系统，RINEX文件读取......，一点点看，拼命的看，坚持下去；刚开始看的慢没关系，过了最初的坎，熟悉起来之后，后面就会慢慢顺起来，之后无论是再继续看别的程序还是自己写都能得心应手。

#### 3. 代码看不懂怎么办

* 现在人工智能越来越强，把 RTKLIB 的代码段扔给 AI，基本都能给你解释解释。
* 网上 RTKLIB 的资料很丰富，基本上能把每一行代码的意思都给你讲明白了；可以先照着博客，把代码快速的过一遍，把博客上的注释、讲解复制到你手头的代码里，自己再看能顺很多。当然，博客大多写的很随意，不严谨，但有个参考总比没有好。
* 学 RTK 可以看硕士论文，学短基线算法、模糊度固定。论文写的不怎么样，但里面基础知识写的详细，很适合初学者阅读，相比比教材更有针对性（讲的基本都是 RTKLIB 用的到的算法），比博客更严谨（算法公式都是仔细检查过的）。
* 学 PPP 算法推荐看吉林大学周昌杰的硕士论文《基于 RTKLIB 的 GNSS 精密单点定位研究》。
* RTKLIB 算法除了模糊度固定，理解起来都没啥难度，只是有些算法你之前可能没听说过，找论文看看就会了。
* 代码量很大，直接看可能会一头雾水，很难一下记住那么复杂的代码逻辑；可以通过流程图、函数调用关系图、思维导图，来辅助理解；通过画图来理清思路，画出的图也可以用来复习。
* 推荐用 Markdown 记笔记，不用费心思调整排版，尤其是想记代码的时候，用 Word 很麻烦的。找视频或者文档花一两个小时学一下，就能上手；Markdown 软件有好多，我懒得折腾，用的 Typora，喜欢折腾想要功能丰富看可以看看 Obsidian、Notable，有共享协作需求可以用飞书。
* Markdown 里可以用 Latex 语法记公式，画一两个小时可以学个大概。Latex 公式的语法比较复杂，输公式很麻烦；短公式还可以直接输，长公式推荐用 Latex 公式编辑器，比如[这个](https://www.latexlive.com/)，如果想把别人的公式记到自己的笔记里，可以截图、拍照，用公式识别软件转 Latex。

#### 4. 看 manual

* RTKLIB 的 manual 有 181 面，比较详细，先介绍附带工具包的使用，然后介绍核心代码库定义的 API，最后介绍算法模型。
* 工具包的使用可以找中文版的看，比如我的[仓库](https://github.com/LiZhengXiao99/Navigation-Learning)里就有。
* 附录 E 介绍 RTKLIB 的模型和算法，要对着公式重点看；不要依赖翻译，直接看英文的，不认识的单词查一下，下次就会了。
* 常用的英文表述最好记下来，不止后面的模型和算法有，还会出现在代码的注释里，看英文论文也经常见。

#### 5. 学习顺序

* 命令行功能的程序和界面程序功能基本对应。界面程序好用，命令行程序代码好读。可以通过界面程序学软件的用法，理解程序运行逻辑；然后再通过阅读命令行程序的源码，来更深入的理解。
* 无论是实时解算还是后处理，都是从 `rtkpos()` 函数开始进行单历元解算。后处理是；实时解算是
* 学的时候先从后处理开始，先看 postpos 的用法，然后顺着 rnx2rtkp 的源码，把从读取 RINEX 文件到算出定位结果整个过程看明白。
* 大部分内容，刚开始看的时候不用太关注内部具体实现，知道原理就好。知道数据存到什么类型里，在哪个函数计算，传入什么数据，计算得到什么数据，就行了。

#### 6. 算法学习顺序

* **矩阵运算**：矩阵都是用一维 double 数组表示、列优先，要熟练掌握矩阵的加减乘除转置求逆，还要会 `matprint()` 输出矩阵用于调试，比如你想看程序运行过程中某个矩阵的值，断点调试直接看肯定不行，矩阵都是指针，得用  `matprint()`  输出。
* **参数估计**：把最小二乘、卡尔曼滤波的四个函数看明白；后处理的时候有前向滤波、反向滤波、正反向结合三种滤波方式，体现在代码上就是有个标记标志前后，取数据的顺序不同。
* **时间系统**：知道基本概念（GPS 时、UTC、周内秒、跳秒、儒略日），理解 `gtime_t` 类型，会用操作 `gtime_t` 的函数，比如算时间差、比较时间先后、输出时间字符串、输出当前北京时间字符串、转周内秒。
* **坐标系统**：矩阵用三维向量表示，要了解 ECEF（XYZ）、LLH（纬经高）、ENU（东北天）的用途、转换函数（包括坐标转换、协方差转换）。
  * ECEF 是直角坐标系，列观测方程计算方便，在 RTKLIB 中一般用 r 表示。
  * LLH 反映了测站在地球椭球上的位置，在 RTKLIB 中一般用 pos 表示。
  * ENU 是站心坐标系，是以测站为原点建立的直角坐标系，方便表示相对关系（卫星相对接收机、流动站相对于基准站），比如计算方位角高度角，视线向量；ENU 表示东北天，生活中常用，比如导航软件告诉你“向东行驶200米左转”；ENU 坐标系都是以某一个 LLH，这个原点 LLH 必须存下来，ENU 才有意义，ENU 转 ECEF、LLH 的时候需要有坐标原点的 LLH。
* **卫星系统定义**：算的时候得知道观测值是哪个卫星系统的，有两套表示方法：
  * 表示卫星系统的字母：GRECJIS；
  * 或者 7 位二进制码 SYS_xxx，对应位写 1 表示有对应的系统，做或算可加系统，做与运算判断有无系统。
* **卫星定义**：解算的时候需要知道观测值是哪颗卫星的，也有两套表示方法：
  * 可以表示为各系统的卫星 ID（系统缩写+PRN）：B02、C21；直观且含义明确，但不好处理。
  * 也可表示为连续的整型数字 satellite number，好处理，方便遍历。
* **观测量定义**：**C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比；`CODE_XXX`：观测值类型定义，用一串连续的数字表示。
* **配置选项**：主要是三个结构体：`prcopt_t` 存处理选项、`filopt_t` 存文件路径、`solopt_t` 存结果输出格式；默认处理选项、结果选项要理解，常用的处理选项要记住。
* **后处理解算大致流程**：结合流程图把 rnx2rtkp、postpos、procpos、rtkpos 看明白，知道配置存到哪、数据存到哪、结果存到哪、哪个函数把数据读进来、SPP/RTK/PPP 分别在哪些函数进行、前向滤波/后向滤波区别。
* **RINEX读取**：不用太细看，对数据格式有个基本的认识，知道读进来的数据以什么形式，存到什么变量里就行。
* **Trace 输出**：知道怎么打开和关闭 Trace 输出、设置 Trace 等级，出了问题能根据 Trace 输出定位到出错位置、看明白出错原因。
* **结果输出**：有两套，一套是输出定位结果，包括位置速度钟差以及它们的协方差等，存在 `sol_t`、`solbuf_t` 中，由 `outsol()` 函数输出；一套输出解算中间结果，包括高度角方位角残差等，存在 `solstat_t`、`solstatbuf_t` 中，由 `outsolstat()` 输出。
* **卫星位置计算**：精密星历和广播星历都是读文件套公式计算，对照着代码看一遍文件格式和公式，有点点印象，知道 BDS、GLONASS 和其它系统计算的区别就可以。
* **卫星钟差计算**：用广播星历里的 $a_0,a_1,a_2$ 二次函数拟合系数算，迭代三次，要做群波延迟校正、相对论效应改正。
* **电离层改正**：当信号通过电离层时，传播速度和传播路径会发生改变，带来电离层延迟；大小与电子密度成正比，对载波和伪距影响相反，不同信号频率延迟不同，影响可达数十米。
  * **克罗布歇模型电离层改正**：
  * **电离层 INOEX 文件**：
  * **估计电离层 STEC**：
* **对流层改正**：信号穿过对流层时，由于传播介质密度的增加，信号传播路径和传播速度会发生改变，带来对流层延迟。与频率无关，对载波和伪距影响相同。对流层延迟一般可分为干延迟和湿延迟，对于载波相位和伪距完全相同，一般在米级大小。
  * **Saastamoninen 模型**：对流层分为两层进行积分，一层温度视为常数、一层温度有变化，然后按照天顶距三角函数展开逐项进行积分, 并把对流层天顶延迟分为对流层干延迟和湿延迟两个分量之和。
  * **标准大气模型**：根据经验模型计算求大气压 P、温度 T、大气水汽压力 e。
  * **GPT 模型**：GPT 模型的气压温度算的准一点，利用欧洲中尺度天气预报中心 长期的再分析气象资料建立的全球气象参数经验模型, 仅需知道测站地理位置信息与年积日便可以获得地表温度、大气压力和水汽压等气象参数。
  * **估计对流层 ZTD**：
* **天线相位改正**：包括卫星端和接收机端、PCO 和 PCV，GNSS 观测量是卫星和接收机天线相位之间的，而不是几何中心，需要转到几何中心，常通过 igs14.atx 文件来改正。不研究这个方向，就不用太细看，
* **天线相位缠绕**：
* **地球自转改正**：也称 Sagnac 效应改正，卫星信号到达地球时 ECEF 坐标系会绕地球时转动 $\omega r$，计算卫星与接收机间的几何距离时需要套公式改正。
* **潮汐改正**：地球并非刚体，会在日月引力、地球负荷作用下产生周期性形变，分为固体潮、极潮、海洋潮，改正的时候先算日月坐标，然后套公式计算。
* **观测值排除**：星历缺失、高度角、信噪比、人为排除卫星、URA。
* **差分码偏差 DCB**：GPS 广播星历是相对 P 码而言，而我们普通用户定位解算的时候用 C/A 码，需要通过 DCB 文件中的参数或者广播星历中的 TGD 来把测量的伪距归化到 P 码。BDS、GLONASS、Galileo 也类似。
* **单频单系统伪距单点定位**：高度角方位角、卫地视线向量、近似距离计算，设计矩阵 H、新息向量 V 的构建，量测协方差阵 var
* **多系统**：多系统涉及到系统间偏差 ISB，以系统间时间偏差为主，还包括硬件延迟，每多一个系统，就要多估计一个相对于 GPS 的 ISB，增广参数向量和设计矩阵。
* **DOP 值计算**：反映卫星的几何分布，GAMP 里用
* **RAIM-FDE**：定位解算迭代若干次之后，残差仍然过大，认为定位解算发射，剔除残差最大的卫星观测值再进行解算，不断重复这个过程，知道解算成功，或者卫星数量过少不足以解算。
* **多频**：多频涉及到频间偏差 IFB；由于 GLONASS  信号频分多址调制，同频还存在频间偏差。
* **周跳检测**：RTKLIB 实现了两套周跳检测 LLI 和 GF，检测到周跳要重置模糊度估计参数，没做周跳修复。
* **差分定位**：
* **模糊度固定**：
* **浮点解 PPP**：理清楚改正了哪些误差，用了哪些文件，估计了哪些参数，参数的排列顺序，每种参数建立什么随机模型，初始噪声过程噪声怎么设置，出现什么情况要重置参数。
* **实时解算流程**：顺着 rtkrcv 的主函数往下看，算法和后处理没啥区别，数据读取。
* **数据流**：包括串口、文件、Ntrip、TCP、UDP 等
* **RTCM、RAW 读取**：简单了解数据格式，知道每种语句都有什么数据，用什么函数能读取到什么类型的哪个变量中。
* **SBAS 改正**：
* **SSR 改正**：

**总结**：

* 矩阵运算、参数估计、时间系统、坐标系统、卫星和观测值的表示，是基础，要熟练掌握。
* 结果输出、Trace 输出、Rinex 和各种其它文件的读取，知道文件格式，知道大概哪个函数就行，基本不需要细致了解。
* 后处理流程要有印象，重点关注定位方程，H、V、R 矩阵的构建。了解模型改正原理，比如对流层、电离层、天线、潮汐、地球自转、引力延迟。
* 偏差的处理也算是重点，DCB、FCB、ISB、IFB，要算多系统多频，肯定得考虑。

---

### 4、manual

RTKLIB 的 manual 有 181 面，先介绍附带工具包的使用，然后介绍核心代码库定义的 API，最后介绍算法模型。

![image-20231012203219194](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012203219194.png)

常用的英文表述最好记下来，不止后面的模型和算法有，还会出现在代码的注释里，看英文论文也经常见。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/9fc06a356efdc14c2a3c4a504d00763c.png" alt="9fc06a356efdc14c2a3c4a504d00763c" style="zoom:80%;" />

---

### 5、后处理程序执行流程（以 RNX2RTKP 为例）

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025155540386.png" alt="image-20231025155540386" style="zoom:80%;" />

### 6、后处理函数调用关系（以 postpos() 函数为例）

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012212121216.png" alt="image-20231012212121216" style="zoom:80%;" />

### 7、实时处理程序执行流程（以 RTKRCV 为例）

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024203128852.png" alt="image-20231024203128852" style="zoom:80%;" />

### 8、实时处理函数调用关系（以 RTKRCV 为例）



---

## 二、VS2022 + Windows 编译调试 RNX2RTKP

RTKLIB 有五大命令行程序：**rnx2rtkp**（后处理定位解算）、**rtkrcv**（实时定位解算）、**str2str**（数据流转发）、**convbin**（数据格式转换）、**pos2kml**（定位结果转谷歌地图格式）。除 `rtkrcv` 外用本章所给步骤都可以编译调试。

rtkrcv 无法直接在 Windows 下编译调试，因为它依赖了一些 Linux 库，这个issue说是用VS编译RTKRCV成功过，可以试试看：https://github.com/tomojitakasu/RTKLIB/issues/407。

一般都是用 VS 编译调试 rnx2rtkp，做后处理定位解算，做二次开发改算法，断点调试很方便。

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

可能会报`使用了可能未初始化的本地指针变量 “sbs” `的错误，解决方式是对指针变量进行初始化，将 ephemeris.c 文件中的第 579 行改为 `const sbssatp_t *sbs=NULL;`，还有些未初始化只报了警告，可以不用理会。

### 5、用示例数据做定位解算

命令行程序都是命令行参数来



完全自己写容易出错，要用的时候建议直接在以前写好的命令基础上改。



* GNSS定位解算必须要输入星历数据和观测数据才能进行

* 默认输出：
* Trace 设置：
* 







### 6、运行程序之后没有结果文件输出

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

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240221191921119.png" alt="image-20240221191921119" style="zoom:50%;" />

---

## 三、CMake+Linux 编译调试

开发中经常要用到 Linux，

WSL 全称 Windows Subsystem for Linux，是在 Widows 电脑上开发调试

### 1、把 RTKLIB 编译成第三方库

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



---

### 2、编译调试命令行程序、链接 RTKLIB

RTKLIB APP 目录下有 5 个命令行程序

* **rnx2rtkp**：后处理定位解算
* **rtkrcv**：实时定位解算
* **str2str**：数据流转换播发
* **convbin**：数据转换
* **pos2kml**：定位结果转谷歌地图数据格式

 可以用一个 CMake 工程同时构建这几个命令行程序：

 ```cmake
 cmake_minimum_required(VERSION 3.0)
 project(RTKLIB-CMake)
 
 # set build flags. 
 set(CMAKE_CXX_FLAGS "-std=c++20" )
 set(CMAKE_CXX_FLAGS "-fpermissive")
 if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
   set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
 endif()
 
 # set output dir
 set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/app/bin)
 set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/app/lib)
 
 # RTKLIB
 add_definitions(-DENAGLO -DENACMP -DENAGAL -DNFREQ=3 -DNEXOBS=3 -DDLL)
 add_subdirectory(rtklib)
 
 # VT
 add_library(vt vt/vt.c)
 target_link_libraries(vt rtklib)
 target_include_directories(vt PUBLIC vt)
 
 # executable
 add_executable(RNX2RTKP app/rnx2rtkp.c)
 target_link_libraries(RNX2RTKP rtklib)
 
 add_executable(RTKRCV app/rtkrcv.c)
 target_link_libraries(RTKRCV rtklib vt)
 
 add_executable(STR2STR app/str2str.c)
 target_link_libraries(STR2STR rtklib)
 
 add_executable(CONVBIN app/convbin.c)
 target_link_libraries(CONVBIN rtklib)
 
 add_executable(POS2KML app/pos2kml.c)
 target_link_libraries(POS2KML rtklib)
 ```



---

### 3、编译调试自己写的程序、链接 RTKLIB

比如我用 C++ 语法，通过调用 RTKLIB 的函数，实现获取当前系统时间，输出年月日时分秒、GPS周 + 周内秒，主函数文件如下：

```C++

```





## 四、QT编译调试





---

## 五、核心代码库介绍

代码库是 RTKLIB 的核心，要不咋能叫"LIB"呢？

RTKLIB 提供许多代码库和 API，包括：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM 函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

头文件 rtklib.h 是库的核心 ，主要有三大部分：**宏定义**、**结构体定义**、**全局变量**、**函数定义**

>  需要注意并非所有函数都可以直接调用，只有加了 EXPORT 前缀，而且在 RTKLIB.h 中声明了才行；想用 static 前缀的函数也很简单，只需要把前缀改成 EXPORT，然后在 rtklib.h 中加上声明。

### 1、宏定义



![rtklib.h 宏定义](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%20%E5%AE%8F%E5%AE%9A%E4%B9%89.png)



各种 ifdef

* **WIN32、WIN_DLL**：用 Windows 下的代码，
* **ENAGLO、ENAGAL、ENAQZS、ENACMP、ENAIRN、ENALEO**：启用除 GPS 外的卫星系统，
* **OBS_100HZ**：判定时间重合的阈值 DTTOL

* **MKL**：
* **LAPACK**：
* **IERS_MODE**L：
* **CPUTIME_IN_GPST**：
* **CLOCK_MONOTONIC_RAW**：
* **RRCENA**：
* **SVR_REUSEADDR**：
* **TIME_64BIT**：



---

### 2、结构体定义

![rtklib.h结构体](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%E7%BB%93%E6%9E%84%E4%BD%93.png)

---

### 3、全局变量

* `extern const double chisqr[];`：**卡方检验表**
* `extern const prcopt_t prcopt_default;`：**默认处理选项**
* `extern const solopt_t solopt_default;`：**默认结果选项**
* `extern const sbsigpband_t igpband1[9][8];`：**SBAS IGP 波段 0-8**
* `extern const sbsigpband_t igpband2[2][5];`：**SBAS IGP 波段 9-10**
* `extern const char *formatstrs[];`：**数据流格式字符串**
* `extern opt_t sysopts[];`：**系统选项表**

---

### 4、基础函数定义

#### 1. 矩阵、向量、最小二乘、卡尔曼滤波

* RTKLIB 中用 double 类型一维数组表示矩阵，不能自动识别矩阵的行列数，每次传矩阵的时候都要传入行数 n、列数 m。

* 用矩阵的时候要先 malloc 开辟空间，用完记得 free 释放空间。

* 要能熟练计算矩阵加减乘除转置求逆。

* RTKLIB 没有实现矩阵加减的函数，用的时候直接写 for 循环，比如把三维向量 dx 加到 X 上：

  ```c
  for (i=0;i<3;i++) X += dx;
  ```

* 矩阵求逆用的 LU 分解法，时间复杂度 $O^3$ ，对于大规模的矩阵，如果利用矩阵的稀疏性和对称性等特性，而且当使用不完全分解方法（例如，只计算到一定程度或使用截断技术）时，LU 分解的效率会更高。

* matprint() 很常用，调试的时候很难直接看的矩阵元素的值（都是指针），得输出到终端或者文件再看。

![image-20231021091639437](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091639437.png)

---

#### 2. 时间和字符串

* RTKLIB 中时间一般都以 `gtime_t` 类型存储，为了提高时间表示的精度，分开存 GPST 时间的整秒数和不足一秒的部分。
* 经常需要做年月日时分秒、周+周内秒、GPST 三种时间之间的转换；输出北京时间要在 UTC 基础上加 8 小时。
* BDT、GLONASST 不用于计算，读完文件就转为 GPS 时间了。

![image-20231021090651319](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021090651319.png)

---

#### 3. 坐标转换

* ECI 用的很少，只在 `sunmoonpos()` 函数中计算日月坐标时候用到了，不用怎么关注。
* ENU、ECEF、LLH 三套坐标系都频繁使用，要熟练掌握他们之间的转换，包括协方差的转换
* ENU 是局部相对坐标系，以某一个 LLH 坐标为原点，坐标转换的时候要传入这个 LLH 坐标。
* ENU 常用 `e`表示、ECEF 常用 `r` 表示、LLH 常用 `pos` 表示。

![image-20231021091756265](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091756265.png)

---

#### 4. 卫星系统、观测值

* **卫星系统表示**：
  * 表示卫星系统的字母：GRECJIS。
  * 7 位二进制码表示，对应位写 1 表示有对应的系统，做与运算可加系统。
* **卫星的表示**：
  * 可以表示为各系统的卫星 ID（系统缩写+PRN）：B02、C21。
  * 也可表示为连续的卫星编号 satellite number，断点调试或者看 Trace 文件的时候，经常只能看到卫星编号。
* **观测值类型**：
  * **C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比。
  * `CODE_XXX`：观测值类型定义，用一串连续的数字表示。
  * `sigind_t`：表示每种卫星系统的载波类型和观测值类型 ，每种类型的系统其实对应的就是一个 `sigind_t` 结构体。
* **观测值优先级**：

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024191221181.png)

---

### 5、选项调试输出

#### 1. 配置选项读取

* 选择主要存在 `prcopt_t`、`solopt_t`、`filopt_t` 三个结构体中。
* 后处理解算程序 rnx2rtkp 和实时解算程序 rtksvr 读取结果文件流程是一样的：
  * 先调用 `resetsysopts()` 重置所有配置为默认。
  * 调用 `loadopts()` 读取配置文件内容，存入 `opt_t` 的 `sysopt` 中。
  * 最后调用 `getsysopts()` 将 `opt_t` 转到 `porcopt_t`/`solopt_t`/`filopt_t`。

![image-20231025205106288](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205106288.png)

---

#### 2. Trace 调试

* 在 rtklib.h 中加入 #define TRACE，启用 trace ，不定义则将 trace 函数全赋空值。
* Trace 信息分五个等级，从 1-5 重要性逐渐降低，通过 tracelevel() 函数可以设置输出的最高等级，设置 2 级意味着只输出 1/2 级信息。
  * **一级 Trace 是致命错误**，出现一级错误基本上意味着程序无法继续执行，比如观测星历文件读取错误、内存分配错误。
  * **二级 Trace 是警告**，出现二级警告程序可能依然能继续执行，但也可能无法进行解算，比如改正文件读取失败，数据解析出错，二进制数据校验出错，某一历元解算失败，缺失解算所需的星历或改正参数等。
  * **三级  Trace 是程序主要执行流程**，主要在函数的开头，告诉我们执行到了这个函数。
  * **四级 Trace 是比三级更深入的程序执行流程**，主要在三级  Trace 函数的中间或者调用的子函数开头，告诉我们执行到了这个操作。
  * **五级 Trace 是解算的中间过程**，具体到每颗卫星，每个频点，每次循环。
* 看 Trace 文件可以辅助断点调试，甚至替代断点调试。程序执行出错，开 2/3 级 Trace，看 Trace 文件里的 error、warring 就能，知道大致出了啥问题，定位出问题的函数，断点调试的时候你就知道该在哪设置断点了。

![image-20231025205158762](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205158762.png)

---

#### 3. 结果输入输出、NMEA

* 输出的结果有两套：
  * 定位结果：坐标、协方差、有效果卫星数、差分龄期
  * 解算中间结果：
* NMEA 读取：

![image-20231025205301277](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205301277.png)

---

### 5、量测数据

#### 1. 导航数据输入

* 用完数据记得释放内存。

![image-20231025205602582](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205602582.png)

---

#### 2. RINEX 文件读写

![image-20231025205701553](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205701553.png)

---

#### 3. 二进制数据读写

* 用于数据流解析。

![image-20231025205800413](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205800413.png)

---

#### 4. 星历数据解析





---

#### 5. RTCM 读写

支持的 RTCM 消息包括：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6a7fd3b4bcb908dfff7b08d7e438bf83.png" alt="6a7fd3b4bcb908dfff7b08d7e438bf83" style="zoom:50%;" />

想进行定位解释至少要有星历，要有观测数据，常用 NAV 配 MSM4（伪距载波信噪比）、MSM7（伪距载波多普勒信噪比）

![image-20231025205952192](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205952192.png)

---

#### 6. 接收机自定义格式读写





---

### 6、解算相关

#### 1. 定位解算入口



![image-20231025210531296](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210531296.png)

---

#### 2. 实时解算



![image-20231025210624595](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210624595.png)

---

### 7、数据流相关

#### 1. 数据流

* 数据流函数用的大部分在 Window 和 Linux 各有一套，涉及到很多系统库，好在现在 AI 发达，可以用来辅助理解。
* 每种数据流关注四个函数：打开、关闭、写数据、读数据。

![image-20231025210846191](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210846191.png)

---

#### 2. 数据流线程管理



![image-20231025210938225](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210938225.png)

---

### 8、模型改正

#### 1. 星历、钟差、DCB、FCB



![image-20231025223050670](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223050670.png)

---

#### 2. SBAS



![image-20231025223134482](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223134482.png)

---

#### 3. 定位模型：计算高度角、方位角、卫地距、DOP 值



![image-20231025223220415](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223220415.png)

---

#### 4. 对流层、电离层模型



![image-20231025223257219](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223257219.png)

---

#### 5. 天线改正：读取天线文件



![image-20231025223330203](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223330203.png)

---

#### 6. 潮汐改正



![image-20231025223344099](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223344099.png)

---

#### 7. 水准面模型



![image-20231025223357868](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223357868.png)

---

#### 8. 高程转换

都是转日本的高程系统，咱们用不到。

![image-20231025223409859](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223409859.png)

---

### 9、其它杂项函数

#### 1. 下载函数

![image-20231025212102845](C:/Users/李郑骁的spin5/AppData/Roaming/Typora/typora-user-images/image-20231025212102845.png)

---

#### 2. 结果格式转换

把定位结果转为 KML、GPX 格式：

* **KML**：
* **GPX**：

![image-20231025212144253](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212144253.png)

---

#### 3. GIS 数据读取

可以读取 shapfile 矢量数据

![image-20231025212339125](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212339125.png)

---

#### 4. 平台相关函数

在 Windows 和 Linux 有完全不同的两套实现

![image-20231025212504258](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212504258.png)

---

#### 5. 用户自定义函数

在 rtklib.h 中声明了，在代码库中使用了，但没有实现，用代码库的时候需要我们自己实现，一般写个空函数就行。编译的时候经常会在这报错，如果说未定义就写三个空实现，如果重定义就把写的实现注释掉。





---

### 10、代码库的使用总结

总结一些命名和函数定义习惯：

* 矩阵做参数时一点要带上维度，矩阵 m 为行、n 为列，一般先传 n 后传 m。
* 带 `const` 的指针一定是输入参数，不带 `const` 的指针是输出参数或者既是输入也是输出。
* 类型命名结尾都带 `_t`，类型传参用的指针名不带 `_t`
* 用指针实现了顺序表，
* 很多读文件的函数都有结尾带 t 和结尾不带 t 两种，带 t 表示要传入开始时间和结束时间的。

* 





### 11、基于RTKLIB二次开发程序示例

下面介绍基于 RTKLIB 二次开发的程序 GAMP、GICI-LIB，着重介绍它的项目结构、CMakeLists.txt 文件和用到 RTKLIB 的部分；如果你也想基于 RTKLIB 进行二次开发，可以参考它的程序组织形式。

#### 1. GAMP

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些多余的函数、代码简洁化，精简出后处理 PPP 部分，并对算法进行改进增强。简化后代码比 RTKLIB 原版还要简单，对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的。使用也很方便，软件包里提供了 VS 工程，和组织好的配置文件、数据文件；设置好 pthreads 库，简单改改文件路径就能算出结果。

![GAMP](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GAMP.png)



相较 RTKLIB 的增强：

* **非差非组合的 PPP 模型**
* **钟跳修复**
* **北斗多路径延迟改正**
* **观测值信号量支持更多**
* **抗差估计**
* **伪距观测值质量检测**
* **MW + GF 周跳检测**
* **利用残差粗差探测**
* **计算了更多的 DOP 值**
* **对流层 GPT 模型**
* **GLONASS 伪距 IFB** 
* **输出结果更多**
* **GPT对流层模型**

#### 2. GICI-LIB



* **时间系统**：用 `gtime_t` 作为量测数据的时间戳，时间转换都用 RTKLIB 提供的接口。
* **坐标转换**：我程序的坐标都用 Eigen 库的 Vector3d 向量表示；为方便调用，我对 ENU、ECEF、BLH 坐标之间的转换函数做了一层封装，接口为 Eigen 形式。
* **结果输出**：为了能输出姿态，扩展了 sol_t 结构体，加上三个欧拉角，输出结果的语句上加上欧拉角；然后拓展 rtkplot，把姿态角结果也画出来。
* **GNSS相关的类型定义**：卫星系统、卫星、观测值定义
* **配置选项**：GNSS 相关的 
* **数据读取**：RINEX、RTCM、NMEA
* **数据流**：
* 



项目文件结构和 CMakeLists.txt 文件内容如下：



