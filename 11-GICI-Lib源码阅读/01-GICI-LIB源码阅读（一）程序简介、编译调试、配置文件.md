>  原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、GICI-LIB 简介

**作者的介绍**：为了阐明 GNSS 的算法模型，加快在多源融合应用中针对 GNSS 的开发效率，我们开源了 GICI-LIB，并辅以详尽的文档和全面的数据集。GICI-LIB 以可扩展的设计理念，实现了GIC传感器之间多种形式的松紧组合。评估结果表明，GIC 系统能够在多种复杂环境下，提供分米到米级的高精度导航。

### 1、程序概述

GICI-LIB 全称 **G**NSS/**I**NS/**C**amera **I**ntegrated Navigation Library，是上海交大 23 年开源的一套基于图优化的 GNSS+INS+Camera 集成导航定位库。基于 RTKLIB 处理 I/O 流、编解码；基于 OKVIS 因子图优化类型封装；基于 SVO 做特征提取。以 GNSS 为主，再加入 INS、Camera 做组合，支持相当多的数据格式、定位模式，包含很多 GNSS 因子、惯导因子、视觉因子及运动约束。以处理实时数据为主，后处理也采用模拟实时数据处理的方式进行。典型的应用方式如下图：

![1689512108793](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689512108793.png)

### 2、资源获取

- **论文**：[GICI-LIB: A GNSS/INS/Camera Integrated Navigation Library](https://arxiv.org/abs/2306.13268)，可以[在这](https://arxiv.org/pdf/2306.13268.pdf)下载
- **源码**：[https://github.com/chichengcn/gici-open](https://github.com/chichengcn/gici-open)
- **数据**：[https://github.com/chichengcn/gici-open-dataset](https://github.com/chichengcn/gici-open-dataset)
- **manual 的参考论文**：
  * Keyframe-based visual–inertial odometry using nonlinear optimization，[下载](https://sci-hub.st/10.1177/0278364914554813)
  * SVO: Semidirect visual odometry for monocular and multicamera systems，[下载](https://sci-hub.st/10.1109/TRO.2016.2623335)
  * Ionospheric time-delay algorithm for single-frequency GPS users，[下载](https://sci-hub.ru/10.1109/taes.1987.310829)
  * GPS meteorology: Mapping zenith wet delays onto precipitable water，[下载](https://journals.ametsoc.org/view/journals/apme/33/3/1520-0450_1994_033_0379_gmmzwd_2_0_co_2.xml?tab_body=pdf)
  * Global Mapping Function (GMF): A new empirical mapping function based on numerical weather model data，[下载](https://agupubs.onlinelibrary.wiley.com/doi/pdfdirect/10.1029/2005GL025546)
  * Features from Accelerated Segment Test ( FAST )，[下载](https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/AV1011/AV1FeaturefromAcceleratedSegmentTest.pdf)
  * An iterative image registration technique with an application to stereo vision，[下载](http://andrewd.ces.clemson.edu/courses/cpsc482/papers/LK81_stereoRegistration.pdf)
  * On-manifold preintegration for real-time visual–inertial odometry，[下载](https://sci-hub.ru/10.1109/tro.2016.2597321)
  * MLAMBDA: A modified LAMBDA method for integer least-squares estimation，[下载](https://sci-hub.ru/10.1007/s00190-005-0004-x)

### 3、功能简介

- 支持非 ROS 模式和 ROS 模式，ROS 模式下可以使用 ROS 话题、RVIZ 显示轨迹。

- 支持多种**数据形式**：serial 串口、TCP/IP server、TCP/IP client、Ntrip server、Ntrip client、[V4L2](https://baike.baidu.com/item/V4L2/10054109?fr=ge_ala)、file 文件。

- 支持多种**数据编码**：RTCM2、RTCM3、 Ublox raw、Septentrio raw、Tersus raw、NMEA、DCB file、ATX file、V4L2 image pack、GICI image pack、GICI IMU pack 

- 支持多种**数据传输方式**：I/O 端口、串口、ROS topics、TCP/IP、NTRIP、V4L2、文件

- 支持多种**定位模式**：SPP、SDGNSS、DGNSS、RTK、PPP、SPP-based LC GINS、TC GINS、RTK-based LC GINS、SRR GVINS、RRR GVINS。

- 支持多种**因子**：

  - **GNSS 松组合因子**：Position Error Factor、Velocity Error Factor
  - **GNSS 紧组合因子**：Pseudorange Error Factors、Carrier Phase Error Factors、Doppler Error Factors
  - **Camera 因子**：Reprojection Error Factor
  - **INS 因子**：Pre-integration Factors、Zero Motion Update Factor、Heading Measurement Constraint Factor、Non-holonomic Constraint Factor
  - **公共因子**：Parameter Error Factors、Relative Constant Error Factors、Relative Integral Error Factors

- 支持三个层级的 **SSR** 服务：

  - **一级**：提供精密星历、精密钟差、码偏差。
  - **二级**：除一级改正信息外，还提供载波相位偏差。
  - **三级**：除二级改正信息外，还提供大气延迟信息，能够支持 PPP-AR、PPP-RTK。

- GICI 不支持时间系统偏差的估计，需要进行硬件层面的时间对准，且最好精度要达到 5ms。

###  4、代码分析

GICI-LIB 主要使用 C++ 编写，且大量使用 C++ 高级语法，CMake 文件里规定以 C++11 的标准编译，采用多线程、面向对象程序设计方法，写法"很C++"，想看懂源码需要有一定的 C++ 基础。C++11 语法的内容可以看下图：

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/HCMO0D7ARSWD9IG%5D8~7SO04.jpg) 

具体来说，还有以下特点：

* 程序面向对象设计，大量使用继承、多态、友元、虚函数等面向对象特性。
* 大量使用模板泛型技术，用了很多 STL 容器、迭代器，用了 `std::function`、`std::bind`。
* 大量使用智能指针管理内存，还用 using 给智能指针起别名 xxxPtr。
* 使用多线程技术，接受数据和解算由不同的线程来做。
* 比较友好的地方是代码的注释还算完善，基本每个文件开头、每个函数、程序的关键代码段都有注释。
* 代码质量很高，但还可以再整理整理，比如有的紧挨着的地方上面用了 auto 而下面没用、有些函数名太长、构造函数写太长、streamer 和 estimator 线程有好几种命名，看着有点乱。

用 cloc 统计 include、src、tools 文件夹**代码量**（不包括 RTKLIB），结果如下：

|     语言     | 文件数  |   空行   |  注释行  |  代码行   |
| :----------: | :-----: | :------: | :------: | :-------: |
|     C++      |   110   |   3704   |   4547   |   26189   |
| C/C++ Header |   104   |   2746   |   4841   |   8452    |
|    MATLAB    |   16    |   111    |   232    |    666    |
|      C       |   10    |    98    |    93    |    394    |
|    CMake     |    9    |    71    |    36    |    262    |
|     YAML     |    4    |    8     |    17    |    185    |
|  Gencat NLS  |   13    |    0     |    0     |    94     |
|     XML      |    2    |    8     |    0     |    56     |
|   **总计**   | **268** | **6746** | **9766** | **36298** |

src 和 tools 文件夹内各子**文件夹功能信息**如下表所示：

|   文件名    |                         功能                         | 代码行数 |
| :---------: | :--------------------------------------------------: | :------: |
|  estimate   |               因子图优化相关类型、函数               |   2451   |
|   fusion    |      多源融合导航相关：估计器、初始化器类型定义      |   2872   |
|    gnss     |    GNSS相关：误差改正、模糊度固定、SPP、PPP、RTK     |   9347   |
|     imu     |             IMU相关：数值更新、误差改正              |   1725   |
|   stream    |           数据流相关、Node处理相关类型定义           |   2829   |
|   utility   |   全局变量、配置选项、信号处理函数、 Spin线程定义    |   1159   |
|   vision    | 视觉相关：初始化、特征提取、处理、跟踪、相对位置计算 |   1900   |
| conversions |                  时间转换、坐标转换                  |   175    |
| edit_binary |             二进制数据处理、处理采样间隔             |   1137   |
| evaluation  |                       NMEA相关                       |   1672   |
|     ros     |           ros 相关、话题消息发布、msg定义            |   2149   |
| matlab_plot |                   MATLAB 画图脚本                    |   666    |

### 5、第三方库

* **RTKLIB**：开源 GNSS 软件包，由一个程序库和多个应用程序工具库组成。
* **fast**：Features from Accelerated Segment Test，特征检测算法。
* **svo**：半直接发法稀疏直接法视觉里程计（Visual Odometry）算法，用到其中函数做视觉前端
* **OKVIS**：Open Keyframe-based Visual-Inertial SLAM，用到了其中的因子图函数
* **Eigen**：线性代数库，用于处理矩阵和向量的计算，它提供了许多线性代数运算的功能，包括矩阵运算、向量运算、特征值分解、奇异值分解、矩阵求逆等。 
* **Ceres**：Google 的非线性最小二乘库，用于实现因子图优化解算。
* **glog**：Google 的日志库。
* **OpenCV**：Open Source Computer Vision Library，跨平台计算机视觉库。
* **yaml-cpp**：GICI 采用 YAML 配置文件格式，YAML-CPP 是一个 C++ 的 YAML 库，用于 yaml 格式的解析和生成。

### 6、manual

GICI-LIB 的 manual 足足有 117 面，挺详细的，内容如下：

* 1~2：对软件简单介绍。
* 3~5：软件**编译**，下文会对每个步骤详细展开介绍。
* 5~39：介绍**配置文件格式**，下文会做总结，具体设置看文档。
* 40~45：介绍**常见使用场景及配置**，内容主要就是本文开头的图。
* 47~58：**硬件设置**：输入数据方式和格式、ROS 系统、硬件时间同步。
* 60~64：介绍**因子图优化基本模型**，下文会详细展开描写。
* 62~98：介绍各种**因子**。
* 98~110：介绍各种**定位模式**，下文会简单概括，详细的公式还是要看文档和代码。
* 111~113：**坐标系框架定义和转换**，多源融合涉及很多的坐标系，各种坐标系的统一至关重要。值得注意的是：导航坐标系（n 系）用ENU（东北天）、载体坐标系（b 系）用RFU（右前上）。
* 114~115：GNSS 线性组合和差分模型。
* 116：参考文献，上文已经给出下载链接。

### 7、程序执行流程图

![image-20230807135432767](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230807135432767.png)

### 8、定位模式

#### 1. GNSS

![1689675695644](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689675695644.png)

- **Single Point Positioning（SPP）**：用单历元的伪距、多普勒测量值，解算接收机的位置、速度、钟差。只实现了单频非差伪距的 SPP，没做消电离层，因为多频组合要额外估计 IFB，模型复杂的同时也提高不了多少 SPP 的精度。
- **Real-Time Differential（RTD）**：用单历元的双差伪距、非差多普勒测量值，解算接收机位置、速度。RTD 中可以很方便的使用多频观测值，因为 IFB 在双差中被大大消除，可以通过更改配置文件中 estimator 节点的输入项来设置。
- **Real-Time Kinematic（RTK）**：用多历元的双差伪距，双差载波，非差多普勒观测值，计算接收机位置、速度。估计了单差模糊度，如果有 m 颗共视卫星、n 个共同频率，需要估计 $m \times n$ 个单差模糊度。除了图优化估计，RTK 还有模糊度固定的步骤，GICI-LIB 支持部分模糊度固定。
- **Precise Point Positioning（PPP）**：使用非差的伪距、载波、多普勒观测值，估计接收机速度、位置、钟差、大气延迟、IFB。GICI-LIB 支持 PPP-AR，但没有完全测试，由于 PPP 比 RTK 浮点解的噪声更大，想固定必须要有偏差产品，并进行改正。
- **Global Frame Initialization**：纯 GNSS 解算在 ECEF 框架进行，而进行组合需要在 ENU 框架进行，因此需要进行转换。全局框架初始化定义一个基准点，用于将 ECEF 框架中的状态转换为 ENU 框架中的状态。基准点也可用于计算重力加速度，用于惯导惯导。设置基准点有两种方法：
  - 我们运行 SPP 估计，将基准点设置为该估计的第一个有效解。
  - 如果将 `force initial global position` 选项设置为 true，我们通过 `initial global position` 选项从配置文件中加载点坐标。 

#### 2. GNSS+ INS

![1689675885876](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689675885876.png)

- **Loosely Integration（LC）**：GNSS/INS 松组合用 GNSS 解算结果（位置、速度），和 INS 的原始数据（比力、角增量）进行组合解算。
- **Tightly Integration（TC）**：GNSS/INS 紧组合用 GNSS 的原始数据（伪距、载波、多普勒），和 INS 的原始数据（比力、角增量）进行组合解算。
- **Initialization**：GNSS/INS 初始化会估计速度、位置和偏差；为了提高效率，无论松紧组合都使用松组合的方程。先用加速度计比力量测值计算俯仰角和横滚角，当有足够比力量测值之后再对航向角进行估计，因子图结构与松组合一致，不同的是，我们使用多普勒来计算初始位置的增量，因为初始位置噪声过大，而多普勒噪声较低。

#### 3. GNSS + INS + Camera

![1689675982558](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689675982558.png)

> 摄像机状态之间没有相互联系，因为相应的参数是时变的。我们在图中保留了连接，以表示由于跟踪地标的切换，各次的估计参数会发生变化。

- **Solution/Raw/Raw Integration（SRR）**：SRR 采用 GNSS 的解算结果（速度、位置），和 INS 的原始数据（比力、角增量），以及 Camera 数据（特征点），进行组合解算。
- **Raw/Raw/Raw Integration（RRR）**：RRR 采用 GNSS 的原始数据（伪距、载波、多普勒），和 INS 的原始数据（比力、角增量），以及 Camera 数据（特征点），进行组合解算。
- **Initialization**：GNSS/INS/Camera 初始化位置、速度、偏差、特征点位置。分两步进行，先进行 GNSS/INS 初始化，再通过空间交汇对特征点追踪。

#### 4. Estimator 类型封装

GICI-LIB 提供的多种传感器在不同定位模式下的很多因子，并封装了一个基类，为所有传感器提供基础的函数去操作变量因子和量测因子，具体可以看**各种 `xxx_estimator_base.h`，看算法的时候肯定重点要看这些文件**。

![1690024555067](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690024555067.png)

## 二、GICI-LIB 编译

> * `git clone` 下载不了，可以去 `git clone` 后面的网站上手动下载。
>
> * 本文只介绍非 ROS 版。
>
> * 我使用的环境是 VSCode + WSl，很多导航定位的开源软件都基于 Linux，比起虚拟机，VSCode + WSl 要流畅一些，不熟悉的推荐看这篇博客：
>
>   > https://blog.csdn.net/donghening/article/details/124611881)。
>
> * 确保之前已经配置好 C++ 环境（g++、Cmake、VScode 插件）。
>
> * 看别人的博客，都提到了曾经安装过 glog/gflag 会出问题：
>
>   > GICI-OPEN 多源融合导航框架编译及问题说明：https://blog.csdn.net/zhaolewen/article/details/133245621
>
> * 构建、编译的时候找不到库，可能是因为库装到`/usr/local/lib` 里了，试试创建软链接到  `/usr/lib`
>
>   ```bash
>   ln -s /usr/local/lib/库名.a /usr/lib/库名.a
>   ```

### 1、安装需要的库

#### 1. 安装 Eigen

```bash
sudo apt-get install libeigen3-dev
```

#### 2. 安装 OpenCV

- 文件下载

  - OpenCV：https://opencv.org/releases/，下载太慢可以去：<https://www.raoyunsoft.com/wordpress/index.php/2020/03/09/opencvdownload/> 
  - 下载opencv_contrib（与 OpenCV 版本一致）：https://opencv.org/releases/
  - 两个都下载好以后，把opencv_contrib放到解压后opencv文件夹里面 

- 环境配置

  ```bash
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev
  ```

- 进入 opencv 目录编译安装

  ```bash
  cd opencv
  mkdir build
  cd build
  
  sudo cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
  
  sudo make -j8
  sudo make install
  ```

- 将 OpenCV 的库添加到路径，从而可以让系统找到 

  ```bash
  sudo vim /etc/ld.so.conf.d/opencv.conf
  
  在文件中加上并保存退出 /usr/local/lib
  
  sudo ldconfig
  ```

- 配置 bash

  ```bash
  sudo vim /etc/bash.bashrc
  
  # 在打开的文件最末尾添加以下代码并保存退出
  PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig  
  export PKG_CONFIG_PATH 
  
  source /etc/bash.bashrc
  ```

- 执行 `pkg-config --cflags opencv` 如果报错，可以看

  > https://blog.csdn.net/PecoHe/article/details/97476135)

#### 3. 安装 glfg、glog

- 下载 glfg

  ```bash
  git clone https://github.com/gflags/gflags
  ```

- 进入 glfg 目录编译安装

  ```bash
  cd glfg
  mkdir build
  cd build/
  
  cmake -DBUILD_SHARED_LIBS=ON -DBUILD_STATIC_LIBS=ON -DINSTALL_HEADERS=ON -DINSTALL_SHARED_LIBS=ON -DINSTALL_STATIC_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/ ..
  
  make
  sudo make install
  ```

- 下载 glog 

  ```bash
  git clone https://github.com/google/glog
  ```

- 进入 glog 目录编译安装

  ```bash
  cd glog
  mkdir build
  cd build 
  
  cmake -DGFLAGS_NAMESPACE=google -DCMAKE_CXX_FLAGS=-fPIC -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/opt/glog ..
  
  make
  sudo make install
  ```

- 为使 glog 库生效，需要在 `/etc/ld.so.conf.d` 下新建配置文件并使其生效 

  ```bash
  cd /etc/ld.so.conf.d
  sudo vim glog.conf
  输入 /opt/glog/lib
  
  sudo ldconfig
  ```

#### 4. 安装 Yaml-cpp

- 下载

  ```bash
  git clone https://github.com/jbeder/yaml-cpp.git
  ```

- 进入 Yaml-cpp 目录编译安装

  ```bash
  cd yaml-cpp
  mkdir build 
  cd build
  
  cmake -D BUILD_SHARED_LIBS=ON ..
  
  make -j16
  sudo make install
  ```

#### 5. 安装 ceres-solver

- 下载

  ```bash
  git clone https://ceres-solver.googlesource.com/ceres-solver
  ```

- 进入 ceres-solver 目录编译安装

  ```bash
  mkdir ceres-bin
  cd ceres-bin
  cmake ../ceres-solver-2.1.0
  make -j3
  make test
  make install
  ```

### 2、GICI-LIB 编译

- 下载

  ```bash
  git clone https://github.com/chichengcn/gici-open
  ```

- 编译

  在工程目录下打开终端，输入以下命令：

  ```bash
  mkdir build
  cd build
  cmake ..
  make -j4 	# 编译需要一段时间
  ```

## 四、glog 日志系统

glog 即 Google Log ，是一个 Google 开源的日志库，它提供了一个轻量级的、可扩展的、跨平台的日志系统。 glog 的用法包括：

- **引入头文件**：需要包含 glog 的头文件：

  ```c++
  include <glog/logging.h>
  ```

- **初始化库**：在开始使用 glog 之前， 初始化库，例如：

  ```c++
  google::InitGoogleLogging(argv[0])
  ```

- **配置日志**：可以通过配置文件或代码来配置 glog 的参数，例如：

  ```c++
  google::SetLogDestination(LOG_TO_FILE, "/path/to/logfile.log")
  ```

- **输出日志**：使用 `LOG(level)` 宏函数来输出日志。`level` 表示日志的严重程度，可以是以下几个级别之一：`INFO`：一般信息、`WARNING`：警告信息、`ERROR`：错误信息、`FATAL`：致命错误信息，输出后会终止程序。例如输出一般信息：

  ```c++
  LOG(INFO) << "This is an informational message."
  ```

- **条件输出日志**：使用 `LOG_IF()`、`LOG_EVERY_N()` 和 `LOG_FIRST_N()`  宏函数来条件输出日志。例如：

  ```c++
  LOG_IF(INFO, num_cookies > 10) << "Got lots of cookies"
  ```

- **关闭日志**：在程序结束之前，关闭 glog：

  ```c++
  google::ShutdownGoogleLogging()
  ```

## 五、YMAL 配置文件

在手册的 9~39 面，详细的介绍了配置文件的具体内容。GICI-LIB 采用 YAML 配置文件格式，下面先对 YMAL 做个简单介绍。

### 1、YAML 简介

> 链接时找不到 yaml-cpp，可以参考博客：[error while loading shared libraries的解决方案](https://blog.csdn.net/weixin_42310458/article/details/125180410)，在 `/etc/ld.so.conf` 文件中加上 `/usr/local/lib`

YAML（YAML Ain't Markup Language）是一种轻量级的数据序列化格式，可以用于配置文件、数据交换、API请求等多种场景。它是一种简单易用的数据序列化格式，使得数据可以以人类可读的方式进行存储和传输。YAML的语法非常简单，它使用缩进和符号来表示数据结构。以下是一些YAML的基本语法：

1. **字符串**：用引号括起来的文本，例如："hello world"。

2. **数字**：没有引号的数字，例如：42。

3. **布尔值**：用 true 或 false 表示的真或假。

4. **缩进**：YAML使用缩进来表示嵌套关系，每个缩进级别用空格数表示。例如，下面的代码段表示一个包含两个列表的字典：

5. **字典/对象/键值对**：用短横线 `-` 或中括号`[]`表示的键值对的集合。例如：`{name: John, age: 30}`或`- name: John age: 30`。多层对象可表示为：

   ```yaml
   key: {key1: value1, key2: value2}
   ```

   或者

   ```yaml
   key:
     key1: value1
     key2: value2
   ```

6. **数组/列表**：用短横线 `-` 或中括号 `[]` 表示的值的列表。例如：`[apple, banana, orange]` 或 `- apple - banana - orange`。复杂一点的如：

   ```yaml
   streamers:
       - streamer:
           tag: str_gnss_rov
           output_tags: [fmt_gnss_rov]
           type: file
           path: <data-directory>/gnss_rover.bin
       - streamer:
           tag: str_gnss_ref
           output_tags: [fmt_gnss_ref]
           type: file
           path: <data-directory>/gnss_reference.bin
   ```

7. **引用**：`&` 用来建立锚点，`<<` 表示合并到当前数据，`*` 用来引用锚点。 

8. **注释**：在YAML中，使用 `#` 表示注释。

> YAML 需要特别注意的几个点：
>
> - 大小写敏感；
> - 缩进不允许使用 tab，只允许空格；
> - 缩进的空格数不重要，只要相同层级的元素左对齐即可；

### 2、读取 YAML 的语法

1. YMAL 在 C++ 中以 Node 类表示。

2. **LoadFile()**：从文件中加载 YAMl 到 C++ 中 Node 对象：

   ```c++
   yaml_node = YAML::LoadFile(文件名); 
   ```

3. **[]**：Node 对象可以理解为是树形的，用中括号可以取出里面的子数，创建一个新的 Node 对象：

   ```c++
   YAML::Node logging_node = yaml_node["logging"];
   ```

4. **safeGet()**：第一个参数为 Node，第二个参数为关键字，判断配置文件的 Node 里有没有你要的那个关键字，有的话再把对应的值作为第三个参数返回。

   ```c++
   option_tools::safeGet(logging_node, "log_to_stderr", &FLAGS_logtostderr);
   ```

5. **checkSubOption()**：第一个参数为 Node、第二个参数为子配置选项，检查参一中是否存在参二子配置选项，如果不存在有两种处理：

   - 当参三为 true 时，LOG(FATAL) 退出程序。
   - 当参三为 false 时，LOG(INFO) 输出错误到日志文件。

下面三个函数都是写了函数模板，然后重载写了很多的 ：

1. **convert()**：参一传入 YAML 配置种类字符串，转换成 StreamerType、FormatorType 等枚举值作为参二返回。
2. **sensorType()**：传入 formator_role 字符串，转换成传感器种类枚举值 SensorType 返回。
3. **loadOptions()**：参一传入文档 18~39 对应 estimate 的选项 Node，转换成对应的 ImuParameters、AmbiguityResolutionOptions 等选项结构体作为参二返回。

### 3、GICI-LIB 配置文件结构

配置文件以 **数组 + 键值对** 的方式组织，每一个键值对都是一个配置项，用多级数组来分模块组织配置项键值对。有三大模块：stream、estimate、logging，其中 stream 模块内还分为三个子模块 steamers、formators、replay。如下图：

![1689518262550](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689518262550.png)

### 4、示例配置文件

option 文件夹里有一些配置文件，以伪实时定位解算为主，对应于下面的应用方式，图上每个模块都对应着咱们要配置的内容。

![1689512108793](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689512108793.png)

**使用方式**：

* 建立 data、output 文件夹，存放数据和输出。

- 将 yaml 配置文件中的 `<data-directory>`、`<gici-root-directory>` 和 `<output-directory>` 分别换成你的`数据文件夹路径`、`gici-open 文件夹的路径`、和 `输出文件夹路径`。
- 改配置中的 `start_time`，起始时间。
- `streamer` 写了两套，非 ROS 模式和 ROS 模式，想用哪套就把另一套注释掉。
- 注意看 `streamers` 里一项项 `streamer` 的路径项 `path`，确保数据文件夹中都有对应的数据。
- 有些 `streamer` 中路径设置在 option 文件夹中，程序会从 `gici-open 文件夹的路径` 找 option 文件夹，确保 option 文件夹和里面数据在对应位置，最好不要动 option 文件夹。
- 程序运行前，把 yaml 配置文件的路径加到命令行参数中。
- 结果在 output 文件夹下，`xxx_solution.txt` 文件可以直接用 `rtkplot` 打开查看结果。用`matlab_plot` 里的脚本应该也行。

> 一定注意，配置文件中有好几处 `<data-directory>`、`<gici-root-directory>` 、 `<output-directory>` 、`start_time` 要改，不能漏，我在这卡了很久。报错了，在YAML文件里Ctrl+F搜搜这几个，看看还有没有没改的。
>
> 没运行成功，仔细看看报错信息，INFO不用看，关注ERROR，看对应配置是否正确。

## 六、数据集

> 对 GitHub 上的介绍简单做个翻译

### 1、数据集介绍

* Github地址：https://github.com/chichengcn/gici-open-dataset
* 百度网盘下载：https://pan.baidu.com/share/init?surl=xZS-C_42LrGtUB0x6Bw_0A&pwd=6ncd，提取码：6ncd

作者专为开发 GICI-LIB 而搭建的数据采集的平台如下图所示：

![image-20230902170048417](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230902170048417.png)

开发了一块 GICI 板，用于收集 IMU 和摄像头数据，并在整个平台中应用了与其他传感器同步的硬件。板载 IMU 和摄像头分别为博世 BMI088 和 Onsemi MT9V034。GNSS 接收器为 Tersus David30 多频接收器。我们还从千寻 SI 数据流中收集了参考站数据，用于 RTD 和 RTK ，并从国际 GNSS 服务（IGS）数据流中收集了状态空间表示（SSR）数据，用于 PPP。光纤 IMU 通过对其数据和 GNSS 原始数据进行后处理来提供参考值。

收集了两种数据集：不同场景的短期（几分钟）实验（1.1 ~ 4.3）和涵盖多个场景的长期（几十分钟）实验（5.1 ~ 5.2）。在短期实验中，我们将场景分为 4 类： 开阔天空、绿树成荫、典型城市和密集城市。对于每个场景，我们提供 2 ~ 3 条轨迹。在长期实验中，我们提供了在上海市中心收集到的涵盖这些场景的两个轨迹。

| ID   | Scene         | Size   | Date       | Scene View                                                   |
| ---- | ------------- | ------ | ---------- | ------------------------------------------------------------ |
| 1.1  | Open-sky      | 0.7 GB | 2023.03.20 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_1.1.md) |
| 1.2  | Open-sky      | 0.5 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_1.2.md) |
| 2.1  | Tree-lined    | 1.4 GB | 2023.03.20 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_2.1.md) |
| 2.2  | Tree-lined    | 0.6 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_2.2.md) |
| 3.1  | Typical urban | 1.7 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_3.1.md) |
| 3.2  | Typical urban | 1.4 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_3.2.md) |
| 3.3  | Typical urban | 1.9 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_3.3.md) |
| 4.1  | Dense urban   | 1.4 GB | 2023.05.21 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_4.1.md) |
| 4.2  | Dense urban   | 0.8 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_4.2.md) |
| 4.3  | Dense urban   | 1.6 GB | 2023.03.27 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_4.3.md) |
| 5.1  | Long-term     | 8.2 GB | 2023.05.21 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_5.1.md) |
| 5.2  | Long-term     | 5.8 GB | 2023.05.21 | [Images](https://github.com/chichengcn/gici-open-dataset/blob/master/figures/typical_scene/README_5.2.md) |

> 用对应数据的时候记得改时间

### 2、非 ROS 方式使用数据集

提供了各种 YAML 配置文件示例，在` <gici-root-directory>/option`。请记住替换所有 `<path> `和 `"start_time"`。通过命令来运行软件处理数据集：

```bash
./gici_main <gici-config-file>
```

要将实时输出流连接到 RTKLIB，应执行以下步骤：

1. 指定 NMEA 格式的 TCP 服务器输出。配置示例见pseudo_real_time_estimation_RTK_RRR.yaml
2. 在 Windows 计算机中打开 RTKPLOT。想要访问 Linux 计算机的 IP 地址，Windows 计算机必须位于同一网段。
3. 单击文件->连接设置。启用 TCP 客户端。单击选项配置 TCP 客户端选项。填写服务器地址（Linux 计算机的 IP）和端口（在 GICI YAML 文件中配置）。
4. 点击文件->连接，形成连接。然后就可以看到实时绘制的结果图了。

### 3、把原始数据转为 rosbag

我们提供了一个将 bin 文件转换为 rosbags 的工具，请参见 `<gichi-root-directory>/tools/ros/gici_tools/src/gici_files_to_rosbag.cpp`。其配置文件位于 `<gici-root-directory>/tools/ros/gici_tools/option/convert_rosbags.yaml` 中。请记住替换所有 `<path>` 和` "start_time"`。

可以通过以下方式编译转换器：

```bash
cd \<gici-root-directory\>/tools/ros
catkin_make -DCMAKE_BUILD_TYPE=Release
```

然后可以通过以下方式运行转换器：

```bash
./devel/lib/gici_tools/gici_files_to_rosbag <config-file>
```

### 4、ROS 方式

YAML 配置文件示例，请参见 `<gichi-root-directory>/ros_wrapper/src/gici/option`。使用前要替换所有`<path>`和 `"start_time"`。在运行 ROS 可执行文件之前，请记得运行一个 roscore。然后，可以通过以下方式运行可执行文件：

```bash
rosrun gici_ros gici_ros_main <gici-config-file>
```

或者：

```bash
cd \<gici-root-directory\>/ros_wrapper
./devel/lib/gici_ros/gici_ros_main <gici-config-file>
```

之后，您可以通过以下方式播放从我们的 bin 文件转换而来的 rosbags

```bash
rosbag play <data1.bag> <data2.bag> <data3.bag> ...
```

为了实现可视化，您可以通过以下方式运行我们的 RVIZ 配置：

```bash
rviz -d \<gici-root-directory\>/ros_wrapper/src/gici/rviz/gici_gic.rviz
```

### 5、结果评估

我们为每个数据集提供 ground_truth.txt。参考数据采用光纤 IMU 的框架。在比较结果之前，您应该进行坐标转换。对于包含 IMU 的估计器，GICI 以 IMU 框架输出解决方案。我们提供将参考值转换为 IMU 框架的工具。首先要编译这个工具：

```bash
cd \<gici-root-directory\>tools/evaluation/alignment
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8

cd \<gici-root-directory\>tools/evaluation/format_converters
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

然后，您可以通过以下方法转换参考值：

```bash
\<gici-root-directory\>tools/evaluation/format_converters/build/ie_to_nmea ground_truth.txt
\<gici-root-directory\>tools/evaluation/alignment/build/nmea_pose_to_pose ground_truth.txt.nmea
```

nmea_pose_to_pose.cpp 中的默认设置是将姿势从光纤 IMU 帧转换为数据集的 IMU 帧。如果您有其他要求，应修改 nmea_pose_to_pose.cpp 中的参数。现在，您将获得以 NMEA 格式转换的参考值文件 ground_truth.txt.nmea.transformed。为了便于可视化，您可以通过以下方法将该文件转换为 TUM 格式

```bash
\<gici-root-directory\>tools/evaluation/format_converters/build/nmea_to_tum ground_truth.txt.nmea.transformed
```

还可以将 GICI NMEA 输出转换为 TUM 格式，然后用任何软件进行比较。

对于纯 GNSS 估计器，GICI 以 GNSS 天线框架输出解决方案。您应进一步将参考值转换为 GNSS 天线，方法是

```bash
\<gici-root-directory\>tools/evaluation/alignment/build/nmea_pose_to_position ground_truth.txt.nmea.transformed
```

现在您会得到一个参考值文件 ground_truth.txt.nmea.transformed.translated。然后就可以继续上面的操作了。
