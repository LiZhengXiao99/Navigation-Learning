[TOC]

## 一、程序简介

### 1、ORB-SLAM 概述

ORB 指 **O**riented FAST and **r**otated **B**RIEF，是一种结合 FAST 和 BRIEF，并引入旋转不变性的一种特征点和描述子；SLAM 指 **S**imultaneous **L**ocalization **a**nd **M**apping，指的是同时进行实时定位和地图构建。

ORB-SLAM3 是**迄今为止，最完整的视觉惯性 SLAM 系统系统**，它是第一个集成了单目相机、双目相机、RGB-D相机，以及单目相机结合 IMU、双目相机结合 IMU 的 SLAM 系统。并且在 ORB-SLAM2 的基础上，改进了相机模型，使其不再局限于传统的小孔成像模型，而是可以**扩展到鱼眼模型**。在与 IMU 的结合上，它根据运动模型在流形上进行 **IMU 的预积分**的方式，然后采用非线性优化的思想，**将 IMU 的预积分结果和视觉 SLAM 的重投影模型一同进行图优化，使得预积分残差以及重投影误差共同达到最小**，以此来完成视觉信息和惯导系统的**紧耦合**。并且它采用了更为快速的**初始化**方法，以及丢失跟踪后利用惯导系统快速**重定位**方法。此外，它还采用**地图集**的方式，实现了对大场景的定位建图。这也是如今众多开源方案中，功能最强大、最精准的方法。系统框图如下：

![image-20230815102741960](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815102741960.png)

### 2、ORB-SLAM3 历史与演变

![1690952265203](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690952265203.png)

#### 1. PTAM（Parallel Tracking and Mapping）

基于关键帧，首次将跟踪（Tracking）和建图（Mapping）分成两个独立线程：

- **跟踪线程**负责跟踪相机位姿，同时绘制虚拟的模型。
- **建图线程**负责建立场景的模型和绘制场景的地图。

#### 2. ORB-SLAM

在 PTAM 基础上，提出基于特征点的单目 SLAM 算法

- **跟踪线程**：提取 ORB 特征，根据上一帧进行初始位姿估计，或者通过全局重定位初始化相机位姿，然后跟踪已重建的局部地图来优化位姿，最后根据一些规则输出关键帧。
- **建图线程**：包括插入关键帧、验证最近生成的地图点并进行筛选，同时生成新的地图点，使用局部 BA，最后对插入关键帧进行筛选，去除多余的关键帧。
- **回环检测线程**：分为回环检测和回环校正。首先通过 BOW 加速闭环匹配帧的选择，然后通过 Sim3 计算相似变换，最后通过回环融合和本质图优化，实现闭环检测功能。

![1690940910132](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690940910132.png)

#### 3. ORB-SLAM-VI

加入 IMU，针对单目 SLAM 缺少尺度信息，提出了新的 IMU 初始化方法，以高精度地图快速计算尺度、重力方向、速度以及陀螺仪和加速度计偏差，并重用地图，在已建图区域实现零漂移定位。

![1690941287964](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690941287964.png)

#### 4. ORB-SLAM2

![image-20230815105030876](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815105030876.png)

- 是第一个适用于单目、双目、RGB-D相机，还包括闭环控制、重定位、地图重用的开源 SLAM 系统。
- 在 RGB-D 模式下使用 BA 算法，准确率高于基于迭代最近点和光度深度误差最小化算法。
- 利用远近双目点和单目量测，使得在双目相机的情况下，算法的准确率高于以前最好的直接法双目视觉 SLAM。
- 设置了一个轻量级定位模式，能暂停地图构建线程，同时能有效地重复利用已建的地图，

#### 5. ORB-SLAM-Atlas

实现多地图合并，包含一个鲁棒的地图合并算法，能处理无限数量非连续的子地图系统。

#### 6. ORB-SLAM3

支持视觉、视觉+惯导、混合地图的 SLAM 系统，可以在单目、双目、RGB-D 相机上利用针孔和鱼眼模型运行。

### 3、资源获取

* 源码：https://github.com/UZ-SLAMLab/ORB_SLAM3

* 推荐博客：[史上最全slam从零开始-总目录](https://blog.csdn.net/weixin_43013761/article/details/123092806)
* 网上有很多 ORB-SLAM 源码解析的课，

### 4、代码分析



文件结构如下：

* Examples 和Exampleold 根据传感器类型，分别存放新的和旧的代码实例。
* include 和 src 分别存放代码的 .h 头文件和 cc/cpp 原文件。
* Thirdparty 存放了 DBOW2、Sophus 和 g2o。
  * DBOW2 是词袋模型，推荐博客：[DBoW2库介绍](https://www.cnblogs.com/luyb/p/6033196.html)
  * Sophus 是李代数库，
  * g2o 是图优化库，
* Vocabulary 存放ORB词典。





## 二、基础知识点

### 1、视觉SLAM简介

#### 1. 定位与建图关系

![1690934934097](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690935198158.png)

代表性 SLAM系统包括：

![image-20230815104820277](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815104820277.png)

#### 2. 单目、双目、RGB-D

* **单目相机**：单目相机在环境中移动，不断获得图像帧，通过两帧之间三角化计算，将空间的点投影到相机空间，对环境进行建模。
* **双目相机**：左右相机同时能得到两图像帧，直接可以获得距离。
* **RGB-D相机**：实现方式有多种：
  * **Kinect1**：使用红外光，根据返回结构光图像，计算距离。
  * **Kinect2**：用脉冲信号，根据收发信号时间差计算距离。

#### 3. 特征点法、直接法

* **特征点法**：处理的是特征点，先提取图像特征，通过特征匹配估计相机运动，优化的是重投影误差，常见开源方案有 ORB-SLAM。
* **直接法**：处理的是像素，根据像素灰度信息估计相机的运动，可以不用计算关键点和描述子，优化的是光度误差；根据使用的像素数量可以分为稀疏、半稠密和稠密三种，常见的开源方案有 LSD-SLAM、DSO 等。

![1690937665104](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690937665104.png)

#### 4. 框架

当前主流的 SLAM 系统由 4 个阶段组成，分别为前端视觉里程计(visual odometry, VO)、后端优化(optimization)、回环检测( loop closing) 和三维地图建立(mapping)。

* **视觉里程计**（Visual Odometry，VO）：估算相邻图像间相机的运动，以及局部地图，求解传感器在各个时刻的位置信息。也称前端（Front END）。
* **后端优化**（Optimization）：接收不同时刻视觉里程计测量的相机位姿，以及回环检测的信息，根据它们进行优化，得到全局一致的轨迹和地图。也称后端（Back End）。
* **回环检测**（Loop Closing）：判断出载体是否到达过先前的位置，如果检测到就将数据传给后端。
* **建图**（Mapping）：根据估计的轨迹，建立与任务要求对应的地图。

### 2、ORB 图像特征

ORB 特征包括特征点和描述子：

* **特征点**：可以理解为图像中比较显著的点，如轮廓点、暗区的亮点、亮区的暗点等，采用 FAST 得到。
* **描述子**：获得特征点后，要以某种方式描述特征点的属性，这些属性的描述称之为该特征点的描述子，采用 BRIEF 得到。

#### 1. FAST

FAST 核心思想就是找出哪些有代表的点，即拿一个点与周围的点比较，如果和周围大部分点都不一样（有明显的像素值变化）就可以认为是一个特征点。

![1690943970181](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690943970181.png)

FAST 的计算过程，即逐个判断像素是否为特征点：

* 设定一个合适的阙值 $\mathrm{t}$：当 2 个点的灰度值之差的绝对值大于 $\mathrm{t}$ 时，则认为这 2 个点不相同。
* 考虑该像素点周围的 16 个像素。如果这 16 个点中有连续的 $n$ 个点 都和点不同，那么它就是一个角点。这里 n 设定为 12。
* 现在提出一个高效的测试，来快速排除一大部分非特征点的点。 该测试仅仅检查在位置 1、9、5 和 13 四个位置的像素。如果是一个角点，那么上述四个像素点中至少有 3 个应该和点相同。如果都不满足，那么不可能是一个角点。

#### 2. BRIEFF

BIREF 算法的核心思想是在关键点 P 的周围以一定模式选取 N 个点对，把这 N 个点对的比较结果组合起来作为描述子。

![1690954752414](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690954786672.png)

BIREF 算法**计算流程**：

* 以关键点 $P$ 为原点，以 $d$ 为半径作圆 $O$。

* 在圆 $O$ 内某一模式选取 $N$ 个点对。这里为方便说明，$N=4$，实际应用中 $N$ 可以取 512 。

* 假设当前选取4个点，分别标记为：$P_{1}(A, B) , P_{2}(A, B)  ,P_{3}(A, B) , P_{4}(A, B)$。

* 定义操作 T：$ T(P(A, B))=\left\{\begin{array}{ll}1 & I_{A} \\ 0 & I_{B}\end{array}\right.$，其中 $I_A$ 表示 $A$ 的灰度。

* 分别对已选取的点进行 T 操作，将获得的结果进行组合。比如：
  $$
  \begin{array}{ll}
  T\left(P_{1}(A, B)\right)=1 & T\left(P_{2}(A, B)\right)=0 \\
  T\left(P_{3}(A, B)\right)=1 & T\left(P_{4}(A, B)\right)=1
  \end{array}
  $$

* 则最终描述子为：1011

#### 3. ORB 改进 BRIEF

实际情况下，从不同的距离，不同的方向、角度，不同的光照条件下观察一个物体时，物体的大小，形状 ，明暗都会有所不同。但我们的大脑依然可以判断它是同一件物体。

**理想的特征描述子**应该具备这些性质。即，在大小、 方向、明暗不同的图像中，同一特征点应具有足够相似的描述子，称之为描述子的**可复现性**。

当以某种理想的方式分别计算上图中红色点的描述子时，应该得出同样的结果。即描述子应该对光照 (亮度) 不敏感，具备尺度一致性（大小），**旋转一致性** (角度) 等。

当图像发生旋转时，用上面方法计算得到的描述子显然会改变，缺乏旋转一致性。ORB 在计算 BRIEF 描述子时建立的坐标系以关键点为圆心，以关键点和选取区域质心（灰度值加权中心）的连线为 X 轴建立二维坐标系。如下图：PQ 作为坐标轴，在不同旋转角度下，同样的几个特征点的描述子是不变的，就解决了旋转一致性问题。

![1690955776407](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690956331103.png)

#### 4. 特征匹配

例如，特征点 A、B 的描述子分别是：10101011、10101010。首先设定一个**阈值**，比如 85%，当 A、B 的描述子相似程度大于 85% 时，认为是相同特征点。该例中相似度 87.5%，A、B 是匹配的。

### 3、图像关键帧

关键帧是图像帧中具有代表性的帧，目的在于降低信息冗余度、减少计算机资源的消耗、保证系统的实时性。

#### 1. 关键帧的选取原则

* **图像质量**：画面清晰、特征点多、分布均匀。
* **连接关系**：与其它关键帧直接有一定的共视关系，同时重复度也不能太高。

#### 2. 共视

* **共视关键帧**：某一关键帧与该关键帧共同观测（共视）到的地图点的关键帧，也称一级关键帧。共同观测到相同特征点的数目称为**共视程度**。
* **共视图**：共视图的节点是关键帧，若两个关键帧的共视地图点超过某一阈值（>=N)，则用边连接，构成无向加权图，边的权重为共视点的数量。

![1690957281149](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690957388374.png)

#### 4. 共视图、生成树、本质图的关系

相同点在于都是以关键帧作为节点，帧与帧之间通过边来连接的模型。不同点在于：

* 共视图(Covisibility Graph)最稠密，本质图(Essential Graph)次之，生成树(Spanning tree)最稀疏。
* 共视图保存了关键帧与所有共视关键点大于某一阈值的的共视帧之间的关系。
* 本质图包含生成树的连接关系、形成闭环的连接关系、共视关系很好的连接关系(共视关键点 $>=100$ )。
* 生成树只包含了关键帧与附近共视关系最好的关键帧。

![1690957525842](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690957751633.png)

![image-20230814160947346](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814160947346.png)

### 4、IMU 预积分

IMU（Inertial measurement unit）惯性测量单元，包括**加速度计**和**角速度**。加速度计用于测量物体的加速度，陀螺仪可以测量物体的三轴角速度。IMU 与视觉 SLAM 的互补：

* IMU 可以为视觉 SLAM 提供：**尺度信息**、**重力方向**、**速度**。
* 视觉 SLAM 可以得到 IMU 的 **Bias**。

IMU 得到的数据包括角速度和加速度、可以通过**积分**得到速度和位置：
$$
\begin{aligned}{ }_{\mathrm{B}} \tilde{\boldsymbol{\omega}}_{\mathrm{WB}}(t) & ={ }_{\mathrm{B}} \boldsymbol{\omega}_{\mathrm{WB}}(t)+\mathbf{b}^{g}(t)+\boldsymbol{\eta}^{g}(t) \\ { }_{\mathrm{B}} \tilde{\mathbf{a}}(t) & =\mathrm{R}_{\mathrm{wB}}^{\top}(t)\left({ }_{\mathrm{w}} \mathbf{a}(t)-{ }_{\mathrm{w}} \mathbf{g}\right)+\mathbf{b}^{a}(t)+\boldsymbol{\eta}^{a}(t),\end{aligned}
$$
由于 IMU 得到的数据时离散的，所以通过**累加和**的方式得到旋转、速度和平移数据：
$$
\begin{array}{l}\mathrm{R}_{j}=\mathrm{R}_{i} \prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \Delta t\right) \\ \mathbf{v}_{j}=\mathbf{v}_{i}+\mathbf{g} \Delta t_{i j}+\sum_{k=i}^{j-1} \mathrm{R}_{k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t \\ \mathbf{p}_{j}=\mathbf{p}_{i}+\sum_{k=i}^{j-1}\left[\mathbf{v}_{k} \Delta t+\frac{1}{2} \mathbf{g} \Delta t^{2}+\frac{1}{2} \mathrm{R}_{k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\eta_{k j}^{a d}\right) \Delta t_{j}^{2}\right]\end{array}
$$
上式旋转、速度和平移量都是耦合在一起的，计算量太大。可以将这三个量**解耦**，只需通过 IMU 的读数和前一时刻的预积分量就可以更新当前的**预积分**量：
$$
\begin{aligned} \Delta \mathrm{R}_{i j} & \doteq \mathrm{R}_{i}^{\top} \mathrm{R}_{j}=\prod_{k=i}^{j-1} \operatorname{Exp}\left(\left(\tilde{\boldsymbol{\omega}}_{k}-\mathbf{b}_{k}^{g}-\boldsymbol{\eta}_{k}^{g d}\right) \Delta t\right) \\ \Delta \mathbf{v}_{i j} & \doteq \mathrm{R}_{i}^{\top}\left(\mathbf{v}_{j}-\mathbf{v}_{i}-\mathbf{g} \Delta t_{i j}\right)=\sum_{k=i}^{j-1} \Delta \mathrm{R}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t \\ \Delta \mathbf{p}_{i j} & \doteq \mathrm{R}_{i}^{\top}\left(\mathbf{p}_{j}-\mathbf{p}_{i}-\mathbf{v}_{i} \Delta t_{i j}-\frac{1}{2} \sum_{k=i}^{j-1} \mathbf{g} \Delta t^{2}\right) \\ & =\sum_{k=i}^{j-1}\left[\Delta \mathbf{v}_{i k} \Delta t+\frac{1}{2} \Delta \mathrm{R}_{i k}\left(\tilde{\mathbf{a}}_{k}-\mathbf{b}_{k}^{a}-\boldsymbol{\eta}_{k}^{a d}\right) \Delta t^{2}\right]\end{aligned}
$$

### 5、VIO 视觉惯性里程计

视觉里程计 VO 通过最小化相机帧中地标的重投影误差，计算得到相机的位姿和地标的位置。IMU 对相邻两位姿直接进行约束，而且对没有帧添加了状态量：陀螺仪和加速度计的 Bias 及速度。

![1690953898017](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690953949051.png)

对于这样的结构，建立一个包含重投影误差和 IMU 误差项的统一损失函数进行联合优化：
$$
J(x)=\underbrace{\sum_{i=1}^{l} \sum_{k=1}^{K} \sum_{j \in J(i, k)} e_{r}^{i, j, k T} W_{r}^{i, j, k} e_{r}^{i, j, k}}_{\text {visual }}+\underbrace{\sum_{k=1}^{K-1} e_{s}^{k T} W_{s}^{k} e_{s}^{k}}_{\text {inertial }}
$$
其中， $i, j, k$ 分别表示相机、特征点和关键帧的索引， $W_{r}^{i, j, k}$ 表示特征的信息矩阵， $W_{s}^{k}$ 表示IMU 误差的信息矩阵，而 $e_{r}^{i, j, k}$ 为视觉重投影误差， $e_{s}^{k}$ 为 $\mathrm{IMU}$ 误差项。

### 6、相机模型

相机将三维世界的坐标点投影到二维平面的过程可以用一个几何模型表示，其中最简单的模型是针孔模型，即物理中的小孔成像原理。

![image-20230814155112837](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814155112837.png)

$O-x-y-z$ 为**相机坐标系**， $O-x^{\prime}-y^{\prime}-z$ 为**物理成像平面**。空间点 $P$ 通过小孔 $O$ 投影到物理成像平面 ，成像点为 $P^{\prime}$ 。$P$ 的坐标为 $[X, Y, Z], P^{\prime}$ 的坐标为 $\left[X^{\prime}, Y^{\prime}, Z^{\prime}\right]$，物理成像平面到小孔的距离 (即焦距) 为 $f$ 。根据三角形的相似关系，存在：
$$
\frac{Z}{f}=-\frac{X}{X^{\prime}}=-\frac{Y}{Y^{\prime}}
$$
其中负号表示成像是倒立的。为简化模型，可以将成像平面对称到相机前面 ，即将空间点 $P$ 一起放在相机坐标的同一侧。

![image-20230814155425601](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814155425601.png)

则有：
$$
\frac{Z}{f}=\frac{X}{X^{\prime}}=\frac{Y}{Y^{\prime}} \Longrightarrow \begin{aligned} X^{\prime} & =f \frac{X}{Z} \\ Y^{\prime} & =f \frac{Y}{Z}\end{aligned}
$$
在相机中获得的像素，需要对物理成像平面上对成像进行采样和量化。因此，假设在物理成像平面上存在一 个像素平面 $O-u-v$ ，设空间点 $P$ 在像素平面的投影为 $P^{\prime}$ ，其坐标为 $[u, v]^{T}$ 。像素坐标系与成像平面之间，相差一个缩放和原点的平移。
$$
P^{\prime} \text { 的坐标 }\left\{\begin{array}{l}
u=\alpha X^{\prime}+c_{x} \\
v=\beta Y^{\prime}+c_{y}
\end{array}\right.
$$
其中， $\alpha$ 和 $\beta$ 是缩放系数， $\left[c_{x}, c_{y}\right]$ 是原点的平移。
$$
\begin{array}{ll}
X^{\prime}=f \frac{X}{Z} & f_{x}=\alpha f \\
Y^{\prime}=f \frac{Y}{Z} & f_{y}=\beta f
\end{array} \Rightarrow\left\{\begin{array}{l}
u=f_{x} \frac{X}{Z}+c_{x} \\
v=f_{y} \frac{Y}{Z}+c_{y}
\end{array}\right.
$$

$$
\left\{\begin{array}{l}u=f_{x} \frac{X}{Z}+c_{x} \\ v=f_{y} \frac{Y}{Z}+c_{y}\end{array} \stackrel{\text { 矩阵形式 }}{\rightleftarrows}\left(\begin{array}{l}u \\ v \\ 1\end{array}\right)=\frac{1}{Z}\left(\begin{array}{ccc}f_{x} & 0 & c_{x} \\ 0 & f_{y} & c_{y} \\ 0 & 0 & 1\end{array}\right)\left(\begin{array}{c}X \\ Y \\ Z\end{array}\right) \triangleq \frac{1}{Z} \boldsymbol{K} \boldsymbol{P}\right.
$$

将 $Z$ 移到左边：
$$
Z\left(\begin{array}{l}
u \\
v \\
1
\end{array}\right)=\left(\begin{array}{ccc}
f_{x} & 0 & c_{x} \\
0 & f_{y} & c_{y} \\
0 & 0 & 1
\end{array}\right)\left(\begin{array}{l}
X \\
Y \\
Z
\end{array}\right) \triangleq \boldsymbol{K}  \Rightarrow   \boldsymbol{P}_{u v}=Z\left[\begin{array}{l}
u \\
v \\
1
\end{array}\right]=\boldsymbol{K}\left(\boldsymbol{R} \boldsymbol{P}_{w}+\boldsymbol{t}\right)=\boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{w}
$$
中间矩阵称为**相机的内参数矩阵** (Camera Intrinsics) $\boldsymbol{K}$ ，相机的位姿 $R, t$ 又称为**相机的外参数** (Camera Extrinsics)。相比于不变的内参，**外参会随着相机运动发生改变**，同时也是 **SLAM 中待估计的目标**，代表相机 的运动轨迹。
$$
Z \boldsymbol{P}_{u v}=Z\left[\begin{array}{c}
u \\
v \\
1
\end{array}\right]=\boldsymbol{K}\left(\boldsymbol{R} \boldsymbol{P}_{w}+\boldsymbol{t}\right)=\boldsymbol{K} \boldsymbol{T} \boldsymbol{P}_{w}
$$
上式表明：可以把一个世界坐标点先转换到相机坐标系，再去掉它最后一维的数值 (即该点距离相机成像平面的深度）。相当于归一化处理，得到点 $P$ 在相机归一化平面上的投影：
$$
\left(R P_{w}+t\right)=\underbrace{[X, Y, Z]}_{\text {相机坐标 }} \rightarrow \frac{[X / Z, Y / Z, 1]}{[\text { 归一化坐标 }}
$$
归一化坐标可以看成相机前方 $z=1$ 处的平面上的一点， $z=1$ 平面又称为归一化平面。归一化坐标左乘相机内参 $K$ ，就得到像素坐标。因此，也可以把像素坐标 $[u, v]^{T}$ 看成对归一化平面上点进行量化测量的结果。

### 7、多视图几何

多视图几何是计算机视觉领域一种应用广泛的建模方法，它可以通过多帧图像之间特征的匹配关系建立拍摄这些图像时的相机位姿之间的联系，目的是进行运动估计，根据特征点匹配情况，求解出旋转矩阵 $\mathrm{R}$ 和平移向量 $t$。多视图几何方法从几何的角度构造特征坐标的等量约束关系，而相机位姿信息则蕴含在这些约束关系之中。多视图几何包括对极几何、单应性与三焦张量：

* **对极几何**：描述同一个空间点在两个相机视图中的坐标关系，连接此点与一个相机光心所构成的线段将在另一个相机的视图中形成一条线，称作极线。此点在第二个相机视图中所成的像必然位于此极线上：

  ![image-20230815112907230](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815112907230.png)

* **单应性**：同样描述两个相机视图之间的映射关系，考虑处于空间中一个平面上的特征点，它一个相机中所成的像必然可以通过一个投影变换转化为其在另一个相机图像中的像，而此投影变换正是由其所在的平面来决定：

  ![image-20230815113017596](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815113017596.png)

* **三焦张量**：描述同一个空间点在三个相机视图中的坐标关系，可以将这三个视图分成两组，分别按照对极几何或单应性进行几何约束关系的构造，最终即可整合成一个整体的约束关系：

  ![image-20230815113124376](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815113124376.png)

针对特征点匹配的情况，运动估计可以分为：2D-2D、3D-2D、3D-3D。

#### 1. 2D-2D

![image-20230814211100817](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814211100817.png)

2D-2D 主要是针对单目相机的初始化过程，在不知道空间中 3D 点的情况下（如末进行初始化）通过两帧间匹配的特征点进行帧间相机运动估计。当相机为**单目相机**，只知道 2D 的像素坐标，如何**根据两组 2D 点估计相机运动**，该问题用**对极几何**解决。

![image-20230814204932980](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814204932980.png)

如图， $I_{1}$ 是第一帧， $I_{2}$ 是相机运动后的第二帧， $O_{1}$ 和 $O_{2}$ 是运动前后的相机中心。假设第一帧到到第二帧的运动 **(旋转和平移) 为 $[R, t]$** 。

- $I_{1}$ 中有一个**特征点 $p_{1}$** ，它在 $I_{2}$ 中对应的是**特征点 $p_{2}$** ，这 两个点是同一个**空间点 $P$ 在两个成像平面上的投影**。

- 连线 $\overrightarrow{O_{1} p_{1}}$ 和连线 $\overrightarrow{O_{1} p_{2}}$ 在三维空间中相交于点 $P$ ，这时 $O_{1}, O_{2} ， P$ 三个点可以确定一个**平面 $O_{1} O_{2} P$ ，称为极平面 (Epipolar plane)**。
- $O_{1} O_{2}$ 连线和像平面 $I_{1} ， I_{2}$ 的交点分别为 $e_{1}, e_{2} 。 O_{1} O_{2}$ 被称为**基线**， $e_{1}$ 和 $e_{2}$ 称为**极点**(Epipoles)。
- 极平面 $O_{1} O_{2} P$ 与两个像平面 $I_{1} ， I_{2}$ 之间的相交线 $l_{1} ， l_{2}$ 为**极线** (Epipolar line)。

几何方法：主要是根据对极几何理论得到两帧间的对应关系根据针孔相机模型，在齐次坐标情况下：
$$
\boldsymbol{p}_{1}=\boldsymbol{K} \boldsymbol{P}, \quad \boldsymbol{p}_{2}=\boldsymbol{K}(\boldsymbol{R P}+\boldsymbol{t})
$$
现在，取：
$$
\boldsymbol{x}_{1}=\boldsymbol{K}^{-1} \boldsymbol{p}_{1}, \quad \boldsymbol{x}_{2}=\boldsymbol{K}^{-1} \boldsymbol{p}_{2} .
$$
其中， $x_{1}, x_{2}$ 是两个像素点的归一化平面上的坐标。 代入上式子，可得：
$$
x_{2}=\boldsymbol{R} x_{1}+\boldsymbol{t} .
$$
两边同时左乘 $t$ 的反对称矩阵 $t^{\wedge}$ ，接着同时左乘 $x_{2}^{T}$ ，可得：
$$
\boldsymbol{x}_{2}^{T} \boldsymbol{t}^{\wedge} \boldsymbol{x}_{2}=\boldsymbol{x}_{2}^{T} \boldsymbol{t}^{\wedge} \boldsymbol{R} \boldsymbol{x}_{1}
$$
$t^{\wedge} x_{2}$ 是一个与 $t$ 和 $x_{2}$ 都垂直得到向量，则 $x_{2}^{T} t^{\wedge} R x_{1}=0$ 可得：
$$
\boldsymbol{p}_{2}^{T} \boldsymbol{K}^{-T} \boldsymbol{t}^{\wedge} \boldsymbol{R} \boldsymbol{K}^{-1} \boldsymbol{p}_{1}=0
$$
上述两个式子称为**对极约束**，它的几何意义是 $O_{1}, O_{2}, P$ 三者共面。对极约束中同时包括**平移**和**旋转**。将中间部分记作两个矩阵，**基础矩阵**(Fundamental Matrix) $\boldsymbol{F}$ 和 **本质矩阵**(Essential Matrix) $\boldsymbol{E}$。进一步简化对极约束：
$$
\begin{array}{c}
\boldsymbol{E}=\boldsymbol{t}^{\wedge} \boldsymbol{R} \\
\boldsymbol{F}=\boldsymbol{K}^{-T} \boldsymbol{E} \boldsymbol{K}^{-1} \\
\boldsymbol{x}_{2}^{T} \boldsymbol{E} \boldsymbol{x}_{1}=\boldsymbol{p}_{2}^{T} \boldsymbol{F} \boldsymbol{p}_{1}=0
\end{array}
$$
相机位姿估计问题变为如下两步：

1. 根据配对点的像素位置，求出 $E$ 或 $F$
2. 根据 $E$ 或 $F$，求出 $R$ 和 $t$

由于 $E$ 和 $F$ 只差相机内参 $K$，而内参由相机提供，通常已知。所以**实际情况中，采用形式更简单的本质矩阵 $E$**。

本质矩阵是一个 $3 \times 3$ 的矩阵，内有 9 个末知数。由于旋转和平移各有 3 个自由度，故 $t^{\wedge} \boldsymbol{R}$ 共有 6 个自由度，但由于尺度等价性， $E$ 实际上有 5 个自由度。估算 $E$ 通常使用 8 对点，也称为八点法(Eight-Point-Algorithm)。八点法只利用 $E$ 的线性性质，因此可以在线性代数框架下求解。

根据估算得到本质矩阵 $E$ ，恢复出相机的运动 $R$ 和 $t$ 。 这个过程采用奇异值分解，假设 $E$ 的SVD 为：
$$
\boldsymbol{E}=\boldsymbol{U} \boldsymbol{\Sigma} \boldsymbol{V}^{T}
$$
其中 $U$ 和 $V$ 是正交阵， $\boldsymbol{\Sigma}$ 是奇异值矩阵。可以求得：
$$
\begin{array}{l}
\boldsymbol{t}_{1}^{\hat{1}}=\boldsymbol{U} \boldsymbol{R}_{Z}\left(\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{T}, \quad \boldsymbol{R}_{1}=\boldsymbol{U} \boldsymbol{R}_{Z}^{T}\left(\frac{\pi}{2}\right) \boldsymbol{V}^{T} \\
\boldsymbol{t}_{\hat{2}}^{\hat{}}=\boldsymbol{U} \boldsymbol{R}_{Z}\left(-\frac{\pi}{2}\right) \boldsymbol{\Sigma} \boldsymbol{U}^{T}, \quad \boldsymbol{R}_{2}=\boldsymbol{U} \boldsymbol{R}_{Z}^{T}\left(-\frac{\pi}{2}\right) \boldsymbol{V}^{T}
\end{array}
$$
从 $E$ 分解到 $R$ 和 $t$，存在 $4$ 个可能的解：

![image-20230814210707814](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814210707814.png)

$P$ 在两个相机中都有正的深度是正确的解。实际中 ，将一个点代入 4 种解中，检测该点在两个相机下的深度，即可确定哪个是正确的解。

> 【总结】2D-2D
>
> * 八点法求本质矩阵 $E$；
> * 本质矩阵奇异值分解得到 4 个解；
> * 把一个点带入 4 个解中，看哪个解深度为正值，即为正确的解。

#### 2. 3D-2D

![image-20230814211202142](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814211202142.png)

如果一组为 3D，一组为 2D，即知道了一些 3D 点和它们在相机的投影位置，也能估计相机的运动。该问题用 **PnP**(Perspective-n-Point) 解决。$\mathrm{PnP}$ 是求解 3D 到 2D 点对运动的方法。它描述了当知道 $n$ 个 $3 \mathrm{D}$ 空间点及其投影位置时，如何估计相机的位姿。

* 在双目或 RGB-D 的视觉里程计中，可以直接使用 PnP 估计相机运动;
* 在单目视觉里程计中，必须先进行初始化，才能使用 PnP。

PnP 有如下解法：

* **P3P**：3 对匹配点，需要相机内参。
* **DLT**：不需要相机内参，4 点法求单应矩阵，DLT 分解出 K、R、t。
* **EPnP**：最少4个点，性价比高，精度较高，需要相机内参。
* **UPnP**：估计出焦距，适合末标定场景。
* **BA**：构建最小二乘法优化相机位姿。

其中 P3P 需要利用给定的 3 个点的几何关系，它的输入数据为 3 对 2D-2D 的匹配点。

![image-20230814212320673](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814212320673.png)

其中：3D点 (世界坐标系) : $A, B, C$ 。2D点 (相机坐标系)： $a, b, c$ 分别对应 $A, B, C$ 在相机成像平面上的投影。

$P n P$ 问题转换为 ICP问题:

- 通过余弦定理，可以得到 $O A, O B, O C$ 的长度；
- 3D点在相机坐标下的坐标能够计算出；
- 3D (相机坐标系) - 3D (相机坐标系) 的对应点；
- PnP 问题转换为 ICP问题。

除了线性方法，还可以将 **PnP 问题**构建为关于**重投影误差的非线性最小二乘法问题**。线性方法往往先求**相机位姿**，再求**空间点位置**，而非线性优化则是把它们都作为优化变量一起优化。 这一类**把相机和三维点放在一起进行最小化**的问题，称为**光束法平差** (Bundle Adjustment，**BA**)

![image-20230814212522583](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230814212522583.png)

考虑到相机位姿末知及观测点的噪声，将误差和构建为最小二乘法问题，**使误差最小化**，**寻找最优的相机位姿**：
$$
\boldsymbol{\xi}^{*}=\arg \min _{\boldsymbol{\xi}} \frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{u}_{i}-\frac{1}{s_{i}} \boldsymbol{K} \exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{P}_{i}\right\|_{2}^{2}
$$
该问题的误差项是将 $3 D$ 点的投影位置与观测位置作差，因此，称为**重投影误差**。

#### 3. 3D-3D

3D-2D 当相机为双目、RGB-D 时，或者通过某种方式得到了距离信息，**根据两组 3D 点估计相机运动**。该问题用 ICP(Iterative Closest Point) 解决。

假设有一组配对好的 3D 点：
$$
\boldsymbol{P}=\left\{\boldsymbol{p}_{1}, \ldots, \boldsymbol{p}_{n}\right\}, \quad \boldsymbol{P}^{\prime}=\left\{\boldsymbol{p}_{1}^{\prime}, \ldots, \boldsymbol{p}_{n}^{\prime}\right\}
$$
现在希望找到一个欧式变换 $R, t$ ，使得：
$$
\forall i, \boldsymbol{p}_{i}=\boldsymbol{R} \boldsymbol{p}_{i}^{\prime}+\boldsymbol{t}
$$
该问题使用迭代最近点 ICP 求解。与 PnP 类似，ICP 的求解也分为两种形式：线性优化 SVD、非线性优化：BA。

先来看 SVD 法，分为三个步骤求解：

1. 计算两组点的质心位置 $\boldsymbol{p}, \boldsymbol{p}^{\prime}$, 然后计算每:个点的去质心坐标：
  $$
  \boldsymbol{q}{i}=\boldsymbol{p}{i}-\boldsymbol{p}, \quad \boldsymbol{q}{i}^{\prime}=\boldsymbol{p}{i} ({\prime}-\boldsymbol{p}){\prime}
  $$

2. 根据以下优化问题计算旋转矩阵：
  $$
  \boldsymbol{R}^{*}=\arg \min {\boldsymbol{R}} \frac{1}{2} \sum{i=1}^{n}\left|\boldsymbol{q}{i}-\boldsymbol{R} \boldsymbol{q}{i} ({\prime}\right\|){2}
  $$

3. 根据第二步的 $\boldsymbol{R}$, 计算 $\boldsymbol{t}$：
  $$
  t^{*}=p-R p^{\prime}
  $$

展开关于 $R$ 的误差项：
$$
\frac{1}{2} \sum_{i=1}^{n}\left\|\boldsymbol{q}_{i}-\boldsymbol{R} \boldsymbol{q}_{i}^{\prime}\right\|^{2}=\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{q}_{i}^{T} \boldsymbol{q}_{i}+\boldsymbol{q}_{i}^{\prime T} \boldsymbol{R}^{T} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}-2 \boldsymbol{q}_{i}^{T} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}
$$
第一项与 $R$ 无关，第二项 $R^{T} R=I$ ，也与 $R$ 无关。 则实际的优化目标函数为：
$$
\sum_{i=1}^{n}-\boldsymbol{q}_{i}^{T} \boldsymbol{R} \boldsymbol{q}_{i}^{\prime}=\sum_{i=1}^{n}-\operatorname{tr}\left(\boldsymbol{R} \boldsymbol{q}_{i}^{\prime} \boldsymbol{q}_{i}^{T}\right)=-\operatorname{tr}\left(R \sum_{i=1}^{n} \boldsymbol{q}_{i}^{\prime} \boldsymbol{q}_{i}^{T}\right)
$$
除此之外，求解 ICP 的非线性方法，用迭代的方式寻找最优值。利用李代数优化相机位姿时，目标函数为:
$$
\min {\boldsymbol{\xi}}=\frac{1}{2} \sum{i=1}^{n}\left|\left(\boldsymbol{p}{i}-\exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}{i} ({\prime}\right)\right\|_{2}){2}
$$
使用李代数扰动模型，可得：
$$
\frac{\partial \boldsymbol{e}}{\partial \delta \boldsymbol{\xi}}=-\left(\exp \left(\boldsymbol{\xi}^{\wedge}\right) \boldsymbol{p}_{i}{ } ({\prime}\right)){\odot}
$$
通过不断迭代，最小化误差，就能找到极小值。一个像素的深度可能有，也可能测量不到，实际情况中常常混合着使用 PnP 和 ICP 优化。

- 深度已知的特征点，建模 3D-3D 误差；
- 深度末知的特征点，建模 3D-2D 的重投影误差。



## 三、编译使用

























