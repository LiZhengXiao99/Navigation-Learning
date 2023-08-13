[TOC]

## 一、程序简介

### 1、ORB-SLAM 概述



### 2、资源获取



### 3、代码分析



文件结构如下：

* Examples 和Exampleold 根据传感器类型，分别存放新的和旧的代码实例。
* include 和 src 分别存放代码的 .h 头文件和 cc/cpp 原文件。
* Thirdparty 存放了 DBOW2、Sophus 和 g2o。DBOW2 是词袋模型，Sophus 是李代数库，g2o 是图优化库。
* Vocabulary 存放ORB词典。

### 5、第三方库







## 二、基础知识点

### 1、视觉SLAM简介

#### 1. 定位与建图关系

![1690934934097](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690935198158.png)

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

* **传感器数据读取**
* **视觉里程计**（Visual Odometry，VO）：估算相邻图像间相机的运动，以及局部地图。也称前端（Front END）。
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
* 现在提出一个高效的测试，来快速排除一大部分非特征点的点。 该测试仅仅检查在位置 1、9、5 和 13 四个位置的像素。如果是一个角点，那么上述四个像素点中至少有 3 个应该和点相同。如果都 不满足，那么不可能是一个角点。

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



![1691021656755](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1691021656755.png)











### 7、多视图几何











## 三、ORB-SLAM3 历史与演变

![1690952265203](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690952265203.png)

### 1、PTAM（Parallel Tracking and Mapping）

基于关键帧，首次将跟踪（Tracking）和建图（Mapping）分成两个独立线程：

- **跟踪线程**负责跟踪相机位姿，同时绘制虚拟的模型。
- **建图线程**负责建立场景的模型和绘制场景的地图。

### 2、ORB-SLAM

在 PTAM 基础上，提出基于特征点的单目 SLAM 算法

- **跟踪线程**：提取 ORB 特征，根据上一帧进行初始位姿估计，或者通过全局重定位初始化相机位姿，然后跟踪已重建的局部地图来优化位姿，最后根据一些规则输出关键帧。
- **建图线程**：包括插入关键帧、验证最近生成的地图点并进行筛选，同时生成新的地图点，使用局部 BA，最后对插入关键帧进行筛选，去除多余的关键帧。
- **回环检测线程**：分为回环检测和回环校正。首先通过 BOW 加速闭环匹配帧的选择，然后通过 Sim3 计算相似变换，最后通过回环融合和本质图优化，实现闭环检测功能。

![1690940910132](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690940910132.png)

### 3、ORB-SLAM-VI

加入 IMU，针对单目 SLAM 缺少尺度信息，提出了新的 IMU 初始化方法，以高精度地图快速计算尺度、重力方向、速度以及陀螺仪和加速度计偏差，并重用地图，在已建图区域实现零漂移定位。

![1690941287964](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690941287964.png)

### 4、ORB-SLAM2

- 是第一个适用于单目、双目、RGB-D相机，还包括闭环控制、重定位、地图重用的开源 SLAM 系统。
- 在 RGB-D 模式下使用 BA 算法，准确率高于基于迭代最近点和光度深度误差最小化算法。
- 利用远近双目点和单目量测，使得在双目相机的情况下，算法的准确率高于以前最好的直接法双目视觉 SLAM。
- 设置了一个轻量级定位模式，能暂停地图构建线程，同时能有效地重复利用已建的地图，

### 5、ORB-SLAM-Atlas

实现多地图合并，包含一个鲁棒的地图合并算法，能处理无限数量非连续的子地图系统。

### 6、ORB-SLAM3

支持视觉、视觉+惯导、混合地图的 SLAM 系统，可以在单目、双目、RGB-D 相机上利用针孔和鱼眼模型运行。









