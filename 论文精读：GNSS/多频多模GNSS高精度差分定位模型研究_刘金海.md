> 以往看的很多GNSS算法都是针对单GPS系统的，通过这篇博士论文学习一下多频多系统的处理，学习**不同系统观测值的定位偏差**、**不同频率观测值的定位偏差**、**差分系统间偏差**和**差分频率间偏差**的处理策略

论文分为7章，各章节内容为：

* 第 1 章：**绪论**。本章主要介绍了多频多模 GNSS 高精度差分定位模型的研究 背景及意义，然后在分析本课题国内外研究现状的基础上，指出现存的一些问题， 最后引出本论文的主要内容。
* 第 2 章：**实时动态差分技术**。本章将详细阐述 GNSS 经典 RTK 模型，包括 观测模型和随机模型；其次，在此基础上给出了系统间差分和频率间差分的 RTK 模型，并详细叙述了差分系统间偏差（Differential Inter-System Bias，DISB）和差 分频率间偏差（Differential Inter-Frequency Bias，DIFB）的处理策略；最后针对 误差处理和参数估计进行阐述。 
* 第 3 章：**一种适用于不同长度基线的 RTK 定位模型**。本章提出了一种适用 于不同长度基线的 RTK 定位模型，并通过实测数据进行论证与分析。
* 第 4 章：**顾及不同系统和不同频率观测值定位偏差的 RTK 模型**。本章主要 对相对定位中不同系统观测值的定位偏差和不同频率观测值的定位偏差的性质 展开分析，并提出了直接作差法和参数估计法，然后通过实测数据进行验证分析。
* 第 5 章：**顾及差分系统间偏差和差分频率间偏差的 RTK 模型**。本章主要对 相对定位中差分系统间偏差和差分频率间偏差的性质展开分析，并提出了模型改 正法和参数估计法，然后通过实测数据进行验证分析。
* 第 6 章：**BDS-3 新信号的双差网络 RTK 与非差网络 RTK 性能评估**。本章首先从传统双差网络 RTK 技术、非差网络 RTK 技术、传统双差网络 RTK 与非差 网络 RTK 的比较来详细进行阐述，然后根据区域参考站网接收到的 BDS-3 新信 号的伪距和相位观测值，初步评估分析了 BDS-3 新信号的双差网络 RTK 和非差 网络 RTK 定位性能。
* 第 7 章：**总结与展望**。概括本文的主要工作成果，总结主要创新点，探讨进 一步的研究方向。

## 摘要

### 1、发展了一种适用于不同长度基线的 RTK 定位模型LWLC 

#### 1.方法

* 先利用**伪距**观测值和**相位宽巷组合**观测值进行**双差宽巷模糊度**的计算和固定。
* 将双差宽巷整周模糊度固定的相位宽巷组合观测值作为测距精度较高的伪距观测值。
* 用该**相位宽巷组合观测值**（视为伪距观测值）和**相位无电离层组合观测值**进行双差载波相位整周模糊度的固定，进而实现 RTK 定位。

#### 2.结果

* 短基线： LWLC方法和**单频**伪距和载波相位组合 P1L1方法定位精度相当，而伪距和载波相位**无电离层**组合 PCLC 定位结果相对较差。
* 中、长基线：LWLC方法的定位精度**略优于** PCLC 方法，但**显著优于** P1L1 方法。
* 说明 LWLC 方法在不同长度基线条件下都可以提供高精度的 RTK 定位结果。

### 2、建立了不同系统观测值定位偏差、不同频率观测值定位偏差的处理模型

处理方法分为**直接作差**和**参数估计**

#### 1.不同系统相对定位偏差特性

* **不同基线**所求得的不同系统观测值定位偏差存在**差异。**
* **相同基线不同天**所求得的不同系统观测值定位偏差表现出**一致性和周期性**。
* 不同系统观测值定位偏差的**直接作差法**和**参数估计模型**都可以得到**与单系统一致**的定位结果。
* 直接作差法和参数估计模型得到的不同系统观测值定位偏差在**连续多天相同时间段内保持一致**且相对稳定。

#### 2.不同频率相对定位偏差特性
* 不同频率观测值的定位结果存在 **mm-cm** 的差异。
* GPS L1 和 L2 观测值的定位偏差比 BDS B1 和 B2 观 测值的定位偏差小，并且更加稳定。
* 不同频率观测值定位偏差的直接作差法和参数估计模型都可以得到与单一频率一致的定位结果。
* 直接作差法和参数估计模型得到的不同频率观测值的定位偏差在**连续多天相同时间段内保持一致**且相对稳定。

### 3、提出了差分系统间偏差DISB和差分频率间偏差DIFB的处理模型

#### 1.差分系统间偏差DISB处理模型

* GPS 与 BDS 系统间差分模型中，不同频率（L1-B1、L2-B2）上伪距和相位 DISB 不同。
* 在一个连续的观测时间段，伪距和相位 DISB 均保持稳定。
* 伪距 DISB 序列相对平均值的波动在 ±1m 以内，STD 小于 0.45m。
* 相位 DISB 序列相对平均值的波动在 ±0.05 周以内，STD 小于 0.01 周。
* 在接收机类型不同的基线中，伪距 DISB 数值可达数米量级。
* DISB 改正模型和实时估计模型可以提高定位结果的 一致性，与经典系统内差分模型的定位结果相比，定位精度可以提高约 30％。

#### 2.差分频率间偏差DIFB处理模型

* GPS 和 BDS 频率间差分模型中，GPS L1-L2 和 BDS B1-B2 对应的伪距和相位的 DIFB 不同。
* 在一个连续的观测时间段，伪距和相位 DIFB 相对稳定。
* 伪距 DIFB 序列相对平均值的波动在 ±2 m 以内，STD 小于 0.5m。
* 相位 DIFB 序列相对平均值的波动在 ±0.1 周以内，STD 小于 0.015 周。
* DIFB 改正模型和实时估计模型均可以提高定位结果的一致性，与经典频率内差分模型的定位结果相比，定位精度可以提高约 30％。

### 4、分析验证了 BDS-3 新信号的双差网络 RTK 和非差网络 RTK 定位性能

* 基于区域参考站网接收到的 BDS-3 新信号的伪距和相位观测值，可以获得厘米级精度的双差大气延迟改正数。
* 在双差网络 RTK（Network Real-Time  Kinematic，NRTK）模式下，使用参考站网得到的双差电离层延迟和对流层延迟改正数，对流动站附近的虚拟参考站的双差大气延迟改正数，从而生成虚拟参考 站的观测值，并与流动站构成超短基线 RTK 定位可以实现流动站厘米级的定位。
* 在非差网络 RTK（Undifferenced Network RTK，URTK）模式下，利用用户站的 近似坐标插值出来的非差改正数，对用户站非差观测值进行改正，然后采用 PPP 模式进行瞬时模糊度固定，亦可以实现厘米级的定位。


## 一、绪论

### 1、选题背景及意义

* 随着多模 GNSS 的建设推进和不断 完善，各系统都将采用三个或三个以上的频率，卫星导航已经迈进了多系统多频率并存与互操作时代。
* PPP和RTK是高精度GNSS定位中的核心技术。
  * PPP需要高精度定位产品支持，且需要较长时间收敛，应用受到限制。
  * RTK在 GNSS 参考站的基础上，通过差分技术，能够为流动站用户提供实时厘米级定位服务，得到了广泛应用。
* RTK 的有效作用范围一般在 20 公里以内，随着距离的增加，空间相关性减弱，简单通过双差组合的方法是很难准确固定载波相位整周模糊度，因而定位性能变差。
* 对于短基线来说，通常忽略双差大气延迟残差的影响，直接采用非组合双差观测方程进行定位解算。
* 对于中长基线来说，通常将天顶方向的**对流层**延迟模型化，然后投影到信号传播方向，而**电离层**延迟通常采用双频无电离层组合观测值消除其一阶项的影响，或者对双差电离层延迟残差进行先验约束或参数估计。
* 多频多模观测值融合是 GNSS 高精度相对定位技术的发展趋势：
  * 几何倍数增长的可视卫星数，大幅改善了卫星空间几何分布，有效提高了遮挡环境下的定位可用性和可靠性。
  * 更多的观测值信息，大大增加了冗余观测量，有利于强化参数估计模型，实现快速准确固定模糊度，从而提升定位的精度。
  * 但不同系统观测值的定位偏差、不同频率观测值的定位偏差、差分系统间偏差以及差分频率间偏差需要进行处理。

### 2、国内外研究现状

#### 1.差分定位模型方面

分为系统内双差模型（也称为松组合模型）和系统间双差模型（也称为紧组合模型）。

* **系统内差分模型**：每个系统内选择一颗参考卫星，在同一卫星系统同类观测值之间进行差分，组成站间单差、星间单差或站星双差观测值。
* **系统间差分模型**：仅选择某个系统的某颗卫星为参考卫星，在不同卫星系统同类观测值之间进行差分，组成站间单差、星间单差或站星双差观测值。











## 二、实时动态差分技术

### 1、经典RTK模型

#### 1.非差观测方程
$$
\begin{array}{l}
\quad P_{r, f_{i}^{S Y S}}^{s}=\rho_{r}^{s}-c\left(\delta t_{r}-\delta t^{s}-\delta \tau^{S Y S}\right)+{\color{green}d_{r}^{s}}-\\{\color{green}d^{s}}+I_{r, f_{i}^{S Y S}}^{s}+T_{r}^{s}+O_{r}^{s}+R_{r}^{s}+ 
S_{r}^{s}+M_{r}^{S}+\varepsilon_{r, f_{i}^{S Y S}}^{s}
\end{array}
$$
$$
\begin{array}{r}
\lambda_{f_{i}^{S Y S}}^{s} \phi_{r, f_{i}^{S Y S}}^{s}=\rho_{r}^{s}-c\left(\delta t_{r}-\delta t^{s}-{\color{pink}\delta \tau^{S Y S}}\right)+\lambda_{f_{i}^{S Y S}}^{s}\left(\color{red}\left(\varphi_{r, f_{i}^{S Y S}}-\varphi_{f_{i}^{S Y S}}^{s}\right)+\right. \\
\left.{\color{red}\left(\delta_{r, f_{i}^{S Y S}}-{\delta_{{f i}^{S Y S}}^S}\right)}+{\color{blue}N_{r, f_{i}^{S Y S}}^{s}}\right)-I_{r, f_{i}^{S Y S}}^{s}+T_{r}^{S}+O_{r}^{s}+R_{r}^{s}+S_{r}^{s}+\boldsymbol{m}_{r}^{s}+\xi_{r, f_{i}^{S Y S}}^{s}
\end{array}
$$

其中：

* $P$ 表示以米为单位的**伪距**观测值
* $\phi$ 表示以周为单位的**相位**观测值
* $r$ 和 $s$ 分别表示**接收机**和**卫星**
* $i$ 表示不同**频带**，以GPS为例，$i$ = L1，L2，L5
* $SYS$ 表示卫星**导航系统**，可以表示 GPS、BDS、GLONASS、Galileo 等
* $\rho_{r}^{s}$ 表示接收机 $r$ 与卫星 $s$ 之间的**几何距离**
* $c$ 表示光在真空中传播的速度
* $\delta t_{r}$ 表示**接收机钟差**
* $\delta t_{s}$ 表示**卫星钟差**
* ${\color{pink}\delta \tau^{S Y S}}$ 表示接收机参考时间与卫星系统时之间的差异
* $\lambda$ 表示**波长**
* ${\color{blue}N}$ 表示**整周模糊度**
* $f$ 表示卫星观测值**频率**
* $I$ 表示**电离层延迟**
* $T$ 表示**对流层延迟**
* ${\color{green} d_{r}^{s}}$ 、${\color{green}d^s}$  分别表示接收机和卫星端的**伪距硬件延迟**
* ${\color{red}\delta_r}$ 、${\color{red}\delta^s}$ 分别表示接收机端和卫星的**相位硬件延迟**
* ${\color{red}\varphi_{r}}$、${\color{red}\varphi_{s}}$  分别表示接收机端和卫星端的**初始相位**
* $O$ 表示卫星**轨道误差**
* $R$ 表示**相对论效应**
* $S$ 表示**地球自转效应**
* $M$、$m$ 分别表示伪距和相位观测值中的**多路径效应**
* $\varepsilon, \xi$ 分别表示伪距和相位观测值中的**测量噪声**

观测方程中的所有误差项都必须仔细处理，特别是对于载波相位高精度定位。 

* 某些项在相对定位中可以通过**观测值作差来消除或削弱**，双差观测模型中完全消除了接收机和卫星**钟差**，大大削弱了卫星**轨道误差**、**电离层延迟**误差和**对流层延迟**误差，不过在中长基线条件下，不能忽略电离层延迟和对流层延迟残差的影响。
* 某些项可以**用物理模型进行精确修正**，比如：**相对论效应**、**地球自转效应**等误差。虽然在观测方程中没有列出卫星和接收机**天线相位中心**偏差（PCO）及其变化（PCV）、**潮汐改正**、**相位缠绕**等误差项，但是这些误差项也可以通过相应的误差改正模型进行精确改正。
* 而某些项（位置参数、模糊度参数等）必须**作为参数进行估计**。 需要指出的是，对流层延迟误差的干分量可以通过模型进行改正，而湿分量与大气湿度和高度角相关，很难用模型对其改正，因此在中长基线条件下需要进行参数估计。

#### 2.站间单差观测方程

$$
\begin{array}{c}\Delta P_{b r, f_{i}^{S Y S}}^{s}=\Delta \rho_{b r}^{s}-c \delta t_{b r}+{\color{green}\Delta d_{b r}^{s}}+\Delta I_{b r, f_{i}^{S Y S}}^{s}+\Delta T_{b r}^{s}+\Delta \varepsilon_{b r, f_{i}^{S Y S}}^{s} \\ \lambda_{f_{i}^{S Y S}}^{s}\Delta \phi_{b r, f_{i}^{S Y S}}^{s}=\Delta \rho_{b r}^{s}-c \delta t_{b r}+\lambda_{f_{i}^{S Y S}}^{s}\left({\color{red}\Delta \varphi_{b r, f_{i}^{S Y S}}}+{\color{red}\Delta \delta_{b r, f_{i}^{S Y S}}}+{\color{blue}\Delta N_{b r, f_{i}^{S Y S}}^{s}}\right)- \\
\Delta I_{b r, f_{i}^{S Y S}}^{S}+\Delta T_{b r}^{s}+\Delta \xi_{b r, f_{i}^{S Y S}}^{S}\end{array}
$$
式中：$∆$ 表示站间单差算子，$b$ 和 $r$ 表示不同的接收机。

* **完全消除了与卫星有关的误差**（包括：卫星钟差、卫星端的初始相位与硬件延迟误差）和系统间时间偏差 $\delta \tau^{S Y S}$ 。
* 卫星端的伪距和相位硬件延迟以及初始相位偏差在连续观测时段内可以视为常数，因此站间单差数据处理过程中能够消除。
* 站间单差伪距硬件延迟 $\Delta d_{b r}^{s}$ 不仅包括接收机码偏差，如果两台接收机采用不同的码类型和跟踪模型的话，还包括卫星差分码偏差（DCB）。
* 对于相同频率的卫星 s，站间单差相位硬件延迟 $\Delta \delta_{b r}^{s}$ 只包括接收机间的差分延迟，因为消除了卫星端的硬件延迟。

#### 3.星间单差观测方程

$$
\begin{array}{l}\nabla P_{r, f_{i}}^{s k}=\nabla \rho_{r}^{s k}+c \delta \nabla t^{s k}-{\color{green}\nabla d^{s k}}+\nabla I_{r, f_{i}}^{s k}+\nabla T_{r}^{s k}+\nabla \varepsilon_{r, f_{i}^{S Y S}}^{s k} \\ \lambda_{f_{i}^{S Y S}}^{k} \phi_{r, f_{i}}^{k Y S}-\lambda_{f_{i}^{S Y S}}^{s} \phi_{r, f_{i}^{S Y S}}^{s}=\nabla \rho_{r}^{S k}+c \delta t^{S k}+\lambda_{f_{i}^{S Y S}}^{k}{\left({\color{red}\left(\varphi_{r, f_{i}^{S Y S}}-\varphi_{f_{i}^{S Y S}}^{k}\right)}+\right.} \\
\left.{\color{red}\left(\delta_{r, f_{i}^{S Y S}}^{S}-\delta_{f_{i}^{S Y S}}^{k}\right)}+{\color{blue}N_{r, f_{i}^{S Y S}}^{k}}\right)-\lambda_{f_{i}^{S Y S}}^{S}\left({\color{red}\left(\varphi_{r, f_{i}^{S Y S}}-\varphi_{f_{i}^{S Y S}}^{S}\right)}+{\color{red}\left(\delta_{r, f_{i}}^{S Y S}-\delta_{f_{i}^{S Y S}}^{S}\right)}+\right. \\
\left.{\color{blue}N_{r, f_{i}^{S Y S}}^{S}}\right)-\nabla I_{r, f_{i}^{S Y S}}^{s k}+\nabla T_{r}^{s k}+\nabla \xi_{r, f_{i}^{S Y S}}^{s k} \\\end{array}
$$
式中：$∇$ 表示星间单差算子，$s$ 和 $k$ 表示不同的卫星。

* **完全消除了与接收机有关的误差**（包括：接收机钟差、接收机端的初始相位与硬件延迟误差等）和系统间时间偏差 $\delta \tau^{S Y S}$ 。
* 如果这两颗卫星使用相同的频率，即：$\lambda_{f_{i}^{S Y S}}^{k} = \lambda_{f_{i}^{S Y S}}^{s}$ ，两个星间单差模糊度可以合并为一个站星双差模糊度。

#### 4.站星双差观测方程

$$
\nabla \Delta P_{b r, f_{i}}^{S Y S}=\nabla \Delta \rho_{b r}^{s k}+{\color{green}\nabla \Delta d_{b r, f_{i}^{S Y S}}^{s k}}+\nabla \Delta I_{b r, f_{i}}^{s k}+\nabla \Delta T_{b r}^{s k}+\nabla \Delta \varepsilon_{b r, f_{i}^{S Y S}}^{s k}
$$
$$
\begin{array}{c}
\lambda_{f_{i}^{S Y S}}^{k} \Delta \phi_{b r, f_{i}^{S Y S}}^{k}-\lambda_{f_{i}^{S Y S}}^{s} \Delta \phi_{b r, f_{i}^{S Y S}}^{s}=
\nabla \Delta \rho_{b r}^{s k}+\lambda_{f_{i}^{S Y S}}^{k}\left({\color{red}\Delta \varphi_{b r, f_{i}^{S Y S}}^{S Y}+\Delta \delta_{b r, f_{i}^{S Y S}}^{k}}+\right. 
\left.{\color{blue}\Delta N_{b r, f_{i}^{k S S}}^{k Y}}\right)-\\
\lambda_{f_{i}^{S Y S}}^{s}\left({\color{red}\Delta \varphi_{b r, f_{i}^{S Y S}}^{S Y}+\Delta \delta_{b r, f_{i}^{S Y S}}^{S}}+{\color{blue}\Delta N_{b r, f_{i}^{S Y S}}^{S}}\right)-\nabla \Delta I_{b r, f_{i}^{S Y S}}^{s k}+\nabla \Delta T_{b r}^{s k}+ 
\nabla \Delta \xi_{b r, f_{i}^{S Y S}}^{S k}
\end{array}
$$

式中：$\nabla \Delta$ 表示双差算子

* 消除了接收机钟差和卫星钟差，而且还消除了卫星的初始相位与伪距硬件延迟误差以及系统间时间偏差 $\delta \tau^{S Y S}$ 。
* 如果这两观测值频率相同，即 $\lambda_{f_{i}^{S Y S}}^{S}=\lambda_{f_{i}^{S Y S S}}^{k}$ ，站间单差模糊度 $\Delta N_{b r, f_{i}^{S Y S}}^{S }, \Delta N_{b r, f_{i}^{S Y S}}^{k}$ 可以合并为站星双差模糊度 $\nabla \Delta N_{b r, f_{i}}^{SYS}$ 。
* 对于采用**码分多址**（CDMA）技术的系统（GPS、Galileo、BDS）系统内双差模型，消除了卫星和接收机端的硬件延迟以及初始相位偏差，不需要考虑不同类型或品牌的接收机对上述因素的影响。
* 对于采用**频分多址**（FDMA）技术的 GLONASS 系统内双差模型，消除了卫星端的硬件延迟和初始相位偏差，但是至少在基线两端接收机类型不同的情况下，**不能消除接收机端的硬件延迟和初始相位偏差**。

#### 5.双频之间实数组合

在双频观测中，由 $f_1、f_2$ 通过如下线性组合得到新的观测值频率，记作：
$$
f_{(m, n)}=m \cdot f_{1}+n \cdot f_{2}
$$
新的观测值所对应的**波长**和**模糊度**分别表示为：
$$
\begin{aligned}
\lambda_{(m, n)} & =\frac{c}{f_{(m, n)}}=\frac{\lambda_{1} \cdot \lambda_{2}}{n \cdot \lambda_{1}+m \cdot \lambda_{2}} \\
N_{(m, n)} & =m \cdot N_{1}+n \cdot N_{2}
\end{aligned}
$$
新的伪距和相位线性组合观测值分别为：
$$
\begin{aligned}
P_{(m, n)} & =\frac{m f_{1}}{m f_{1}+n f_{2}} P_{1}+\frac{n f_{2}}{m f_{1}+n f_{2}} P_{2} \\
\Phi_{(m, n)} & =\frac{m f_{1}}{m f_{1}+n f_{2}} \Phi_{1}+\frac{n f_{2}}{m f_{1}+n f_{2}} \Phi_{2}
\end{aligned}
$$
假设不同频率的伪距观测值测量噪声的方差相等、不同频率的相位观测值测量噪声的方差也相等，根据**误差传播定律**，可以得到线性组合后的伪距和相位观测值测量噪声的方差，分别表示为：
$$
\begin{array}{l}
\sigma_{P_{(m, n)}}^{2}=\left(\left(\frac{m f_{1}}{m f_{1}+n f_{2}}\right)^{2}+\left(\frac{n f_{2}}{m f_{1}+n f_{2}}\right)^{2}\right) \sigma_{\mathrm{P}}^{2} \\
\sigma_{\Phi_{(m, n)}}^{2}=\left(\left(\frac{m f_{1}}{m f_{1}+n f_{2}}\right)^{2}+\left(\frac{n f_{2}}{m f_{1}+n f_{2}}\right)^{2}\right) \sigma_{\Phi}^{2}
\end{array}
$$
常用的组合有：无电离层组合、无几何信息组合、宽巷组合、窄巷组合、MW 组合


### 2、多频多系统差分 RTK 模型

#### 1.偏差

* **DCB**（Differential Code Bias）差分码偏差，不同的**测距码类型**和**跟踪模型**造成，不同类型的导航信号在卫星和接收终端不同 通道产生的时间延迟（也称为硬件延迟）并不完全 一致，由此带来的不同导航信号观测量之间的延迟也难以做到完全一致。最 大可达到 ±20 左右。
	* 任意两种码观测量之间均存在上述偏差，由卫星端和接收机端两部分组成。
	* **频内偏差**：表示同一频率不同类型信号观测量之间的相对偏差。在卫星精密钟差估计中采用 ＭＷ 组合估计获得或利用观测值直接计算。
	* **频间偏差**：表示不同频率信号观测量之间的相对偏差。通常包括两种估计方法，一是与电离层总电子含量参数同步建模估计；二是基于模型数据扣除电离层延迟后直接求解 DCB 参数
	* 由于存在模糊度参数，上述不一致性导致的差异表现为**初始相位偏差**，或称之为**未校正的小数偏差**。

* **ISB**（Inter-System Bias）系统间偏差，主要由**硬件延迟**组成，由 GNSS 设备中的不同信号路径产生，这取决于接收机内部的相关性
* **IFB**（Inter-Frequency Bias）频间偏差，
* **DISB**（Differential Inter-System Bias）差分系统间偏差
* **DIFB**（Differential Inter-Frquecny Bias）差分频率间偏差
* **DIFCB**（Differential Inter-Frquecny Code Bias）伪距差分频率间偏差
* **DIFPB**（Differential Inter-Frquecny Phase Bias）相位差分频率间偏差
* **DISPB**（Differential Inter-System Phase Bias）相位差分系统间偏差
* **PBDFO**（Positionig Bias of Different Frequency Observations）不同频率观测值的定位偏差，使用不同频率的观测值进行 RTK 定位的结果存在差异，这是一个综合偏差，不区分天线的相位中心变化、信号传输的不同延迟以及局部多径效应。
* **PBDSO**（Positionig Bias of Different System Observations）不同系统观测值的定位偏差
* **UPD**（Uncalibrated Phase Delay）未校准相位延迟

#### 2.影响

* 伪距 DISB 序列相对平均值的波动在 ±1m 以内，STD 小于 0.45m。
* 相位 DISB 序列相对平均值的波动在 ±0.05周 以内，STD 小于 0.01 周。
* 伪距 DIFB 序列相对平均值的波动在 ±2m 以内，STD 小于 0.5m。
* 相位 DIFB 序列相对平均值的波动在 ±0.1周 以内，STD 小于 0.015 周。

#### 3.相同频率的系统间双差模型

假设接收机 $b$ 和 $r$ 可以同时跟踪到 GNSS 系统 $\mathrm{A}$ 的卫星 $s_{A}=1_{A}, 2_{A}, \ldots, m_{A}$ 以 及 GNSS 系统 B 的卫星 $s_{B}=1_{B}, 2_{B}, \ldots, m_{B}$ 。不妨假设选取系统 A 作为基准系统, 同时选取卫星 $1_{A}$ 作为系统 $\mathrm{A}$ 和 $\mathrm{B}$ 的参考卫星。

* 首先需要将不同系统的观测值统一或**转换到相同的时空参考基准**。
* **系统间偏差**（ISB）主要由**硬件延迟**组成，由 GNSS 设备中的不同信号路径产生，这取决于接收机内部的相关性，如果可以合理处理伪距和相位**差分系统间偏差**（DISB），那么使用系统间差分模型可以获得最大化冗余。

频率相同，相位观测值的**波长相等**，相位可以先做差再乘波长，**单差模糊度可以合并在一起**。然而，至少在接收机类型不一致的基线情况下，**接收机硬件延迟不能消除**。因此，观测方程分别表示为：
$$
\begin{array}{l}
\nabla \Delta P_{b r, f_{i}^{A}}^{1_{A} S_{B}}=\nabla \Delta \rho_{b r}^{1 A S_{B}}+{{\color{green}\nabla \Delta d_{b r, f_{i}^{A}}^{A B}}}+\nabla \Delta I_{b r, f_{i}^{A}}^{1_{A} S_{B}}+\nabla \Delta T_{b r}^{1{ }^{A} S_{B}}+\nabla \Delta \varepsilon_{b r, f_{i}^{A}}^{1_{A} S_{B}} \\
\lambda_{f_{i}^{A}} \nabla \Delta \phi_{b r, f_{i}^{A}}^{1_{A} s_{B}}=
\nabla \Delta \rho_{b r}^{1 s_{B}}+\lambda_{f_{i}^{A}}\left({\color{red}\nabla \Delta \delta_{b r, f_{i}^{A}}^{A B}}+{\color{blue}\nabla \Delta N_{b r, f_{i}^{A}}^{1_{A} s_{B}}}\right)-\\
\nabla \Delta I_{b r, f_{i}^{A}}^{1_{A} s_{B}}+\nabla \Delta T_{b r}^{1_{A} s_{B}}+ \nabla \Delta \xi_{b r, f_{i}^{A}}^{1_{A} s_{B}} 
\end{array}
$$

式中，${\color{green}\nabla \Delta d_{b r, f_{i}^{A}}^{A B}} \text { 和 } {\color{red}\nabla \Delta \delta_{b r, f_{i}^{A}}^{A B}}$ 分别表示接收机间伪距与相位的差分系统间偏差（DISB）：
$$
\begin{array}{c}
{\color{green}\nabla \Delta d_{b r, f_{i}^{A}}^{A B}}=\Delta d_{b r, f_{i}^{A}}^{B}-\Delta d_{b r, f_{i}^{A}}^{A} \\
{\color{red}\nabla \Delta \delta_{b r, f_{i}^{A}}^{A B}}=\left(\Delta \varphi_{b r, f_{i}^{A}}^{B}+\Delta \delta_{b r, f_{i}^{A}}^{B}\right)-\left(\Delta \varphi_{b r, f_{i}^{A}}^{A}+\Delta \delta_{b r, f_{i}^{A}}^{A}\right)
\end{array}
$$
* DISB 在接收机类型相同时数值接近于 0
* 在接收机类型不同时数值一般较大，不过其在时域上表现出长期稳定性

#### 4.相同系统的频率间差分模型

除了标准的系统内双差观测值外，引入了不同频率之间的双差观测值， 从而增加了观测冗余，有助于提高 GNSS 定位的可靠性和可用性，尤其是在仅跟踪少量卫星的情况下。但必须考虑差分频率间偏差（DIFB）的影响。

假设接收机 $b$ 和 $r$ 可以跟踪到 GNSS 系统 $A$ 的卫星，$s_{A}=1_{A}, 2_{A}, \ldots, m_{A}$ ，选取卫星 $1_{A}$ 作为系统 A 的参考卫星，伪距和相位观测值频率间差分表达式分别为：
$$
\Delta \nabla P_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}}=\Delta \nabla \rho_{b r}^{1_{A} s_{A}}+{\color{green}\Delta \bar{d}_{b r, f_{i}^{A} f_{j}^{A}}^{A}}+\nabla \Delta I_{b r, f_{i} f_{j}}^{1_As_A}+\nabla \Delta T_{b r}^{1_As_A}+\Delta \nabla \varepsilon_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}}
$$
$$
\begin{array}{c}\lambda_{f_{j}^{A}}^{s_{A}} \Delta \phi_{b r, f_{j}^{A}}^{s_{A}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \phi_{b r, f_{i}^{A}}^{1_{A}}=\Delta \nabla \rho_{b r}^{1_{A} s_{A}}+{\color{red} \left(\lambda_{f_{j}^{A}} \Delta \varphi_{b r, f_{j}^{A}}-\lambda_{f_{i}^{A}} \Delta \varphi_{b r, f_{i}^{A}}\right)}+ \\
{\color{red}\left(\lambda_{f_{j}^{A}} \Delta \delta_{b r, f_{j}^{A}}^{A}-\lambda_{f_{i}^{A}} \Delta \delta_{b r, f_{i}^{A}}^{A}\right)}-{\color{red} \left(\lambda_{f_{j}^{A}} \Delta \delta_{f_{j}^{A}}^{S_{A}}-\lambda_{f_{i}^{A}} \Delta \delta_{f_{i}^{A}}^{1_{A}}\right)}+{\color{blue}\left(\lambda_{f_{j}^{A}} \Delta N_{b r, f_{j}^{A}}^{s_{A}}-\right. \left.\lambda_{f_{i}^{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}\right) }-\\
\nabla \Delta I_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}}+\nabla \Delta T_{b r}^{1_{A} s_{A}}+\Delta \nabla e_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}}\end{array}
$$
其中，伪距 DIFB为两频率单差硬件延迟之差 + 双差DCB，DCB 表示差分码偏差，包括卫星硬件延迟。
$${\color{green}\Delta \bar{d}_{b r, f_{i}^{A} f_{j}^{A}}^{A}}=\left(\Delta d_{b r, f_{j}^{A}}^{A}-\Delta d_{b r, f_{i}^{A}}^{A}\right)+\Delta \nabla D C B_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}}$$

因为卫星端的硬件延迟、接收机端的硬件延迟以及初始相位小数偏差项耦合在一起，难以将其分离，通过引入**相位差分频间偏差**（DIFPB）进行重参数化来吸收接收机初始相位偏差的影响：
$$
\begin{array}{c}{\color{red} \Delta \nabla \delta_{b r, f_{i}^{A} f_{j}^{A}}^{A} }=  \left(\begin{array}{c}\left(\lambda_{f_{j}^{A}} \Delta \varphi_{b r, f_{j}^{A}}-\lambda_{f_{i}^{A}} \Delta \varphi_{b r, f_{i}^{A}}\right)+\left(\lambda_{f_{j}^{A}} \Delta \delta_{b r, f_{j}^{A}}^{A}-\lambda_{f_{i}^{A}} \Delta \delta_{b r, f_{i}^{A}}^{A}\right) \\ -\left(\lambda_{f_{j}^{A}} \Delta \delta_{f_{j}^{A}}^{S_{A}}-\lambda_{f_{i}^{A}} \Delta \delta_{f_{i}^{A}}^{A_{A}}\right)\end{array}\right) / \lambda_{f_{j}^{A}}\end{array}
$$
此外，两个站间单差模糊度参数也可以重新参数化：
$$
\begin{array}{c} {\color{blue}\lambda_{f_{j}^{A}} \Delta N_{b r, f_{j}^{A}}^{s_{A}}-\lambda_{f_{i}^{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}}=\left(\lambda_{f_{j}^{A}} N_{r, f_{j}^{A}}^{s_{A}}-\lambda_{f_{j}^{A}} N_{b, f_{j}^{A}}^{s_{A}}\right)-\lambda_{f_{i}^{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}= \\ 
\left(\lambda_{f_{j}^{A}} N_{r, f_{j}^{A}}^{s_{A}}-\lambda_{f_{j}^{A}} N_{r, f_{j}^{A}}^{1_{A}^{}}\right)-\left(\lambda_{f_{j}^{A}} N_{b, f_{j}^{A}}^{s_{A}}-\lambda_{f_{j}^{A}} N_{b, f_{j}^{A}}^{1_{A}^{}}\right)+\left(\lambda_{f_{j}^{A}} N_{r, f_{j}^{A}}^{1_{A}^{}}-\lambda_{f_{j}^{A}} N_{b, f_{j}^{A}}^{1_{A}^{}}\right)-  \lambda_{f_{i}{ }^{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}^{}}=\\
{\color{blue}\lambda_{f_{j}^{A}} \Delta \nabla N_{b r, f_{j}^{A}}^{1_{A} s_{A}}+\left(\lambda_{f_{j}^{A}} \Delta N_{b r, f_{j}^{A}}^{1_{A}}-\lambda_{f_{i}^{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}\right)}\end{array}
$$
所以相位观测值频率间差分表达式可以表示为：
$$
\begin{array}{l}\lambda_{f_{j}^{A}}^{s_{A}} \Delta \phi_{b r, f_{j}^{A}}^{s_{A}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \phi_{b r, f_{i}^{A}}^{1_{A}}=\Delta \nabla \rho_{b r}^{1_{A} s_{A}}+{\color{red}\lambda_{f_{j}^{A}} \Delta \nabla \bar{\delta}_{b r, f_{i}^{A} f_{j}^{A}}^{A}}+\lambda_{f_{j}^{A}} \Delta \nabla N_{b r, f_{j}^{A}}^{1_{A} s_{A}}+  \Delta \nabla e_{b r, f_{i}^{A} f_{j}^{A}}^{1_{A} s_{A}} \end{array}
$$
其中：
$$
\Delta \nabla \bar{\delta}_{b r, f_{i}^{A} f_{j}^{A}}^{A}={\color{red}\Delta \nabla \delta_{b r, f_{i}^{A} f_{j}^{A}}^{A}}+\left(\Delta N_{b r, f_{j}^{A}}^{1_{A}}-\frac{\lambda_{f_{i}^{A}}}{\lambda_{f_{j}^{G}}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}\right)
$$

#### 5.不同频率的系统间双差模型

假设接收机 $b$ 和 $r$ 可以同时跟踪到 GNSS 系统 $\mathrm{A}$ 的卫星 $s_{A}=1_{A}, 2_{A}, \ldots, m_{A}$ 以 及 GNSS 系统 B 的卫星 $s_{B}=1_{B}, 2_{B}, \ldots, m_{B}$ 。不妨假设选取系统 A 作为基准系统, 同时选取卫星 $1_{A}$ 作为系统 $\mathrm{A}$ 和 $\mathrm{B}$ 的参考卫星。

由于频率的差异，波长不等，相位得先乘对应的波长做差，**卫星端的硬件延迟**、**接收机端的硬件延迟**以及**初始相位偏差**在差分过程中无法被消除。观测方程表示为：
$$
\begin{array}{c}
\nabla \Delta P_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}=\nabla \Delta \rho_{b r}^{1_{A} s_{B}}+{\color{green}\Delta \nabla \bar{d}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}+\nabla \Delta I_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}+\nabla \Delta T_{b r}^{1_{A} s_{B}}+\nabla \Delta \varepsilon_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}} \\
\lambda_{f_{j}^{B}}^{s_{B}} \Delta \phi_{b r, f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}^{}} \Delta \phi_{b r, f_{i}^{A}}^{1_{A}}=\nabla \Delta \rho_{b r}^{1_{A} s_{B}}+
{\color{blue}\lambda_{f_{j}^{B}}^{s_{B}} \Delta N_{b r, f_{j}^{B}}^{s_{B}}}-
{\color{blue} \lambda_{f_{i}^{A}}^{1_{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}}+ \\ {\color{red}\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \varphi_{b r, f_{j}^{B}}-\lambda_{f_{i}^{A}}^{1_{A}^{}} \Delta \varphi_{b r, f_{i}^{A}}\right)+\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \delta_{b r, f_{j}^{B}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \delta_{b r, f_{i}^{A}}\right)}-\\
{\color{red}{{\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \delta_{f_{j}^{B}}^{s_{B}^{}}-\right.} }\left.\lambda_{f_{i}^{A}}^{\mathbf{1}_{A}} \Delta {\delta}_{\boldsymbol{f}_{i}^{A}}^{\mathbf{1}_{A}}\right)}-\nabla \Delta I_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}+\nabla \Delta T_{b r}^{1_{A} s_{B}}+\nabla \Delta \xi_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}
\end{array}
$$
其中，伪距偏差由两部分组成： $\Delta \nabla d_{b r, f_{i}^{A} f_{j}^{B}}^{A B}$ **系统间伪距偏差**，主要是接收机伪距硬件延迟，$IFCB_{b r}^{A B}$ 表示**频间伪距偏差**，包括了卫星端的伪距硬件延迟：
$$
{\color{green}\Delta \nabla \bar{d}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}=\Delta \nabla d_{b r, f_{i}^{A} f_{j}^{B}}^{A B}+I F C B_{b r}^{A B}
$$

载波相位偏差的**卫星端的硬件延迟**、**接收机端的硬件延迟**以及**初始相位偏差**，耦合在一 起，难以将其分离，所以考虑将其重组为一个参数，即重新参数化的**差分系统间相位偏差**（DISPB）项来吸收初始相位小数偏差的影响：
$$
{\color{red}\nabla \Delta \delta_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}=\left(\begin{array}{c}
\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \varphi_{b r, f_{j}^{B}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \varphi_{b r, f_{i}^{A}}\right)+\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \delta_{b r, f_{j}^{B}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \delta_{b r, f_{i}^{A}}\right) \\
-\left(\lambda_{f_{j}^{B}}^{s_{B}} \Delta \delta_{f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta \delta_{f_{i}^{A}}^{1_{A}}\right)
\end{array}\right) / \lambda_{f_{j}^{B}}^{s_{B}}
$$
由于不同的波长，单差模糊度不能进一步合并为双差模糊度。在这种情况下，可以**重写模糊度**项：
$$
\begin{array}{c}
{\color{blue}\lambda_{f_{j}^{B}}^{s_{B}} \Delta N_{b r, f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}}=\lambda_{f_{j}^{B}}^{s_{B}} \Delta N_{b r, f_{j}^{B}}^{s_{B}}-\lambda_{f_{j}^{B}}^{s_{B}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}+\lambda_{f_{j}^{B}}^{s_{B}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}- \lambda_{f_{i}^{A}}^{1_{A}} \Delta N_{b r, f_{i}^{A}}^{1_{A}}=\\
{\color{blue}\lambda_{f_{j}^{B}}^{s_{B}} \Delta \nabla N_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}+\left(\lambda_{f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}}\right) \Delta N_{b r, f_{i}^{A}}^{1_{A}}}
\end{array}
$$
于是，相位观测方程可表示为：
$$
\begin{array}{c}
\lambda_{f_{j}^{B}}^{s_{B}} \Delta \phi_{b r, f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}^{}} \Delta \phi_{b r, f_{i}^{A}}^{1_{A}}=\\
\nabla \Delta \rho_{b r}^{1_{A} s_{B}}+{\color{red}\lambda_{f_{j}^{B}}^{s_{B}} \nabla \Delta \bar{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}+
{\color{blue}\lambda_{f_{j}^{B}}^{s_{B}} \Delta \nabla N_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}}- \nabla \Delta I_{b r, f_{i}^{A} f_{i}^{B}}^{1_{A} s_{B}}+\nabla \Delta T_{b r}^{1_{A} s_{B}}+\nabla \Delta \xi_{b r, f_{i}^{A} f_{i}^{B}}^{1_{A} s_{B}}
\end{array}
$$
其中，
$$
{\color{red}\nabla \Delta \bar{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}={\color{red}\nabla \Delta \delta_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}+{\color{blue}\left(1-\frac{\lambda_{f_{i}^{A}}^{1_{A}}}{\lambda_{f_{i}^{B}}^{S_B}}\right) \Delta N_{b r, f_{i}^{A}}^{1_{A}}}
$$
进一步地，系统间的双差模糊度 ${\color{blue}\Delta \nabla N_{b r, f_{i}^{A}}^{1_{A} S_{B}} f_{j}^{B}}$ 作如下变换，拆分为系统 B 参考卫星与系统 A 参考卫星之间的双差模糊度 $\color{blue}\Delta \nabla N_{b r, f_{i}^{A} f_{j}^{1_{A} 1_{B}}}$ 和系统 B 的系统内双差模糊度 $\color{blue}\Delta \nabla N_{b r, f_{j}^{B}}^{1_{B} s_{B}}$ 之和，即： 
$$
{\color{blue}\Delta \nabla N_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}}=\Delta N_{b r, f_{j}^{B}}^{s_{B}}-\Delta N_{b r, f_{j}^{B}}^{1_{B}}+\Delta N_{b r, f_{j}^{B}}^{1_{B}}-\Delta N_{b r, f_{i}^{A}}^{1_{A}}={\color{blue}\Delta \nabla N_{b r, f_{j}^{B}}^{1_{B} s_{B}}+\Delta \nabla N_{b r, f_{i}^{A}}^{1_{A} 1_{B}} f_{j}^{B}}
$$
因此观测方程可表示为：
$$
\begin{array}{c}
\lambda_{f_{j}^{B}}^{s_{B}} \Delta \phi_{b r, f_{j}^{B}}^{s_{B}}-\lambda_{f_{i}^{A}}^{1_{A}^{}} \Delta \phi_{b r, f_{i}^{A}}^{1_{A}}=\\\nabla \Delta \rho_{b r}^{1_{A} s_{B}}+{\color{red}\lambda_{f_{j}^{B}}^{s_{B}} \nabla \Delta \hat{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}+
{\color{blue}\lambda_{f_{j}^{B}}^{s_{B}} \Delta \nabla N_{b r, f_{j}^{B}}^{1_{B} s_{B}}}- \nabla \Delta I_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}+\nabla \Delta T_{b r}^{1_{A} s_{B}}+\nabla \Delta \xi_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} s_{B}}
\end{array}
$$
其中：
$$
\begin{array}{l}
{\color{red}\nabla \Delta \hat{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}={\color{red}\nabla \Delta \bar{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}}+{\color{blue}\Delta \nabla N_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} 1_{B}}}=
\nabla \Delta \delta_{b r, f_{i}^{A} f_{j}^{B}}^{A B}+\Delta \nabla N_{b r, f_{i}^{A} f_{j}^{B}}^{1_{A} 1_{B}}+\left(1-\frac{\lambda_{f_{i}^{A}}^{1_{A}}}{\lambda_{f_{i}^{B}}^{S_B}}\right) \Delta N_{b r, f_{i}^{A}}^{1_{A}}
\end{array}
$$
可以看出，
* 相位 DISB 参数 $\nabla \Delta \hat{\delta}_{b r, f_{i}^{A B}}^{A B} f_{j}^{B}$ 同时吸收了两个系统参考卫星之间的双差模糊度 $\Delta \nabla N_{b r, f_{i}^{A}}^{1_{A}{ }^{A}} f_{j}^{B}$ 以及基准系统参考卫星的站间单差模糊度 $\Delta N_{b r, f_{i}^{A}}^{1_{A}}$ 。
* 相位 DISB 参数由整数部分和小数部分组成，而且其整数部分可以归算到整周模糊度中，因此要想研究相位 DISB 在时域的稳定性，只需要分析其小数部分的稳定性。
* 当基准系统 A 或非基准系统 B 的参考卫星变化时，估计得出的相位差分系统间偏差 $\nabla \Delta \hat{\delta}_{b r, f_{i}^{A} f_{j}^{B}}^{A B}$ 也将发生变化，其中非基准系统的参考星只影响其整数部分，而基准系统的参考星同时影响其整数部分和小数部分。


### 3、双差观测量误差处理

双差观测模型不但消除了卫星钟差和接收机钟差，而且大大削弱了卫星轨道误差、电离层延迟误差和对流层延迟误差。由于电离层延迟误差和对流层延迟误差具有强空间相关性，在中长基线条件下不能忽略其残差的影响。

#### 1.电离层延迟误差

常用的电离层改正模型包括：Klobuchar 模型及其改进模型、NeQuick 模型、NTCM 模型及其修正模型 MNTCM-BC、BDS 广播模型及其改进模型。但是，电离层预报模型的精度较低。与电离层预报模型相比，由 IGS 的联合分析中心 IAAC 提供的最终全球电离层格网模型（Global Ionosphere Maps，GIM）具有 2-8 TECU（Total Electron  Content Unit）的更高精度，提升超过了 50%。但是，这种最终 GIM 不能用于实时定位，因为其延迟时间为 1-14 天。

对于短基线（<20 km）而言，可以忽略双差电离层延迟残差的影响。当基线长度为 50 km 左右时，双差电离层延迟残差的最大值达到 0.7 m；当基线长度为 200 km 左右时，双差电离层延迟残差的最大值达到 2.5 m。所以对于中长基线，双差电离层延迟残差不能忽略不计，通常采用以下三类方法对其处理：

* **参数估计法**：将双差电离层延迟残差作为未知参数，与位置参数和模糊度参数一起进行参 数估计
* **无电离层组合法**：由于电离层延迟与频率有关，因此电离层延迟的一阶项可以采用双频无电离层组合观测值进行消除。而对于具有三频信号的卫星导航系统，电离层延迟的二阶项也可以通过无电离层组合观测值进行消除。
* **先验约束法**：将双差电离层延迟残差作为未知数，同时构造虚拟观 测方程对双差电离层延迟残差进行先验约束，联合非组合双差观测方程进行参数估计。合理地设置双差电离层延迟残差的先验方差，有助于快速实现模糊度固定

#### 2.对流层延迟误差

对于短基线（<20 km）而言，对流层延迟残差可以忽略不计。当基线长度为 50 km 左右时，双差对流层延迟残差基本在 5 cm 以内； 当基线长度为 200 km 左右时，双差对流层延迟残差的最大值接近 20 cm；因此，对于中长基线来说，双差对流层延迟残差的影响不能忽略不计，通常将天顶方向的对流层延迟模型化，然后投影到信号传播方向。

* 常用的**对流层延迟模型**包括：Hopfield 模型、Saastamoinen 模型、UNB3 模型、全球气压温度模型
* 常用的**投影函数模型**包括：Marini、Chao、NMF、VMF1、GMF 

对流层延迟误差的**干分量**可以通过模型进行改正，而**湿分量**与大气湿度和高度角相关，很难用模型对其改正，因此需要进行参数估计。天顶对流层延迟估计方法包括：分段线性函数法、分段常数法和随机游走法。


## 三、一种适用于不同长度基线的 RTK 定位模型

* 先利用**伪距**观测值和**相位宽巷组合**观测值进行**双差宽巷模糊度**的计算和固定
* 将双差宽巷整周模糊度固定的相位宽巷组合观测值作为测距精度较高的伪距观测值
* 用该**相位宽巷组合观测值**（视为伪距观测值）和**相位无电离层组合观测值**进行双差载波相位整周模糊度的固定，进而实现 RTK 定位

### 1、单频伪距和载波相位组合（P1L1）

双差伪距和载波相位 L1 观测方程分别表示为：
$$
\begin{array}{c}\nabla \Delta P_{1, A B}^{i j}=\nabla \Delta \rho_{A B}^{i j}+\nabla \Delta I_{A B}^{i j}+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \varepsilon_{1, A B}^{i j} \\ \lambda_{1} \nabla \Delta \phi_{1, A B}^{i j}=\nabla \Delta \rho_{A B}^{i j}+\lambda_{1} \nabla \Delta N_{1, A B}^{i j}-\nabla \Delta I_{A B}^{i j}+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \xi_{1, A B}^{i j}\end{array}
$$

*  $\nabla \Delta(\cdot)$ 为双差运算符; 
* $P 、 \phi$ 分别为伪距和相位观测值; 
* 上标 $i 、 j$ 表示不同的卫星; 
* 下标 1 表示频率; 
* 下标 A、B 表示不同的测站; 
* $\rho$ 为测站到卫星之间的距离; 
* $I$ 为电离层延迟误差;
* $\nabla \bar{M}_{W, A B}^{i j}$ 为两测站星间单差对流层湿延迟投影系数的平均数;
* $\bar{T}_{W, A B}$ 为两测站平均天顶对流层湿延迟; 
* $\lambda$ 为载波波长;
* $N$ 为模糊度;
* $\varepsilon$ 、 $\xi$ 分别为伪距和相位观测值的测量噪声。

### 2、伪距和载波相位无电离层组合（PCLC）

双差伪距和相位无电离层组合观测方程分别表示为：
$$
\begin{array}{c}\nabla \Delta P_{I F, A B}^{i j}=\nabla \Delta \rho_{A B}^{i j}+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \varepsilon_{I F, A B}^{i j} \\ \lambda_{I F} \nabla \Delta \phi_{I F, A B}^{i j}=\nabla \Delta \rho_{A B}^{i j}+\lambda_{I F} \nabla \Delta N_{I F, A B}^{i j}+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \xi_{I F, A B}^{i j}= \\ \nabla \Delta \rho_{A B}^{i j}+ \lambda_{I F}\left(\nabla \Delta N_{1, A B}^{i j}+\frac{f_{2}}{f_{1-f_{2}}} \nabla \Delta N_{W, A B}^{i j}\right)+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \xi_{I F, A B}^{i j}
\end{array}
$$
式中：下标 IF 表示无电离层组合；$\nabla \Delta_{IF}$ 为双差无电离层模糊度，可以拆分为**双差 L1 模糊度**和**宽巷模糊度**的表达式，在整周**宽巷**模糊度准确固定后，再进行相位 L1 浮点模糊度解算，采用 LAMBDA 算法搜索 L1 模糊度，并采用 Ratio 值进行检验，然后进行定位结果的解算。

### 3、宽巷相位和载波相位无电离层组合（LWLC）

双差宽巷组合观测方程表示为：
$$
\lambda_{W} \nabla \Delta \phi_{W, A B}^{i j}=\nabla \Delta \rho_{A B}^{i j}+\lambda_{W} \nabla \Delta N_{W, A B}^{i j}+\frac{f_{1}}{f_{2}} \nabla \Delta I_{A B}^{i j}+\nabla \bar{M}_{W, A B}^{i j} \bar{T}_{W, A B}+\nabla \Delta \xi_{W, A B}^{i j}
$$

### 4、高度角定权

$$
D_{i}=\left[\begin{array}{cccc}1+\frac{\sin E_{1}}{\sin E_{r}} & 1 & \cdots & 1 \\ 1 & 1+\frac{\sin E_{2}}{\sin E_{r}} & \cdots & 1 \\ \vdots & \vdots & \ddots & \vdots \\ 1 & 1 & \cdots & 1+\frac{\sin E_{n-1}}{\sin E_{r}}\end{array}\right]
$$
式中: $E_{r}$ 是两测站参考卫星的平均高度角; $E_{1}, \cdots, E_{n-1}$ 为两测站其它 $n-1$ 颗共视卫星的平均高度角。

## 四、顾及不同系统和不同频率观测值定位偏差的RTK 模型

### 1、顾及不同系统观测值定位偏差的 RTK 模型

多系统组合后的系统内差分相对定位中不同系统观测值的定位偏差（PBDSO），其数量级相对较小，但确实存在，

$$
\left\{\begin{array}{c}P_{b r}^{k j, *_{1}}=l_{r}^{k j, *_{1}} \Delta X_{r}+m_{r}^{k j, *_{1}} \Delta Y_{r}+n_{r}^{k j, *_{1}} \Delta Z_{r}+\delta I_{b r}^{k j, *_{1}}+\delta T_{b r}^{k j, *_{1}}+\rho_{b r}^{k j, *_{1}}+\varepsilon_{P}^{*_{1}} \\ P_{b r}^{k j, *_{2}}=l_{r}^{k j, *_{2}} \Delta X_{r}+m_{r}^{k j, *_{2}} \Delta Y_{r}+n_{r}^{k j, *_{2}} \Delta Z_{r}+\delta I_{b r}^{k j, *_{2}}+\delta T_{b r}^{k j, *_{2}}+\rho_{b r}^{k j, *_{2}}+\varepsilon_{P}^{*_{2}} \\ \lambda^{*_{1}} \phi_{b r}^{k j, *_{1}}=l_{r}^{k j, *_{1}} \Delta X_{r}+m_{r}^{k j, *_{1}} \Delta Y_{r}+n_{r}^{k j, *_{1}} \Delta Z_{r}-\delta I_{b r}^{k j, *_{1}}+\delta T_{b r}^{k j, *_{1}}+\rho_{b r}^{k j, *_{1}}-\lambda^{*_{1}} N_{b r}^{k j, *_{1}}+\varepsilon_{\phi}^{*_{1}} \\ \lambda^{*_{2}} \phi_{b r}^{k j, G_{2}}=l_{r}^{k j, *_{2}} \Delta X_{r}+m_{r}^{k j, *_{2}} \Delta Y_{r}+n_{r}^{k j, *_{2}} \Delta Z_{r}-\delta I_{b r}^{k j, *_{2}}+\delta T_{b r}^{k j, *_{2}}+\rho_{b r}^{k j, *_{2}}-\lambda^{*_{2}} N_{b r}^{k j, *_{2}}+\varepsilon_{\phi}^{*_{2}}\end{array}\right.
$$




$$
\left\{\begin{array}{c}

P_{b r}^{k j, G_{1}}=l_{r}^{k j, G_{1}} \Delta X_{r}+m_{r}^{k j, G_{1}} \Delta Y_{r}+n_{r}^{k j, G_{1}} \Delta Z_{r}+\delta I_{b r}^{k j, G_{1}}+\delta T_{b r}^{k j, G_{1}}+\rho_{b r}^{k j, G_{1}}+\varepsilon_{p}^{G_{1}} \\

P_{b r}^{k j, G_{2}}=l_{r}^{k j, G_{2}} \Delta X_{r}+m_{r}^{k j, G_{2}} \Delta Y_{r}+n_{r}^{k j, G_{2}} \Delta Z_{r}+\delta I_{b r}^{k j, G_{2}}+\delta T_{b r}^{k j, G_{2}}+\rho_{b r}^{k j, G_{2}}+\varepsilon_{p}^{G_{2}} \\

\lambda^{G_{1}} \phi_{b r}^{k j, G_{1}}=l_{r}^{k j, G_{1}} \Delta X_{r}+m_{r}^{k j, G_{1}} \Delta Y_{r}+n_{r}^{k j, G_{1}} \Delta Z_{r}-\delta I_{b r}^{k j, G_{1}}+\delta T_{b r}^{k j, G_{1}}+\rho_{b r}^{k j, G_{1}}-\lambda^{G_{1}} N_{b r}^{k j, G_{1}}+\varepsilon_{\phi}^{G_{1}} \\

\lambda^{G_{2}} \phi_{b r}^{k j, G_{2}}=l_{r}^{k j, G_{2}} \Delta X_{r}+m_{r}^{k j, G_{2}} \Delta Y_{r}+n_{r}^{k j, G_{2}} \Delta Z_{r}-\delta I_{b r}^{k j, G_{2}}+\delta T_{b r}^{k j, G_{2}}+\rho_{b r}^{k j, G_{2}}-\lambda^{G_{2}} N_{b r}^{k j, G_{2}}+\varepsilon_{\phi}^{G_{2}} \\

P_{b r}^{k j, C_{1}}=l_{r}^{k j, C_{1}} \Delta X_{r}+m_{r}^{k j, C_{1}} \Delta Y_{r}+n_{r}^{k j, C_{1}} \Delta Z_{r}+\delta I_{b r}^{k j, C_{1}}+\delta T_{b r}^{k j, C_{1}}+\rho_{b r}^{k j, C_{1}}+\varepsilon_{P}^{C_{1}} \\

P_{b r}^{k j, C_{2}}=l_{r}^{k j, C_{2}} \Delta X_{r}+m_{r}^{k j, C_{2}} \Delta Y_{r}+n_{r}^{k j, C_{2}} \Delta Z_{r}+\delta I_{b r}^{k j, C_{2}}+\delta T_{b r}^{k j, C_{2}}+\rho_{b r}^{k j, C_{2}}+\varepsilon_{P}^{C_{2}} \\

\lambda^{C_{1}} \phi_{b r}^{k j, C_{1}}=l_{r}^{k j, C_{1}} \Delta X_{r}+m_{r}^{k j, C_{1}} \Delta Y_{r}+n_{r}^{k j, C_{1}} \Delta Z_{r}-\delta I_{b r}^{k j, C_{1}}+\delta T_{b r}^{k j, C_{1}}+\rho_{b r}^{k j, C_{1}}-\lambda^{c_{1}} N_{b r}^{k j, C_{1}}+\varepsilon_{\phi}^{c_{1}} \\

\lambda^{c_{2}} \phi_{b r}^{k j, C_{2}}=l_{r}^{k j, C_{2}} \Delta X_{r}+m_{r}^{k j, C_{2}} \Delta Y_{r}+n_{r}^{k j, C_{2}} \Delta Z_{r}-\delta I_{b r}^{k j, C_{2}}+\delta T_{b r}^{k j, C_{2}}+\rho_{b r}^{k j, C_{2}}-\lambda^{C_{2}} N_{b r}^{k j, C_{2}}+\varepsilon_{\phi}^{c_{2}}

\end{array}\right.
$$
