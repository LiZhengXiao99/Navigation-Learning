> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning



[TOC]



## 一、pppos

![image-20231028182626796](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028182626796.png)







## 二、模型改正

### 1、satantpcv()：卫星天线相位中心改正

GNSS 的距离测量值为接收机天线至卫星天线的几何距离，而一般精密产品给出的卫星的坐标以卫星质量中心为参考，我们把相位中心和质量中心之间的差异称为卫星天线相位误差。实际测量中，天线相位引起的误差随时间变化，在对其处理时，我们一般将其分为常量和变量两个部分。常量部分称为卫星天线相位中心偏差（Phase Center Offset，PCO），表示卫星质量中心和卫星平均相位中心的差异。变量部分称为相位中心变化（Phase Center Variation，PCV），表示天线瞬时相位中心和平均相位中心之间的差异。从 2006 年 11 月 5 日，IGS 开始使用绝对相位中心改正模型 IGS_05，该模型给出了与天底角相关的卫星 PCV 和不同型号接收机的 PCO[85]。当前最新的IGS 天线改正文件是 igs14_2186_plus.atx（ftp://sopac-ftp.ucsd.edu/archive/garner/ gamit/tables/igs14_2186_plus.atx），其包括最新 GNSS 卫星和接收机的天线 PCO和 PCV 改正信息。





### 2、antmodel()：接收机天线相位中心改正

和卫星相似，接收机端也存在由天线相位中心引起的误差 PCO 和 PCV。和卫星不同的是，接收机的 PCO 定义为天线相位中心和天线参考点之间的差异，因为在实际测量中都是以天线参考点为基准进行对中整平的，而不是接收机质心。对于常见型号的接收机，IGS 给出了 GPS 和 GLONASS 的改正信息，可从天线改正文件中获取（当前最新版文件为 igs14_2186_plus.atx）。由于缺少 BDS 的接收机天线相位中心改正值，在 PPP 数据处理中，一般将 GPS 的接收机 PCO 和 PCV 信息用于 BDS。



### 3、model_phw()：天线相位缠绕改正

GNSS 载波相位是右旋圆极化电磁波，当接收机天线和卫星天线发生相对旋转时，载波相位观测值会因此产生误差，即相位缠绕（phase wind-up）。相位缠绕误差最大可达载波相位的一个波长，其改正模型如下：
$$
\left\{\begin{array}{l}
\mathbf{D}=\mathbf{x}-\mathbf{e}_{r}^{s}\left(\mathbf{e}_{r}^{s} \cdot \mathbf{x}\right)+\mathbf{e}_{r}^{s} \cdot \mathbf{y} \\
\mathbf{D}^{\prime}=\mathbf{x}^{\prime}-\mathbf{e}_{r}^{s}\left(\mathbf{e}_{r}^{s} \cdot \mathbf{x}^{\prime}\right)+\mathbf{e}_{r}^{s} \cdot \mathbf{y}^{\prime} \\
\Delta \Phi=\operatorname{sign}\left(\mathbf{e}_{r}^{s} \cdot\left(\mathbf{D}^{\prime} \cdot \mathbf{D}\right)\right) \arccos \left[\left(\mathbf{D}^{\prime} \cdot \mathbf{D}\right) /\left(\left|\mathbf{D}^{\prime}\right| \cdot|\mathbf{D}|\right)\right]
\end{array}\right.
$$
式中，$\mathbf{e}_{r}^{s}$ 表示卫星指向接收机的单位向量；$\mathbf{x}, \mathbf{y}$ 和 $\mathbf{x}^{\prime}, \mathbf{y}^{\prime}$ 分别表示接收机和卫星的两个有效偶极矢量；sign 表示符号函数。



### 4、tidedisp()：潮汐改正

由于地球不是理想刚体，形状会因其他天体的引力发生形变，所以即使将接收机固定在地球表面，接收机和地心的相位位置也会发生变化，接收机在地固系中的坐标随之改变，由此生的测量误差称为地球潮汐效应。GNSS 数据处理一般将潮汐分为三个部分，地球固体潮、海洋负荷潮和极潮。

* **地球固体潮**：是指地球固体在其他天体的引力作用下发生周期变化的现象，对接收机水平和高程方向的影响分别可达数厘米和数分米
* **海洋负荷潮**：是指在太阳和月球引力作用下地球海洋发生的周期性变化，对接收机水平和高程方向的影响都在厘米级。
* **极潮**：是指地球自转轴发生的瞬时变化引起的测量误差，对接收机影响在厘米级。

关于以上潮汐改正，目前主要通过国际地球自转协议（International Earth Rotation Service，IERS）的 IERS Convention 技术文档中的模型改正。





### 5、sagnac()：地球自转效应改正

GNSS 的 MEO 卫星轨道高度大约 20000km。导航信号由卫星发出到接收机接受需要数十微秒，对于 GEO 卫星信号传播时间更长。在导航信号传播过程中，由于地球自转的影响，地固坐标系已随地球旋转了一定角度，由此给观测值造成的误差可达数十米，其影响不可忽略。因此需要对观测值进行距离改正，改正数的计算方法如下：
$$
\Delta D=\frac{\omega}{c}\left[y^{S}\left(x_{R}-x^{S}\right)-x^{S}\left(y_{R}-y^{S}\right)\right]
$$
式中，$x^{S}, y^{S}, x_{R}, y_{R}$ 分别表示信号发射时刻卫星和接收机在地固坐标系下的坐标，$\omega$ 表示地球自转角速度，单位为弧度每秒。

### 6、model_trop()：

对流层一般指距离地面 50km 内的大气层，是大气层质量的主要部分。当导航信号穿过对流层时，由于传播介质密度的增加，信号传播路径和传播速度会发生改变，由此引起的 GNSS 观测值误差称为对流层延迟。对流层延迟一般可分为干延迟和湿延迟，对于载波相位和伪距完全相同，一般在米级大小，可通过模型改正和参数估计的方法来削弱其影响。修正模型如下：
$$
T=M_{d r y} T_{d r y}+M_{w e t} T_{w e t}
$$
式中，$T_{d r y}, T_{w e t}$ 分别表示接收机天顶对流层的干延迟和湿延迟；$M_{d r y}, M_{w e t}$ 分别表示干延迟和湿延迟的投影函数。对流层干延迟比较稳定，主要与测站高度、大气温度和大气压相关，可通过模型改正，常用模型有 Saastamoninen 模型、Hopfield 模型等。湿延迟不同于干延迟，变化较大，主要与水汽含量相关，一般估计天顶对流层湿延迟，通过投影函数计算各卫星的电离层湿延迟，常用的投影函数有全球投影函数（Global Mapping Function，GMF）、Niell 投影函数（NMF）和 Vienna投影函数（Vienna Mapping Function，VMF）等。本文使用 Saastamoninen 模型改正对流层干延迟，使用 GMF 投影函数估计接收机天顶对流层湿延迟。





### 7、model_iono()：

受太阳辐射的影响，距地面 60km 以上的大气层处于部分电离或完全电离状态，该区域被称为电离层。当电磁波信号通过电离层时，传播速度和传播路径会发生改变，给 GNSS 观测值带来误差，即电离层延迟。电离层延迟大小由电子密度和信号频率决定，影响可达数十米。由于 GNSS 信号的特性，同一频率载波相位和伪距观测值电离层延迟大小相等，符号相反。在一些条件下，可使用经验模型改正或约束观测值中的电离层延迟，常用的电离层延迟经验模型有 Klobuchar 模型、Bent 模型和电离层格网模型等。本文对电离层延迟的处理使用最常用的两种方法：使用非差非组合模型估计各卫星第一频率倾斜电离层延迟；使用无电离层组合消除电离层延迟一阶项，忽略二阶及以上项。



### 8、gravitationalDelayCorrection()：







### 9、corr_meas()：









## 三、周跳检测

目前用于非差周跳探测最常用的方法是联合使用 Geometry-free (GF) 和 MW 组合观测值进行周跳探测，其充分利用了双频观测值线性组合的特点。GF 和 MW 组合观测值分别为：
$$
\begin{array}{c}
L_{\mathrm{GF}}(i)=\lambda_{1} \Phi_{1}(i)-\lambda_{2} \Phi_{2}(i)=\left(\gamma_{2}-1\right) I_{1}(i)+\left(\lambda_{1} N_{1}-\lambda_{2} N_{2}\right) \\
\left\{\begin{array}{l}
\lambda_{\delta} \Phi_{\delta}(i)=\left(f_{1} \lambda_{1} \Phi_{1}(i)-f_{2} \lambda_{2} \Phi_{2}(i)\right) /\left(f_{1}-f_{2}\right)=\rho(i)+f_{1} f_{2} /\left(f_{1}^{2}-f_{2}^{2}\right) \cdot I_{1}(i)+\lambda_{\delta} N_{\delta} \\
P_{\delta}(i)=\left(f_{1} P_{1}(i)+f_{2} P_{2}(i)\right) /\left(f_{1}+f_{2}\right)=\rho(i)+f_{1} f_{2} /\left(f_{1}^{2}-f_{2}^{2}\right) \cdot I_{1}(i) \\
N_{\delta}=N_{1}-N_{2}=\Phi_{\delta}(i)-P_{\delta}(i) / \lambda_{\delta} \\
\lambda_{\delta}=c /\left(f_{1}-f_{2}\right)
\end{array}\right.
\end{array}
$$
式中，$i$ 表示观测历元号; $\lambda_{\delta}$ 和 $N_{\delta}$ 分别为宽巷波长和宽巷模糊度。 可以看出，MW 组合的精度受伪距观测噪声和多路径效应的影响，可通过下述递推公式减弱其影响，第 $i$ 个历元的 MW 组合观测量的平均值及方差为：
$$
\begin{array}{c}
\left\langle N_{\delta}\right\rangle_{i}=\left\langle N_{\delta}\right\rangle_{i-1}+\frac{1}{i}\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right) \\
\sigma_{i}^{2}=\sigma_{i-1}^{2}+\frac{1}{i}\left\{\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right)^{2}-\sigma_{i-1}^{2}\right\}
\end{array}
$$
式中, 〈〉表示多个历元的平滑值。对于 GF 组合，利用当前历元组合观测值与前一历元组合观测值的差值的绝对值 $\left|L_{\mathrm{GF}}(i)-L_{\mathrm{GF}}(i-1)\right|$ 作为检验量进行周跳探测。对于 MW 组合，将当前历元 $i$ 的 MW 观测量 $N_{\delta i}$ 与前 $i-1$ 历元宽巷模糊度平滑值 $\left\langle N_{\delta}\right\rangle_{i-1}$ 差值的绝对值进行比较判断是否发生周跳。顾及观测数据的采样率和高度角，给出确定周跳探测经验阈值：
$$
\begin{array}{l}\begin{array}{l}R_{\mathrm{GF}}(E, R)=\left\{\begin{array}{cc}(-1.0 / 15.0 \cdot E+2) \cdot b_{\mathrm{GF}}, & E \leq 15^{\circ} \\ b_{\mathrm{GF}}, & E>15^{\circ}\end{array}\right. \\ b_{\mathrm{GF}}(R)=\left\{\begin{array}{cc}0.05 \mathrm{~m}, & 0<R \leq 1 \mathrm{~s} \\ 0.1 / 20.0 \cdot R+0.05 \mathrm{~m}, & 1<R \leq 20 \mathrm{~s} \\ 0.15 \mathrm{~m}, & 20<R \leq 60 \mathrm{~s} \\ 0.25 \mathrm{~m}, & 60<R \leq 100 \mathrm{~s} \\ 0.35 \mathrm{~m}, & \text { 其它 }\end{array}\right.\end{array} \\ R_{\mathrm{MW}}(E, R)=\left\{\begin{array}{cc}(-0.1 \cdot E+3) \cdot b_{\mathrm{MW}}, & E \leq 20^{\circ} \\ b_{\mathrm{MW}}, & E>20^{\circ}\end{array}\right. \\ b_{\text {MW }}(R)=\left\{\begin{array}{cc}2.5 \mathrm{c}, & 0<R \leq 1 \mathrm{~s} \\ 2.5 / 20.0 \cdot R+2.5 \mathrm{c}, & 1<R \leq 20 \mathrm{~s} \\ 5.0 \mathrm{c}, & 20<R \leq 60 \mathrm{~s} \\ 7.5 \mathrm{c}, & \text { 其它 }\end{array}\right. \\\end{array}
$$
式中，$R_{\mathrm{GF}}$ (单位: $\mathrm{m}$ 或米) 和 $R_{\mathrm{MW}}$ (单位: $\mathrm{c}$ 或周) 分别为 $\mathrm{GF}$ 组合和 $\mathrm{MW}$ 组合周跳检验量的阈值；$E 、 R$ 分别为卫星高度角 (单位：度) 和观测值采样间隔 (单位：$s$ )。



### 1、detecs()





## 四、PPP 时间更新

### 1、udstate_ppp()：Kalman 滤波时间更新





### 2、udpos_ppp()：位置参数时间更新





### 3、udclk_ppp()：钟差参数时间更新





### 4、udtrop_ppp()：对流层参数时间更新





### 5、udiono_ppp()：电离层参数时间更新





### 6、uddcb_ppp()：DCB 参数时间更新





### 7、udbias_ppp()：模糊度参数时间更新













## 五、ppp_res()









## 六、filter()：EKF 估计





抗差 Kalman 滤波通过构造等价权对含有小周跳或粗差的观测值进行控制，降低异常观测值对于参数估计的影响。本文使用 IGGIII 等价权函数：
$$
\bar{p}_{i}=\left\{\begin{array}{cc}
p_{i}, & \left|\tilde{v}_{i}\right| \leq k_{0} \\
p_{i} \frac{k_{0}}{\left|\tilde{v}_{i}\right|}\left(\frac{k_{1}-\left|\tilde{v}_{i}\right|}{k_{1}-k_{0}}\right)^{2}, & k_{0}<\left|\tilde{v}_{i}\right| \leq k_{1} \\
0, & \left|\tilde{v}_{i}\right|>k_{1}
\end{array}\right.
$$


式中，$p_{i}$ 为观测量 $l_{i}$ 对应的权; $\tilde{v}_{i}$ 为标准化残差；$k_{0}$ 和 $k_{1}$ 为阈值常量，一般取 $k_{0}=1.0 \sim 1.5, \quad k_{1}=2.0 \sim 3.0$ 。























