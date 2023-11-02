> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

![image-20231101211450651](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231101211450651.png)

[TOC]

## 一、PPP 模型

### 1、非差非组合 PPP



以非差非组合双频 PPP 模型为例，假如某一时刻观测到 $\mathrm{m}$ 颗卫星，则可建立 $4 \mathrm{~m}$ 个观测方程，其误差方程为：
$$
\underset{4 m \times 1}{\mathbf{V}}=\underset{4 m \times n}{\mathbf{H}} \cdot \mathbf{X}-\underset{n \times 1}{\mathbf{l}}
$$
式中，$\mathrm{V}$ 为观测值残差向量；$\mathbf{H}$ 为系数矩阵；1 为观测量减去计算量。状态向量 $\mathbf{X}$包含接收机位置坐标增量、接收机钟差改正、天顶对流层湿延迟、倾斜电离层延迟以及 $\mathrm{L}_{1}$ 和 $\mathrm{L}_{2}$ 上的载波相位模糊度六类基本参数 $(n=5+3 m)$ ，即：
$$
\mathbf{X}=[\underbrace{\Delta x, \Delta y, \Delta z}_{\text {位置 }}, c d t_{r}, \underbrace{\mathrm{ZWD}^{2}}_{\text {天顶对流层延迟 }}, \overbrace{\mathrm{I}_{r, 1}^{1}, \cdots, \bar{I}_{r, 1}^{m}}^{L_{1} \text { 上的斜电离层延迟 }}, \underbrace{\bar{N}_{r, 1}^{1}, \cdots, \bar{N}_{r, 1}^{m},}_{L_{1} \text { 上的相位模糊度 }} \overbrace{\bar{N}_{r, 2}^{1}, \cdots, \bar{N}_{r, 2}^{m}}^{L_{2} \text { 上的相位模糊度 }}]^{T}
$$
系数矩阵 $\mathbf{H}$ 可表示为：
$$
\mathbf{H}=\left[\begin{array}{c}
\partial f_{\left(\mathbf{X}, p_{1}\right)}^{1} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, l_{1}\right)}^{1} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, p_{2}\right)}^{1} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, l_{2}\right)}^{1} / \partial \mathbf{X} \\
\vdots \\
\partial f_{\left(\mathbf{X}, p_{1}\right)}^{m} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, l_{1}\right)}^{m} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, p_{2}\right)}^{m} / \partial \mathbf{X} \\
\partial f_{\left(\mathbf{X}, l_{2}\right)}^{m} / \partial \mathbf{X}
\end{array}\right]=\left[\begin{array}{cccccccccccccc}
a^{1} & b^{1} & e^{1} & 1 & \mathrm{Mw}_{r}^{1} & 1 & \cdots & 0 & 0 & \cdots & 0 & 0 & \cdots & 0 \\
a^{1} & b^{1} & e^{1} & 1 & \mathrm{Mw}_{r}^{1} & -1 & \cdots & 0 & 1 & \cdots & 0 & 0 & \cdots & 0 \\
a^{1} & b^{1} & e^{1} & 1 & \mathrm{Mw}_{r}^{1} & \gamma_{1} & \cdots & 0 & 0 & \cdots & 0 & 0 & \cdots & 0 \\
a^{1} & b^{1} & e^{1} & 1 & \mathrm{Mw}_{r}^{1} & -\gamma_{1} & \cdots & 0 & 0 & \cdots & 0 & 1 & \cdots & 0 \\
\vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots & \vdots \\
a^{m} & b^{m} & e^{m} & 1 & \mathrm{Mw}_{r}^{m} & 0 & \cdots & 1 & 0 & \cdots & 0 & 0 & \cdots & 0 \\
a^{m} & b^{m} & e^{m} & 1 & \mathrm{Mw}_{r}^{m} & 0 & \cdots & -1 & 0 & \cdots & 1 & 0 & \cdots & 0 \\
a^{m} & b^{m} & e^{m} & 1 & \mathrm{Mw}_{r}^{m} & 0 & \cdots & \gamma_{1} & 0 & \cdots & 0 & 0 & \cdots & 0 \\
a^{m} & b^{m} & e^{m} & 1 & \mathrm{Mw}_{r}^{m} & 0 & \cdots & -\gamma_{1} & 0 & \cdots & 0 & 0 & \cdots & 1
\end{array}\right]
$$
式中，$a, b$ 和 $e$ 为卫星-接收机连线的方向余弦。

观测噪声的方差矩阵 $\mathbf{R}_{k}$ 可根据卫星高度角或信噪比随机模型确定，即:
$$
\mathbf{R}_{k}=\left[\begin{array}{ccccccccc}\left(\sigma_{p_{1}}^{1}\right)^{2} & 0 & 0 & 0 & & & & & \\ 0 & \left(\sigma_{l_{1}}^{1}\right)^{2} & 0 & 0 & & & & & \\ 0 & 0 & \left(\sigma_{p_{2}}^{1}\right)^{2} & 0 & & & & & \\ 0 & 0 & 0 & \left(\sigma_{l_{2}}^{1}\right)^{2} & & & & & \\ & & & & \ddots & & & & \\ & & & & & \left(\sigma_{p_{1}}^{m}\right)^{2} & 0 & 0 & 0 \\ & & & & & 0 & \left(\sigma_{l_{1}}^{m}\right)^{2} & 0 & 0 \\ & & & & & 0 & 0 & \left(\sigma_{p_{2}}^{m}\right)^{2} & 0 \\ & & & & & 0 & 0 & 0 & \left(\sigma_{l_{2}}^{m}\right)^{2}\end{array}\right]
$$
式中，$\sigma_{p_{1}}$ 和 $\sigma_{l_{1}}$ 分别表示卫星伪距和相位观测值的噪声。

对于 PPP 定位，其状态转移矩阵 $\boldsymbol{\Phi}_{k, k-1}$ 为单位阵，因此状态方程可描述为：
$$
\hat{\mathbf{X}}_{k, k-1}=\mathbf{X}_{k-1}
$$
其中，相位模糊度参数当作常数处理，其初值由伪距和载波观测值扣除电离层延迟得到，初始方差设为 $10^{4} \mathrm{~m}^{2}$；考虑到接收机钟差主要表现为高频信号，采用白噪声过程来描述它较为合适，每历元的初值由伪距单点定位确定，初始方差设为 $10^{4} \mathrm{~m}^{2}$；对流层延迟和电离层延迟参数利用随机游走参数过程估计，其初值分别由 Saastamoinen 模型和双频伪距观测值得到，初始方差分别设为 $0.6 \mathrm{~m}^{2}$ 和 $10^{4} \mathrm{~m}^{2}$，过程噪声分别设为 $10^{-8} \mathrm{~m}^{2} / \mathrm{s}^{[98]}$ 和 $0.0016 \mathrm{~m}^{2} / \mathrm{s}$ 。静态坐标参数作为常数估计，伪距单点定位结果作为初值，初始方差设为 $10^{4} \mathrm{~m}^{2}$；动态坐标参数作为白噪声估计，每历元的初值由伪距单点定位解得到，初始方差设为 $10^{4} \mathrm{~m}^{2}$ 。





### 参数数量、下标宏定义

状态向量 $\mathbf{X}$ 包含接收机位置坐标增量、接收机钟差改正、天顶对流层湿延迟、倾斜电离层延迟以及 $\mathrm{L}_{1}$ 和 $\mathrm{L}_{2}$ 上的载波相位模糊度六类基本参数 $(n=5+3 m)$ ，即：
$$
\mathbf{X}=[\underbrace{\Delta x, \Delta y, \Delta z}_{\text {位置 }}, c d t_{r}, \underbrace{\mathrm{ZWD}^{2}}_{\text {天顶对流层延迟 }}, \overbrace{\mathrm{I}_{r, 1}^{1}, \cdots, \bar{I}_{r, 1}^{m}}^{L_{1} \text { 上的斜电离层延迟 }}, \underbrace{\bar{N}_{r, 1}^{1}, \cdots, \bar{N}_{r, 1}^{m},}_{L_{1} \text { 上的相位模糊度 }} \overbrace{\bar{N}_{r, 2}^{1}, \cdots, \bar{N}_{r, 2}^{m}}^{L_{2} \text { 上的相位模糊度 }}]^{T}
$$
在 gamp.h 

N 开头的宏定义是参数数量：

* **NF**：**频率数量**：电离层与双频的线性组合时为 1，否则为设置的频率数 

  ```cpp
  #define NF(opt)     ((opt)->ionoopt==IONOOPT_IF12?1:(opt)->nf)
  ```

* **NP**：**位置参数数量**：正常为 3（ XYZ坐标），dynamics 动力学模式还有估计速度加速度为 9。

  ```cpp
  #define NP(opt)     ((opt)->dynamics?9:3)
  ```

* **NC**：**系统数量、钟差参数数量**：设为卫星系统数量，GPS 对应的是接收机钟差，其它系统对应的是与 GPS 之间的系统间偏差 ISB（包括系统间时间偏差和硬件延迟等）。

  ```cpp
  #define NC(opt)     (NSYS)
  ```

* **ND**：**接收机 DCB 参数数量**：如果频率数量 nf>=3，或者双频非差非组合，设为 1，否则为 0。

  ```cpp
  #define ND(opt)     ((opt)->nf>=3||(opt)->ionoopt==IONOOPT_UC12?1:0)
  ```

* **NICB**：**GLONASS 伪距频间偏差 ICB 数量**：不估计是为 0，线性模型为 1，二次多项式模型为 2，每颗卫星估计一个 GLONASS 卫星数 NSATGLO，否则是 13（13 是啥意思，没看懂）。

  ```cpp
  #define NICB(opt)   ((opt)->gloicb==GLOICB_OFF?0:((opt)->gloicb==GLOICB_LNF?1:((opt)->gloicb==GLOICB_QUAD?2:((opt)->gloicb==GLOICB_1SAT?NSATGLO:13))))
  ```

* **NT**：**对流层参数数量**：不估计对流层时为 0，`TROPOPT_EST`时为 1，`TROPOPT_ESTG`时为 3。

  ```cpp
  #define NT(opt)     ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt==TROPOPT_EST?1:3))
  ```

* **NI**：**电离层参数数量**：非差非组合模式为最大卫星数 MAXSAT，否则为 0 不估计电离层。

  ```cpp
  #define NI(opt)     ((opt)->ionoopt==IONOOPT_UC1||(opt)->ionoopt==IONOOPT_UC12?MAXSAT:0)
  ```

* **NR**：**除模糊度之外的参数数量**：上面的几个相加。

  ```cpp
  #define NR(opt)     (NP(opt)+NC(opt)+ND(opt)+NICB(opt)+NT(opt)+NI(opt))
  ```

* **NB**：**模糊度参数数量**：频率数 NF 乘以卫星数 MAXSAT。

  ```cpp
  #define NB(opt)     (NF(opt)*MAXSAT)
  ```

* **NX**：**总参数数量**：除模糊度之外的参数数量 NR + 总参数数量 NX。

  ```cpp
  #define NX(opt)     (NR(opt)+NB(opt))
  ```

I 开头的宏定义是参数下标：

> 为啥直接从 IC 开始，没有 IP？因为位置参数在最前面，就是前 3 或者前 9，直接取就行。

* **IC**：**钟差、ISB 参数下标**：

  ```cpp
  #define IC(s,opt)   (NP(opt)+(s))
  ```

* **ID**：**DCB 参数下标**：

  ```cpp
  #define ID(opt)     (NP(opt)+NC(opt))
  ```

* **IICB**：**GLONASS 伪距频间偏差 ICB 下标**：

  ```cpp
  #define IICB(s,opt) (NP(opt)+NC(opt)+ND(opt)+(s)-1)
  ```

* **IT**：**对流层参数下标**：

  ```cpp
  #define IT(opt)     (NP(opt)+NC(opt)+ND(opt)+NICB(opt))
  ```

* **II**：**电离层参数下标**：

  ```cpp
  #define II(s,opt)   (NP(opt)+NC(opt)+ND(opt)+NICB(opt)+NT(opt)+(s)-1)
  ```

* **IB**：**模糊度参数下标**：

  ```cpp
  #define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)
  ```

  



## 一、pppos

![image-20231028182626796](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028182626796.png)











## 二、周跳检测

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
式中，$i$ 表示观测历元号；$\lambda_{\delta}$ 和 $N_{\delta}$ 分别为宽巷波长和宽巷模糊度。 可以看出，MW 组合的精度受伪距观测噪声和多路径效应的影响，可通过下述递推公式减弱其影响，第 $i$ 个历元的 MW 组合观测量的平均值及方差为：
$$
\begin{array}{c}
\left\langle N_{\delta}\right\rangle_{i}=\left\langle N_{\delta}\right\rangle_{i-1}+\frac{1}{i}\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right) \\
\sigma_{i}^{2}=\sigma_{i-1}^{2}+\frac{1}{i}\left\{\left(N_{\delta i}-\left\langle N_{\delta}\right\rangle_{i-1}\right)^{2}-\sigma_{i-1}^{2}\right\}
\end{array}
$$
式中，〈〉表示多个历元的平滑值。对于 GF 组合，利用当前历元组合观测值与前一历元组合观测值的差值的绝对值 $\left|L_{\mathrm{GF}}(i)-L_{\mathrm{GF}}(i-1)\right|$ 作为检验量进行周跳探测。对于 MW 组合，将当前历元 $i$ 的 MW 观测量 $N_{\delta i}$ 与前 $i-1$ 历元宽巷模糊度平滑值 $\left\langle N_{\delta}\right\rangle_{i-1}$ 差值的绝对值进行比较判断是否发生周跳。顾及观测数据的采样率和高度角，给出确定周跳探测经验阈值：
$$
\begin{array}{l}\begin{array}{l}R_{\mathrm{GF}}(E, R)=\left\{\begin{array}{cc}(-1.0 / 15.0 \cdot E+2) \cdot b_{\mathrm{GF}}, & E \leq 15^{\circ} \\ b_{\mathrm{GF}}, & E>15^{\circ}\end{array}\right. \\ b_{\mathrm{GF}}(R)=\left\{\begin{array}{cc}0.05 \mathrm{~m}, & 0<R \leq 1 \mathrm{~s} \\ 0.1 / 20.0 \cdot R+0.05 \mathrm{~m}, & 1<R \leq 20 \mathrm{~s} \\ 0.15 \mathrm{~m}, & 20<R \leq 60 \mathrm{~s} \\ 0.25 \mathrm{~m}, & 60<R \leq 100 \mathrm{~s} \\ 0.35 \mathrm{~m}, & \text { 其它 }\end{array}\right.\end{array} \\ R_{\mathrm{MW}}(E, R)=\left\{\begin{array}{cc}(-0.1 \cdot E+3) \cdot b_{\mathrm{MW}}, & E \leq 20^{\circ} \\ b_{\mathrm{MW}}, & E>20^{\circ}\end{array}\right. \\ b_{\text {MW }}(R)=\left\{\begin{array}{cc}2.5 \mathrm{c}, & 0<R \leq 1 \mathrm{~s} \\ 2.5 / 20.0 \cdot R+2.5 \mathrm{c}, & 1<R \leq 20 \mathrm{~s} \\ 5.0 \mathrm{c}, & 20<R \leq 60 \mathrm{~s} \\ 7.5 \mathrm{c}, & \text { 其它 }\end{array}\right. \\\end{array}
$$
式中，$R_{\mathrm{GF}}$ (单位: $\mathrm{m}$ 或米) 和 $R_{\mathrm{MW}}$ (单位: $\mathrm{c}$ 或周) 分别为 $\mathrm{GF}$ 组合和 $\mathrm{MW}$ 组合周跳检验量的阈值；$E 、 R$ 分别为卫星高度角 (单位：度) 和观测值采样间隔 (单位：$s$ )。



### 1、detecs()



### 2、calCsThres()：计算 MW、GF 周跳检测阈值

$$
\begin{array}{l}\begin{array}{l}R_{\mathrm{GF}}(E, R)=\left\{\begin{array}{cc}(-1.0 / 15.0 \cdot E+2) \cdot b_{\mathrm{GF}}, & E \leq 15^{\circ} \\ b_{\mathrm{GF}}, & E>15^{\circ}\end{array}\right. \\ b_{\mathrm{GF}}(R)=\left\{\begin{array}{cc}0.05 \mathrm{~m}, & 0<R \leq 1 \mathrm{~s} \\ 0.1 / 20.0 \cdot R+0.05 \mathrm{~m}, & 1<R \leq 20 \mathrm{~s} \\ 0.15 \mathrm{~m}, & 20<R \leq 60 \mathrm{~s} \\ 0.25 \mathrm{~m}, & 60<R \leq 100 \mathrm{~s} \\ 0.35 \mathrm{~m}, & \text { 其它 }\end{array}\right.\end{array} \\ R_{\mathrm{MW}}(E, R)=\left\{\begin{array}{cc}(-0.1 \cdot E+3) \cdot b_{\mathrm{MW}}, & E \leq 20^{\circ} \\ b_{\mathrm{MW}}, & E>20^{\circ}\end{array}\right. \\ b_{\text {MW }}(R)=\left\{\begin{array}{cc}2.5 \mathrm{c}, & 0<R \leq 1 \mathrm{~s} \\ 2.5 / 20.0 \cdot R+2.5 \mathrm{c}, & 1<R \leq 20 \mathrm{~s} \\ 5.0 \mathrm{c}, & 20<R \leq 60 \mathrm{~s} \\ 7.5 \mathrm{c}, & \text { 其它 }\end{array}\right. \\\end{array}
$$

```c
extern int calCsThres(prcopt_t *opt, const double sample)
{
	int b=0;

	if (sample>0.0) {
		if (PPP_Glo.prcOpt_Ex.bUsed_gfCs==1&&fabs(PPP_Glo.prcOpt_Ex.csThresGF)<0.01) {
			if (sample<=1.0)        PPP_Glo.prcOpt_Ex.csThresGF=0.05;
			else if (sample<=20.0)  PPP_Glo.prcOpt_Ex.csThresGF=(0.10)/(20.0-0.0)*sample+0.05;
			else if (sample<=60.0)  PPP_Glo.prcOpt_Ex.csThresGF=0.15;
			else if (sample<=100.0) PPP_Glo.prcOpt_Ex.csThresGF=0.25;
			else                    PPP_Glo.prcOpt_Ex.csThresGF=0.35;

			b=1;
		}
		if (PPP_Glo.prcOpt_Ex.bUsed_mwCs==1&&fabs(PPP_Glo.prcOpt_Ex.csThresMW)<0.01) {
			if (sample<=1.0)        PPP_Glo.prcOpt_Ex.csThresMW=2.5;
			else if (sample<=20.0)  PPP_Glo.prcOpt_Ex.csThresMW=(2.5)/(20.0-0.0)*sample+2.5;
			else if (sample<=60.0)  PPP_Glo.prcOpt_Ex.csThresMW=5.0;
			else                    PPP_Glo.prcOpt_Ex.csThresMW=7.5;

			b=1;
		}

		return b;
	}
	else {
		//sample<=0.0
		PPP_Glo.prcOpt_Ex.csThresGF=0.15;
		PPP_Glo.prcOpt_Ex.csThresMW=5.0;
		b=0;
	}

	return b;
}
```



### 3、wlAmbMeas()







### 4、gfmeas()







### 5、detslp_mw()







### 6、detslp_gf()







## 三、模型改正

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

```c
static int model_phw(gtime_t time, int sat, const char *type, int opt,
                     const double *rs, const double *rr, double *phw)
{
    double exs[3],eys[3],ek[3],exr[3],eyr[3],eks[3],ekr[3],E[9];
    double dr[3],ds[3],drs[3],r[3],pos[3],cosp,ph;
    int i;
    
    if (opt<=0) return 1; /* no phase windup */
    
    // 首先调用 sat-yaw 函数，根据卫星的姿态模型计算出卫星本体坐标系 X,Y 方向的单位矢量exs、eys，即上面公式里的SX、SY
    /* satellite yaw attitude model */
    if (!sat_yaw(time,sat,type,opt,rs,exs,eys)) return 0;
    
    // 计算卫星至接收机的单位矢量
    /* unit vector satellite to receiver */
    for (i=0;i<3;i++) r[i]=rr[i]-rs[i];
    if (!normv3(r,ek)) return 0;
    
    // 计算接收机天线在当地坐标系的北向、西向单位矢量
    /* unit vectors of receiver antenna */
    ecef2pos(rr,pos);
    xyz2enu(pos,E);
    exr[0]= E[1]; exr[1]= E[4]; exr[2]= E[7]; /* x = north */
    eyr[0]=-E[0]; eyr[1]=-E[3]; eyr[2]=-E[6]; /* y = west  */
    
    // 根据公式以及前一次的相位缠绕误差计算当前时刻相位缠绕误差
    /* phase windup effect */
    cross3(ek,eys,eks);
    cross3(ek,eyr,ekr);
    for (i=0;i<3;i++) {
        ds[i]=exs[i]-ek[i]*dot(ek,exs,3)-eks[i];
        dr[i]=exr[i]-ek[i]*dot(ek,exr,3)+ekr[i];
    }
    cosp=dot(ds,dr,3)/norm(ds,3)/norm(dr,3);
    if      (cosp<-1.0) cosp=-1.0;
    else if (cosp> 1.0) cosp= 1.0;
    ph=acos(cosp)/2.0/PI;
    cross3(ds,dr,drs);
    if (dot(ek,drs,3)<0.0) ph=-ph;
    
    *phw=ph+floor(*phw-ph+0.5); /* in cycle */
    return 1;
}
```

#### sat_yaw()：





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











### 四、ppp_res()：残差计算、设计矩阵构建

![image-20231102142834322](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231102142834322.png)







## 五、PPP 时间更新

时间更新函数对理解 PPP 模型至关重要，



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


式中，$p_{i}$ 为观测量 $l_{i}$ 对应的权；$\tilde{v}_{i}$ 为标准化残差；$k_{0}$ 和 $k_{1}$ 为阈值常量，一般取 $k_{0}=1.0 \sim 1.5, \quad k_{1}=2.0 \sim 3.0$ 。























