[TOC]

## 一、信息滤波

### 1、模型

函数模型
$$
\left\{\begin{array}{l}
\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\
\boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}
\end{array}\right.
$$
随机模型
$$
\left\{\begin{array}{lr}  \mathrm{E}\left[\boldsymbol{W}_{k}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{W}_{j}^{\mathrm{T}}\right]=\boldsymbol{Q}_{k} \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\mathbf{0} & \end{array}\right.
$$

> 选择合适的系统噪声分配矩阵 $\boldsymbol{\Gamma}_{k}$，总可以保证系统噪声方差阵正定，$\boldsymbol{Q}_{k}>0$

"信息"的含义：信息和方差是互逆的，即 $I_k=p_k^{-1}$ ；估计越准，方差越小，信息量越大：
$$
\begin{array}{ll}\boldsymbol{P}_{k}=\mathrm{E}\left[\left(\boldsymbol{X}_{k}-\hat{\boldsymbol{X}}_{k}\right)\left(\boldsymbol{X}_{k}-\hat{\boldsymbol{X}}_{k}\right)^{\mathrm{T}}\right] \\ \hat{\boldsymbol{X}}_{k}-\boldsymbol{X}_{k} \rightarrow 0 ， \boldsymbol{P}_{k} \rightarrow \mathbf{0}\left(\boldsymbol{P}_{k}^{-1} \rightarrow \infty\right) \\ \hat{\boldsymbol{X}}_{k}-\boldsymbol{X}_{k} \rightarrow \infty ， \boldsymbol{P}_{k} \rightarrow \infty\left(\boldsymbol{P}_{k}^{-1} \rightarrow \mathbf{0}\right)\end{array}
$$
用 $I_k$ 替换原始Kalman滤波中的 $p_k^{-1}$ ，得信息滤波：
$$
\mathrm{KF}\left\{\begin{array}{l}\boldsymbol{P}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{F}_{k-1}^{\mathrm{T}} \\ \boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{k / k-1} \\ \boldsymbol{K}_{k}=\boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} \\ \hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1} \\ \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right)\end{array} \Longrightarrow \mathrm{IKF}\left\{\begin{array}{l}\boldsymbol{I}_{k / k-1}=\left(\boldsymbol{\Phi}_{k / k-1} \boldsymbol{I}_{k-1}^{-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right)^{-1} \\ \boldsymbol{I}_{k}=\boldsymbol{I}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k} \\ \boldsymbol{K}_{k}=\boldsymbol{I}_{k}^{-1} \boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \\ \hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1} \\ \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right)\end{array}\right.\right.
$$

> 两个问题：
>
> * 右边公式缺点是相比左边公式求逆公式更多。
> * 滤波开始时，对初始参数 $X_0$ 的状态一无所知，方差  $P_0$ 应该取无穷大，无穷大不好表示，且无穷大分之一等于 $0$ ，两边的公式都无法执行。

### 2、信息滤波公式改写

针对这种情况对右边的公式修改：

**信息预测改写**

提取出 $\color{green}M_{k-1}$ ，红色部分用矩阵求逆引理得：

> $$
> \begin{array}{l}\left(A_{11}-A_{12} A_{22}^{-1} A_{21}\right)^{-1}=A_{11}^{-1}+A_{11}^{-1} A_{12}\left(A_{22}-A_{21} A_{11}^{-1} A_{12}\right)^{-1} A_{21} A_{11}^{-1}  \\=\left[I+A_{11}^{-1} A_{12}\left(A_{22}-A_{21} A_{11}^{-1} A_{12}\right)^{-1} A_{21}\right] A_{11}^{-1}\end{array}
> $$
>

$$
\begin{aligned} \boldsymbol{I}_{k / k-1} & =\left(\boldsymbol{\Phi}_{k / k-1} \boldsymbol{I}_{k-1}^{-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right)^{-1} \quad {\color{green}\boldsymbol{M}_{k-1}=\boldsymbol{\Phi}_{k / k-1}^{-\mathrm{T}} \boldsymbol{I}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{-1}} \\ & \left.={\color{red}\left(\boldsymbol{I}+\boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right)^{-1}} \boldsymbol{M}_{k-1} \\=[I-\boldsymbol{\boldsymbol { M } _ { k - 1 }} \boldsymbol{\Gamma}_{k-1}\left(\boldsymbol{Q}_{k-1}^{-1}+\boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1}\right)^{-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right] \boldsymbol{M}_{k-1} \\ & =\left(\boldsymbol{I}-\hat{N}_{k-1}\right) \boldsymbol{M}_{k-1} \end{aligned}
$$

信息矩阵的更新就无需求逆了。

**量测更新改写**

将卡尔曼滤波量测更新改写为预测和量测加权平均的形式：
$$
\begin{aligned} \hat{\boldsymbol{X}}_{k} & =\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right) \\ & =\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right) \hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k} \boldsymbol{Z}_{k} \\ & =\boldsymbol{P}_{k} \boldsymbol{P}_{k / k-1}^{-1} \hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{P}_{k} \boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}\end{aligned}
$$
两边除以 $P_k$ 得：
$$
\boldsymbol{P}_{k}^{-1} \hat{\boldsymbol{X}}_{k}=\boldsymbol{P}_{k / k-1}^{-1} \hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}
$$
用 $I_k$ 替换 $p_k^{-1}$ ，将 $\boldsymbol{I}_{k} \hat{\boldsymbol{X}}_{k}$ 记为 $\hat{\boldsymbol{S}}_{k}$，得：
$$
\hat{\boldsymbol{S}}_{k}=\boldsymbol{I}_{k} \hat{\boldsymbol{X}}_{k}=\boldsymbol{I}_{k / k-1} \hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}=\hat{\boldsymbol{S}}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}
$$
$S_{k-1}$ 到 $S_k$ 的递推为：
$$
\begin{array}{l}
\hat{\boldsymbol{S}}_{k / k-1}=\boldsymbol{I}_{k / k-1} \hat{\boldsymbol{X}}_{k / k-1}=\left(\boldsymbol{I}-\boldsymbol{N}_{k-1}\right) \boldsymbol{M}_{k-1} \boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1} \\
=\left(\boldsymbol{I}-\boldsymbol{N}_{k-1}\right) \boldsymbol{\Phi}_{k / k-1}^{-\mathrm{T}} \boldsymbol{I}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{-1} \boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1}=\left(\boldsymbol{I}-\boldsymbol{N}_{k-1}\right) \boldsymbol{\Phi}_{k / k-1}^{-\mathrm{T}} \hat{\boldsymbol{S}}_{k-1} \\
\end{array}
$$

### 3、IKF公式汇总

滤波流程转换成了 $S$ 和 $I$ 的递推：$\hat{\boldsymbol{S}}_{k-1}, \boldsymbol{I}_{k-1}, \boldsymbol{Z}_{k} \longrightarrow \hat{\boldsymbol{S}}_{k}, \boldsymbol{I}_{k}$
$$
\left\{\begin{array}{l}\boldsymbol{I}_{k / k-1}=\left(\boldsymbol{\Phi}_{k / k-1} \boldsymbol{I}_{k-1}^{-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right)^{-1} \\ \boldsymbol{I}_{k}=\boldsymbol{I}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k} \\ \boldsymbol{K}_{k}=\boldsymbol{I}_{k}^{-1} \boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \\ \hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1} \\ \hat{\boldsymbol{X}}_{k-1}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right)\end{array} \quad \Longrightarrow\left\{\begin{array}{l}\boldsymbol{M}_{k-1}=\boldsymbol{\Phi}_{k / k-1}^{-\mathrm{T}} \boldsymbol{I}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{-1} \\ \left.\boldsymbol{N}_{k-1}=\boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1}\left(\boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1}+\boldsymbol{Q}_{k-1}^{-1}\right)^{-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right) \\ \boldsymbol{I}_{k / k-1}=\left(\boldsymbol{I}-\boldsymbol{N}_{k-1}\right) \boldsymbol{M}_{k-1} \\ \boldsymbol{I}_{k}=\boldsymbol{I}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k} \\ \hat{\boldsymbol{S}}_{k / k-1}=\left(\boldsymbol{I}-\boldsymbol{N}_{k-1}\right) \boldsymbol{\Phi}_{k / k-1}^{-\mathrm{T}} \hat{\boldsymbol{S}}_{k-1} \\ \hat{\boldsymbol{S}}_{k}=\hat{\boldsymbol{S}}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}\end{array}\right.\right.
$$

* $I_0=0$ 可以实现计算了，解决了初始方差阵无穷大的问题。
* 矩阵求逆也变少了，状态转移矩阵的求逆可以认为没有：$\phi=I+FT_S \Longrightarrow \phi^{-1}=I-FT_S$
* 最终输出：$\left\{\begin{array}{l}\boldsymbol{I}_{k}=\boldsymbol{P}_{k}^{-1} \\ \hat{\boldsymbol{S}}_{k}=\boldsymbol{I}_{k} \hat{\boldsymbol{X}}_{k}\end{array} \Rightarrow\left\{\begin{array}{l}\boldsymbol{P}_{k}=\boldsymbol{I}_{k}^{-1} \\ \hat{\boldsymbol{X}}_{k}=\boldsymbol{P}_{k} \hat{\boldsymbol{S}}_{k}\end{array}\right.\right.$，在前几次滤波递推中，可能 $I_k$ 不可逆，得不到均值方差。

### 4、KF与IKF的对偶关系

$$
\begin{array}{l}\mathrm{KF}:\left\{\begin{array}{l}\boldsymbol{P}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \\ \boldsymbol{P}_{k}=\left[\boldsymbol{I}-\boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} \boldsymbol{H}_{k}\right] \boldsymbol{P}_{k / k-1}\end{array}\right. \\ \text { IKF }:\left\{\begin{array}{l}\boldsymbol{I}_{k / k-1}=\left[\boldsymbol{I}-\boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1}\left(\boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \boldsymbol{M}_{k-1} \boldsymbol{\Gamma}_{k-1}+\boldsymbol{Q}_{k-1}^{-1}\right)^{-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right] \boldsymbol{M}_{k-1} \\ \boldsymbol{I}_{k=}=\boldsymbol{I}_{k / k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\end{array}\right.\end{array}
$$

* KF时间更新使状态估计误差**增大**，IKF量测更新使信息量**增大**；
* KF量测更新使状态估计误差**减小**，IKF时间更新使信息量**减小**；
* 系统噪声使状态估计误差**增大**，使状态估计信息量**减小**；
* 量测噪声使状态估计误差**减小**，使状态估计信息量**增大**。

## 二、信息融合

### 1、信息融合方法

可以看成特殊 N 组量测最小二乘问题，H是单位阵，从很多方面对参数进行量测，每一个量测有一个噪声，所有噪声之间不相关。需要将这些信息融合，得到最优的参数估计。
$$
\left\{\begin{array}{lrr}\boldsymbol{X}^{(1)}=\boldsymbol{X}+\boldsymbol{V}_{1} & \\ \boldsymbol{X}^{(2)}=\boldsymbol{X}+\boldsymbol{V}_{2} & \mathrm{E}\left[\boldsymbol{V}_{i}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{V}_{i} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{P}_{i} \delta_{i j} \quad(i, j=1,2, \cdots, N) \\ \vdots & &\boldsymbol{P}_{i}>0 \quad i, j \text { 表示信息来源/渠道 } \\ \boldsymbol{X}^{(N)}=\boldsymbol{X}+\boldsymbol{V}_{N} & \end{array}\right.
$$
可以用递推最小二乘RLS解决，也可以用信息递推最小二乘IRLS解决：
$$
\begin{array}{l}\operatorname{RLS}\left\{\begin{array}{l}\boldsymbol{P}_{k}^{-1}=\boldsymbol{P}_{k-1}^{-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k} \\ \boldsymbol{P}_{k}^{-1} \hat{\boldsymbol{X}}_{k}=\boldsymbol{P}_{k-1}^{-1} \hat{\boldsymbol{X}}_{k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}\end{array}\right. \\ \text { IRLS }\left\{\begin{array}{l}\boldsymbol{I}_{k}=\boldsymbol{I}_{k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k} \\ \hat{\boldsymbol{S}}_{k}=\hat{\boldsymbol{S}}_{k-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}\end{array}\right.\end{array}
$$
一直往前递推，得到总/全局信息量级状态估计：
$$
\begin{aligned} \boldsymbol{I}_{g} & =\boldsymbol{I}_{N-1}+\boldsymbol{P}_{N}^{-1} & \hat{\boldsymbol{S}}_{g} & =\hat{\boldsymbol{S}}_{N-1}+\boldsymbol{P}_{N}^{-1} \hat{\boldsymbol{X}}^{(N)} \\ & =\left(\boldsymbol{I}_{N-2}+\boldsymbol{P}_{N-1}^{-1}\right)+\boldsymbol{P}_{N}^{-1} & & =\left(\hat{\boldsymbol{S}}_{N-2}+\boldsymbol{P}_{N-1}^{-1} \hat{\boldsymbol{X}}^{(N-1)}\right)+\boldsymbol{P}_{N}^{-1} \hat{\boldsymbol{X}}^{(N)} \\ & =\quad \cdots & & \cdots \\ & =\left(\boldsymbol{I}_{1}+\boldsymbol{P}_{2}^{-1}\right)+\cdots+\boldsymbol{P}_{N-1}^{-1}+\boldsymbol{P}_{N}^{-1} & & =\left(\hat{\boldsymbol{S}}_{1}+\boldsymbol{P}_{2}^{-1} \hat{\boldsymbol{X}}^{(2)}\right)+\cdots+\boldsymbol{P}_{N-1}^{-1} \hat{\boldsymbol{X}}^{(N-1)}+\boldsymbol{P}_{N}^{-1} \hat{\boldsymbol{X}}^{(N)} \\ & =\boldsymbol{P}_{1}^{-1}+\boldsymbol{P}_{2}^{-1}+\cdots+\boldsymbol{P}_{N-1}^{-1}+\boldsymbol{P}_{N}^{-1} & & =\boldsymbol{P}_{1}^{-1} \hat{\boldsymbol{X}}^{(1)}+\boldsymbol{P}_{2}^{-1} \hat{\boldsymbol{X}}^{(2)}+\cdots+\boldsymbol{P}_{N-1}^{-1} \hat{\boldsymbol{X}}^{(N-1)}+\boldsymbol{P}_{N}^{-1} \hat{\boldsymbol{X}}^{(N)}\end{aligned}
$$
即信息融合公式：
$$
\begin{array}{l}
\boldsymbol{I}_{g}=\boldsymbol{P}_{1}^{-1}+\boldsymbol{P}_{2}^{-1}+\cdots+\boldsymbol{P}_{N-1}^{-1}+\boldsymbol{P}_{N}^{-1} \\
\hat{\boldsymbol{S}}_{g}=\boldsymbol{P}_{1}^{-1} \hat{\boldsymbol{X}}^{(1)}+\boldsymbol{P}_{2}^{-1} \hat{\boldsymbol{X}}^{(2)}+\cdots+\boldsymbol{P}_{N-1}^{-1} \hat{\boldsymbol{X}}^{(N-1)}+\boldsymbol{P}_{N}^{-1} \hat{\boldsymbol{X}}^{(N)} \\
\Longrightarrow\left\{\begin{array} { l } 
{ \boldsymbol { P } _ { g } ^ { - 1 } = \sum _ { k = 1 } ^ { N } \boldsymbol { P } _ { k } ^ { - 1 } } \\
{ \boldsymbol { P } _ { g } ^ { - 1 } \hat { \boldsymbol { X } } _ { g } = \sum _ { k = 1 } ^ { N } \boldsymbol { P } _ { k } ^ { - 1 } \hat { \boldsymbol { X } } ^ { ( k ) } }
\end{array} \Longrightarrow \left\{\begin{array}{l}
\boldsymbol{P}_{g}=\left(\sum_{k=1}^{N} \boldsymbol{P}_{k}^{-1}\right)^{-1} \\
\hat{\boldsymbol{X}}_{g}=\boldsymbol{P}_{g} \sum_{k=1}^{N} \boldsymbol{P}_{k}^{-1} \hat{\boldsymbol{X}}^{(k)}
\end{array}\right.\right.
\end{array}
$$
本质上就是加权平均，特别的，当 $N=2$ 时有
$$
\begin{array}{l}
\boldsymbol{P}_{\text {fusion }}=\left(\boldsymbol{P}_{1}^{-1}+\boldsymbol{P}_{2}^{-1}\right)^{-1} \\
\hat{\boldsymbol{X}}_{\text {fusion }}=\boldsymbol{P}_{g}\left(\boldsymbol{P}_{1}^{-1} \boldsymbol{X}^{(1)}+\boldsymbol{P}_{2}^{-1} \boldsymbol{X}^{(2)}\right)=\frac{\boldsymbol{P}_{2}}{\boldsymbol{P}_{1}+\boldsymbol{P}_{2}} \boldsymbol{X}^{(1)}+\frac{\boldsymbol{P}_{1}}{\boldsymbol{P}_{1}+\boldsymbol{P}_{2}} \boldsymbol{X}^{(2)}
\end{array}
$$

### 2、信息融合推导Kalman滤波

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/63d198b0e14549a19ee29a1518deee93.png)


**状态估计融合**
$$
\begin{aligned}
\hat{\boldsymbol{X}}_{k} & =\frac{\left(\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\right)^{-1}}{\boldsymbol{P}_{k / k-1}+\left(\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\right)^{-1}} \underline{\boldsymbol{\Phi}_{k / k-1}} \hat{\boldsymbol{X}}_{k-1}+\frac{\boldsymbol{P}_{k / k-1}}{\boldsymbol{P}_{k / k-1}+\left(\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\right)^{-1}} \underline{\left(\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\right)^{-1} \boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{Z}_{k}} \\
\end{aligned}
$$
**均方差阵融合**
$$
\boldsymbol{P}_{k}=\left[\boldsymbol{P}_{k / k-1}^{-1}+\left(\boldsymbol{P}_{k}^{\prime}\right)^{-1}\right]^{-1}=\left(\boldsymbol{P}_{k / k-1}^{-1}+\boldsymbol{H}_{k}^{\mathrm{T}} \boldsymbol{R}_{k}^{-1} \boldsymbol{H}_{k}\right)^{-1}
$$