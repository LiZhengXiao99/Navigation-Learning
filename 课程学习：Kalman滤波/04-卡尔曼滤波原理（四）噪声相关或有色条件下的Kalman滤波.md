[TOC]

## 一、系统噪声与量测噪声相关的Kalman滤波

### 1、模型

**函数模型**
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.
$$
**随机模型**
$$
\left\{\begin{array}{l}\mathrm{E}\left[\boldsymbol{W}_{k}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{W}_{j}^{\mathrm{T}}\right]=\boldsymbol{Q}_{k} \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{l}^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \delta_{k j} \\ {\color{red}\mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{S}_{k} \delta_{k j}}\end{array}\right.
$$

### 2、通过改写状态方程去相关

多了相关矩阵 $\boldsymbol{S}_{k} \delta_{k j}$，想假设相关就必须有这个相关矩阵。

> 表示这个矩阵挺困难的，所以实际处理时，即使有一定相关，也直接把它当成不相关处理。

从理论上分析，它们相关，可以通过对状态方程进行改写，处理成不相关形式：
$$
\begin{aligned} \boldsymbol{X}_{k} & =\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}+\boldsymbol{J}_{k-1}{\color{green}\left(\boldsymbol{Z}_{k-1}-\boldsymbol{H}_{k-1} \boldsymbol{X}_{k-1}-\boldsymbol{V}_{k-1}\right) }\\ & =\left(\boldsymbol{\Phi}_{k / k-1}-\boldsymbol{J}_{k-1} \boldsymbol{H}_{k-1}\right) \boldsymbol{X}_{k-1}+\boldsymbol{J}_{k-1} \boldsymbol{Z}_{k-1}+\left(\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}-\boldsymbol{J}_{k-1} \boldsymbol{V}_{k-1}\right) \\ & =\boldsymbol{\Phi}_{k / k-1}^{*} \boldsymbol{X}_{k-1}+{\color{brown}\boldsymbol{J}_{k-1} \boldsymbol{Z}_{k-1}}+\boldsymbol{W}_{k-1}^{*}\end{aligned}
$$
绿色部分实质上等于 $0$  ，$J_{k-1}$ 是一个待定系数矩阵。由此得到一个新的状态方程，此状态方程含有输入 ${\color{brown}\boldsymbol{J}_{k-1} \boldsymbol{Z}_{k-1}}$ ，即把上一时刻的量测当做已知的输入。现在的问题是：系数矩阵 $J_{k-1}$ 取什么值合适？

我们是希望通过对状态方程进行改写，处理成不相关形式，也就是说要找到一个系数矩阵 $J_{k-1}$ 使 $\mathrm{E}\left[\boldsymbol{W}_{k}^{*} \boldsymbol{V}_{j}^{\mathrm{T}}\right]$ 等于 $0$ ，即：
$$
\begin{array}{l}\mathrm{E}\left[\boldsymbol{W}_{k}^{*}\right]=\mathrm{E}\left[\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}-\boldsymbol{J}_{k-1} \boldsymbol{V}_{k-1}\right]=\mathbf{0} \\ \mathrm{E}\left[\boldsymbol{W}_{k}^{*}\left(\boldsymbol{W}_{j}^{*}\right)^{\mathrm{T}}\right]=\left(\boldsymbol{\Gamma}_{k} \boldsymbol{Q}_{k} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}+\boldsymbol{J}_{k} \boldsymbol{R}_{k} \boldsymbol{J}_{k}^{\mathrm{T}}-\boldsymbol{\Gamma}_{k} \boldsymbol{S}_{k} \boldsymbol{J}_{k}^{\mathrm{T}}-\boldsymbol{J}_{k} \boldsymbol{S}_{k}^{\mathrm{T}} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}\right) \boldsymbol{\delta}_{k j} \\ \mathrm{E}\left[\boldsymbol{W}_{k}^{*} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\left(\boldsymbol{\Gamma}_{k} \boldsymbol{S}_{k}-\boldsymbol{J}_{k} \boldsymbol{R}_{k}\right) \delta_{k j} \Longrightarrow {\color{red}\boldsymbol{J}_{k}=\boldsymbol{\Gamma}_{k} \boldsymbol{S}_{k} \boldsymbol{R}_{k}^{-1}}\end{array}
$$
把系数矩阵 $J_{k-1}$ 带回，得到新的系统噪声方差阵：
$$
\begin{aligned} \boldsymbol{Q}_{k}^{*} & =\mathrm{E}\left[\boldsymbol{W}_{k}^{*}\left(\boldsymbol{W}_{j}^{*}\right)^{\mathrm{T}}\right]=\left(\boldsymbol{\Gamma}_{k} \boldsymbol{Q}_{k} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}-\boldsymbol{J}_{k} \boldsymbol{S}_{k}^{\mathrm{T}} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}\right) \delta_{k j} \\ & =\left(\boldsymbol{\Gamma}_{k} \boldsymbol{Q}_{k} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}-\boldsymbol{J}_{k} \boldsymbol{R}_{k} \boldsymbol{R}_{k}^{-1} \boldsymbol{S}_{k}^{\mathrm{T}} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}\right) \boldsymbol{\delta}_{k j}=\left(\boldsymbol{\Gamma}_{k} \boldsymbol{Q}_{k} \boldsymbol{\Gamma}_{k}^{\mathrm{T}}-\boldsymbol{J}_{k} \boldsymbol{R}_{k} \boldsymbol{J}_{k}^{\mathrm{T}}\right) \delta_{k j}\end{aligned}
$$

### 3、噪声相关条件下Kalman滤波公式汇总

**新函数模型**
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1}^{*} \boldsymbol{X}_{k-1}+\boldsymbol{J}_{k-1} \boldsymbol{Z}_{k-1}+\boldsymbol{W}_{k-1}^{*} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.
$$
其中：
$$
\begin{array}{l}
\boldsymbol{J}_{k-1}=\boldsymbol{\Gamma}_{k-1} \boldsymbol{S}_{k-1} \boldsymbol{R}_{k-1}^{-1} \quad \\\boldsymbol{\Phi}_{k / k-1}^{*}=\boldsymbol{\Phi}_{k / k-1}-\boldsymbol{J}_{k-1} \boldsymbol{H}_{k-1} \\
\boldsymbol{Q}_{k-1}^{*}=\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}-\boldsymbol{J}_{k-1} \boldsymbol{R}_{k-1} \boldsymbol{J}_{k-1}^{\mathrm{T}}
\end{array}
$$
**新随机模型**
$$
\left\{\begin{array}{lc}\mathrm{E}\left[\boldsymbol{W}_{k}^{*}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{W}_{k}^{*}\left(\boldsymbol{W}_{j}^{*}\right)^{\mathrm{T}}\right]=\boldsymbol{Q}_{k}^{*} \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{W}_{k}^{*} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\mathbf{0} & \end{array}\right.
$$
**滤波公式**
$$
\left\{\begin{array}{l}
\hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1}^{*} \hat{\boldsymbol{X}}_{k-1}+\boldsymbol{J}_{k-1} \boldsymbol{Z}_{k-1} \\
\boldsymbol{P}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1}^{*} \boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{*}\right)^{\mathrm{T}}+\boldsymbol{Q}_{k-1}^{*} \\
\boldsymbol{K}_{k k}=\boldsymbol{P}_{k / k-1}^{\mathrm{T}} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} \\
\hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right) \\
\boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{k / k-1}
\end{array}\right.
$$

## 二、系统噪声有色时的Kalman滤波

> 有色噪声含义非常广，并不是任何有色噪声都可以处理，能建模成状态方程形式的有色噪声才能处理。

### 1、模型

**函数模型**
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.
$$
**将噪声分成白噪声 $W_{w,k}$ 和有色噪声 $W_{c,k}$** ：
$$
\boldsymbol{\Gamma}_{k} \boldsymbol{W}_{k}=\left[\begin{array}{ll}
\boldsymbol{\Gamma}_{w, k} & \boldsymbol{\Gamma}_{c, k}
\end{array}\right][\begin{array}{l}
\boldsymbol{W}_{w, k} \\
\boldsymbol{W}_{c, k}
\end{array}]
$$
当然，有色噪声和白噪声之间不相关：
$$
\mathrm{E}\left[\boldsymbol{W}_{w, k} \boldsymbol{W}_{w, j}^{\mathrm{T}}\right]=\boldsymbol{Q}_{w, k} \delta_{k j}, \quad \mathrm{E}\left[\boldsymbol{W}_{w, k} \boldsymbol{W}_{c, j}^{\mathrm{T}}\right]=\mathbf{0}
$$

### 2、将有色噪声建模成状态方程形式

当前时刻的有色噪声 $W_{c,k}$ 等于上一时刻的有色噪声 $W_{c,k-1}$ ，乘噪声的状态转移矩阵 ${\Pi}_{k / k-1}$ ，再加上一个新的系统白噪声 ${\zeta}_{k-1}$
$$
\boldsymbol{W}_{c, k}=\boldsymbol{\Pi}_{k / k-1} \boldsymbol{W}_{c, k-1}+\boldsymbol{\zeta}_{k-1}
$$
**状态增广新模型**：
$$
\left\{\begin{array}{l}{\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{W}_{c, k}\end{array}\right]=\left[\begin{array}{cc}\boldsymbol{\Phi}_{k / k-1} & \boldsymbol{\Gamma}_{c, k-1} \\ \mathbf{0} & \boldsymbol{\Pi}_{k / k-1}\end{array}\right]\left[\begin{array}{c}\boldsymbol{X}_{k-1} \\ \boldsymbol{W}_{c, k-1}\end{array}\right]+\left[\begin{array}{cc}\boldsymbol{\Gamma}_{w, k-1} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{I}\end{array}\right]\left[\begin{array}{c}\boldsymbol{W}_{w, k-1} \\ \boldsymbol{\zeta}_{k-1}\end{array}\right]} \\ \boldsymbol{Z}_{k}=\left[\begin{array}{ll}\boldsymbol{H}_{k} & \mathbf{0}\end{array}\right]\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{W}_{c, k}\end{array}\right]+\boldsymbol{V}_{k}\end{array}\right.
$$

### 3、系统噪声有色时的Kalman滤波公式汇总

**新模型**
$$
\left\{\begin{array}{l}{\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{W}_{c, k}\end{array}\right]=\left[\begin{array}{cc}\boldsymbol{\Phi}_{k / k-1} & \boldsymbol{\Gamma}_{c, k-1} \\ \mathbf{0} & \boldsymbol{\Pi}_{k / k-1}\end{array}\right]\left[\begin{array}{c}\boldsymbol{X}_{k-1} \\ \boldsymbol{W}_{c, k-1}\end{array}\right]+\left[\begin{array}{cc}\boldsymbol{\Gamma}_{w, k-1} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{I}\end{array}\right]\left[\begin{array}{c}\boldsymbol{W}_{w, k-1} \\ \zeta_{k-1}\end{array}\right]} \\ \boldsymbol{Z}_{k}=\left[\begin{array}{ll}\boldsymbol{H}_{k} & \mathbf{0}\end{array}\right]\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{W}_{c, k}\end{array}\right]+\boldsymbol{V}_{k}\end{array}\right.
$$
若简记：
$$
\boldsymbol{X}_{k}^{a}=\left[\begin{array}{c}
\boldsymbol{X}_{k} \\
\boldsymbol{W}_{c, k}
\end{array}\right], \boldsymbol{\Phi}_{k / k-1}^{a}=\left[\begin{array}{cc}
\boldsymbol{\Phi}_{k / k-1} & \boldsymbol{\Gamma}_{c, k-1} \\
\mathbf{0} & \boldsymbol{\Pi}_{k / k-1}
\end{array}\right], \boldsymbol{\Gamma}_{k}^{a}=\left[\begin{array}{cc}
\boldsymbol{\Gamma}_{w, k} & \mathbf{0} \\
\mathbf{0} & \boldsymbol{I}
\end{array}\right], \boldsymbol{W}_{k}^{a}=\left[\begin{array}{c}
\boldsymbol{W}_{w, k} \\
\boldsymbol{\zeta}_{k}
\end{array}\right], \boldsymbol{H}_{k}^{a}=\left[\begin{array}{ll}
\boldsymbol{H}_{k} & \mathbf{0}
\end{array}\right]
$$
则有：
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}^{a}=\boldsymbol{\Phi}_{k / k-1}^{a} \boldsymbol{X}_{k-1}^{a}+\boldsymbol{\Gamma}_{k-1}^{a} \boldsymbol{W}_{k-1}^{a} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k}^{a} \boldsymbol{X}_{k}^{a}+\boldsymbol{V}_{k}\end{array}\right.
$$
其中：
$$
\quad\left\{\begin{array}{l}\mathrm{E}\left[\boldsymbol{W}_{k}^{a}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{W}_{k}^{a}\left(\boldsymbol{W}_{j}^{a}\right)^{\mathrm{T}}\right]=\boldsymbol{Q}_{k}^{a} \delta_{k j}=\operatorname{diag}\left(\boldsymbol{Q}_{w, k} \boldsymbol{Q}_{c, k}\right) \delta_{k j} \\ \mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \boldsymbol{\delta}_{k j} \\ \mathrm{E}\left[\boldsymbol{W}_{j}^{a} \boldsymbol{V}_{k}^{\mathrm{T}}\right]=\mathbf{0}\end{array}\right.
$$
后面直接用Kalman滤波方程就可以了

## 三、量测噪声有色时的Kalman滤波方程

> GNSS量测噪声，考虑当前和上一历元有相关性，可以当做有色噪声处理

函数模型
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.
$$

### 1、状态增广法

将量测噪声分成有色的 ${V}_{c, k}$ 和无色的 ${V}_{w, k}$ 两部分，${V}_{c, k}$ 表示成状态方程：
$$
\begin{array}{l}
\boldsymbol{V}_{k}=\left[\begin{array}{ll}
\boldsymbol{\Theta}_{w, k}  \boldsymbol{\Theta}_{c, k}
\end{array}\right]\left[\begin{array}{l}
\boldsymbol{V}_{w, k} \\
\boldsymbol{V}_{c, k}
\end{array}\right] \\
 \mathrm{E}\left[\boldsymbol{V}_{w, k} \boldsymbol{V}_{w, j}^{\mathrm{T}}\right]=\boldsymbol{R}_{w, k} \delta_{k j}, \quad \mathrm{E}\left[\boldsymbol{V}_{w, k} \boldsymbol{V}_{c, j}^{\mathrm{T}}\right]=\mathbf{0} \\
 {\color{green}\boldsymbol{V}_{c, k}=\boldsymbol{\Psi}_{c, k / k-1} \boldsymbol{V}_{c, k-1}+\boldsymbol{\varsigma}_{k-1}} \quad \mathrm{E}\left[\boldsymbol{\varsigma}_{k}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{\varsigma}_{k} \boldsymbol{\varsigma}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{c, k} \boldsymbol{\delta}_{k j}
 \end{array}
$$
状态增广：
$$
\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{V}_{c, k-1}\end{array}\right]=\left[\begin{array}{cc}\boldsymbol{\Phi}_{k / k-1} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{\Psi}_{c, k-1 / k-2}\end{array}\right]\left[\begin{array}{c}\boldsymbol{X}_{k-1} \\ \boldsymbol{V}_{c, k-2}\end{array}\right]+\left[\begin{array}{cc}\boldsymbol{\Gamma}_{k-1} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{I}\end{array}\right]\left[\begin{array}{c}\boldsymbol{W}_{k-1} \\ \boldsymbol{\varsigma}_{k-2}\end{array}\right]
$$
新的量测方程为：
$$
\begin{aligned}
\boldsymbol{Z}_{k} & =\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{\Theta}_{w, k} \boldsymbol{V}_{w, k}+\boldsymbol{\Theta}_{c, k}\left(\boldsymbol{\Psi}_{c, k / k-1} \boldsymbol{V}_{c, k-1}+\varsigma_{k-1}\right) \\
& =\left[\begin{array}{ll}
\boldsymbol{H}_{k} & \boldsymbol{\Theta}_{c, k} \boldsymbol{\Psi}_{c, k / k-1}
\end{array}\right]\left[\begin{array}{c}
\boldsymbol{X}_{k} \\
\boldsymbol{V}_{c, k-1}
\end{array}\right]+\left(\boldsymbol{\Theta}_{w, k} \boldsymbol{V}_{w, k}+\boldsymbol{\Theta}_{c, k} \varsigma_{k-1}\right)
\end{aligned}
$$
若记：
$$
\boldsymbol{X}_{k}^{a}=\left[\begin{array}{c}\boldsymbol{X}_{k} \\ \boldsymbol{V}_{c, k-1}\end{array}\right], \boldsymbol{\Phi}_{k / k-1}^{a}=\left[\begin{array}{cc}\boldsymbol{\Phi}_{k / k-1} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{\Psi}_{c, k-1 / k-2}\end{array}\right], \boldsymbol{\Gamma}_{k}^{a}=\left[\begin{array}{cc}\boldsymbol{\Gamma}_{k} & \mathbf{0} \\ \mathbf{0} & \boldsymbol{I}\end{array}\right], \boldsymbol{W}_{k}^{a}=\left[\begin{array}{c}\boldsymbol{W}_{k} \\ \boldsymbol{\varsigma}_{k-1}\end{array}\right]\\
\boldsymbol{H}_{k}^{a}=\left[\begin{array}{ll}
\boldsymbol{H}_{k} & \boldsymbol{\Theta}_{c, k} \boldsymbol{\Psi}_{c, k / k-1}
\end{array}\right], \boldsymbol{V}_{k}^{a}=\boldsymbol{\Theta}_{w, k} \boldsymbol{V}_{w, k}+\boldsymbol{\Theta}_{c, k} \boldsymbol{\varsigma}_{k-1}
$$
得到新的函数模型：
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}^{a}=\boldsymbol{\Phi}_{k / k-1}^{a} \boldsymbol{X}_{k-1}^{a}+\boldsymbol{\Gamma}_{k-1}^{a} \boldsymbol{W}_{k-1}^{a} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k}^{a} \boldsymbol{X}_{k}^{a}+\boldsymbol{V}_{k}^{a}\end{array}\right.
$$
新的随机模型：
$$
\quad\left\{\begin{array}{l}\mathrm{E}\left[\boldsymbol{W}_{k}^{a}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{W}_{k}^{a}\left(\boldsymbol{W}_{j}^{a}\right)^{\mathrm{T}}\right]=\boldsymbol{Q}_{k}^{a} \delta_{k j}=\operatorname{diag}\left(\boldsymbol{Q}_{ k}\quad \boldsymbol{R}_{c, k-1}\right) \delta_{k j} \\ 
\mathrm{E}\left[\boldsymbol{V}_{k}^a\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{V}_{k}^a(\boldsymbol{V}_{j}^a)^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \boldsymbol{(Q_{w,k}R_{w,k}Q_{w,k}^T+Q_{c,k}R_{c,k}Q_{c,k}^T) \delta_{k j}} \\
\mathrm{E}\left[\boldsymbol{W}_{j}^{a} \boldsymbol{V}_{k}^{\mathrm{T}}\right]=\operatorname{diag}\left(0 \quad  \boldsymbol{R}_{c, k-1} \boldsymbol{Q}_{c, k}^T\right) \delta_{k j} \end{array}\right.
$$
得出的系统噪声和量测噪声相关，**之后还需要再去相关**。

### 2、量测求差法

> 如果前后时刻相关，两时刻间差分消去相关性

$$
\left\{\begin{array}{l}
\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\
\boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}
\end{array}\right.
$$

假设 ${\color{red}\boldsymbol{\psi}_{k / k-1}}$ 为相邻两时刻噪声转移矩阵，$\boldsymbol{\xi}_{k-1}$ 为量测的激励噪白声
$$
\boldsymbol{V}_{k}={\color{red}\boldsymbol{\psi}_{k / k-1}} \boldsymbol{V}_{k-1}+\boldsymbol{\xi}_{k-1}\\
\mathrm{E}\left[\boldsymbol{\xi}_{k}\right]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{\xi}_{k} \boldsymbol{\xi}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{\xi, k} \delta_{k j}
$$
前后相邻量测求差
$$
\begin{aligned}
\boldsymbol{Z}_{k}-{\color{red}\boldsymbol{\psi}_{k / k-1}} \boldsymbol{Z}_{k-1} & =\left[\boldsymbol{H}_{k}\left(\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right)+\left({\color{red}\boldsymbol{\psi}_{k \mid k-1}} \boldsymbol{V}_{k-1}+\boldsymbol{\xi}_{k-1}\right)\right]-{\color{red}\boldsymbol{\psi}_{k / k-1}}\left(\boldsymbol{H}_{k-1} \boldsymbol{X}_{k-1}+\boldsymbol{V}_{k-1}\right) \\
& =\left(\boldsymbol{H}_{k} \boldsymbol{\Phi}_{k / k-1}-\boldsymbol{\psi}_{k \mid k-1} \boldsymbol{H}_{k-1}\right) \boldsymbol{X}_{k-1}+\left(\boldsymbol{H}_{k} \boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}+\boldsymbol{\xi}_{k-1}\right)
\end{aligned}
$$
得到新的量测噪声 $\boldsymbol{H}_{k} \boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}+\boldsymbol{\xi}_{k-1}$ 为白噪声

简记
$$
\begin{array}{l}
\boldsymbol{Z}_{k}^{*}=\boldsymbol{Z}_{k}-\boldsymbol{\psi}_{k / k-1} \boldsymbol{Z}_{k-1}\\
 \boldsymbol{H}_{k}^{*}=\boldsymbol{H}_{k} \boldsymbol{\Phi}_{k \mid k-1}-\boldsymbol{\psi}_{k \mid k-1} \boldsymbol{H}_{k-1} \\
 \boldsymbol{V}_{k}^{*}=\boldsymbol{H}_{k} \boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}+\boldsymbol{\xi}_{k-1}\\
 \boldsymbol{X}_{k}^{*}=\boldsymbol{X}_{k-1}, \boldsymbol{\Phi}_{k / k-1}^{*}=\boldsymbol{\Phi}_{k-1 / k-2}, \boldsymbol{\Gamma}_{k}^{*}=\boldsymbol{\Gamma}_{k-1}, \boldsymbol{W}_{k}^{*}=\boldsymbol{W}_{k-1}
 \end{array}
$$
得到新的函数模型：
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}^{*}=\boldsymbol{\Phi}_{k / k-1}^{*} \boldsymbol{X}_{k-1}^{*}+\boldsymbol{\Gamma}_{k-1}^{*} \boldsymbol{W}_{k-1}^{*} \\ \boldsymbol{Z}_{k}^{*}=\boldsymbol{H}_{k}^{*} \boldsymbol{X}_{k}^{*}+\boldsymbol{V}_{k}^{*}\end{array}\right.
$$
新的随机模型：
$$
\quad\left\{\begin{array}{ll}\mathrm{E}\left[W_{k}^{*}\right]=0,\quad \mathrm{E}\left[W_{k}^{*}\left(W_{j}^{*}\right)^{\top}\right]=Q_{k-1} \delta_{k j} \\ \mathrm{E}\left[V_{k}^{*}\right]=0, \quad \mathrm{E}\left[V_{k}^{\top}\left(V_{j}^{*}\right)^{\mathrm{T}}\right]=\left(\boldsymbol{H}_{k} \Gamma_{k-1} Q_{k-1} I_{k-1}^{\top} H_{k}^{\top}+\boldsymbol{R}_{\xi k-1}\right) \delta_{k j} \\ {\color{red}\left.\mathrm{E}\left[W_{k}^{*}\left(V_{j}^{*}\right)^{\top}\right)^{\mathrm{T}}\right]=\boldsymbol{Q}_{k-1} I_{k-1}^{\mathrm{T}} \boldsymbol{H}_{k}^{\top} \delta_{k j}}\end{array}\right.
$$
得出的系统噪声和量测噪声相关，**之后还需要再去相关**。