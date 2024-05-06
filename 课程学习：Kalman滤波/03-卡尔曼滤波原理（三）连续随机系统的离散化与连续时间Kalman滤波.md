[TOC]
实际建模中给的可能是连续的状态方程、连续的量测方程；要用Kalman滤波得先进行离散化。

## 一、连续时间系统方程离散化

### 1、连续时间模型

$$
\dot{\boldsymbol{X}}(t)=\boldsymbol{F}(t) \boldsymbol{X}(t)+\boldsymbol{G}(t) \boldsymbol{w}(t) \quad \mathrm{E}[\boldsymbol{w}(t)]=0, \quad \mathrm{E}\left[\boldsymbol{w}(t) \boldsymbol{w}^{\mathrm{T}}(\tau)\right]=\boldsymbol{q}(t) \delta(t-\tau)
$$

> 连续时间的白噪声不太好理解，它处处连续，处处不可导，幅值无穷大，任何两个时间不相关，且是高斯白噪声

在 $t_{k-1}$ 到 $t_{k}$ 做积分，将 $t_{k-1}$ 到 $t_{k}$ 时间内连续的状态方程，转化为 $k-1$ 到 $k$ 两时刻间的状态方程，得到离散化形式：
$$
\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\eta}_{k-1}
$$
其中：
$$
\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{X}\left(t_{k}\right) \\ \boldsymbol{\Phi}_{k / k-1}=\boldsymbol{\Phi}\left(t_{k}, t_{k-1}\right) \approx \mathrm{e}^{\int_{t_{k-1}}^{t_{k}} F(\tau) \mathrm{d} \tau} \\ \boldsymbol{\eta}_{k-1}=\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{w}(\tau) \mathrm{d} \tau\end{array}
$$
### 2、状态转移矩阵计算

泰勒展开之后取常数项和一阶项；舍弃后面计算量大且收益小的高阶项，得：
$$
\boldsymbol{\Phi}_{k / k-1} \approx \mathrm{e}^{F\left(t_{k-1}\right) T_{s}}={\color{red}\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}}+\boldsymbol{F}^{2}\left(t_{k-1}\right) \frac{I_{s}^{2}}{\partial !}+\boldsymbol{F}^{3}\left(t_{k-1}\right) \frac{I_{s}}{\partial !}+\cdots \approx \boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}
$$
### 3、激励噪声的等效计算

$$
\boldsymbol{\eta}_{k-1}=\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{w}(\tau) \mathrm{d} \tau
$$
连续时间系统的噪声是高斯白噪声，经过线性变换（微积分也是线性变换），得到的还是高斯白噪声。想了解它，只要知道它的一阶矩和二阶矩就行了

**激励噪声均值**：
$$
\mathrm{E}\left[\boldsymbol{\eta}_{k-1}\right]=\mathrm{E}\left[\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{w}(\tau) \mathrm{d} \tau\right]=\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \mathrm{E}[\boldsymbol{w}(\tau)] \mathrm{d} \tau=\mathbf{0}
$$
**激励噪声协方差**：
$$
\mathrm{E}\left[\boldsymbol{\eta}_{k-1} \boldsymbol{\eta}_{j-1}^{\mathrm{T}}\right]=\mathbf{0} \quad k \neq j
$$

$$
\begin{aligned} \mathrm{E}\left[\boldsymbol{\eta}_{k-1} \boldsymbol{\eta}_{k-1}^{\mathrm{T}}\right] & =\mathrm{E}\left\{\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{w}(\tau) \mathrm{d} \tau \cdot\left[\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, s\right) \boldsymbol{G}(s) \boldsymbol{w}(s) \mathrm{d} s\right]^{\mathrm{T}}\right\} \quad k=j \\ & =\mathrm{E}\left[\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{w}(\tau) \int_{t_{k-1}}^{t_{k}} \boldsymbol{w}^{\mathrm{T}}(s) \boldsymbol{G}^{\mathrm{T}}(s) \boldsymbol{\Phi}^{\mathrm{T}}\left(t_{k}, s\right) \mathrm{d} s \mathrm{~d} \tau\right] \\ & =\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \int_{t_{k-1}}^{t_{k}} \mathrm{E}\left[\boldsymbol{w}(\tau) \boldsymbol{w}^{\mathrm{T}}(s)\right] \boldsymbol{G}^{\mathrm{T}}(s) \boldsymbol{\Phi}^{\mathrm{T}}\left(t_{k}, s\right) \mathrm{d} s \mathrm{~d} \tau \\ & =\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \int_{t_{k-1}}^{t_{k}} \boldsymbol{q}(\tau) \delta(\tau-s) \boldsymbol{G}^{\mathrm{T}}(s) \boldsymbol{\Phi}^{\mathrm{T}}\left(t_{k}, s\right) \mathrm{d} s \mathrm{~d} \tau \\ & =\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{q}(\tau) \boldsymbol{G}^{\mathrm{T}}(\tau) \boldsymbol{\Phi}^{\mathrm{T}}\left(t_{k}, \tau\right) \mathrm{d} \tau\end{aligned}
$$

想往下算，得做一些假设，在 $t_{k-1}$ 到 $t_k$ 这一小段时间内，$\boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau)$ 都为常值，时不变；不是常值可以取一阶项
$$
\begin{aligned} \mathrm{E} & {\left[\boldsymbol{\eta}_{k-1} \boldsymbol{\eta}_{k-1}^{\mathrm{T}}\right]=\int_{t_{k-1}}^{t_{k}} \boldsymbol{\Phi}\left(t_{k}, \tau\right) \boldsymbol{G}(\tau) \boldsymbol{q}(\tau) \boldsymbol{G}^{\mathrm{T}}(\tau) \boldsymbol{\Phi}^{\mathrm{T}}\left(t_{k}, \tau\right) \mathrm{d} \tau } \\ \approx & \int_{t_{k-1}}^{t_{k}}\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right)\left(t_{k}-\tau\right)\right] \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right)\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right)\left(t_{k}-\tau\right)\right]^{\mathrm{T}} \mathrm{d} \tau \\ = & \int_{t_{k-1}}^{t_{k}} \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q} \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \mathrm{d} \tau+\int_{t_{k-1}}^{t_{k}} \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right)\left(t_{k}-\tau\right) \mathrm{d} \tau \\ & \quad+\int_{t_{k-1}}^{t_{k}} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right)\left(t_{k}-\tau\right) \mathrm{d} \tau+\int_{t_{k-1}}^{t_{k}} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right)\left(t_{k}-\tau\right)^{2} \mathrm{~d} \tau \\ = & \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}+\frac{1}{2} \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}^{2} \\ & \quad+\frac{1}{2} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}^{2}+\frac{1}{3} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}^{3} \\ = & {\left[\boldsymbol{I}+\frac{1}{2} \boldsymbol{F}\left(t_{k-1}\right) T_{s}\right] \cdot\left[\boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) T T_{s}\right] \cdot\left[\boldsymbol{I}+\frac{1}{2} \boldsymbol{F}\left(t_{k-1}\right) T_{s}\right]^{\mathrm{T}}+\frac{1}{12} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}^{3} } \\ \approx & \left\{\left[\boldsymbol{I}+\frac{1}{2} \boldsymbol{F}\left(t_{k-1}\right) T_{s}\right] \boldsymbol{G}\left(t_{k-1}\right)\right\} \cdot\left[\boldsymbol{q}\left(t_{k-1}\right) T_{s}\right] \cdot\left\{\left[\boldsymbol{I}+\frac{1}{2} \boldsymbol{F}\left(t_{k-1}\right) T_{s}\right] \boldsymbol{G}\left(t_{k-1}\right)\right\}^{\mathrm{T}}\end{aligned}
$$
如果时间极短，还可以再简化：
$$
\mathrm{E}\left[\boldsymbol{\eta}_{k-1} \boldsymbol{\eta}_{k-1}^{\mathrm{T}}\right] \approx \boldsymbol{G}\left(t_{k-1}\right) \cdot\left[\boldsymbol{q}\left(t_{k-1}\right) T_{s}\right] \cdot \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) \quad \boldsymbol{F}\left(t_{k-1}\right) T_{s} \ll \boldsymbol{I}
$$
### 4、最终离散化结论

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/22ff41fce26d46deb90a6c3aaf872ee5.png)



或等价表示为：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/d7521c9337a44609b66b5178688f2d65.png)



两者都满足：
$$
\mathrm{E}\left[\left(\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right)\left(\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right)^{\mathrm{T}}\right]=\boldsymbol{G}\left(t_{k-1}\right) \cdot\left[\boldsymbol{q}\left(t_{k-1}\right) T_{s}\right] \cdot \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right)=\mathrm{E}\left[\left(\boldsymbol{\Gamma}_{k-1}^{\prime} \boldsymbol{W}_{k-1}^{\prime}\right)\left(\boldsymbol{\Gamma}_{k-1}^{\prime} \boldsymbol{W}_{k-1}^{\prime}\right)^{\mathrm{T}}\right]
$$

### 5、常见简单随机过程离散化

1.一阶马尔可夫：$\dot{X}(t)=-\beta X(t)+w(t) \quad X_{k}=a_{1} X_{k-1}+W_{k-1}$

2.二阶马尔可夫：$\ddot{X}(t)=-2 \beta \dot{X}(t)-\beta^{2} X(t)+w(t) \quad X_{k}=a_{1} X_{k-1}+a_{2} X_{k-2}+W_{k-1}$

3.随机游走：$\dot{X}(t)=w(t) \quad X_{k}=X_{k-1}+W_{k-1}$

4.随机常值：$\dot{X}(t)=0 \quad X_{k}=X_{0}$

### 6、实际物理信号的噪声单位

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f7b4d9b17ba845c087b1275afd729620.png)



* $\tau$ 表示相关时间，很多时候等于 $1$，所以 $-1 / \tau$ 可能被省略 
* $\sqrt{q}-\mathrm{U} / \mathrm{s} / \sqrt{Hz } \mathrm{}$ 为激励噪声密度，描述激励噪声本身。$\mathrm{U} / \sqrt{ S}$ 随机游走噪声系数，是想描述 $X$ 的噪声。描述的主体不同。比如惯导中有：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/9148093d86cb4fe58de65fc8f75e779c.png)



## 二、连续时间量测方程离散化

> 连续量测方程的只是理论上的抽象，没有实际意义

**连续量测模型**：
$$
\boldsymbol{Z}(t)=\boldsymbol{H}(t) \boldsymbol{X}(t)+\boldsymbol{v}(t) \quad \mathrm{E}[\boldsymbol{v}(t)]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{v}(t) \boldsymbol{v}^{\mathrm{T}}(\tau)\right]=\boldsymbol{r}(t) \delta(t-\tau)
$$
在 $t_{k-1}$ 到 $t_{k}$ 时间段，两边积分平均：
$$
\frac{1}{T_{s}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{Z}(\tau) \mathrm{d} \tau=\frac{1}{T_{v}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{H}(\tau) \boldsymbol{X}(\tau)+\boldsymbol{v}(\tau) \mathrm{d} \tau=\frac{1}{T_{v}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{H}(\tau) \boldsymbol{X}(\tau) \mathrm{d} \tau+\frac{1}{T_{s}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{v}(\tau) \mathrm{d} \tau
$$
离散化记为：$\boldsymbol{Z}_{k} \approx \boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}$

**观测噪声均值**：
$$
\mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathrm{E}\left[\frac{1}{T_{s}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{v}(\tau) \mathrm{d} \tau\right]=\frac{1}{T_{s}} \int_{t_{k-1}}^{t_{k}} \mathrm{E}[\boldsymbol{v}(\tau)] \mathrm{d} \tau=\mathbf{0}
$$
**观测噪声协方差**：
$$
\begin{aligned} \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right] & =\mathrm{E}\left[\left(\frac{1}{T_{s}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{v}(\tau) \mathrm{d} \tau\right)\left(\frac{1}{T_{s}} \int_{t_{j-1}}^{t_{j}} \boldsymbol{v}(s) \mathrm{d} s\right)^{\mathrm{T}}\right] \\ & =\frac{1}{T_{s}^{2}} \int_{t_{k-1}}^{t_{k}} \int_{t_{j-1}}^{t_{j}} \mathrm{E}\left[\boldsymbol{v}(\tau) \boldsymbol{v}^{\mathrm{T}}(s)\right] \mathrm{d} s \mathrm{~d} \tau=\frac{1}{T_{s}^{2}} \int_{t_{k-1}}^{t_{k}} \int_{t_{j-1}}^{t_{j}} \boldsymbol{r}(\tau) \delta(s-\tau) \mathrm{d} s \mathrm{~d} \\ & =\frac{1}{T_{s}^{2}} \int_{t_{k-1}}^{t_{k}} \boldsymbol{r}(\tau) \delta_{k j} \mathrm{~d} \tau \approx \frac{\boldsymbol{r}\left(t_{k}\right)}{T_{s}} \delta_{k j} \triangleq \boldsymbol{R}_{k} \delta_{k j}\end{aligned}
$$
离散化量测间隔越小，等效出来的量测方差 $\boldsymbol{R}_{k}$ 越大

## 三、连续时间Kalman滤波

连续的函数可以认为是离散函数，但离散间隔 $t_{k-1}$ 到 $t_{k}$ 趋于 $0$ ；推导不是太严格，但比较直观。

### 1、连续状态空间模型

**函数模型**：
$$
\left\{\begin{array}{l}\dot{\boldsymbol{X}}(t)=\boldsymbol{F}(t) \boldsymbol{X}(t)+\boldsymbol{G}(t) \boldsymbol{w}(t) \\ \boldsymbol{Z}(t)=\boldsymbol{H}(t) \boldsymbol{X}(t)+\boldsymbol{v}(t)\end{array}\right.
$$
**随机模型**：
$$
\left\{\begin{array}{l}\mathrm{E}[\boldsymbol{w}(t)]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{w}(t) \boldsymbol{w}^{\mathrm{T}}(\tau)\right]=\boldsymbol{q}(t) \delta(t-\tau) \\ \mathrm{E}[\boldsymbol{v}(t)]=\mathbf{0}, \quad \mathrm{E}\left[\boldsymbol{v}(t) \boldsymbol{v}^{\mathrm{T}}(\tau)\right]=\boldsymbol{r}(t) \delta(t-\tau) \\ \mathrm{E}\left[\boldsymbol{w}(t) \boldsymbol{v}^{\mathrm{T}}(\tau)\right]=\mathbf{0}\end{array}\right.
$$

### 2、离散时间Kalman滤波

1. 状态一步预测：$\hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{k-1}$
2. 状态一步预测均方误差：$\boldsymbol{P}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}$
3. 滤波增益：$\boldsymbol{K}_{k}=\boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1}$
4. 状态估计：$\hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k / k-1}\right)$
5. 状态估计均方误差：$\boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{k / k-1}$

### 3、增益矩阵的连续化

将离散方程里的量测噪声 $\boldsymbol{R}_{k}$ 换成连续方程里的 $\frac{\boldsymbol{r}\left(t_{k}\right)}{T_{s}}$ 得：
$$
\boldsymbol{K}_{k}=\boldsymbol{P}_{k} \boldsymbol{H}_{k}^{\mathrm{T}} {\color{red}\boldsymbol{R}_{k}^{-1}}=\boldsymbol{P}_{k} \boldsymbol{H}_{k}^{\mathrm{T}}{\color{red}\left[\frac{\boldsymbol{r}\left(t_{k}\right)}{T_{s}}\right]}^{-1}={\color{red}T_{s} }\boldsymbol{P}_{k} \boldsymbol{H}_{k}^{\mathrm{T}} {\color{red}\boldsymbol{r}^{-1}\left(t_{k}\right)}
$$
将 $T_s$ 除到左边，求 $T_s$ 趋于 $0$ 的极限：
$$
\boldsymbol{K}(t)=\lim \limits_{T_{s} \rightarrow 0} \frac{\boldsymbol{K}_{k}}{T_{s}}=\lim \limits_{T_{s} \rightarrow 0} \boldsymbol{P}_{k} \boldsymbol{H}_{k}^{\mathrm{T}} {\color{red}\boldsymbol{r}^{-1}\left(t_{k}\right)}=\boldsymbol{P}(t) \boldsymbol{H}^{\mathrm{T}}(t) {\color{red}\boldsymbol{r}^{-1}(t)}
$$
即离散方程的增益 ${K}_{k}$ 除以时间间隔 $T_s$ 记成连续方程的增益；如果 $T_s$ 趋于 $0$ ,${K}_{k}$ 也趋于 0，时间很短，预测很小，量测的修正自然也很小。

### 4、状态估计的连续化

将一阶的状态转移矩阵带入：
$$
\begin{aligned} \hat{\boldsymbol{X}}_{k} & ={\color{green}\boldsymbol{\Phi}_{k / k-1} }\hat{\boldsymbol{X}}_{k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} {\color{green}\boldsymbol{\Phi}_{k / k-1}} \hat{\boldsymbol{X}}_{k-1}\right) \\ & ={\color{green}\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}\right]} \hat{\boldsymbol{X}}_{k-1}+\boldsymbol{K}_{k}\left\{\boldsymbol{Z}_{k}-\boldsymbol{H}_{k}{\color{green}\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}\right]} \hat{\boldsymbol{X}}_{k-1}\right\} \\ & ={\color{red}\hat{\boldsymbol{X}}_{k-1}}+\boldsymbol{F}\left(t_{k-1}\right) \hat{\boldsymbol{X}}_{k-1} T_{s}+\boldsymbol{K}_{k}\left[\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k-1}-\boldsymbol{H}_{k} \boldsymbol{F}\left(t_{k-1}\right) \hat{\boldsymbol{X}}_{k-1} T_{s}\right]\end{aligned}
$$
将 ${\color{red}\hat{\boldsymbol{X}}_{k-1}}$ 减到左边， $k$ 时刻的值，减去 $k-1$ 时刻的值，除以 $T_s$，求 $T_s$ 趋于 $0$ 的极限，就是连续的方程：
$$
\begin{aligned} \dot{\hat{\boldsymbol{X}}}(t) & =\lim \limits_{T_{s} \rightarrow 0} \frac{\boldsymbol{X}_{k}-\boldsymbol{X}_{k-1}}{T_{s}}=\lim \limits_{T_{s} \rightarrow 0} \boldsymbol{F}\left(t_{k-1}\right) \hat{\boldsymbol{X}}_{k-1}+\frac{\boldsymbol{K}_{k}}{T_{s}}\left[\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{k-1}-\boldsymbol{H}_{k} \boldsymbol{F}\left(t_{k-1}\right) \hat{\boldsymbol{X}}_{k-1} T_{s}\right] \\ & =\boldsymbol{F}(t) \hat{\boldsymbol{X}}(t)+\boldsymbol{K}(t)[\boldsymbol{Z}(t)-\boldsymbol{H}(t) \hat{\boldsymbol{X}}(t)]\end{aligned}
$$

### 5、均方差阵的连续化

将一阶的状态转移矩阵带入：
$$
\begin{aligned} \boldsymbol{P}_{k} & =\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right)\left({\color{green}\boldsymbol{\Phi}_{k / k-1}} \boldsymbol{P}_{k-1} {\color{green}\boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}}\right) \\ & =\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right)\left\{{\color{green}\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}\right]} \boldsymbol{P}_{k-1}{\color{green}\left[\boldsymbol{I}+\boldsymbol{F}\left(t_{k-1}\right) T_{s}\right]^{\mathrm{T}}}+\boldsymbol{G}\left(t_{k-1}\right) \cdot \boldsymbol{q}\left(t_{k-1}\right) T_{s} \cdot \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right)\right\} \\ & ={\color{red}\boldsymbol{P}_{k-1}}+\boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{P}_{k-1} T_{s}+\boldsymbol{P}_{k-1} \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}+\boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right) T_{s}+O\left(T_{s}^{2}\right)-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\left[{\boldsymbol{P}_{k-1}}+O\left(T_{s}\right)\right]\end{aligned}
$$
同理，将 ${\color{red}P_{k-1}}$ 减到左边， $k$  时刻的值，减去 $k-1$ 时刻的值，除以 $T_s$，求 $T_s$ 趋于 $0$ 的极限，就是连续的方程：
$$
\begin{aligned} \dot{\boldsymbol{P}}(t) & =\lim \limits_{T_{s} \rightarrow 0} \frac{\boldsymbol{P}_{k}-\boldsymbol{P}_{k-1}}{T_{s}}=\lim \limits_{T_{s} \rightarrow 0} \boldsymbol{F}\left(t_{k-1}\right) \boldsymbol{P}_{k-1}+\boldsymbol{P}_{k-1} \boldsymbol{F}^{\mathrm{T}}\left(t_{k-1}\right)+\boldsymbol{G}\left(t_{k-1}\right) \boldsymbol{q}\left(t_{k-1}\right) \boldsymbol{G}^{\mathrm{T}}\left(t_{k-1}\right)-\frac{\boldsymbol{K}_{k}}{T_{s}} \boldsymbol{H}_{k} \boldsymbol{P}_{k-1} \\ & =\boldsymbol{F}(t) \boldsymbol{P}(t)+\boldsymbol{P}(t) \boldsymbol{F}^{\mathrm{T}}(t)+\boldsymbol{G}(t) \boldsymbol{q}(t) \boldsymbol{G}^{\mathrm{T}}(t)-\boldsymbol{K}(t) \boldsymbol{H}(t) \boldsymbol{P}(t) \\ & =\boldsymbol{F}(t) \boldsymbol{P}(t)+\boldsymbol{P}(t) \boldsymbol{F}^{\mathrm{T}}(t)-\boldsymbol{K}(t) \boldsymbol{r}(t) \boldsymbol{K}^{\mathrm{T}}(t)+\boldsymbol{G}(t) \boldsymbol{q}(t) \boldsymbol{G}^{\mathrm{T}}(t)\end{aligned}
$$

此方程也称：矩阵黎卡蒂（Riccati）方程，下面简单介绍一下Riccati方程：

**标量Riccati方程**：

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/25ee6776be9844718d766f7721bc9a7e.png)


**矩阵Riccati方程**：
$$
\dot{\boldsymbol{P}}(t)=\boldsymbol{F}(t) \boldsymbol{P}(t)+\boldsymbol{P}(t) \boldsymbol{F}^{\mathrm{T}}(t)-\boldsymbol{P}(t) \boldsymbol{H}^{\mathrm{T}}(t) \boldsymbol{r}^{-1}(t) \boldsymbol{H}(t) \boldsymbol{P}(t)+\boldsymbol{G}(t) \boldsymbol{q}(t) \boldsymbol{G}^{\mathrm{T}}(t)
$$
Kalman滤波里的 $P$ 一定是对称阵，转置还是本身，所以都没写

简记为：$\dot{\boldsymbol{P}}=\boldsymbol{F P}+\boldsymbol{P F} \boldsymbol{F}^{\mathrm{T}}-\boldsymbol{P} \boldsymbol{R P}+\boldsymbol{Q}$

它的解等价于 (化非线性问题为线性求解)：
$$
\boldsymbol{P}=\boldsymbol{Y} \boldsymbol{D}^{-1} \quad \text { 其中 } \quad\left[\begin{array}{c}
 \dot{\boldsymbol{Y}} \\
 \dot{\boldsymbol{D}}
 \end{array}\right]=\left[\begin{array}{cc}
 \boldsymbol{F} & \boldsymbol{Q} \\
 \boldsymbol{R} & -\boldsymbol{F}^{\mathrm{T}}
 \end{array}\right]\left[\begin{array}{l}
 \boldsymbol{Y} \\
 \boldsymbol{D}
 \end{array}\right]
$$
 > 验证如下：
 > $$
 > \begin{aligned} \boldsymbol{Y} & =\boldsymbol{P D} \\ \dot{\boldsymbol{Y}} & =\dot{\boldsymbol{P}} \boldsymbol{D}+\boldsymbol{P} \dot{\boldsymbol{D}} \\ \dot{\boldsymbol{P}} & =\dot{\boldsymbol{Y}} \boldsymbol{D}^{-1}-\boldsymbol{P} \dot{\boldsymbol{D}} \boldsymbol{D}^{-1}=(\boldsymbol{F} \boldsymbol{Y}+\boldsymbol{Q D}) \boldsymbol{D}^{-1}-\boldsymbol{P}\left(\boldsymbol{R} \boldsymbol{Y}-\boldsymbol{F}^{\mathrm{T}} \boldsymbol{D}\right) \boldsymbol{D}^{-1} \\ & =\boldsymbol{F} \boldsymbol{Y} \boldsymbol{D}^{-1}+\boldsymbol{Q}-\boldsymbol{P} \boldsymbol{R} \boldsymbol{Y} \boldsymbol{D}^{-1}+\boldsymbol{P} \boldsymbol{F}^{\mathrm{T}}=\boldsymbol{F P}+\boldsymbol{Q}-\boldsymbol{P} \boldsymbol{R} \boldsymbol{P}+\boldsymbol{P} \boldsymbol{F}^{\mathrm{T}}\end{aligned}
 > $$
 >

但线性化后此矩阵微分方程没有初等解，只有毕卡积分级数解：

 > 时变状态/矩阵方程 $\dot{\boldsymbol{X}}(t)=\boldsymbol{F}(t) \boldsymbol{X}(t)$ 的毕卡积分级数解：
 >
 >
 > 上面方程两边积分得：
 > $$
 > \boldsymbol{X}(t)  =\boldsymbol{X}(0)+\int_{0}^{t} \boldsymbol{F}(\tau) {\color{red}\boldsymbol{X}(\tau)} \mathrm{d} \tau
 > $$
 > 但此方程右边还含有未知量，迭代一次，将  ${\color{red}\boldsymbol{X}(\tau)}$ 替换成上式
 > $$
 > \boldsymbol{X}(t)  =\boldsymbol{X}(0)+\int_{0}^{t} \boldsymbol{F}(\tau)\left[\boldsymbol{X}(0)+\int_{0}^{\tau} \boldsymbol{F}\left(\tau_{1}\right) {\color{red}\boldsymbol{X}\left(\tau_{1}\right) }\mathrm{d} \tau_{1}\right] \mathrm{d} \tau
 > $$
 > 里面又含未知变量 ${\color{red}\boldsymbol{X}\left(\tau_{1}\right) }$ ，再进行替换。反复迭代，一直存在未知变量，最后就变成无穷级数，而且还是无穷重积分
 > $$
 > \begin{aligned}
 > & =\boldsymbol{X}(0)+\int_{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau \boldsymbol{X}(0)+\int_{0}^{t} \boldsymbol{F}(\tau) \int_{0}^{\tau} \boldsymbol{F}\left(\tau_{1}\right) \boldsymbol{X}\left(\tau_{1}\right) \mathrm{d} \tau_{1} \mathrm{~d} \tau=\cdots \\
 > & =\left[\boldsymbol{I}+\int_{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau+\int_{0}^{t} \boldsymbol{F}(\tau) \int_{0}^{\tau} \boldsymbol{F}\left(\tau_{1}\right) \mathrm{d} \tau_{1} \mathrm{~d} \tau+\int_{0}^{t} \boldsymbol{F}(\tau) \int_{0}^{\tau} \boldsymbol{F}\left(\tau_{1}\right) \int_{0}^{\tau_{1}} \boldsymbol{F}\left(\tau_{2}\right) \mathrm{d} \tau_{2} \mathrm{~d} \tau_{1} \mathrm{~d} \tau+\cdots\right] \boldsymbol{X}(0) \\
 > \end{aligned}
 > $$
 > 前面括号内的矩阵就是从 $0$ 时刻转移到 $t$ 时刻的状态转移矩阵：
 > $$
 > \boldsymbol{\Phi}(t, 0)  =\boldsymbol{I}+\int{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau+\int{0}^{t} \boldsymbol{F}(\tau) \int{0}^{\tau} \boldsymbol{F}\left(\tau{1}\right) \mathrm{d} \tau{1} \mathrm{~d} \tau+\int{0}^{t} \boldsymbol{F}(\tau) \int{0}^{\tau} \boldsymbol{F}\left(\tau{1}\right) \int{0}^{\tau{1}} \boldsymbol{F}\left(\tau{2}\right) \mathrm{d} \tau{2} \mathrm{~d} \tau_{1} \mathrm{~d} \tau+\cdots
 > $$
 >
 > >可以省略高阶项进行近似，或者有时候高阶项就为 $0$ 
 >
 > * 一般情况下不可交换 $\boldsymbol{F}(\tau) \boldsymbol{F}\left(\tau_{1}\right) \neq \boldsymbol{F}\left(\tau_{1}\right) \boldsymbol{F}(\tau)$, 上式即为最终毕卡级数解;
 >
 > * 可交换时 $\boldsymbol{F}(\tau) \boldsymbol{F}\left(\tau_{1}\right)=\boldsymbol{F}\left(\tau_{1}\right) \boldsymbol{F}(\tau)$ (特殊如常值 $\left.\boldsymbol{F}(\tau)=\boldsymbol{F}\right)$, 才有闭合解:
 >
 >   可交换时，多重积分等于单重积分的 $n$ 次方，除以 $n$ 的阶乘分之一
 >
 > $$
 > \begin{array}{l}
 >  \int_{0}^{\tau_{1}} \boldsymbol{F}\left(\tau_{1}\right) \cdots \int_{0}^{\tau_{n}} \boldsymbol{F}\left(\tau_{n}\right) \mathrm{d} \tau_{n} \cdots \mathrm{d} \tau_{1}=\frac{1}{n !}\left[\int_{0}^{\tau_{1}} \boldsymbol{F}(\tau) \mathrm{d} \tau_{1}\right]^{n} \\
 >  \boldsymbol{\Phi}(t, 0)=\boldsymbol{I}+\frac{1}{1 !} \int_{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau+\frac{1}{2 !}\left[\int_{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau\right]^{2}+\frac{1}{3 !}\left[\int_{0}^{t} \boldsymbol{F}(\tau) \mathrm{d} \tau\right]^{3}+\cdots=\mathrm{e}^{\int_{0}^{t} F(\tau) \mathrm{d} \tau} 40
 >  \end{array}
 > $$
### 6、连续时间Kalman滤波方程汇总

$$
\begin{array}{l}\boldsymbol{K}(t)=\boldsymbol{P}(t) \boldsymbol{H}^{\mathrm{T}}(t) \boldsymbol{r}^{-1}(t) \ \dot{\hat{\boldsymbol{X}}}(t)=\boldsymbol{F}(t) \hat{\boldsymbol{X}}(t)+\boldsymbol{K}(t)[\boldsymbol{Z}(t)-\boldsymbol{H}(t) \hat{\boldsymbol{X}}(t)] \ \dot{\boldsymbol{P}}(t)=\boldsymbol{F}(t) \boldsymbol{P}(t)+\boldsymbol{P}(t) \boldsymbol{F}^{\mathrm{T}}(t)-\boldsymbol{K}(t) \boldsymbol{r}(t) \boldsymbol{K}^{\mathrm{T}}(t)+\boldsymbol{G}(t) \boldsymbol{q}(t) \boldsymbol{G}^{\mathrm{T}}(t) \end{array}
$$

连续时间Kalman滤波方程中的状态估计和方差都是微分方程，相求结果得解微分方程。

> 离散时间Kalman滤波方程中的状态估计和方差可以认为是差分方程。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/092f12b0dec44e8e8407ac91f4be6309.png)



> 与确定性系统的状态观测器非常像
>
> ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/27cfb60e8ecb48aa9bf9efcd157be727.png)