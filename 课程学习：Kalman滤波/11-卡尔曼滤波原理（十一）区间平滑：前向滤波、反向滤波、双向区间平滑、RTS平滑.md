最优预测、估计与平滑之间的关系：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f19c5ae8d6e141f2871ddf61224efe4b.png" alt="[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-kZC8JtPg-1686234324707)(卡尔曼滤波与组合导航原理（十一）区间平滑.assets/1686229884025.png)]" style="zoom:67%;" />


三种平滑方式：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/8a347802fce04134a7106b7afc078975.png" style="zoom: 80%;" />


**函数模型和随机模型**
$$
\left\{\begin{array} { l } 
{ \boldsymbol { X } _ { k } = \boldsymbol { \Phi } _ { k / k - 1 } \boldsymbol { X } _ { k - 1 } + \boldsymbol { \Gamma } _ { k - 1 } \boldsymbol { W } _ { k - 1 } } \\
{ \boldsymbol { Z } _ { k } = \boldsymbol { H } _ { k } \boldsymbol { X } _ { k } + \boldsymbol { V } _ { k } }
\end{array} \quad \left\{\begin{array}{ll}
\mathrm{E}\left[\boldsymbol{W}_{k}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{W}_{j}^{\mathrm{T}}\right]=\boldsymbol{Q}_{k} \delta_{k j} \\
\mathrm{E}\left[\boldsymbol{V}_{k}\right]=\mathbf{0}, & \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\boldsymbol{R}_{k} \delta_{k j} \\
\mathrm{E}\left[\boldsymbol{W}_{k} \boldsymbol{V}_{j}^{\mathrm{T}}\right]=\mathbf{0} &
\end{array}\right.\right.
$$
将量测序列分成两段：$\bar{Z}_{M}=\underbrace{\left[\boldsymbol{Z}_{1} \boldsymbol{Z}_{2} \cdots \boldsymbol{Z}_{j}\right.}_{\overline{\boldsymbol{Z}}_{\mathrm{1} \cdot j}} \underbrace{\mathbf{Z}_{j+1} \boldsymbol{Z}_{j+2} \cdots \boldsymbol{Z}_{M}}_{\bar{Z}_{j+1+M}}]^{\mathrm{T}}$ 

> 先看固定点平滑，固定区间平滑只是将点的范围扩大了，将点前后平移即可

### 1.正向滤波（forward）

通过前面一段做正向滤波，就是普通的Kalman滤波，下标都加了 $f$ 表示正向，：
$$
\left\{\begin{array}{l}
\hat{\boldsymbol{X}}_{f, k / k-1}=\boldsymbol{\Phi}_{k / k-1} \hat{\boldsymbol{X}}_{f, k-1} \\
\boldsymbol{P}_{f, k / k-1}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{f, k-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \\
\boldsymbol{K}_{f, k}=\boldsymbol{P}_{f, k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{f, k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} \quad k=1,2, \cdots, j \\
\hat{\boldsymbol{X}}_{f, k}=\hat{\boldsymbol{X}}_{f, k / k-1}+\boldsymbol{K}_{f, k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{f, k / k-1}\right) \\
\boldsymbol{P}_{f, k}=\left(\boldsymbol{I}-\boldsymbol{K}_{f, k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{f, k / k-1}
\end{array}\right.
$$
求得 $j$ 时刻估计 $\hat{\boldsymbol{X}}_{f, j}, \boldsymbol{P}_{f, j}$ 

### 2.反向滤波（backward）

从后往前推，需要对Kalman滤波模型改写：把状态转移矩阵变为求逆的形式，表示由后时刻预测前时刻：
$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}={\color{red}\boldsymbol{\Phi}_{k / k-1}} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array} \Longrightarrow\left\{\begin{array}{l}\boldsymbol{X}_{k}={\color{red}\boldsymbol{\Phi}_{k+1 / k}^{-1}} \boldsymbol{X}_{k+1}-{\color{red}\boldsymbol{\Phi}_{k+1 / k}^{-1}} \boldsymbol{\Gamma}_{k} \boldsymbol{W}_{k} \\ \boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.\right.
$$
令 $\boldsymbol{\Phi}_{k+1 / k}^{*}=\boldsymbol{\Phi}_{k+1 / k}^{-1}$ ，$\boldsymbol{\Gamma}_{k}^{*}=-\boldsymbol{\Phi}_{k+1 / k}^{-1} \boldsymbol{\Gamma}_{k}$ ，$\boldsymbol{W}_{k+1}^{*}=\boldsymbol{W}_{k}$ ，得新的函数模型：
$$
\left\{\begin{array}{l}
\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k+1 / k}^{*} \boldsymbol{X}_{k+1}+\boldsymbol{\Gamma}_{k}^{*} \boldsymbol{W}_{k+1}^{*} \\
\boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}
\end{array}\right.
$$
对新的函数模型做Kalman滤波：
$$
\left\{\begin{array}{l}
\hat{\boldsymbol{X}}_{b, k / k+1}=\boldsymbol{\Phi}_{k / k+1}^{*} \hat{\boldsymbol{X}}_{b, k+1} \\
\boldsymbol{P}_{b, k / k+1}=\boldsymbol{\Phi}_{k / k+1}^{*} \boldsymbol{P}_{b, k+1}\left(\boldsymbol{\Phi}_{k / k+1}^{*}\right)^{\mathrm{T}}+\boldsymbol{\Gamma}_{k}^{*} \boldsymbol{Q}_{k} \boldsymbol{\Gamma}_{k}^{*} \\
\boldsymbol{K}_{b, k}=\boldsymbol{P}_{b, k / k+1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{b, k / k+1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} \quad k=M-1, M-2, \cdots, j+1 \\
\hat{\boldsymbol{X}}_{b, k}=\hat{\boldsymbol{X}}_{b, k / k+1}+\boldsymbol{K}_{b, k}\left(\boldsymbol{Z}_{k}-\boldsymbol{H}_{k} \hat{\boldsymbol{X}}_{b, k / k+1}\right) \\
\boldsymbol{P}_{b, k}=\left(\boldsymbol{I}-\boldsymbol{K}_{b, k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{b, k / k+1}
\end{array}\right.
$$
求得 $j$ 时刻反向一步预测 $\hat{\boldsymbol{X}}_{b, j / j+1}, \boldsymbol{P}_{b, j / j+1}$ 

### 3、j 时刻固定点信息融合(smoothing)

将利用前面观测值正向滤波得到的观测值 $\hat{\boldsymbol{X}}_{f, j}, \boldsymbol{P}_{f, j}$ ，和反向滤波得到的观测值 $\hat{\boldsymbol{X}}_{b, j / j+1}, \boldsymbol{P}_{b, j / j+1}$ 做信息融合（加权平均），并且认为两段滤波的结果不相关：
$$
\begin{array}{c}
\left\{\begin{array} { l } 
{ \hat { \boldsymbol { X } } _ { f , j } = \boldsymbol { X } _ { j } + \boldsymbol { \Delta } _ { f , j } } \\
{ \hat { \boldsymbol { X } } _ { b , j / j + 1 } = \boldsymbol { X } _ { j } + \boldsymbol { \Delta } _ { b , j / j + 1 } }
\end{array} \quad \left\{\begin{array}{l}
\boldsymbol{\Delta}_{f, j} \sim \mathrm{N}\left(\mathbf{0}, \boldsymbol{P}_{f, j}\right), \\
\boldsymbol{\Delta}_{b, j / j+1} \sim \mathrm{N}\left(\mathbf{0}, \boldsymbol{P}_{b, j / j+1}\right), \\
\operatorname{cov}\left(\boldsymbol{U}_{f, j} \boldsymbol{\Delta}_{b, j / j+1}^{\mathrm{T}}\right)=\mathbf{0}
\end{array}\right.\right. \\
\Longrightarrow\left\{\begin{array}{l}
\boldsymbol{P}_{s, j}=\left(\boldsymbol{P}_{f, j}^{-1}+\boldsymbol{P}_{b, j / j+1}^{-1}\right)^{-1} \\
\hat{\boldsymbol{X}}_{s, j}=\boldsymbol{P}_{s, j}\left(\boldsymbol{P}_{b, j / j+1}^{-1} \hat{\boldsymbol{X}}_{f, j}+\boldsymbol{P}_{f, j}^{-1} \hat{\boldsymbol{X}}_{b, j / j+1}\right)
\end{array}\right.
\end{array}
$$

### 4、基于正反向滤波的区间平滑

1. 由前往后**正向滤波**, 获得并存储 $\hat{\boldsymbol{X}}_{f, j}, \boldsymbol{P}_{f, j}(j=1,2, \cdots, M)$;
2. 由后往前**反向滤波**, 获得 $\hat{\boldsymbol{X}}_{b, j / j+1}, \boldsymbol{P}_{b, j / j+1}$, 融合获得 $\hat{\boldsymbol{X}}_{s, j}, \boldsymbol{P}_{s, j}$;
3. 当 $(j=M-1, M-2, \cdots, 1)$ 完成整个**区间平滑**。

### 5、RTS区间平滑算法

> H.**R**auch, F.**T**ung, C.**S**triebel, 1965 ，推导比较复杂

不需要时间更新，先由前往后正向滤波, 获得并存储 $\boldsymbol{\Phi}_{j / j-1}^{\mathrm{T}}, \hat{\boldsymbol{X}}_{f, j / j-1}$, $\boldsymbol{P}_{f, j / j-1}, \hat{\boldsymbol{X}}_{f, j}, \boldsymbol{P}_{f, j}(j=1,2, \cdots, M)$ 再按由后往前的量测顺序执行如下RTS算法：
$$
\left\{\begin{array}{ll}\boldsymbol{K}_{s, k}=\boldsymbol{P}_{f, k} \boldsymbol{\Phi}_{k+1 / k}^{\mathrm{T}} \boldsymbol{P}_{f, k+1 / k}^{-1} & \text { 初值 } \hat{\boldsymbol{X}}_{s, M}=\hat{\boldsymbol{X}}_{f, M}, \boldsymbol{P}_{s, M}=\boldsymbol{P}_{f, M} \\ \hat{\boldsymbol{X}}_{s, k}=\hat{\boldsymbol{X}}_{f, k}+\boldsymbol{K}_{s, k}\left(\hat{\boldsymbol{X}}_{s, k+1}-\hat{\boldsymbol{X}}_{f, k+1 / k}\right) & k=M-1, M-2, \cdots, 1 \\ \boldsymbol{P}_{s, k}=\boldsymbol{P}_{f, k}+\boldsymbol{K}_{s, k}\left(\boldsymbol{P}_{s, k+1}-\boldsymbol{P}_{f, k+1 / k}\right) \boldsymbol{K}_{s, k}^{\mathrm{T}} & \end{array}\right.
$$
计算量和正反向滤波的区间平滑差不多，存储量增加不少

### 6、平滑精度比较

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/51dfacc488584ff39fb482710ad71461.png" alt="[外链图片转存失败,源站可能有防盗链机制,建议将图片保存下来直接上传(img-U8jzdNUC-1686234324709)(卡尔曼滤波与组合导航原理（十一）区间平滑.assets/1686232739636.png)]" style="zoom: 67%;" />

* 可平滑性问题：受系统噪声影响的状态才具可平滑性。随机常数就没有可平滑性。
* 滤波要存的数据量比较大，可能几个G，工程上双向滤波+ P 阵对角线加权平均，可有效降低存储量。

我的理解，RTS和双向滤波等价，比单向滤波精度高，全高斯白噪声理想情况精度高一倍；双向滤波是从前往后算一遍，存下状态向量、协方差，再从后往前算一遍，前后的结果取加权平均；RTS也是先从前往后算一遍，存的量更多，但从后往前算可以直接出平滑后的结果。