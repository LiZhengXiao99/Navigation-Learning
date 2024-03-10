> 量测维数很高，而且能写成很多分量，每一个分量可以看成一个小量测，可以序贯进行量测更新
>
> * 优点是：计算快，数字稳定性更好，我们知道矩阵求逆是和维数的三次方成正比，分成小矩阵求逆快（都分解成一维的，求逆就变为求导）
> * 缺点是：如果量测相关且每一时刻变化，每一时刻要做 Cholesky分解。（其实很多时候量测噪声都是定常数矩阵，只要在启动的时候做一次分解就行）

函数模型
$$
\left\{\begin{array}{l}
\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\
\boldsymbol{Z}_{k}=\boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}
\end{array}\right.
$$
其中
$$
\begin{array}{l}
{\left[\begin{array}{c}
\boldsymbol{Z}_{k}^{(1)} \\
\boldsymbol{Z}_{k}^{(2)} \\
\vdots \\
\boldsymbol{Z}_{k}^{(N)}
\end{array}\right]=\left[\begin{array}{c}
\boldsymbol{H}_{k}^{(1)} \\
\boldsymbol{H}_{k}^{(2)} \\
\vdots \\
\boldsymbol{H}_{k}^{(N)}
\end{array}\right] \boldsymbol{X}_{k}+\left[\begin{array}{c}
\boldsymbol{V}_{k}^{(1)} \\
\boldsymbol{V}_{k}^{(2)} \\
\vdots \\
\boldsymbol{V}_{k}^{(N)}
\end{array}\right]} \\
\boldsymbol{R}_{k}=\left[\begin{array}{cccc}
\boldsymbol{R}_{k}^{(1)} & & & \\
& \boldsymbol{R}_{k}^{(2)} & & \\
& & \ddots & \\
& & & \boldsymbol{R}_{k}^{(N)}
\end{array}\right]
\end{array}
$$
序贯滤波执行框图：

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/8bfcc790b07647528cbbe1fc60312a9f.png)


如果历元间量测噪声相关，量测噪声不是对角阵，它总可以作平方根分解（Cholesky分解）
$$
\boldsymbol{R}_{k}=\boldsymbol{L}_{k} \boldsymbol{L}_{k}^{\mathrm{T}}
$$
使用 $\boldsymbol{L}_{k}^{-1}$ 同时左乘量测方程两边, 得：
$$
\boldsymbol{L}_{k}^{-1} \boldsymbol{Z}_{k}=\boldsymbol{L}_{k}^{-1} \boldsymbol{H}_{k} \boldsymbol{X}_{k}+\boldsymbol{L}_{k}^{-1} \boldsymbol{V}_{k}
$$
得到新的量测方程：
$$
\boldsymbol{Z}_{k}^{*}=\boldsymbol{H}_{k}^{*} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}^{*}
$$
其中：
$$
\quad \boldsymbol{Z}_{k}^{*}=\boldsymbol{L}_{k}^{-1} \boldsymbol{Z}_{k}, \quad \boldsymbol{H}_{k}^{*}=\boldsymbol{L}_{k}^{-1} \boldsymbol{H}_{k}, \quad \boldsymbol{V}_{k}^{*}=\boldsymbol{L}_{k}^{-1} \boldsymbol{V}_{k} 
$$
从而可以将量测方差阵对角化（单位阵！）：
$$
\begin{aligned}
\boldsymbol{R}_{k}^{*} & =\mathrm{E}\left[\boldsymbol{V}_{k}^{*}\left(\boldsymbol{V}_{k}^{*}\right)^{\mathrm{T}}\right]=\mathrm{E}\left[\left(\boldsymbol{L}_{k}^{-1} \boldsymbol{V}_{k}\right)\left(\boldsymbol{L}_{k}^{-1} \boldsymbol{V}_{k}\right)^{\mathrm{T}}\right] \\
& =\boldsymbol{L}_{k}^{-1} \mathrm{E}\left[\boldsymbol{V}_{k} \boldsymbol{V}_{k}^{\mathrm{T}}\right]\left(\boldsymbol{L}_{k}^{-1}\right)^{\mathrm{T}}=\boldsymbol{L}_{k}^{-1} \boldsymbol{R}_{k}\left(\boldsymbol{L}_{k}^{-1}\right)^{\mathrm{T}}=\boldsymbol{I}
\end{aligned}
$$
满足序贯滤波模型 $\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k l k-1} \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}^{*}=\boldsymbol{H}_{k}^{*} \boldsymbol{X}_{k}+\boldsymbol{V}_{k}^{*}\end{array}\right.$  