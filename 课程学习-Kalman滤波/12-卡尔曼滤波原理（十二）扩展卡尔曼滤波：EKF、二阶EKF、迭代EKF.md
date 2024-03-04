[TOC]

## 一、多元向量的泰勒级数展开
$$
\left\{\begin{array}{c}
y_{1}=f_{1}(\boldsymbol{X})=f_{1}\left(x_{1}, x_{2}, \cdots x_{n}\right) \\
y_{2}=f_{2}(\boldsymbol{X})=f_{2}\left(x_{1}, x_{2}, \cdots x_{n}\right) \\
\quad \vdots \\
y_{m}=f_{m}(\boldsymbol{X})=f_{m}\left(x_{1}, x_{2}, \cdots x_{n}\right)
\end{array}\right.
$$

上面线性方差组，写成向量形式就是：
$$
\boldsymbol{Y} = \boldsymbol{f} \left( \boldsymbol{X}\right)
$$
泰勒展开式为：
$$
\boldsymbol{Y}=\boldsymbol{f}\left(\boldsymbol{X}^{(0)}\right)+\sum_{i=1}^{\infty} \frac{1}{i !}\left(\nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X}\right)^{i} \boldsymbol{f}\left(\boldsymbol{X}^{(0)}\right)
$$
其中 $\delta \boldsymbol{X}=\boldsymbol{X}-\boldsymbol{X}^{(0)}$ ，$\nabla=\left[\begin{array}{llll}\frac{\partial}{\partial x{1}} & \frac{\partial}{\partial x{2}} & \cdots & \frac{\partial}{\partial x_{n}}\end{array}\right]^{\mathrm{T}}$

> 一阶导代表梯度、二阶导代表曲率半径

 **一阶展开项的详细展开式**：

标量的求导运算对向量求导是对向量里的每一个元素求导，$m$ 列多元函数对 $n$ 行的自变量求导：
$$
\begin{array}{l}
\frac{1}{1}\left(\nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X}\right)^{\prime} \boldsymbol{f}\left(\boldsymbol{X}^{(0)}\right)=\left.\left(\frac{\partial}{\partial x_{1}} \delta x_{1}+\frac{\partial}{\partial x_{2}} \delta x_{2}+\cdots+\frac{\partial}{\partial x_{n}} \delta x_{n}\right) \boldsymbol{f}(\boldsymbol{X})\right|_{\boldsymbol{X}=\boldsymbol{X}^{(0)}} \\
=\left[\begin{array}{c}
\frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{1}} \delta x_{1}+\frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{2}} \delta x_{2}+\cdots+\frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{n}} \delta x_{n} \\
\frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{1}} \delta x_{1}+\frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{2}} \delta x_{2}+\cdots+\frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{n}} \delta x_{n} \\
\frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{1}} \delta x_{1}+\frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{2}} \delta x_{2}+\cdots+\frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{n}} \delta x_{n}
\end{array}\right]_{\boldsymbol{X}=\boldsymbol{X}^{(0)}}=\left[\begin{array}{cccc}
\frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{1}} & \frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{2}} & \cdots & \frac{\partial f_{1}(\boldsymbol{X})}{\partial x_{n}} \\
\frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{1}} & \frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{2}} & \cdots & \frac{\partial f_{2}(\boldsymbol{X})}{\partial x_{n}} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{1}} & \frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{2}} & \cdots & \frac{\partial f_{m}(\boldsymbol{X})}{\partial x_{n}}
\end{array}\right]^{\delta}\left[\begin{array}{c}
\delta x_{1} \\
\delta x_{2} \\
\vdots \\
\delta x_{n}
\end{array}\right]_{X=X^{(0)}} \\
=\left.J(\boldsymbol{f}(\boldsymbol{X})) \delta \boldsymbol{X}\right|_{X=X^{(0)}} \quad  \\
\end{array}
$$
其中 $\int J(\boldsymbol{f}(\boldsymbol{X}))=\frac{\partial \boldsymbol{f}(\boldsymbol{X})}{\partial \boldsymbol{X}^{\mathrm{T}}}$ 可称之为雅各比矩阵，$m$ 列多元函数对 $n$ 行的自变量求一阶导得到的矩阵。

**二阶展开项详细表达式**：
$$
\begin{array}{l}
\frac{1}{2}\left(\nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X}\right)^{2} \boldsymbol{f}\left(\boldsymbol{X}^{(0)}\right)=\left.\frac{1}{2}\left(\frac{\partial}{\partial x_{1}} \delta x_{1}+\frac{\partial}{\partial x_{2}} \delta x_{2}+\cdots+\frac{\partial}{\partial x_{n}} \delta x_{n}\right)^{2} \boldsymbol{f}(\boldsymbol{X})\right|_{X=X^{(0)}} \\
=\left.\frac{1}{2} \operatorname{tr}\left(\left[\begin{array}{cccc}
\frac{\partial^{2}}{\partial x_{1}^{2}} & \frac{\partial^{2}}{\partial x_{1} \partial x_{2}} & \cdots & \frac{\partial^{2}}{\partial x_{1} \partial x_{n}} \\
\frac{\partial^{2}}{\partial x_{2} \partial x_{1}} & \frac{\partial^{2}}{\partial x_{2}^{2}} & \cdots & \frac{\partial^{2}}{\partial x_{2} \partial x_{n}} \\
\vdots & \vdots & \ddots & \vdots \\
\frac{\partial^{2}}{\partial x_{n} \partial x_{1}} & \frac{\partial^{2}}{\partial x_{n} \partial x_{2}} & \cdots & \frac{\partial^{2}}{\partial x_{n}^{2}}
\end{array}\right]\left[\begin{array}{cccc}
\delta x_{1}^{2} & \delta x_{1} \delta x_{2} & \cdots & \delta x_{1} \delta x_{n} \\
\delta x_{2} \delta x_{1} & \delta x_{2}^{2} & \cdots & \delta x_{2} \delta x_{n} \\
\vdots & \vdots & \ddots & \vdots \\
\delta x_{n} \delta x_{1} & \delta x_{n} \delta x_{2} & \cdots & \delta x_{n}^{2}
\end{array}\right]\right) \boldsymbol{f}(\boldsymbol{X})\right|_{X=\mathbf{X}^{(0)}} \\
\begin{array}{l}
=\left.\frac{1}{2} \operatorname{tr}\left(\nabla \nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right) \boldsymbol{f}(\boldsymbol{X})\right|_{X=X^{(0)}}=\left.\frac{1}{2} \operatorname{tr}\left(\nabla \nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right) \sum_{j=1}^{m} \boldsymbol{e}_{j} f_{j}(\boldsymbol{X})\right|_{X=X^{(0)}} \\
=\left.\frac{1}{2} \sum_{j=1}^{m} \boldsymbol{e}_{j} \operatorname{tr}\left(\nabla \nabla^{\mathrm{T}} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right) f_{j}(\boldsymbol{X})\right|_{X=X^{(0)}}=\frac{1}{2} \sum_{j=1}^{m} \boldsymbol{e}_{j} \operatorname{tr}\left(\left.\nabla \nabla^{\mathrm{T}} f_{j}(\boldsymbol{X})\right|_{X=X^{(0)}} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)
\end{array} \\
=\frac{1}{2} \sum_{j=1}^{m} \boldsymbol{e}_{j} \operatorname{tr}\left(\mathscr{H}\left(\left.f_{j}(\boldsymbol{X})\right|_{X=X^{(0)}} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right) \quad\right.  \\
\end{array}
$$

$$
\boldsymbol{Y}=\boldsymbol{f}\left(\boldsymbol{X}^{(0)}\right)+J\left(f\left(X^{(0)}\right)\right) \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{m} e_{i} \operatorname{tr}\left(\mathscr{H}\left(f_{i}\left(X^{(0)}\right)\right) \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)+O^{3}
$$

其中 $\mathscr{H}$ 为海森矩阵，$m$ 列多元函数对 $n$ 行的自变量求二阶导得到的矩阵，m 个向量函数有 m 个海森矩阵。

## 二、扩展Kalman滤波

遇到非线性的状态空间模型时，对状态方程和量测方程做线性化处理，取泰勒展开的一阶项。

**状态空间模型**(加性噪声，噪声和状态直接是相加的) 

> 这里简单的用加性噪声模型表示，一般形式为 $\boldsymbol{X}_{k}=\boldsymbol{f}\left(\boldsymbol{X}_{k-1}, \boldsymbol{W}_{k-1}, k-1\right)$ ，非线性更强，更一般，可以表示噪声和状态之间复杂的关系

$$
\left\{\begin{array}{l}\boldsymbol{X}_{k}=\boldsymbol{f}\left(\boldsymbol{X}_{k-1}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \boldsymbol{Z}_{k}=\boldsymbol{h}\left(\boldsymbol{X}_{k}\right)+\boldsymbol{V}_{k}\end{array}\right.
$$

选择 $k-1$ 时刻参考值 $\boldsymbol{X}_{k-1}^{n}$ ，参考点和真实值偏差 $\delta \boldsymbol{X}_{k-1}=\boldsymbol{X}_{k-1}-\boldsymbol{X}_{k-1}^{n}$ 

**状态一步预测**：上一时刻参考点带入状态方程 $\boldsymbol{X}_{k / k-1}^{n}=\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)$ ，预测值的偏差 $\delta \boldsymbol{X}_{k}=\boldsymbol{X}_{k}-\boldsymbol{X}_{k / k-1}^{n}$ 。

**量测一步预测**：$\boldsymbol{Z}_{k / k-1}^{n}=\boldsymbol{h}\left(\boldsymbol{X}_{k / k-1}^{n}\right)$ ，偏差 $\delta \boldsymbol{Z}_{k}=\boldsymbol{Z}_{k}-\boldsymbol{Z}_{k / k-1}^{n}$ 。

**展开得状态偏差方程**：
$$
\begin{array}{ll}
& \boldsymbol{X}_{k} \approx \boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{J}\left(\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)\right)\left(\boldsymbol{X}_{k-1}-\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\
& \boldsymbol{X}_{k}-\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right) \approx \boldsymbol{\Phi}_{k / k-1}^{n}\left(\boldsymbol{X}_{k-1}-\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\
& \delta \boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k / k-1}^{n} \delta \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}
\end{array}
$$
**展开得量测偏差方程**：
$$
\begin{array}{ll}
& \boldsymbol{Z}_{k} \approx \boldsymbol{h}\left(\boldsymbol{X}_{k / k-1}^{n}\right)+\boldsymbol{J}\left(\boldsymbol{h}\left(\boldsymbol{X}_{k / k-1}^{n}\right)\right)\left(\boldsymbol{X}_{k}-\boldsymbol{X}_{k / k-1}^{n}\right)+\boldsymbol{V}_{k} \\
& \delta \boldsymbol{Z}_{k}=\boldsymbol{H}_{k}^{n} \delta \boldsymbol{X}_{k}+\boldsymbol{V}_{k}
\end{array}
$$
**得偏差状态空间估计模型**：
$$
\left\{\begin{array}{l}\delta \boldsymbol{X}_{k}=\boldsymbol{\Phi}_{k \mid k-1}^{n} \delta \boldsymbol{X}_{k-1}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1} \\ \delta \boldsymbol{Z}_{k}=\boldsymbol{H}_{k}^{n} \delta \boldsymbol{X}_{k}+\boldsymbol{V}_{k}\end{array}\right.
$$
偏差量是线性的了，可以用Kalman滤波进行处理：

**滤波更新方程**：${\color{red}\delta \hat{\boldsymbol{X}}_{k}}=\boldsymbol{\Phi}_{k / k-1}^{n} {\color{green}\delta \hat{\boldsymbol{X}}_{k-1}}+\boldsymbol{K}_{k}^{n}\left(\delta \boldsymbol{Z}_{k}-\boldsymbol{H}_{k}^{n} \boldsymbol{\Phi}_{k / k-1}^{n} {\color{green}\delta \hat{\boldsymbol{X}}_{k-1}}\right)$ 
$$
\begin{array}{l}
\underset{\delta \hat{\boldsymbol{X}}_{k-1}=\hat{\boldsymbol{X}}_{k-1}-\boldsymbol{X}_{k-1}^{n}}{\underset{\delta \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k}-\boldsymbol{X}_{k / k-1}^{n}}{\longrightarrow}} \hat{\boldsymbol{X}}_{k}=\boldsymbol{X}_{k / k-1}^{n}+\boldsymbol{K}_{k}^{n}\left(\boldsymbol{Z}_{k}-\boldsymbol{Z}_{k / k-1}^{n}\right)+\left(\boldsymbol{I}-\boldsymbol{K}_{k}^{n} \boldsymbol{H}_{k}^{n}\right) \boldsymbol{\Phi}_{k / k-1}^{n}\left(\hat{\boldsymbol{X}}_{k-1}-\boldsymbol{X}_{k-1}^{n}\right) \\
\stackrel{\boldsymbol{X}_{k-1}^{n}=\hat{\boldsymbol{X}}_{k-1}}{\longrightarrow}=\boldsymbol{X}_{k / k-1}^{n}+\boldsymbol{K}_{k}^{n}\left(\boldsymbol{Z}_{k}-\boldsymbol{Z}_{k / k-1}^{n}\right)
\end{array}
$$
将 ${\color{green}\delta \hat{\boldsymbol{X}}_{k-1}=\hat{\boldsymbol{X}}_{k-1}-\boldsymbol{X}_{k-1}^{n}}$ 和 ${\color{red}\delta \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k}-\boldsymbol{X}_{k / k-1}^{n}}$ 带入得：
$$
\hat{\boldsymbol{X}}_{k}=\boldsymbol{X}_{k / k-1}^{n}+\boldsymbol{K}_{k}^{n}\left(\boldsymbol{Z}_{k}-\boldsymbol{Z}_{k / k-1}^{n}\right)+{\color{pink} \left(\boldsymbol{I}-\boldsymbol{K}_{k}^{n} \boldsymbol{H}_{k}^{n}\right) \boldsymbol{\Phi}_{k / k-1}^{n}\left(\hat{\boldsymbol{X}}_{k-1}-\boldsymbol{X}_{k-1}^{n}\right)}
$$
此公式就相当于对状态进行操作。前面部分就是Kalman滤波方程。后面多出的粉色部分包含了参考值，令 $\boldsymbol{X}_{k-1}^{n}=\hat{\boldsymbol{X}}_{k-1}$ ，参考点选成上一时刻的估计值，后面一部分就为 $0$ ，得
$$
\boldsymbol{X}_{k / k-1}^{n}+\boldsymbol{K}_{k}^{n}\left(\boldsymbol{Z}_{k}-\boldsymbol{Z}_{k / k-1}^{n}\right)
$$
**EKF滤波公式汇总**
$$
\left\{\begin{array}{ll}\hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right) & \\ \boldsymbol{P}_{k / k-1}={\color{red}\boldsymbol{\Phi}_{k / k-1}} \boldsymbol{P}_{k-1} {\color{red}\boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} & {\color{red}\boldsymbol{\Phi}_{k / k-1}=\boldsymbol{J}\left(\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right)\right)} \\ \boldsymbol{K}_{k}=\boldsymbol{P}_{k / k-1} {\color{green}\boldsymbol{H}_{k}^{\mathrm{T}}}\left({\color{green}\boldsymbol{H}_{k}} \boldsymbol{P}_{k / k-1} {\color{green}\boldsymbol{H}_{k}^{\mathrm{T}}}+\boldsymbol{R}_{k}\right)^{-1} & {\color{green}\boldsymbol{H}_{k}=\boldsymbol{J}\left(\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)\right)} \\ \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left[\boldsymbol{Z}_{k}-\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)\right] & \\ \boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} {\color{green}\boldsymbol{H}_{k}}\right) \boldsymbol{P}_{k / k-1} & \end{array}\right.
$$

* 状态转移矩阵是状态方程在 $k-1$ 处的一阶偏导组成的雅可比矩阵。
* 设计矩阵是量测方程在 $k-1$ 处的一阶偏导组成的雅可比矩阵。

## 三、二阶滤波

**状态方程的二阶泰勒展开**：

$$
\boldsymbol{X}_{k}=\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{\Phi}_{k / k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i}^{-} \cdot \boldsymbol{\delta} \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}
$$
**状态预测**：上面的方程两边求均值得：
$$
\begin{aligned}
\hat{\boldsymbol{X}}_{k / k-1} & =\mathrm{E}\left[\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{\Phi}_{k / k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i} \cdot \boldsymbol{\delta} \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right] \\
& =\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)+\boldsymbol{\Phi}_{k / k-1}^{n} \mathrm{E}[\delta \boldsymbol{X}]+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i} \cdot \mathrm{E}\left[\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right]\right)+\boldsymbol{\Gamma}_{k-1} \mathrm{E}\left[\boldsymbol{W}_{k-1}\right] \\
& =\boldsymbol{f}\left(\boldsymbol{X}_{k-1}^{n}\right)+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i} \boldsymbol{P}_{k-1}\right)
\end{aligned}
$$
状态预测既和状态的传递有关系，也和方差 $P$ 阵有关系，比较麻烦。

**状态预测的误差**：
$$
\begin{aligned}
\tilde{\boldsymbol{X}}_{k / k-1} & =\boldsymbol{X}_{k}-\hat{\boldsymbol{X}}_{k / k-1} \\
& =\boldsymbol{\Phi}_{k / k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}-\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i} \boldsymbol{P}_{k-1}\right) \\
& =\boldsymbol{\Phi}_{k / k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}
\end{aligned}
$$
**状态预测的方差**：比较复杂，最后算下来和四阶矩有关系
$$
\begin{array}{l}
\boldsymbol{P}_{k / k-1}=\mathrm{E}\left[\tilde{\boldsymbol{X}}_{k / k-1} \tilde{\boldsymbol{X}}_{k / k-1}^{\mathrm{T}}\right] \\
=\mathrm{E}\left[\left[\boldsymbol{\Phi}_{k \mid k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right]\right. \\
\left.\times\left[\boldsymbol{\Phi}_{k l k-1}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{W}_{k-1}\right]^{\mathrm{T}}\right] \\
=\boldsymbol{\Phi}_{k k / k-1}^{n} \boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{n}\right)^{\mathrm{T}} \\
+\frac{1}{4} \mathrm{E}\left[\sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f i}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)\left(\sum_{i=1}^{n} \boldsymbol{e}_{i} \operatorname{tr}\left(\boldsymbol{D}_{f_{i}}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)\right)^{\mathrm{T}}\right]+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \\
=\boldsymbol{\Phi}_{k / k-1}^{n} \boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{n}\right)^{\mathrm{T}} \\
+\frac{1}{4} \sum_{i=1}^{n} \sum_{j=1}^{n} \boldsymbol{e}_{i} \boldsymbol{e}_{j}^{\mathrm{T}} \mathrm{E}\left[\operatorname{tr}\left(\boldsymbol{D}_{f i}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right) \cdot \operatorname{tr}\left(\boldsymbol{D}_{j j}\left(\delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}-\boldsymbol{P}_{k-1}\right)\right)\right]+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \\
=\boldsymbol{\Phi}_{k / k-1}^{n} \boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{n}\right)^{\mathrm{T}}+\frac{1}{2} \sum_{i=1}^{n} \sum_{j=1}^{n} \boldsymbol{e}_{\boldsymbol{i}} \boldsymbol{e}_{j}^{\mathrm{T}} \operatorname{tr}\left(\boldsymbol{D}_{f i} \boldsymbol{P}_{k-1} \boldsymbol{D}_{f j} \boldsymbol{P}_{k-1}\right)+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} \\
\end{array}
$$
**量测方程的二阶展开**：
$$
Z_{k / k-1}=h\left(X_{k / k-1}^{n}\right)+\boldsymbol{H}_{k}^{n} \delta \boldsymbol{X}+\frac{1}{2} \sum_{i=1}^{m} e_{i} i r\left(D_{h i} \cdot \delta \boldsymbol{X} \delta \boldsymbol{X}^{\mathrm{T}}\right)+V_{k}
$$
**量测预测**：
$$
\hat{\boldsymbol{Z}}_{k / k-1}=\boldsymbol{h}\left(\boldsymbol{X}_{k / k-1}^{n}\right)+\frac{1}{2} \sum_{i=1}^{m} e_{i} \operatorname{tr}\left(\boldsymbol{D}_{h i} \boldsymbol{P}_{k / k-1}\right)
$$
**量测预测方差阵**：
$$
\boldsymbol{P}_{z z, k k-1}=\boldsymbol{H}_{k}^{n} \boldsymbol{P}_{k / k-1}\left(\boldsymbol{H}_{k}^{n}\right)^{\mathrm{T}}+\frac{1}{2} \sum_{i=1}^{m} \sum_{j=1}^{m} \boldsymbol{e}_{i}^{\mathrm{e}} \mathrm{e}_{j}^{\mathrm{t}}\left(\boldsymbol{D}_{h i} \boldsymbol{P}_{k / k-1} \boldsymbol{D}_{hj} \boldsymbol{P}_{k / k-1}\right)+\boldsymbol{R}_{k}
$$
**状态/量测互协方差阵**：
$$
\boldsymbol{P}_{x z, k k-1}=\boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{n}
$$
**最后，得二阶滤波公式**：红色部分是二阶多出来的，运算量巨大收益微小

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/bc4e06cfe64c41e5bec4692b80af3972.png)


## 四、迭代EKF滤波

$$
\left\{\begin{array}{ll}\hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right) & \\ \boldsymbol{P}_{k / k-1}={\color{red}\boldsymbol{\Phi}_{k / k-1}} \boldsymbol{P}_{k-1} {\color{red}\boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} & {\color{red}\boldsymbol{\Phi}_{k / k-1}=\boldsymbol{J}\left(\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right)\right)} \\ \boldsymbol{K}_{k}=\boldsymbol{P}_{k / k-1} {\color{green}\boldsymbol{H}_{k}^{\mathrm{T}}}\left({\color{green}\boldsymbol{H}_{k}} \boldsymbol{P}_{k / k-1} {\color{green}\boldsymbol{H}_{k}^{\mathrm{T}}}+\boldsymbol{R}_{k}\right)^{-1} & {\color{green}\boldsymbol{H}_{k}=\boldsymbol{J}\left(\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)\right)} \\ \hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left[\boldsymbol{Z}_{k}-\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)\right] & \\ \boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} {\color{green}\boldsymbol{H}_{k}}\right) \boldsymbol{P}_{k / k-1} & \end{array}\right.
$$

非线性系统展开点越接近你想研究的那个点，展开的精度越高。普通EKF的展开点是上一时刻 $k-1$，考虑迭代修正展开点。做完一次滤波之后，有 $k$ 时刻的新量测值，可以用新量测值对 $k-1$ 时刻做反向平滑，用反向平滑的值作为新的上一时刻的状态估计值，进行Kalman滤波。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/f2cf8c011635467791f23db0f5d035d4.png)


**预滤波**：
$$
\left\{\begin{array}{ll}
\hat{\boldsymbol{X}}_{k / k-1}^{*}=\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right) \\
\boldsymbol{P}_{k / k-1}^{*}=\boldsymbol{\Phi}_{k / k-1}^{*} \boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{*}\right)^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} & \boldsymbol{\Phi}_{k / k-1}^{*}=\boldsymbol{J}\left(\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right)\right) \\
\boldsymbol{K}_{k}^{*}=\boldsymbol{P}_{k / k-1}^{*}\left(\boldsymbol{H}_{k}^{*}\right)^{\mathrm{T}}\left[\boldsymbol{H}_{k}^{*} \boldsymbol{P}_{k / k-1}^{*}\left(\boldsymbol{H}_{k}^{*}\right)^{\mathrm{T}}+\boldsymbol{R}_{k}\right]^{-1} & \boldsymbol{H}_{k}^{*}=\boldsymbol{J}\left(\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}^{*}\right)\right) \\
\hat{\boldsymbol{X}}_{k}^{*}=\hat{\boldsymbol{X}}_{k / k-1}^{*}+\boldsymbol{K}_{k}^{*}\left[\boldsymbol{Z}_{k}-\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}^{*}\right)\right] &
\end{array}\right.
$$
**反向一步平滑**：可以用RTS算法
$$
\hat{\boldsymbol{X}}_{k-1 / k}=\hat{\boldsymbol{X}}_{k-1}+\boldsymbol{P}_{k-1}\left(\boldsymbol{\Phi}_{k / k-1}^{*}\right)^{\mathrm{T}}\left(\boldsymbol{P}_{k / k-1}^{*}\right)^{-1}\left(\hat{\boldsymbol{X}}_{k}^{*}-\hat{\boldsymbol{X}}_{k / k-1}^{*}\right)
$$
**重做状态一步预测**：$\hat{\boldsymbol{X}}_{k-1 / k}$ 再做泰勒级数展开
$$
\begin{aligned}
\hat{\boldsymbol{X}}_{k / k-1} & =\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right)=\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right)+\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1}\right)-\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right) \\
& \approx \boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right)+\boldsymbol{J}\left(\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right)\right)\left(\hat{\boldsymbol{X}}_{k-1}-\hat{\boldsymbol{X}}_{k-1 / k}\right)
\end{aligned}
$$
**重做量测一步预测**：
$$
\begin{aligned}
\hat{\boldsymbol{Z}}_{k / k-1} & =\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)=\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right)+\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k / k-1}\right)-\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right) \\
& \approx \boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right)+\boldsymbol{J}\left(\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right)\right)\left(\hat{\boldsymbol{X}}_{k / k-1}-\hat{\boldsymbol{X}}_{k}^{*}\right)
\end{aligned}
$$
**迭代滤波**：在此基础上做Kalman滤波
$$
\left\{\begin{array}{ll}
\hat{\boldsymbol{X}}_{k / k-1}=\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right)+\boldsymbol{\Phi}_{k / k-1}\left(\hat{\boldsymbol{X}}_{k-1}-\hat{\boldsymbol{X}}_{k-1 / k}\right) & \\
\boldsymbol{P}_{k / k-1}=\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{\mathrm{T}}+\boldsymbol{\Gamma}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{\Gamma}_{k-1}^{\mathrm{T}} & \boldsymbol{\Phi}_{k / k-1}=\boldsymbol{J}\left(\boldsymbol{f}\left(\hat{\boldsymbol{X}}_{k-1 / k}\right)\right) \\
\boldsymbol{K}_{k}=\boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}\left(\boldsymbol{H}_{k} \boldsymbol{P}_{k / k-1} \boldsymbol{H}_{k}^{\mathrm{T}}+\boldsymbol{R}_{k}\right)^{-1} & \boldsymbol{H}_{k}=\boldsymbol{J}\left(\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right)\right) \\
\hat{\boldsymbol{X}}_{k}=\hat{\boldsymbol{X}}_{k / k-1}+\boldsymbol{K}_{k}\left[\boldsymbol{Z}_{k}-\boldsymbol{h}\left(\hat{\boldsymbol{X}}_{k}^{*}\right)-\boldsymbol{H}_{k}\left(\hat{\boldsymbol{X}}_{k / k-1}-\hat{\boldsymbol{X}}_{k}^{*}\right)\right] \\
\boldsymbol{P}_{k}=\left(\boldsymbol{I}-\boldsymbol{K}_{k} \boldsymbol{H}_{k}\right) \boldsymbol{P}_{k / k-1} & 
\end{array}\right.
$$

还可以多次迭代，但收益小