[TOC]

## 一、蒙特卡洛仿真

蒙特卡洛仿真的基本原理是通过生成大量的随机样本，以近似地估计实际事件的概率和预测结果。 它是以蒙特卡洛赌场命名的，因为它使用随机数和概率统计的方法来模拟现实世界中的各种情况和结果，就像在赌场中抛骰子或发牌一样。 

> 缺点：收敛精度 $\varepsilon \propto \frac{1}{\sqrt{n}}$, 收敛速度慢! 
>
> 优点：和问题的维数无关，只要边界可以描述，就都可以求。


### 【例1】圆周率计算

我们都知道, 单位圆围成的面积为 $\pi$, 若以圆心为原点 建立平面直角坐标系, 则它在第一象限内的面积为 $\pi / 4$, 即有：
$$
\int_{0}^{1} \int_{0}^{\sqrt{1-y^{2}}} 1 \mathrm{~d} x \mathrm{~d} y=\pi / 4
$$
![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5092808c086e4ac4a36229c3f41d5d76.png)


**Matlab仿真**：

```matlab
n = 1000000;				% 十万个点
p = unifrnd(0,1,2,n);		 % 在 0~1 区间均匀随机分布，生成二维点
PI = 4*sum(sum(p.^2)<1)/n     % 求出在四分之一圆内的比例，再乘 4
```

结果：`PI = 3.141092 `，前三个数字是准确的。计算机仿真，可以实现均匀分布，如果是现实仿真，保证不了均匀分布，效果会更差。

### 【例2】体积计算

在单位球体内偏离中心 0.25 的位置挖去一个半径为 0.25 的柱体 (参见图), 试求该球体剩余部分的体积。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5f33830b5ccf4cc887f29f89fd894dab.png)

用一个立方体，包住这个球，生成均匀分布的随机点，

**Matlab仿真**：

```matlab
n = 100000;
p = unifrnd(-1,1,3,n);
V = 2^3*sum(sum(p.^2)<1&(p(1,:).^2 + (p(2,:)-0.25).^2)>0.25^2)/n
```

结果：`V = 3.812905`。

### 【例3】二维随机向量的非线性变换/概论传播

已知 $x$ 的分布，求 $2x$ 的分布比较简单，因为属于线性变换。如果是求 $x^2+2x+e^x$ 就比较难了。用仿真的方法求结果的分布函数也比较难，但如果只追求对其粗略的了解，比如均值、方差，可以实现。

现在有如下的函数，输入 $x_1$、$x_2$ ，转化到 $y_1$、$y_2$ 都是非线性函数， 二维方差是一个对称正定阵，分布是一个椭圆，椭圆就是一个长轴一个短轴，总可以分解得到。
$$
\left\{\begin{array}{l}
y_{1}=\left(x_{1}-1\right) \cdot\left(x_{2}-0.2\right) \\
y_{2}=-\left(x_{1}-1\right)^{2}
\end{array}\quad \quad \quad  E\left[x x^{\mathrm{T}}\right]=\left[\begin{array}{cc}
1 & 0.42 \\
0.42 & 2
\end{array}\right]\right.
$$
可以画出下面的图。左边每个绿点代表一个向量，共500个点，满足蓝色的方差椭圆分布。经过上面的非线性函数得到右边的点，可由点的分布统计得右边的均值、方差。无穷个点得到的真实的方差椭圆是图中的实线，统计出来的是十字线，统计值和真实值算比较接近的了。

用5个红点也能变换过去，也能统计均值和方差，画出误差椭圆是叉线，与500个绿点得到的十字线几乎重合。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/042d476dd58c4e2592f553c17082668c.png)


  ## 二、UT变换

### 1、线性系统的UT变换

假设随机向量的线性变换关系 $\boldsymbol{y}=\boldsymbol{F} \boldsymbol{x}$ 
$$
\left\{\begin{array} { l } 
{ \overline { \boldsymbol { x } } = E [ \boldsymbol { x } ] } \\
{ \boldsymbol { P } _ { x } = E [ ( \boldsymbol { x } - \overline { \boldsymbol { x } } ) ( \boldsymbol { x } - \overline { \boldsymbol { x } } ) ^ { \mathrm { T } } ] }
\end{array} \Longrightarrow \left\{\begin{array}{l}
\overline{\boldsymbol{y}}=\boldsymbol{F} \overline{\boldsymbol{x}} \\
\boldsymbol{P}_{y}=\boldsymbol{F} \boldsymbol{P}_{x} \boldsymbol{F}^{\mathrm{T}} \\
\boldsymbol{P}_{x y}=\boldsymbol{P}_{x} \boldsymbol{F}^{\mathrm{T}}
\end{array}\right.\right.
$$

#### 1.方差阵的Cholesky分解

$\boldsymbol{P}_{x}=\boldsymbol{A} \boldsymbol{A}^{\mathrm{T}} \quad$ 记 $\sqrt{\boldsymbol{P}_{x}}=\boldsymbol{A} \quad\left[\begin{array}{cc}1 & 2 \\ 2 & 13\end{array}\right]=\left[\begin{array}{lll}1 & 0 \\ 2 & 3\end{array}\right]$

 如此，P 就可以写成：$\boldsymbol{P}_{x}=\sqrt{\boldsymbol{P}_{x}}{\sqrt{\boldsymbol{P}_{x}}}^{\mathrm{T}}=\sum_{i=1}^{n}\left(\sqrt{\boldsymbol{P}_{x}}\right)_{i}\left(\sqrt{\boldsymbol{P}_{x}}\right)_{i}^{\mathrm{T}}$ ，向量累加和的形式。

> 如 $\begin{array}{l}{\left[\begin{array}{cc}1 & 2 \\ 2 & 13\end{array}\right]=\left[\begin{array}{cc}1 & 0 \\ 2 & 3\end{array}\right]\left[\begin{array}{ll}1 & 2 \\ 0 & 3\end{array}\right]}  =\left[\begin{array}{l}1 \\ 2\end{array}\right]\left[\begin{array}{ll}1 & 2\end{array}\right]+\left[\begin{array}{l}0 \\ 3\end{array}\right]\left[\begin{array}{ll}0 & 3\end{array}\right]\end{array}$

向量就是图上的点，$p$ 如果是高维的，$\sqrt{\boldsymbol{P}}$ 里的每一列就代表其中一个点。

#### 2.输入近似成有限个（n个）Sigma点的估计形式

$$
\left\{\begin{array}{l}
\overline{\boldsymbol{x}}=\lim \limits_{k \rightarrow \infty} \frac{1}{k} \sum_{i=1}^{k} \chi_{i} \approx \frac{1}{n} \sum_{i=1}^{n} \chi_{i} \\
\boldsymbol{P}_{x}=\lim \limits_{k \rightarrow \infty} \frac{1}{k} \sum_{i=1}^{k}\left(\chi_{i}-\bar{x}\right)\left(\chi_{i}-\overline{\boldsymbol{x}}\right)^{\mathrm{T}} \approx \frac{1}{n} \sum_{i=1}^{n}\left(\chi_{i}-\overline{\boldsymbol{x}}\right)\left(\chi_{i}-\overline{\boldsymbol{x}}\right)^{\mathrm{T}}
\end{array}\right.
$$

式中的 $P_x$ 与上面 $\boldsymbol{P}_{x}=\sum_{i=1}^{n}\left(\sqrt{\boldsymbol{P}_{x}}\right)_{i}\left(\sqrt{\boldsymbol{P}_{x}}\right)_{i}^{\mathrm{T}}$ 的形式对应，比较, 可得 $\frac{1}{\sqrt{n}}\left(\boldsymbol{\chi}_{i}-\overline{\boldsymbol{x}}\right)= \pm\left(\sqrt{\boldsymbol{P}_{x}}\right)_{i}$ ，即 $\chi_{i}=\overline{\boldsymbol{x}} \pm\left(\sqrt{n \boldsymbol{P}_{x}}\right)_{i}$ ，sigma点就是在均值的地方加减 $\left(\sqrt{n \boldsymbol{P}_{x}}\right)_{i}$ 

#### 3.输入近似成有限个（2n个）Sigma点的估计形式

输入取上一步求得的sigma点，带入线性函数中，得：
$$
\left\{\begin{array}{l}
\overline{\boldsymbol{x}}=\frac{1}{2 n} \sum_{i=1}^{2 n} \chi_{i} \\
\boldsymbol{P}_{x}=\frac{1}{2 n} \sum_{i=1}^{2 n}\left(\chi_{i}-\overline{\boldsymbol{x}}\right)\left(\chi_{i}-\overline{\boldsymbol{x}}\right)^{\mathrm{T}}
\end{array}\right.
$$
记Sigma点组成的**输入**矩阵：$\chi=\left[[\overline{\boldsymbol{x}}]_{n}+\sqrt{n \boldsymbol{P}_{x}}\quad[\overline{\boldsymbol{x}}]_{n}-\sqrt{n \boldsymbol{P}_{x}}\right]$ ，n*2n矩阵

得Sigma点组成的**输出**矩阵：$\boldsymbol{\eta}=\left[[\boldsymbol{F} \overline{\boldsymbol{x}}]_{n}+\boldsymbol{F} \sqrt{n \boldsymbol{P}_{x}} \quad[\boldsymbol{F}\overline{\boldsymbol{x}}]_{n}-\boldsymbol{F} \sqrt{n \boldsymbol{P}_{x}}\right]$ 

#### 4.由输入输出Sigma点计算传播统计特征

$$
\left\{\begin{array}{l}
\overline{\boldsymbol{y}}=\frac{1}{2 n} \sum_{i=1}^{2 n} \boldsymbol{\eta}_{i}=\frac{1}{2 n} \sum_{i=1}^{2 n} \boldsymbol{F} \overline{\boldsymbol{x}}=\boldsymbol{F} \overline{\boldsymbol{x}} \\
\boldsymbol{P}_{y}=\frac{1}{2 n} \sum_{i=1}^{2 n}\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)^{\mathrm{T}}=\frac{1}{2 n} \sum_{i=1}^{2 n}\left(\boldsymbol{F} \sqrt{n \boldsymbol{P}_{x}}\right)\left(\boldsymbol{F} \sqrt{n \boldsymbol{P}_{x}}\right)^{\mathrm{T}}=\boldsymbol{F}_{x} \boldsymbol{F}^{\mathrm{T}} \\
\boldsymbol{P}_{x y}=\frac{1}{2 n} \sum_{i=1}^{2 n}\left(\boldsymbol{x}_{i}-\overline{\boldsymbol{x}}\right)\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)^{\mathrm{T}}=\frac{1}{2 n} \sum_{i=1}^{2 n}\left(\sqrt{n \boldsymbol{P}_{x}}\right)\left(\boldsymbol{F} \sqrt{n \boldsymbol{P}_{x}}\right)^{\mathrm{T}}=\boldsymbol{P}_{x} \boldsymbol{F}^{\mathrm{T}}
\end{array}\right.
$$

与理论结果一致，精确地捕获了输出的一、二阶矩。

### 2、非线性函数的UT变换

将这种操作放到非线性函数 $y=f(x)$ 中：只是输入的Sigma点要带到非线性函数中算，输出的操作和线性的一致，只是符号变为约等于号。
$$
\left\{\begin{array}{l}\boldsymbol{\chi}=\left[[\overline{\boldsymbol{x}}]_{n}+\sqrt{n \boldsymbol{P}_{x}} \quad[\overline{\boldsymbol{x}}]_{n}-\sqrt{n \boldsymbol{P}_{x}}\right] \\ \boldsymbol{\eta}_{i}=\boldsymbol{f}\left(\chi_{i}\right) \\ \overline{\boldsymbol{y}} \approx 1 /(2 n) \cdot \sum_{i=1}^{2 n} \boldsymbol{\eta}_{i} \\ \boldsymbol{P}_{y} \approx 1 /(2 n) \cdot \sum_{i=1}^{2 n}\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)^{\mathrm{T}} \\ \boldsymbol{P}_{x y} \approx 1 /(2 n) \cdot \sum_{i=1}^{2 n}\left(\chi_{i}-\overline{\boldsymbol{x}}\right)\left(\boldsymbol{\eta}_{i}-\overline{\boldsymbol{y}}\right)^{\mathrm{T}}\end{array}\right.
$$
可以改成更一般的形式：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/373b08049203416799ae868041e10664.png)

其中：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6c03edede66b43fdb74a4a4fd432f489.png)


非线性如果比较严重，$\alpha$ 就应该取的比较小，使展开的点集中上次均值的附近。

> EKF需要泰勒级数展开；UKF不需要，只要表达式能知道能运算就行了，点总可以运算得到结果，总可以进行统计

### 3、UT变换图示

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/b3b300b4f63041ee9202d5769fc79381.png)


## 三、UKF滤波

**系统模型**
$$
\left\{\begin{array}{l}\boldsymbol{x}_{k}=\boldsymbol{f}\left(\boldsymbol{x}_{k-1}\right)+\boldsymbol{B}_{k-1} \boldsymbol{w}_{k-1} \\ \boldsymbol{y}_{k}=\boldsymbol{h}\left(\boldsymbol{x}_{k}\right)+\boldsymbol{v}_{k}\end{array}\right.
$$
**滤波框架**
$$
\left\{\begin{array}{l}\boldsymbol{K}_{k}=\boldsymbol{P}_{x y, k / k-1} \boldsymbol{P}_{y, k / k-1}^{-1} \\ \hat{\boldsymbol{x}}_{k}=\hat{\boldsymbol{x}}_{k / k-1}+\boldsymbol{K}_{k}\left(\boldsymbol{y}_{k}-\hat{\boldsymbol{y}}_{k / k-1}\right) \\ \boldsymbol{P}_{x, k}=\boldsymbol{P}_{x, k / k-1}-\boldsymbol{K}_{k} \boldsymbol{P}_{y, k / k-1} \boldsymbol{K}_{k}^{\mathrm{T}}\end{array}\right.
$$
**状态预测UT变换**
$$
\left\{\begin{array}{l}\boldsymbol{\chi}_{k-1}=\left[\begin{array}{lll}\hat{\boldsymbol{x}}_{k-1} & {\left[\hat{\boldsymbol{x}}_{k-1}\right]_{n}+\gamma \sqrt{\boldsymbol{P}_{x, k-1}}} & {\left[\hat{\boldsymbol{x}}_{k-1}\right]_{n}-\gamma \sqrt{\boldsymbol{P}_{x, k-1}}}\end{array}\right] \\ \chi_{i, k / k-1}^{*}=\boldsymbol{f}\left(\boldsymbol{\chi}_{i, k-1}\right) \\ \hat{\boldsymbol{x}}_{k / k-1}=\sum_{i=0}^{2 n} W_{i}^{m} \chi_{i, k / k-1}^{*} \\ \boldsymbol{P}_{x, k / k-1}=\sum_{i=0}^{2 n} W_{i}^{c}\left(\chi_{i, k / k-1}^{*}-\hat{\boldsymbol{x}}_{k / k-1}\right)\left(\chi_{i, k / k-1}^{*}-\hat{\boldsymbol{x}}_{k / k-1}\right)^{\mathrm{T}}+\boldsymbol{B}_{k-1} \boldsymbol{Q}_{k-1} \boldsymbol{B}_{k-1}^{\mathrm{T}}\end{array}\right.
$$
**量测预测UT变换** 
$$
\left\{\begin{array}{l}\boldsymbol{\chi}_{k / k-1}=\left[\begin{array}{lll}\hat{\boldsymbol{x}}_{k / k-1} & {\left[\hat{\boldsymbol{x}}_{k / k-1}\right]_{n}+\gamma \sqrt{\boldsymbol{P}_{x, k / k-1}}} & {\left[\hat{\boldsymbol{x}}_{k / k-1}\right]_{n}-\gamma \sqrt{\boldsymbol{P}_{x, k / k-1}}}\end{array}\right] \\ \boldsymbol{\eta}_{i, k / k-1}=\boldsymbol{h}\left(\chi_{i, k / k-1}\right) \\ \hat{\boldsymbol{y}}_{k / k-1}=\sum_{i=0}^{2 n} W_{i}^{m} \boldsymbol{\eta}_{i, k / k-1} \\ \boldsymbol{P}_{y, k / k-1}=\sum_{i=0}^{2 n} W_{i}^{c}\left(\boldsymbol{\eta}_{i, k / k-1}-\hat{\boldsymbol{y}}_{k / k-1}\right)\left(\boldsymbol{\eta}_{i, k / k-1}-\hat{\boldsymbol{y}}_{k / k-1}\right)^{\mathrm{T}}+\boldsymbol{R}_{k} \\ \boldsymbol{P}_{x y, k / k-1}=\sum_{i=0}^{2 n} W_{i}^{c}\left(\boldsymbol{\chi}_{i, k / k-1}-\hat{\boldsymbol{x}}_{k / k-1}\right)\left(\boldsymbol{\eta}_{i, k / k-1}-\hat{\boldsymbol{y}}_{k / k-1}\right)^{\mathrm{T}}\end{array}\right.
$$
每个都实现，带到框架里，完成UKF时间更新、量测更新

### 【例1】一维非线性随机系统

$$
\left\{\begin{array}{l}
x_{k}=\sin \left(x_{k-1}\right)+w_{k-1} \\
y_{k}=\left\{\begin{array}{ll}
x_{k}+v_{k} & x_{k}>0 \\
2 x_{k}+v_{k} & x_{k} \leq 0
\end{array} \quad w_{k} \sim W N\left(0,0.1^{2}\right) \quad v_{k} \sim W N\left(0,0.3^{2}\right)\right.
\end{array}\right.
$$

**仿真结果**：滤波效果与初值的选择关系密切

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6b64b1ce23fc473cbbd0f034a4de0c8a.png)


### 【例2】针对非常复杂的非线性系统模型

> 近似非线性概率传播 比近似非线性函数（泰勒展开线性化）更简单，此时UKF滤波比EKF滤波更具优势。

**比如惯导大失准角误差模型**
$$
\left\{\begin{array}{l}\dot{\boldsymbol{\alpha}}=\boldsymbol{C}_{\omega}^{-1}\left[\left(\boldsymbol{I}-\boldsymbol{C}_{n}^{n^{\prime}}\right) \boldsymbol{\omega}_{i n}^{n}-\boldsymbol{\varepsilon}^{n^{\prime}}\right] \\ \delta \dot{\boldsymbol{v}}^{n}=\left[\boldsymbol{I}-\left(\boldsymbol{C}_{n}^{n^{\prime}}\right)^{\mathrm{T}}\right] \tilde{\boldsymbol{f}}_{s f}^{n^{\prime}}+\left(\boldsymbol{C}_{n}^{n^{\prime}}\right)^{\mathrm{T}} \nabla^{n^{\prime}}\end{array}\right.
$$
其中：
$$
\boldsymbol{C}_{\omega}^{-1}=\frac{1}{c \alpha_{x}}\left[\begin{array}{ccc}c \alpha_{y} c \alpha_{x} & 0 & s \alpha_{y} c \alpha_{x} \\ s \alpha_{y} s \alpha_{x} & c \alpha_{x} & -c \alpha_{y} s \alpha_{x} \\ -s \alpha_{y} & 0 & c \alpha_{y}\end{array}\right]
$$

$$
\boldsymbol{C}_{n}^{n^{\prime}}=\left[\begin{array}{ccc}c \alpha_{y} c \alpha_{z}-s \alpha_{y} s \alpha_{x} s \alpha_{z} & c \alpha_{y} s \alpha_{z}+s \alpha_{y} s \alpha_{x} c \alpha_{z} & -s \alpha_{y} c \alpha_{x} \\ -c \alpha_{x} s \alpha_{z} & c \alpha_{x} c \alpha_{z} & s \alpha_{x} \\ s \alpha_{y} c \alpha_{z}+c \alpha_{y} s \alpha_{x} s \alpha_{z} & s \alpha_{y} s \alpha_{z}-c \alpha_{y} s \alpha_{x} c \alpha_{z} & c \alpha_{y} c \alpha_{x}\end{array}\right]
$$



方程如此复杂，想进行泰勒级数展开很困难，可以用UKF。