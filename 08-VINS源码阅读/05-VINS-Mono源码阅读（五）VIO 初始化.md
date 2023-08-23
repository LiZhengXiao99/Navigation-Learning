## 一、原理

![白话VINS-Mono之初始化（三）](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-9195a1bfc959c5b04473be33da6cbacf_720w.png)

### 1. 为什么要进行初始化？

* 因为求解方式类似与梯度下降，VINS 这种基于优化的模式对初始值非常敏感，初始值不好的话，会陷入局部最优解（或者不是我们想要的部分极小值）。
* 为了保证系统的极小值是我们要的，那么初始值就需要距离正确值比较近，这样迭代次数也少一些，也节约了计算时间和运算资源，也可以保证实时性。

### 2. 需要初始化的变量有哪些？

需要初始化的变量如下，分别是滑窗，状态，和外参：
$$
\begin{aligned} \mathcal{X} & =\left[\mathbf{x}_{0}, \mathbf{x}_{1}, \ldots \mathbf{x}_{n}, \mathbf{x}_{c}^{b}, \lambda_{0}, \lambda_{1}, \ldots \lambda_{m}\right] \\ \mathbf{x}_{k} & =\left[\mathbf{p}_{b_{k}}^{w}, \mathbf{v}_{b_{k}}^{w}, \mathbf{q}_{b_{k}}^{w}, \mathbf{b}_{a}, \mathbf{b}_{g}\right], k \in[0, n] \\ \mathbf{x}_{c}^{b} & =\left[\mathbf{p}_{c}^{b}, \mathbf{q}_{c}^{b}\right]\end{aligned}
$$

* 有些变量不是通过初始化得到的，比如加速度计的零偏，初始化很难得到准确的加速度零偏的结果。
* 平移外参也很难求解，VINS里是不去估计他们的，直接假设已经提前知道了平移外参。
* 加速度计的零偏不好求，就算我们忽略掉，对结果的影响也比较小。在后端中，加速度计的零偏再进行优化。初始的时候零偏当做0直接算，对结果影响不大。
* 为什么不优化加速度计的零偏？因为他很难从重力中区分出来。就算忽略掉，也对结果影响不大。



**初始化之前**：

![image-20230817140225615](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230817140225615.png)







![image-20230817140801194](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230817140801194.png)























