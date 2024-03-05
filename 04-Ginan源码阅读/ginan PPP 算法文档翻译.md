网址：https://geoscienceaustralia.github.io/ginan/page.html?c=on&p=science.index

对应算法文档：

* observations.md
* kalmanFilter.md
* rts.md
* orbits.md
* ionosphere.md
* ambiguities.md
* minimumConstraints.md
* conventions.md

## 一、量测模型

### 1、观测方程

伪距和载波相位的观测方程：
$$
\begin{alignat}{2} 
\label{eq:code_UC_mea}
E(P_{r,f}^s) 
&= \rho_{r}^s 
+ c(dt_{r}^q - dt^s) 
+ \tau_r^s
+ \mu_f I^s_r 
+ d_{r,f}^q
+ d_{f}^s
\\
\label{eq:phase_UC_mea}
E(L_{r,f}^s) 
&= \rho_{r}^s 
+ c(dt_{r}^q - dt^s) 
+ \tau_r^s 
- \mu_f I^s_r
+ b_{r,f}^q 
- b_{f}^s
+ \lambda_{f} z_{r,f}^s  
+ \phi^s_{r,f}
\end{alignat}
$$

*  $P_{r,f}^S$：伪距观测值（m）
* $L_{r,f}^s$：载波相位观测值（m）
* $E()$：期望
* $ \sigma() $：方差
*  $\rho_r^s$：站星几何距离
*  $c$：光速
* 



想要准确定位，需要先计算出准确的几何距离 $\rho_r^s$，其它的误差项都要想办法消除。在 PPP 解算过程中，几何距离先由卫星位置和接收机近似位置，通过距离公式线性化求得：

$$
\rho_{r}^{s} = \sqrt{X^{s-} - X_r^-}+\Delta X^{s} - \Delta X_r
$$






### 2、对流层模型

对流层延迟分为两部分：**干延迟**（hydrostatic delay  取决于温度、压力）、**湿延迟**（wet delay  取决于湿度）。每个部分都可以表示为天顶延迟和高度角投影的乘积：
$$
\tau_r^s = m{H}(\theta{el,r}^s) \tau{ZHD,r} +  m{W}(\theta{el,r}^s) \tau{ZWD,r}
$$
在 Ginan 中，假定干延迟是确定的，天顶湿延迟可以

如果对湿延迟进行估计，





