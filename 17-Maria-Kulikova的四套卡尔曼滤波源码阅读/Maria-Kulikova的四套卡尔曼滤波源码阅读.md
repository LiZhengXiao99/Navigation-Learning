[TOC]

## 一、简介

卡尔曼滤波是由 Swerling(1958) 和 Kalman(1960) 作为线性高斯系统中预测和滤波技术而发明的[^1]。

里斯本大学的 Maria Kulikova



代码很好读，

每种算法都附上了相关的论文，

MATLAB的程序

主要是最大相关熵、各种矩阵分解



卡尔曼滤波的概念：

> * **离散 or 连续**：
> * **线性 or 非线性**：
> * **时变 or 时不变**：
> * **高斯 or 非高斯**：
> * **先验 or 后验**：
> * **高斯滤波 or 非参滤波**：
> * **最优 or 非最优**：
> * **似然函数**：
> * **马尔科夫假设**：
> * **贝叶斯定理**：
> * **蒙特卡洛方法**：

卡尔曼滤波有很多增强：

> * **抗差**：
> * **自适应**：
> * **序贯**：
> * **信息**：
> * **分解（LU、Cholesky、QR、VD、SVD、UD）**：
> * **扩展卡尔曼滤波（EKF）**：
> * **无迹卡尔曼滤波（UKF）**：
> * **容积卡尔曼滤波（CKF）**：
> * **粒子滤波（PF）**：
> * **最大相关熵滤波（MCKF）**：
> * **直方图滤波（HF）**：
> * **贝叶斯滤波（BF）**：

补充常见矩阵分解：

> * **LU分解**：$M=LU$ ，$L$ 是下三角矩阵，$U$ 是上三角矩阵。方便求矩阵的行列式和逆，解线性方程组。
> * **Cholesky分解**：$M=L^TL$ ，$L$ 是上三角矩阵。是 $LU$ 分解的进阶版。但是不同于 $LU$ 分解的是，Cholesky分解只适用于正定的对称阵。在复数域，Cholesky 分解要求矩阵是正定的共轭对称矩阵。要求这么多，就是因为 Cholesky 分解可以看成是给矩阵开平方根。 
> * **QR分解**：$M=QR$ ，其中 $Q$ 是正交矩阵，$R$ 是上三角矩阵。主要有三种方法：Gram-Schmidt 正交化法（QR 分解中的 Q 本身就可以看作是正交化构造出来的），Household 变换法，Givens 变换法。 可以用来求矩阵的特征值以及求解最小二乘法。
> * **特征值分解**：$M=VDV^T$ ，其中 $V$ 是正交阵，$D$ 是由 $M$ 的特征值构成的对角矩阵。只适用于方阵，目的是提取出矩阵最重要的特征。
> * **奇异值分解（SVD分解）**：$M=USV^T$ ，其中 $U$、$V$ 是正交矩阵，$S$ 是由 $M$ 的奇异值构成的对角矩阵。特征值分解只适用于方阵，对于普通矩阵，则可以采用奇异值分解，提取出奇异值。
> * **UD分解**：$M=UDU^T$，$U$ 为上三角且对角线元素为 $1$，$D$ 为对角阵。



### 1、KF-a-priori

* 项目地址：https://github.com/Maria-Kulikova/KF-a-priori

* Park, P. and Kailath, T. (1995) **New square-root algorithms for Kalman filtering**. IEEE Transactions on Automatic Control. 40(5):895-9. 

  > * **《新型平方根卡尔曼滤波算法》1995**
  > * **DOI**：http://doi.org/10.1109/9.384225
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：本文介绍了一些新的平方根算法，这些算法可以更可靠地计算状态估计值，并尽可能使用通过正交运算获得的量。文中给出了协方差量和信息量的新算法，还介绍了一种新的组合算法。

* Morf, M. and Sidhu, G. and Kailath, T. (1974) **Some new algorithms for recursive estimation in constant, linear, discrete-time systems**. IEEE Transactions on Automatic Control. 19(4):315-23. 

  > * **《线性时不变离散系统中递归估计的一些新算法》1974**
  >
  > * **DOI**：http://doi.org/10.1109/TAC.1974.1100576
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：本文将线性连续时不变系统的快速卡尔曼滤波算法被扩展到离散时间系统。其主要是用另一套差分方程组（我们称之为 Chandrasekhar 型）取代了通常用于此类问题的 Riccati 型差分方程组。新算法的总运算次数总体上少于基于 Riccati 方程的卡尔曼滤波器，在一些重要的特殊情况下还能显著减少运算次数。这些算法是通过 Riccati 方程变量增量的因式分解得出的，这种方法也可以扩展到非对称 Riccati 方程。



### 2、KF-a-posteriorihttp

* 项目地址：https://github.com/Maria-Kulikova/KF-a-posteriori

* Kalman, R.E. (1960) **A new approach to linear filtering and prediction problems**. Journal of basic Engineering. 1960 Mar, 82(1):35-45.

  > * **《线性滤波和预测的新方法》1960**
  > * **DOI**：https://doi.org/10.1115/1.3662552
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：利用随机过程的 Bode-Shannon 表示法和动态系统分析的 "状态转换 "方法，重新研究了经典的过滤和预测问题。新成果有 (1) 问题的表述和求解方法不加修改地适用于静态和非静态统计，以及增长记忆和无限记忆滤波器。(2) 导出了最优估计误差协方差矩阵的非线性差分（或微分）方程。根据该方程的解，无需进一步计算即可得到最优线性滤波器差分（或微分）方程的协系数。(3) 滤波问题被证明是无噪声调节器问题的对偶问题。本文开发的新方法应用于两个著名问题，证实并扩展了之前的结果。讨论基本上自成一体，从第一原理出发；随机过程理论的基本概念在附录中进行了回顾。

* Bucy, R.S. and Joseph, P.D. **Filtering for Stochastic Processes, with Applications to Guidance**. New York, John Wiley & Sons, 1968.

  > * **《随机过程滤波及应用指导》1968**
  > * **DOI**：
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：

* Swerling, P. (1959) **First order error propagation in a stagewise differential smoothing procedure for satellite observations**, Journal of Astronautical Sciences, V.6, 46--52.

  > * 
  > * **DOI**：
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：

* Grewal, M.S. and Andrews, A.P. **Kalman filtering: theory and practice using MATLAB**. Prentice-Hall, New Jersey, 4th edn., 2015.

  > * 
  > * **DOI**：https://doi.org/10.1115/1.3662552
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：

* Park, P. and Kailath, T. (1995) **New square-root algorithms for Kalman filtering.** IEEE Transactions on Automatic Control. 40(5):895-9.

  > * 
  > * **DOI**：http://doi.org/10.1109/9.384225
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：

* Kulikova, M.V. (2009) **On scalarized calculation of the likelihood function in array square-root filtering algorithms**. Automation and Remote Control. 70:855-71.

  > * 
  > * **DOI**：http://dx.doi.org/10.1134/S0005117909050129
  >
  > * **下载地址**：https://link.springer.com/content/pdf/10.1134/S0005117909050129.
  >
  > * **摘要翻译**：根据卡尔曼滤波公式的阵列平方根实施方法，提出了一种高效的对数似然函数标量化计算方法。与传统的卡尔曼滤波器相比，这种算法对舍入误差的影响更加稳定。测量标量化技术大大降低了算法的计算复杂度。此外，新的实现方法与阵列滤波算法归为一类，因此面向并行计算。计算结果证实了新算法的有效性。

* Wang, L. and Libert, G. and Manneback, P. (1992) **Kalman filter algorithm based on singular value decomposition**. Proceedings of the 31st IEEE Conference on Decision and Control 1992 Dec 16, pp. 1224-1229.

  > * **《基于奇异值分解的卡尔曼滤波》**
  > * **DOI**：http://doi.org/10.1109/CDC.1992.371522
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：针对离散时间线性滤波问题开发了一种算法。该算法的关键部分是计算非对称矩阵的奇异值分解（SVD），而不明确形成其左因子，因为其维度较高。该算法具有良好的数值稳定性，无需任何额外变换即可处理相关测量噪声。由于该算法是以向量-矩阵和矩阵-矩阵运算的形式制定的，因此也适用于并行计算机。下面给出一个数值示例

* Kulikova, M.V. and Tsyganova, J.V. (2017) **Improved discrete‐time Kalman filtering within singular value decomposition.** IET Control Theory & Applications. 11(15):2412-8.

  > * **《改进离散SVD分解卡尔曼滤波》**
  > * **DOI**：http://doi.org/10.1049/iet-cta.2016.1282
  >
  > * **下载地址**：https://arxiv.org/abs/1611.03686
  >
  > * **摘要翻译**：
  > * 本研究提出了一种新的卡尔曼滤波器（KF）实现方法，它适用于受舍入误差影响，相关里卡提方程数值解的精度可能会严重下降的应用场合。自 20 世纪 60 年代出现卡尔曼滤波器以来，人们已经认识到卡尔曼滤波器的因子形式更适合实际应用。基于误差协方差矩阵 Cholesky 分解的平方根算法是最流行、最有效的技术。另一种重要的矩阵因式分解方法是奇异值分解（SVD），因此，在这种方法下可能会有更多令人鼓舞的实施方案。本文的分析表明，之前提出的基于 SVD 的 KF 变体对舍入误差仍然很敏感，对条件不佳的情况处理不佳，尽管基于 SVD 的策略本质上比传统的 KF 方法更稳定。在本研究中，作者设计了一种新的基于 SVD 的 KF 实现，以增强对舍入误差的鲁棒性，提供了其详细推导，并讨论了数值稳定性问题。为了进行比较研究，还进行了一组数值实验。结果表明，基于 SVD 的新方法在代数上等同于传统 KF 和之前提出的基于 SVD 的方法，但在条件不佳的情况下，其估计精度优于上述技术。
  >

* Kulikova, M.V. and Tsyganova, J.V. and Kulikov, G.Yu. (2021) **SVD-based state and parameter estimation approach for generalized Kalman filtering with application to GARCH-in-Mean estimation.** Journal of Computational and Applied Mathematics. 387:112487.

  > * **《基于 SVD 的广义卡尔曼滤波状态和参数估计方法，并应用于伽马平均估计》**
  > * **DOI**：https://doi.org/10.1016/j.cam.2019.112487
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：本文针对经典状态空间模型的一些扩展，提出了一种基于梯度的自适应滤波技术，用于未知状态和参数估计：(i) 线性时变多输入多输出（LTI MIMO）系统；(ii) 带有相关对卡尔曼滤波器（PKF）的线性对马尔可夫模型（PMM）。鲁棒自适应滤波问题的解决方案是以最近用于经典卡尔曼滤波器的奇异值分解（SVD）为基础的。与我们之前的成果不同，新方法是针对滤波递推和灵敏度计算中涉及的薄矩阵（预阵列）的情况而设计的。此外，本文还讨论了利用所提出的创新自适应滤波器对时变方差模型进行校准的问题。特别是探讨了估计 GARCH-in-Mean(1,1) 的状态空间方法。

### 3、MCCKF-a-priori

* 项目地址：https://github.com/Maria-Kulikova/MCCKF-a-priori

* Izanloo, R. and Fakoorian, S.A. and Yazdi, H.S. and Simon D. (2016) **Kalman filtering based on the maximum correntropy criterion in the presence of non-Gaussian noise**, in: 2016 Annual Conference on Information Science and Systems (CISS), 2016, pp. 500-505.

  > * 《非高斯噪声下最大熵准则下卡尔曼滤波》
  > * **DOI**：https://doi.org/10.1109/CISS.2016.7460553
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：本文讨论了存在非高斯噪声时的状态估计。由于卡尔曼滤波器仅使用二阶信号信息，因此在非高斯噪声环境中并不是最优的。最大熵准则（MCC）是一种利用高阶信号统计信息来衡量两个随机变量相似性的新方法。熵滤波器（C-Filter）使用 MCC 进行状态估计。在本文中，我们首先通过修改 C 滤波器的推导过程来改进其性能，从而得到修正熵滤波器（MC-Filter）。接下来，我们利用 MCC 和加权最小二乘法（WLS），提出了卡尔曼滤波器形式的 MCC 滤波器，我们称之为 MCC-KF。仿真结果表明，在存在两种不同类型的非高斯干扰（射出噪声和高斯混合噪声）的情况下，MCC-KF 与 C-滤波器、MC-滤波器、无香味卡尔曼滤波器、集合卡尔曼滤波器和高斯和滤波器相比都更胜一筹。

* Kulikova, M.V. (2017) **Square-root algorithms for maximum correntropy estimation of linear discrete-time systems in presence of non-Gaussian noise.** Systems and Control Letters, 108: 8-15.

  > * 《非高斯噪声下线性离散时间系统最大熵估计的平方根算法》
  > * **DOI**：https://doi.org/10.1016/j.sysconle.2017.07.016
  >
  > * **下载地址**：https://arxiv.org/abs/1611.03686
  >
  > * **摘要翻译**：在存在非高斯噪声的情况下，随机动态系统状态估计领域的最新发展催生了一种名为最大熵滤波的新方法。根据最大熵准则（MCC）设计的滤波器利用两个随机变量之间的相似度量（或熵）作为成本函数。结果表明，它们能提高估计器对异常值或脉冲噪声的鲁棒性。在本文中，我们探讨了最近在 MCC 方法下提出的线性滤波技术的数值稳定性。由此产生的估计器被称为最大熵准则卡尔曼滤波器（MCC-KF）。这项研究有两个目的。首先，我们对之前推导出的 MCC-KF 方程进行了修订，并证明了相关的卡尔曼相等条件。在此理论基础上，我们对 MCC-KF 技术进行了改进，与之前提出的 MCC-KF 变体相比，新方法在降低计算成本的同时拥有更好的估计质量。其次，我们为新设计的改进估计器设计了一些平方根实现方法。众所周知，平方根算法比传统的卡尔曼算法更稳定，后者在滤波器的每个迭代步骤中都要处理全误差协方差矩阵。此外，根据 KF 界的最新成果，这里的所有平方根算法都采用了所谓的阵列形式。这意味着使用正交变换来递归更新所需的滤波量，因此不会造成精度损失。除了数值稳定性方面的优势外，阵列形式还使现代卡尔曼滤波器更适合并行实施和超大规模集成（VLSI）实施。本文开发的所有 MCC-KF 变体都在两个数值示例中证明优于之前提出的 MCC-KF 版本。

* Kulikova, M.V. (2019) **One-Step Condensed Forms for Square-Root Maximum Correntropy Criterion Kalman Filtering, Proceedings of the 23rd International Conference on System Theory**, Control and Computing (ICSTCC), Sinaia, Romania, pp. 13-18.

  > * **《平方根最大熵滤波的一步简化形式》**
  > * **DOI**：http://doi.org/10.1109/ICSTCC.2019.8885950
  >
  > * **下载地址**：https://arxiv.org/abs/2310.18750
  >
  > * **摘要翻译**：本文针对最大熵准则卡尔曼滤波提出了几种基于 Cholesky 的平方根新算法。与之前获得的结果不同，新算法是以所谓的浓缩形式开发的，与先验滤波相对应。众所周知，在解决条件不佳的估计问题时，平方根滤波器的实现具有更好的调节性和更高的数值稳健性。此外，新算法还能更容易地传播状态估计值，并且在计算估计值时不需要进行反代换。通过使用一个四阶基准导航系统示例，对新型滤波方法的性能进行了检验。

* Kulikova, M.V. (2020) **Chandrasekhar-based maximum correntropy Kalman filtering with the adaptive kernel size selection.** IEEE Transactions on Automatic Control, 65(2): 741-748.

  > * 《基于 Chandrasekhar 的最大熵卡尔曼滤波的自适应核大小选择》
  > * **DOI**：https://doi.org/10.1109/TAC.2019.2919341
  >
  > * **下载地址**：https://arxiv.org/abs/2311.01165
  >
  > * **摘要翻译**：本技术说明旨在推导卡尔曼滤波（KF）最大熵准则（MCC）的 Chandrasekhar 型递推公式。对于经典的卡尔曼滤波，第一个 Chandrasekhar 差分方程是在 20 世纪 70 年代初提出的。它替代了传统使用的里卡提递归，并产生了所谓的快速实现方法，即 Morf-Sidhu-Kailath-Sayed KF 算法。由于在里卡提递推中传播的矩阵大小小于 n × n 误差协方差矩阵，因此这些算法被证明计算成本低廉。在 MCC 估算方法中推导 Chandrasekhar 型递推的问题还从未在工程文献中提出过。在本技术论文中，我们迈出了第一步，推导出了自适应核大小选择策略情况下的 Chandrasekhar MCC-KF 估计器，这意味着一个恒定的标量调整权重。数值示例证明了新提出的 MCC-KF 实现方法的实际可行性和理论推导的正确性。

### 4、MCCKF-a-posteriori

* 项目地址：https://github.com/Maria-Kulikova/MCCKF-a-posteriori

* Izanloo, R. and Fakoorian, S.A. and Yazdi, H.S. and Simon D. (2016) **Kalman filtering based on the maximum correntropy criterion in the presence of non-Gaussian noise**, in: 2016 Annual Conference on Information Science and Systems (CISS), 2016, pp. 500-505. 

  > * **《非高斯噪声下最大熵准则下卡尔曼滤波》**
  > * **DOI**：https://doi.org/10.1109/CISS.2016.7460553
  >
  > * **下载地址**：
  >
  > * **摘要翻译**：本文讨论了存在非高斯噪声时的状态估计。由于卡尔曼滤波器仅使用二阶信号信息，因此在非高斯噪声环境中并不是最优的。最大熵准则（MCC）是一种利用高阶信号统计信息来衡量两个随机变量相似性的新方法。熵滤波器（C-Filter）使用 MCC 进行状态估计。在本文中，我们首先通过修改 C 滤波器的推导过程来改进其性能，从而得到修正熵滤波器（MC-Filter）。接下来，我们利用 MCC 和加权最小二乘法（WLS），提出了卡尔曼滤波器形式的 MCC 滤波器，我们称之为 MCC-KF。仿真结果表明，在存在两种不同类型的非高斯干扰（射出噪声和高斯混合噪声）的情况下，MCC-KF 与 C-滤波器、MC-滤波器、无香味卡尔曼滤波器、集合卡尔曼滤波器和高斯和滤波器相比都更胜一筹。

* Kulikova, M.V. (2017) **Square-root algorithms for maximum correntropy estimation of linear discrete-time systems in presence of non-Gaussian noise.** Systems and Control Letters, 108: 8-15.

  > * **《非高斯噪声下线性离散时间系统最大熵估计的平方根算法》**
  > * **DOI**：https://doi.org/10.1016/j.sysconle.2017.07.016
  >
  > * **下载地址**：https://arxiv.org/abs/1611.03686
  >
  > * **摘要翻译**：在存在非高斯噪声的情况下，随机动态系统状态估计领域的最新发展催生了一种名为最大熵滤波的新方法。根据最大熵准则（MCC）设计的滤波器利用两个随机变量之间的相似度量（或熵）作为成本函数。结果表明，它们能提高估计器对异常值或脉冲噪声的鲁棒性。在本文中，我们探讨了最近在 MCC 方法下提出的线性滤波技术的数值稳定性。由此产生的估计器被称为最大熵准则卡尔曼滤波器（MCC-KF）。这项研究有两个目的。首先，我们对之前推导出的 MCC-KF 方程进行了修订，并证明了相关的卡尔曼相等条件。在此理论基础上，我们对 MCC-KF 技术进行了改进，与之前提出的 MCC-KF 变体相比，新方法在降低计算成本的同时拥有更好的估计质量。其次，我们为新设计的改进估计器设计了一些平方根实现方法。众所周知，平方根算法比传统的卡尔曼算法更稳定，后者在滤波器的每个迭代步骤中都要处理全误差协方差矩阵。此外，根据 KF 界的最新成果，这里的所有平方根算法都采用了所谓的阵列形式。这意味着使用正交变换来递归更新所需的滤波量，因此不会造成精度损失。除了数值稳定性方面的优势外，阵列形式还使现代卡尔曼滤波器更适合并行实施和超大规模集成（VLSI）实施。本文开发的所有 MCC-KF 变体都在两个数值示例中证明优于之前提出的 MCC-KF 版本。

* Kulikova, M.V. (2019) **Factored-form Kalman-like implementations under maximum correntropy criterion.** Signal Processing. 160:328-38.

  > * **《最大熵准则下的因式卡尔曼滤波》**
  > * **DOI**：https://doi.org/10.1016/j.sigpro.2019.03.003
  >
  > * **下载地址**：https://arxiv.org/pdf/2311.02440
  >
  > * **摘要翻译**：最大熵准则（MCC）方法被认为是一种对异常值具有鲁棒性的过滤策略，在非高斯噪声存在的情况下，其估计精度优于经典的卡尔曼滤波器（KF）。然而，新提出的 MCC-KF 估计器在有限精度运算中的数值稳定性问题却鲜有涉及。本文为 MCC-KF 及其改进变体分别推导了一个因子形式（平方根）算法族。该系列传统上包括三种因式分解实现方法：(i) 基于 Cholesky 因式分解的算法，(ii) 修改 Cholesky，即基于 UD 的方法，以及 (iii) 最近建立的基于 SVD 的滤波方法。人们普遍认为，所有这些策略都能增强传统滤波对舍入误差的数值稳健性，因此，在解决可靠性要求较高的应用时，它们是首选的实现方法。此前，人们只设计了基于 Cholesky 的 IMCC-KF 算法。本文通过引入基于 UD 和 SVD 的方法，丰富了因子形式系列。本文特别关注数组算法，事实证明，这些算法在数值上最为稳定，而且适合并行执行。本文讨论了这些算法的理论特性，并进行了数值比较，以确定最可靠的实现方法。
  >
  >
  >   通过www.DeepL.com/Translator（免费版）翻译

* Kulikova, M.V. (2020) **On the stable Cholesky factorization-based method for the maximum correntropy criterion Kalman filtering.** IFAC-PapersOnLine. 53(2):482-7.

  > * **《基于 Cholesky 分解的最大熵卡尔曼滤波》**
  > * **DOI**：https://doi.org/10.1016/j.ifacol.2020.12.264
  >
  > * **下载地址**：https://arxiv.org/pdf/2311.02438
  >
  > * **摘要翻译**：本文继续致力于设计最大熵准则卡尔曼滤波（MCC-KF）的数值稳定平方根实现方法。与之前获得的结果不同，我们在此揭示了基于 Cholesky 因子化方法的第一种稳健方法（与舍入误差有关）。该方法用协方差矩阵的平方根因子来表述，即属于协方差型滤波方法。此外，在算法的每次迭代中都使用了数值稳定的正交变换，以精确传播所涉及的 Cholesky 因子。数值实验结果表明，与传统算法及其之前发布的基于 Cholesky 的变体相比，新型 MCC-KF 实现具有更优越的性能。

* Kulikova, M.V. (2020) **Sequential maximum correntropy Kalman filtering.** Asian Journal of Control. 22(1):25-33.

  > * **《顺序最大熵卡尔曼滤波》**
  > * **DOI**：https://doi.org/10.1002/asjc.1865
  >
  > * **下载地址**：https://onlinelibrary.wiley.com/doi/pdf/10.1002/asjc.1865
  >
  > * **摘要翻译**：本文探讨了非高斯背景下的线性状态估计问题，并提出了一种基于最大熵准则卡尔曼滤波器（MCC-KF）的计算简单的估计方法。第一种 MCC-KF 方法是以约瑟夫稳定形式开发的。它需要进行两次 n × n 和一次 m × m 矩阵反演，其中 n 是要估计的未知动态状态的维数，m 是可用测量矢量的维数。因此，当系统维度增加时，估计器就变得不切实际了。我们之前的工作提出了一种改进的 MCC-KF 估计器（IMCC-KF）及其因式分解（平方根）实现方法，可提高 MCC-KF 估计质量和数值鲁棒性，防止出现舍入误差。然而，所提出的 IMCC-KF 及其平方根实现仍需要在滤波器的每个迭代步骤中进行 m × m 矩阵反演。出于数值稳定性和计算复杂性的考虑，最好避免矩阵反转操作。在本文中，我们提出了一种新的 IMCC-KF 算法，它比原来的 MCC-KF 和之前提出的 IMCC-KF 更精确，计算成本更低。此外，与稳定的平方根算法相比，新方法同样精确，但计算成本更低。数值实验结果证实了新估计器在数值实例中的上述特性。

## 二、





---

## 参考文献

[^1]:概率机器人P30

