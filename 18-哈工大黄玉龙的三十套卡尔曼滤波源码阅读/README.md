<div align="center">
    <a name="Top"></a>
	<h1>哈工大黄玉龙的三十套卡尔曼滤波</h1>
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
</div>
<br/>

### 01-A Novel Robust Student's t-Based Kalman Filter

* **标题翻译**：《基于 Student's t 的抗差卡尔曼滤波》【[程序](https://www.researchgate.net/publication/318707513_Matlab_code_for_the_paper_A_Novel_Robust_Student%27s_t-Based_Kalman_Filter?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：A novel robust Student's t-based Kalman filter is proposed by using the variational Bayesian approach, which provides a Gaussian approximation to the posterior distribution. Simulation results for a manoeuvring target tracking example illustrate that the proposed filter has smaller root mean square error and bias than existing filters.
* **摘要翻译**：本文提出了一种新型的基于 Student's t 的变分贝叶斯卡尔曼滤波，该滤波器提供了后验分布的高斯近似值。针对机动目标跟踪示例的仿真结果表明，与现有的滤波器相比，新提出的滤波器具有更小的均方根误差和偏差。

---

### 02-Robust Student’s t based Stochastic Cubature Filter for Nonlinear Systems with Heavy-tailed Process and Measurement Noises

* **标题翻译**：《针对非线性系统重尾过程噪声和量测噪声的抗差 Student's t 容积卡尔曼滤波》【[程序](https://www.researchgate.net/publication/318862518_Matlab_code_for_the_paper_Robust_Student%27s_t_based_Stochastic_Cubature_Filter_for_Nonlinear_Systems_with_Heavy-tailed_Process_and_Measurement_Noises?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a new robust Student's t based stochastic cubature filter (RSTSCF) is proposed for nonlinear state-space model with heavy-tailed process and measurement noises. The heart of the RSTSCF is a stochastic Student's t spherical radial cubature rule (SSTSRCR), which is derived based on the third-degree unbiased spherical rule and the proposed third-degree unbiased radial rule. The existing stochastic integration rule is a special case of the proposed SSTSRCR when the degrees of freedom parameter tends to infinity. The proposed filter is applied to a manoeuvring bearings-only tracking example, where an agile target is tracked and the bearing is observed in clutter. Simulation results show that the proposed RSTSCF can achieve higher estimation accuracy than the existing Gaussian approximate filter, Gaussian sum filter, Huber-based nonlinear Kalman filter, maximum correntropy criterion based Kalman filter, and robust Student's t based nonlinear filters, and is computationally much more efficient than the existing particle filter.
* **摘要翻译**：本文针对具有重尾过程噪声和量测噪声的非线性状态空间模型，提出了一种新的基于Student's t 的抗差随机容积滤波器（RSTSCF）。RSTSCF 的核心是随机 Student's t 球面径向容积规则（SSTSRCR），它是基于三度无偏球面规则和新提出的三度无偏径向规则推导出来的。当自由度参数趋于无穷大时，现有的随机积分规则是新提出的的 SSTSRCR 的特例。将新提出的滤波器应用于只跟踪方位的机动跟踪示例中，在该示例中跟踪一个快速移动的目标，并在杂波中观测方位。仿真结果表明，与现有的高斯近似滤波器、高斯和滤波器、基于 Huber 的非线性卡尔曼滤波器、基于最大熵准则的卡尔曼滤波器和基于抗差 Student's t 的非线性滤波器相比，新提出的 RSTSCF 可以获得更高的估计精度，而且比现有的粒子滤波计算效率更高。

---

### 03-Quasi-stochastic integration filter for nonlinear estimation

* **标题翻译**：《用于非线性估计的准随机积分滤波器》【[程序](https://www.researchgate.net/publication/319074949_Matlab_code_for_the_paper_Quasi-stochastic_integration_filter_for_nonlinear_estimation?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In practical applications, numerical instability problem, systematic error problem caused by nonlinear approximation, and nonlocal sampling problem for high-dimensional applications, exist in unscented Kalman filter (UKF). To solve these problems, a quasi-stochastic integration filter (QSIF) for nonlinear estimation is proposed in this paper. nonlocal sampling problem is solved based on the unbiased property of stochastic spherical integration rule, which can also reduce systematic error and improve filtering accuracy. In addition, numerical instability problem is solved by using fixed radial integration rule. Simulations of bearing-only tracking model and nonlinear filtering problem with different state dimensions show that the proposed QSIF has higher filtering accuracy and good numerical stability as compared with existing methods, and it can also solve nonlocal sampling problem effectively.
* **摘要翻译**：在实际应用中，无迹卡尔曼滤波（UKF）存在数值不稳定性问题、非线性近似引起的系统误差以及高维应用中的非局部采样问题。为了解决这些问题，本文提出了一种用于非线性估计的准随机积分滤波器（QSIF）。非局部采样问题是基于随机球形积分规则的无偏特性解决的，它还可以减少系统误差，提高滤波精度。此外，还利用固定径向积分规则解决了数值不稳定性问题。对不同状态维数的纯方位跟踪模型和非线性滤波问题的仿真表明，与现有算法相比，新提出的 QSIF 具有更高的滤波精度和良好的数值稳定性，而且还能有效解决非局部采样问题。

---

### 04-A high order unscented Kalman filtering method

* **标题翻译**：《一种高阶无迹卡尔曼滤波》【[程序](https://www.researchgate.net/publication/319074300_Matlab_code_for_the_paper_A_high_order_unscented_Kalman_filtering_method?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：Currently there is still no specific analytical solution for the high order unscented transform(UT), thus high order UT can not be used to obtain high order unscented Kalman filter(UKF) with higher accuracy. In order to solve this problem, an analytical solution of high order UT is obtained by introducing a free parameter κ on the basis of fifth-order cubature transform(CT), and the high order UKF is then obtained. It is illustrated that the existing fifth-order CT and fifth-order UT are two special cases of the high order UT when κ=2 and κ = 6-n, respectively. Furthermore, the optimal choice of parameter κ in the high order UKF is analyzed and discussed for different dimensional systems, and the stability of the proposed method is discussed. Simulations based on the bearings-only tracking model and ballistic object reentry model show that the proposed method is correct and it has better performance as compared with the existing methods.
* **摘要翻译**：目前，高阶无符号变换（UT）还没有具体的解析解，因此无法利用高阶UT得到精度更高的高阶无符号卡尔曼滤波器（UKF）。为了解决这个问题，我们在五阶容积变换（CT）的基础上引入自由参数κ，得到了高阶UT的解析解，进而得到了高阶UKF。结果表明，当 κ=2 和 κ=6-n 时，现有的五阶 CT 和五阶 UT 分别是高阶 UT 的两个特例。此外，针对不同维度的系统，分析和讨论了高阶 UKF 中参数 κ 的最优选择，并讨论了新算法的稳定性。基于纯方位跟踪模型和弹道物体重返模型的仿真表明，新算法是正确的，与现有算法相比具有更好的性能。

---

### 05-Interpolatory cubature Kalman filters

* **标题翻译**：《插值容积卡尔曼滤波》【[程序](https://www.researchgate.net/publication/319073769_Matlab_code_for_the_paper_Interpolatory_cubature_Kalman_filters?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this study, a novel class of interpolatory cubature Kalman filters (ICKFs) with arbitrary degrees of accuracy, including the third-degree ICKF and the fifth-degree ICKF, are proposed based on the interpolatory cubature rule. Existing cubature Kalman filter and unscented Kalman filter can both be deemed as special cases of the proposed ICKFs. Furthermore, the proposed fifth-degree ICKF can achieve higher filtering accuracy than existing fifth-degree methods by properly choosing free parameters, as illustrated in numerical simulation.
* **摘要翻译**：本研究基于插值容积规则，提出了一类具有任意精度的新型插值容积卡尔曼滤波器（ICKF），包括三度 ICKF 和五度 ICKF。现有的容积卡尔曼滤波器和无香精卡尔曼滤波器都可以视为新提出的 ICKF 的特例。此外，如数值模拟所示，通过适当选择自由参数，新提出的五度 ICKF 可以达到比现有五度算法更高的滤波精度。

---

### 06-Embedded cubature Kalman filter with adaptive setting of free parameter

* **标题翻译**：《自由参数自适应的容积卡尔曼滤波》【[程序](https://www.researchgate.net/publication/319072988_Matlab_code_for_the_paper_Embedded_cubature_Kalman_filter_with_adaptive_setting_of_free_parameter?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：The choice of free parameter in embedded cubature Kalman filter (ECKF) is important, and it is difficult to choose an optimal value in practice. To solve this problem, an adaptive method is proposed to determine the value of free parameter of ECKF based on maximum likelihood criterion. By incorporating this method in the third-degree ECKF, a new third-degree adaptive ECKF (AECKF) algorithm is obtained. To further improve the accuracy of the third-degree AECKF, a new fifth-degree AECKF based on the fifth-degree embedded cubature rule is developed. Simulation results show that the proposed algorithms have higher estimation accuracy than existing methods. (C) 2015 Elsevier B.V. All rights reserved.
* **摘要翻译**：嵌入式立体卡尔曼滤波器（ECKF）中自由参数的选择非常重要，在实际应用中很难选择一个最佳值。为了解决这个问题，我们提出了一种基于最大似然准则的自适应算法来确定 ECKF 的自由参数值。通过将该算法纳入三度 ECKF，得到了一种新的三度自适应 ECKF（AECKF）算法。为了进一步提高三度 AECKF 的精确度，还推导了一种基于五度嵌入容积规则的新的五度 AECKF。仿真结果表明，与现有算法相比，新提出的算法具有更高的估计精度。(C) 2015 Elsevier B.V. 版权所有。保留所有权利。

---

### 07-Statistical Similarity Measure-based Adaptive Outlier-Robust State Estimator With Applications

* **标题翻译**：《基于统计相似度量的自适应离群稳态估计器及其应用》【[程序](https://www.researchgate.net/publication/364330082_Code_on_Statistical_Similarity_Measure-based_Adaptive_Outlier-Robust_State_Estimator_With_Applications?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：This article presents an adaptive outlier-robust state estimator (AORSE) under the statistical similarity measures (SSMs) framework. Two SSMs are first proposed to evaluate the similarities between a pair of positive definite random matrices and between a pair of weighted random vectors, respectively. The AORSE is developed by maximizing a hybrid SSMs based cost function, wherein the posterior density function of the hidden state is assumed as a Gaussian distribution with the posterior covariance being approximately determined in a heuristic way. Simulation and experimental examples of moving-target tracking demonstrate the effectiveness of the proposed algorithm.
* **摘要翻译**：本文在统计相似度量（SSMs）框架下提出了一种自适应离群稳健状态估计器（AORSE）。首先提出了两种 SSM，分别用于评估一对正定随机矩阵之间和一对加权随机向量之间的相似性。AORSE 是通过最大化基于混合 SSMs 的成本函数推导的，其中假设隐藏状态的后验密度函数为高斯分布，后验协方差以启发式算法近似确定。移动目标跟踪的仿真和实验实例证明了新算法的有效性。

---

### 08-A robust Gaussian approximate fixed-interval smoother for nonlinear systems with heavy-tailed process and measurement noises

* **标题翻译**：《针对具有重尾过程和量测噪声的非线性抗差高斯近似固定区间平滑器》【[程序](https://www.researchgate.net/publication/320404208_Matlab_code_for_the_paper_A_robust_Gaussian_approximate_fixed-interval_smoother_for_nonlinear_systems_with_heavy-tailed_process_and_measurement_noises?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this letter, a robust Gaussian approximate (GA) fixed-interval smoother for nonlinear systems with heavy-tailed process and measurement noises is proposed. The process and measurement noises are modeled as stationary Student's t distributions, and the state trajectory and noise parameters are inferred approximately based on the variational Bayesian (VB) approach. Simulation results show the efficiency and superiority of the proposed smoother as compared with existing smoothers.
* **摘要翻译**：本文针对具有重尾过程和量测噪声的非线性系统，提出了一种稳健的高斯近似（GA）定值平滑器。过程噪声和量测噪声被建模为静态的 Student's t 分布，状态轨迹和噪声参数是基于变分贝叶斯（VB）算法近似推断的。仿真结果表明，与现有的平滑器相比，新提出的平滑器效率更高、性能更优。

---

### 09-A Novel Adaptive Kalman Filter with Inaccurate Process and Measurement Noise Covariance Matrices

* **标题翻译**：《具有不准确过程和量测噪声协方差矩阵的新型自适应卡尔曼滤波》【[程序](https://www.researchgate.net/publication/322202767_Matlab_code_for_the_paper_A_Novel_Adaptive_Kalman_Filter_with_Inaccurate_Process_and_Measurement_Noise_Covariance_Matrices?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a novel variational Bayesian (VB) based adaptive Kalman filter (VBAKF) for linear Gaussian state-space models with inaccurate process and measurement noise covariance matrices is proposed. By choosing inverse Wishart priors, the state together with the predicted error and measurement noise covariance matrices are inferred based on the VB approach. Simulation results for a target tracking example illustrate that the proposed VBAKF has better robustness to resist the uncertainties of process and measurement noise covariance matrices than existing state-of-the-art filters.
* **摘要翻译**：本文针对过程和量测噪声协方差矩阵不准确的线性高斯状态空间模型，提出了一种基于变分贝叶斯（VB）的新型自适应卡尔曼滤波器（VBAKF）。通过选择逆 Wishart 前验，基于 VB 算法推断出状态以及预测误差和量测噪声协方差矩阵。目标跟踪示例的仿真结果表明，与现有的最先进滤波器相比，新提出的 VBAKF 在面对过程和量测噪声协方差矩阵的不确定性方面具有更好的抗差性。

---

### 10-Design of Gaussian Approximate Filter and Smoother for Nonlinear Systems with Correlated Noises at One Epoch Apart

* **标题翻译**：《为具有相干噪声的非线性系统设计高斯近似滤波器和平滑器》【[程序](https://www.researchgate.net/publication/322365033_Matlab_codes_for_the_paper_Design_of_Gaussian_Approximate_Filter_and_Smoother_for_Nonlinear_Systems_with_Correlated_Noises_at_One_Epoch_Apart?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this study, the authors investigate the filtering and smoothing problems of nonlinear systems with correlated noises at one epoch apart. A pseudomeasurement equation is firstly reconstructed with a corresponding pseudomeasurement noise, which is no longer correlated with the process noise. Based on the reconstructed measurement model, new Gaussian approximate (GA) filter and smoother are derived, from which Kalman filter and smoother can be obtained for linear systems. For nonlinear systems, different GA filters and smoothers can be developed through utilizing different numerical methods for computing Gaussian-weighted integrals involved in the proposed solution. Numerical examples concerning univariate nonstationary growth model, passive ranging problem, and target tracking show the efficiency of the proposed filtering and smoothing methods for nonlinear systems with correlated noises at one epoch apart.
* **摘要翻译**：在这项研究中，作者探讨了非线性系统的滤波和平滑问题，这些非线性系统具有相隔一个纪元的相关噪声。首先用与过程噪声不再相关的相应伪量测噪声重建伪量测方程。根据重建后的量测模型，推导出新的高斯近似（GA）滤波器和平滑器，由此可得到线性系统的卡尔曼滤波器和平滑器。对于非线性系统，通过利用不同的数值算法计算建议解决方案中涉及的高斯加权积分，可以推导出不同的 GA 滤波器和平滑器。有关单变量非平稳增长模型、被动测距问题和目标跟踪的数值示例显示了新提出的滤波和平滑算法对于具有相隔一个纪元的相关噪声的非线性系统的效率。

---

### 11-Improved square-root cubature information filter

* **标题翻译**：《改进的平方根信息容积卡尔曼滤波》【[程序](https://www.researchgate.net/publication/322976256_Matlab_Codes_for_the_paper_Improved_square-root_cubature_information_filter?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a theoretical comparison between existing the sigma-point information filter (SPIF) framework and the unscented information filter (UIF) framework is presented. It is shown that the SPIF framework is identical to the sigma-point Kalman filter (SPKF). However, the UIF framework is not identical to the classical SPKF due to the neglect of one-step prediction errors of measurements in the calculation of state estimation error covariance matrix. Thus SPIF framework is more reasonable as compared with UIF framework. According to the theoretical comparison, an improved cubature information filter (CIF) is derived based on the superior SPIF framework. Square-root CIF (SRCIF) is also developed to improve the numerical accuracy and stability of the proposed CIF. The proposed SRCIF is applied to a target tracking problem with large sampling interval and high turn rate, and its performance is compared with the existing SRCIF. The results show that the proposed SRCIF is more reliable and stable as compared with the existing SRCIF. Note that it is impractical for information filters in large-scale applications due to the enormous computational complexity of large-scale matrix inversion, and advanced techniques need to be further considered.
* **摘要翻译**：本文对现有的 Σ 点信息滤波（SPIF）和无迹信息滤波（UIF）进行了理论比较。结果表明，SPIF 框架与Σ点卡尔曼滤波器（SPKF）相同。但是，由于在计算状态估计误差协方差矩阵时忽略了量测的一步预测误差，UIF 框架与经典的 SPKF 并不相同。因此，与 UIF 框架相比，SPIF 框架更为合理。根据理论比较，在优越的 SPIF 框架基础上推导出了改进的容积信息滤波器（CIF）。为了提高容积信息滤波器的数值精度和稳定性，还推导了平方根容积信息滤波器（SRCIF）。将新提出的 SRCIF 应用于大采样间隔和高转向率的目标跟踪问题，并将其性能与现有的 SRCIF 进行了比较。结果表明，与现有的 SRCIF 相比，新提出的的 SRCIF 更可靠、更稳定。需要注意的是，由于大规模矩阵反演的计算复杂度巨大，因此在大规模应用中用于信息滤波器是不切实际的，需要进一步考虑先进的技术。

---

### 12-A robust and efficient system identification method for a state-space model with heavy-tailed process and measurement noises

* **标题翻译**：《具有重尾过程噪声和量测噪声的状态空间模型的稳健高效系统识别算法》【[程序](https://www.researchgate.net/publication/323445775_Matlab_code_for_the_paper_%27A_robust_and_efficient_system_identification_method_for_a_state-space_model_with_heavy-tailed_process_and_measurement_noises%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In the paper, a robust and efficient system identification method is proposed for a state-space model with heavy-tailed process and measurement noises by using the maximum likelihood criterion. An expectation maximization algorithm for a state-space model with heavy-tailed process and measurement noises is derived by treating auxiliary random variables as missing data, based on which a new nonlinear system identification method is proposed. Noise parameter estimations are updated analytically and model parameter estimations are updated approximately based on the Newton method. The effectiveness of the proposed method is illustrated in a numerical example concerning a univariate non-stationary growth model.
* **摘要翻译**：本文利用最大似然准则，为具有重尾过程和量测噪声的状态空间模型提出了一种稳健高效的系统识别算法。通过将辅助随机变量视为缺失数据，推导出了具有重尾过程和量测噪声的状态空间模型的期望最大化算法，并在此基础上提出了一种新的非线性系统识别算法。噪声参数估计是通过分析更新的，而模型参数估计是基于牛顿法近似更新的。通过一个关于单变量非平稳增长模型的数值示例，证明了新算法的有效性。

---

### 13-Design of High-Degree Student’s t-Based Cubature Filters

* **标题翻译**：《设计基于 Student’s t 的高阶容积卡尔曼滤波》【[程序](https://www.researchgate.net/publication/328394103_Matlab_codes_for_the_paper_%27Design_of_High-Degree_Student%27s_t-Based_Cubature_Filters%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In the paper, a robust and efficient system identification method is proposed for a state-space model with heavy-tailed process and measurement noises by using the maximum likelihood criterion. An expectation maximization algorithm for a state-space model with heavy-tailed process and measurement noises is derived by treating auxiliary random variables as missing data, based on which a new nonlinear system identification method is proposed. Noise parameter estimations are updated analytically and model parameter estimations are updated approximately based on the Newton method. The effectiveness of the proposed method is illustrated in a numerical example concerning a univariate non-stationary growth model.
* **摘要翻译**：本文针对具有重尾过程和量测噪声的状态空间模型，提出了一种基于最大似然准则稳健高效的系统识别算法。通过将辅助随机变量视为缺失数据，推导出了具有重尾过程和量测噪声的状态空间模型的期望最大化算法，并在此基础上提出了一种新的非线性系统识别算法。噪声参数估计是通过分析更新的，而模型参数估计是基于牛顿法近似更新的。通过一个关于单变量非平稳增长模型的数值示例，证明了新算法的有效性。

---

### 14-A robust Gaussian approximate filter for nonlinear systems with heavy tailed measurement noises

* **标题翻译**：《针对重尾量测噪声非线性系统的抗差高斯近似滤波》【[程序](https://www.researchgate.net/publication/329782429_Matlab_code_for_the_paper_A_robust_Gaussian_approximate_filter_for_nonlinear_systems_with_heavy_tailed_measurement_noises%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：The scale matrix and degrees of freedom (dof) parameter of a Student's t distribution are important for nonlinear robust inference, and it is difficult to determine exact values in practical application due to complex environments. To solve this problem, an improved robust Gaussian approximate (GA) filter is derived based on the variational Bayesian approach, where the state together with unknown scale matrix and dof parameter are inferred. The proposed filter is applied to a target tracking problem with measurement outliers, and its performance is compared with an existing robust GA filter with fixed scale matrix and dof parameter. The results show the efficiency and superiority of the proposed filter as compared with the existing filter.
* **摘要翻译**：Student's t 分布的规模矩阵和自由度参数对非线性抗差推理非常重要，但由于环境复杂，在实际应用中很难确定精确值。为了解决这个问题，我们基于变分贝叶斯算法推导出了一种改进的抗差高斯近似（GA）滤波器，在这种滤波器中，状态与未知的尺度矩阵和自由度参数一起被推断出来。将新提出的滤波器应针对具有量测异常值的目标跟踪问题，并将其性能与现有的具有固定尺度矩阵和 dof 参数的抗差 GA 滤波器进行比较。结果表明，与现有滤波器相比，新提出的滤波器效率更高、性能更优。

---

### 15-A novel robust Gaussian-Student's t mixture distribution based Kalman filter

* **标题翻译**：《基于 Gaussian-Student's t 混合分布的新型抗差卡尔曼滤波器》【[程序](https://www.researchgate.net/publication/333704751_Matlab_Codes_for_the_paper_A_novel_robust_Gaussian-Student%27s_t_mixture_distribution_based_Kalman_filter%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a novel Gaussian-Student's t mixture (GSTM) distribution is proposed to model non-stationary heavy-tailed noises. The proposed GSTM distribution can be formulated as a hierarchical Gaussian form by introducing a Bernoulli random variable, based on which a new hierarchical linear Gaussian state-space model is constructed. A novel robust GSTM distribution based Kalman filter is proposed based on the constructed hierarchical linear Gaussian state-space model using the variational Bayesian approach. The Kalman filter and robust Student's t based Kalman filter (RSTKF) with fixed distribution parameters are two existing special cases of the proposed filter. The novel GSTM distributed Kalman filter has the important advantage over the RSTKF that the adaptation of the mixing parameter is much more straightforward than learning the degrees of freedom parameter. Simulation results illustrate that the proposed filter has better estimation accuracy than those of the Kalman filter and RSTKF for a linear state-space model with non-stationary heavy-tailed noises.
* **摘要翻译**：本文提出了一种新的高斯-Student's t 混合（GSTM）分布，针对模拟非平稳重尾噪声。通过引入伯努利随机变量，新提出的 GSTM 分布可被表述为分层高斯形式，并在此基础上构建了一个新的分层线性高斯状态空间模型。基于所构建的分层线性高斯状态空间模型，利用变分贝叶斯算法，提出了一种基于稳健 GSTM 分布的新型卡尔曼滤波器。具有固定分布参数的卡尔曼滤波器和基于抗差 Student's t 的卡尔曼滤波器（RSTKF）是新提出的滤波器的两个现有特例。与 RSTKF 相比，新型的 GSTM 分布式卡尔曼滤波器有一个重要优势，即混合参数的调整比自由度参数的学习更直接。仿真结果表明，对于具有非平稳重尾噪声的线性状态空间模型，新提出的滤波器比卡尔曼滤波器和 RSTKF 具有更好的估计精度。

---

### 16-A Novel Robust Kalman Filtering Framework Based on Normal-Skew Mixture Distribution

* **标题翻译**：《基于正态偏斜混合分布的新型抗差卡尔曼滤波框架》【[程序](https://www.researchgate.net/publication/364342087_A_demo_codes_for_A_Novel_Robust_Kalman_Filtering_Framework_Based_on_Normal-Skew_Mixture_Distribution?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this article, a novel normal-skew mixture (NSM) distribution is presented to model the normal and/or heavy-tailed and/or skew nonstationary distributed noises. The NSM distribution can be formulated as a hierarchically Gaussian presentation by leveraging a Bernoulli distributed random variable. Based on this, a novel robust Kalman filtering framework can be developed utilizing the variational Bayesian method, where the one-step prediction and measurement-likelihood densities are modeled as NSM distributions. For implementation, several exemplary robust Kalman filters (KFs) are derived based on some specific cases of NSM distribution. The relationships between some existing robust KFs and the presented framework are also revealed. The superiority of the proposed robust Kalman filtering framework is validated by a target tracking simulation example.
* **摘要翻译**：本文提出了一种新型的正态-倾斜混合（NSM）分布，针对模拟正态和/或重尾和/或倾斜非平稳分布噪声。通过利用伯努利分布式随机变量，NSM 分布可被表述为分层高斯呈现。在此基础上，可以利用变分贝叶斯算法推导出一种新型的稳健卡尔曼滤波框架，其中一步预测和量测概率密度被建模为 NSM 分布。为便于实施，我们根据 NSM 分布的一些特定情况，推导出了几种典型的抗差卡尔曼滤波器（KF）。此外，还揭示了一些现有抗差卡尔曼滤波器与新提出的框架之间的关系。提出的抗差卡尔曼滤波框架的优越性通过一个目标跟踪仿真实例得到了验证。

---

### 17-Robust Rauch-Tung-Striebel Smoothing Framework for Heavy-tailed and or Skew Noises

* **标题翻译**：《重尾和/或偏斜噪声的抗差 Rauch-Tung-Striebel 平滑框架》【[程序](https://www.researchgate.net/publication/333842423_Matlab_Codes_for_the_paper_Robust_Rauch-Tung-Striebel_Smoothing_Framework_for_Heavy-tailed_andor_Skew_Noises?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：A novel robust Rauch-Tung-Striebel smoothing framework is proposed based on a generalized Gaussian scale mixture (GGScM) distribution for a linear state-space model with heavy-tailed and/or skew noises. The state trajectory, mixing parameters, and unknown distribution parameters are jointly inferred using the variational Bayesian approach. As such, a major contribution of this paper is unifying results within the GGScM distribution framework. Simulation and experimental results demonstrate that the proposed smoother has better accuracy than existing smoothers.
* **摘要翻译**：针对具有重尾和/或偏斜噪声的线性状态空间模型，提出了一种基于广义高斯尺度混合（GGScM）分布的新型抗差 Rauch-Tung-Striebel 平滑框架。使用变分贝叶斯算法共同推断了状态轨迹、混合参数和未知分布参数。因此，本文的主要贡献在于统一了 GGScM 分布框架内的结果。仿真和实验结果表明，新提出的平滑器比现有的平滑器具有更高的精度。

---

### 18-自动化学报论文《带有色厚尾量测噪声的抗差高斯近似滤波器和平滑器》

* **摘要原文**：为了解决带有色厚尾量测噪声的非线性状态估计问题，本文提出了新的抗差高斯近似(Gaussian approximate,GA)滤波器和平滑器。首先，基于状态扩展算法将量测差分后带一步延迟状态和白色厚尾量测噪声的非线性状态估计问题，转化成带厚尾量测噪声的标准非线性状态估计问题。其次，针对量测差分后模型中的噪声尺度矩阵和自由度（Degrees of freedom,DOF)参数未知问题，设计了新的高斯近似滤波器和平滑器，通过建立未知参数和待估计状态的共轭先验分布，并利用变分贝叶斯算法同时估计未知的状态、尺度矩阵、自由度参数。最后，利用目标跟踪仿真验证了本文提出的带有色厚尾量测噪声的抗差高斯近似滤波器和平滑器的有效性以及与现有算法相比的优越性。
* **英文摘要**：In this paper,new robust Gaussian approximate(GA)filter and smoother are proposed to solve the problem of nonlinear state estimation with colored heavy tailed measurement noise.Firstly,the nonlinear state estimation problem with one-step delayed state and white heavy tailed measurement noise after measurement differencing is transformed into a standard nonlinear state estimation problem with heavy tailed measurement noise based on the state augmentation approach.Secondly,new GA filter and smoother are designed for the problem of unknown scale matrix and degrees of freedom (DOF)parameter of noise of the model after measurement differencing.The state,scale matrix and DOF param-eter are estimated simultaneously by building the conjugate prior distributions for unknown parameters and estimated state and using variational Bayesian approach.Finally,the efficiency and superiority of the proposed robust GA filter and smoother with colored heavy tailed measurement noise,as compared with existing method,are shown in the simulation of target tracking.

---

### 19-A Novel Kullback-Leilber Divergence Minimization-Based Adaptive Student's t-Filter

* **标题翻译**：《基于 Kullback-Leilber Divergence Minimization 的新型自适应 Student's t 滤波器》【[程序](https://www.researchgate.net/publication/335652820_Matlab_Codes_for_the_paper_A_Novel_Kullback-Leilber_Divergence_Minimization-Based_Adaptive_Student%27s_t-Filter?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, in order to improve the Student's t-matching accuracy, a novel Kullback-Leilber divergence (KLD) minimization-based matching method is firstly proposed by minimizing the upper bound of the KLD between the true Student's t-density and the approximate Student's t-density. To improve the Student's t-modelling accuracy, a novel KLD minimization-based adaptive method is then proposed to estimate the scale matrices of Student's t-distributions, in which the modified evidence lower bound is maximized. A novel KLD minimization-based adaptive Student's t-filter is derived via combining the proposed Student's t-matching technique and the adaptive method. A manoeuvring target tracking example is provided to demonstrate the effectiveness and potential of the proposed filter.
* **摘要翻译**：为了提高Student's t 匹配的精度，本文首先提出了一种基于 Kullback-Leilber 分歧（KLD）最小化的新型匹配算法，即最小化真实Student's t 密度与近似Student's t 密度之间的 KLD 上界。为了提高Student's t 模型的准确性，还提出了一种基于 KLD 最小化的新型自适应算法来估计Student's t 分布的规模矩阵，其中修正的证据下限被最大化。通过结合新提出的Student's t 匹配技术和自适应算法，得出了一种基于 KLD 最小化的新型自适应Student's t 过滤器。本文提供了一个机动目标跟踪示例，以证明新滤波器的有效性和潜力。

---

### 20-A Novel Progressive Gaussian Approximate Filter for Tightly Coupled GNSS/INS Integration

* **标题翻译**：《用于紧组合 GNSS/INS 的新型渐进高斯滤波器》【[程序](https://www.researchgate.net/publication/335685031_Matlab_codes_for_the_paper_A_Novel_Progressive_Gaussian_Approximate_Filter_for_Tightly_Coupled_GNSSINS_Integration?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this article, we focus on addressing the nonlinear filtering problem with large prior uncertainty but high measurement accuracy, which may be encountered in the application of tightly coupled global navigation satellite system (GNSS)/inertial navigation system (INS) integration. Although the existing methods, such as progressive Gaussian approximate filter (PGAF), can address this problem, it has poor estimation accuracy. To improve the estimation accuracy of PGAF, the step sizes as well as the measurement noise covariance matrix (MNCM) are jointly estimated based on the variational Bayesian approach, from which a novel PGAF with variable step size is developed. Tightly coupled GNSS/INS integration simulations illustrate that the proposed filter outperforms the existing methods both in estimation accuracy and rate of convergence.
* **摘要翻译**：在本文中，我们重点讨论了在紧密耦合的全球导航卫星系统（GNSS）/惯性导航系统（INS）集成应用中可能遇到的先验不确定性大但量测精度高的非线性滤波问题。虽然现有的算法，如渐进高斯近似滤波器（PGAF），可以解决这个问题，但其估计精度较差。为了提高 PGAF 的估计精度，基于变分贝叶斯算法对步长和量测噪声协方差矩阵（MNCM）进行了联合估计，并由此推导出一种步长可变的新型 PGAF。紧密耦合的 GNSS/INS 集成模拟表明，新提出的滤波器在估计精度和收敛速度方面都优于现有算法。

---

### 21-A Novel Adaptive Kalman Filter With Unknown Loss Probability of Measurement

* **标题翻译**：《量测损失概率未知的新型自适应卡尔曼滤波》【[程序](https://www.researchgate.net/publication/337136266_Matlab_code_for_the_paper_A_Novel_Adaptive_Kalman_Filter_With_Unknown_Loss_Probability_of_Measurement?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：A novel variational Bayesian (VB)-based adaptive Kalman filter (AKF) is proposed to solve the filtering problem of a linear system with unknown probability of measurement loss. The sum of two likelihood functions is transformed into an exponential multiplication form, and the state vector, the Bernoulli random variable and the probability of measurement loss are jointly inferred based on the VB approach. Simulation results demonstrate the superiority of the proposed AKF as compared with the existing filtering algorithms with unknown probability of measurement loss.
* **摘要翻译**：本文提出了一种基于变分贝叶斯法（VB）的新型自适应卡尔曼滤波器（AKF），针对解决具有未知量测损失概率的线性系统的滤波问题。两个似然函数之和被转化为指数乘法形式，状态向量、伯努利随机变量和量测损失概率是基于变分贝叶斯算法联合推断的。仿真结果表明，与现有的量测损失概率未知的滤波算法相比，新提出的 AKF 更为优越。

---

### 22-An Improved Kalman Filter with Adaptive Estimate of Latency Probability

* **标题翻译**：《自适应估计延迟概率的改进卡尔曼滤波》【[程序](https://www.researchgate.net/publication/337151890_Matlab_code_for_the_paper_An_Improved_Kalman_Filter_with_Adaptive_Estimate_of_Latency_Probability?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this brief, an improved Kalman filter is proposed for a linear system with one-step randomly delayed measurement and unknown latency probability. The measurement likelihood function which is a weighted sum of two Gaussian distributions is transformed into an exponential multiplication form via importing a discrete Bernoulli random variable. Then, an hierarchical Gaussian form of the state-space model is established. Finally, an improved Kalman filter is deduced to estimate jointly the augmented state vector and the unknown parameters employing the variational Bayesian and state augmentation approaches. Simulation study indicates that the improved method has superior performance in estimation accuracy than the existing methods on the basis of accurate estimation of the unknown and time-varying latency probability.
* **摘要翻译**：本文针对具有一步随机延迟量测和未知延迟概率的线性系统，提出了一种改进的卡尔曼滤波器。量测似然函数是两个高斯分布的加权和，通过导入离散伯努利随机变量，将其转换为指数乘法形式。然后，建立状态空间模型的分层高斯形式。最后，推导出一种改进的卡尔曼滤波器，利用变分贝叶斯法和状态增强法联合估计增强状态向量和未知参数。仿真研究表明，在准确估计未知参数和时变延迟概率的基础上，改进算法的估计精度优于现有算法。

---

### 23-A New Robust Kalman Filter with Adaptive Estimate of Time-Varying Measurement Bias

* **标题翻译**：《量测偏差时变自适应估计功能的新型抗差卡尔曼滤波》【[程序](https://www.researchgate.net/publication/340503702_Matlab_code_for_the_paper_A_New_Robust_Kalman_Filter_with_Adaptive_Estimate_of_Time-Varying_Measurement_Bias?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：To better model the non-Gaussian heavy-tailed measurement noise with unknown and time-varying bias, a new Student's t-inverse-Wishart (STIW) distribution is presented. The STIW distribution is firstly written as a Gaussian, inverse-Wishart and normal-Gamma hierarchical form, from which a new robust Kalman filter is then derived based on the variational Bayesian method. Simulation results illustrate the potentials of the new derived robust Kalman filter for addressing the above measurement noise.
* **摘要翻译**：为了更好地模拟具有未知和时变偏差的非高斯重尾量测噪声，本文提出了一种新的Student's t-inverse-Wishart（STIW）分布。STIW 分布首先被写成高斯、反-Wishart 和正-Gamma 分层形式，然后基于变分贝叶斯算法从中导出一个新的抗差卡尔曼滤波器。仿真结果表明了新推导出的抗差卡尔曼滤波器在解决上述量测噪声方面的潜力。

---

### 24-A Slide Window Variational Adaptive Kalman Filter

* **标题翻译**：《滑动窗口变分自适应卡尔曼滤波》【[程序](https://www.researchgate.net/publication/342466130_Implementation_codes_for_the_paper_A_Slide_Window_Variational_Adaptive_Kalman_Filter?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：A slide window variational adaptive Kalman filter is presented in this paper based on adaptive learning of inaccurate state and measurement noise covariance matrices, which is composed of the forward Kalman filtering, the backward Kalman smoothing, and the online estimates of noise covariance matrices. By imposing an approximation on the smoothing posterior distribution of slide window state vectors, the posterior distributions of noise covariance matrices can be analytically updated as inverse Wishart distributions by exploiting the variational Bayesian method, which avoids the fixed-point iterations and achieves good computational efficiency. Simulation comparisons demonstrate that the proposed method has better filtering accuracy and consistency than the existing cutting-edge method.
* **摘要翻译**：本文提出了一种基于不精确状态和量测噪声协方差矩阵自适应学习的滑动窗口变分自适应卡尔曼滤波器，它由前向卡尔曼滤波、后向卡尔曼平滑和噪声协方差矩阵在线估计组成。通过对滑动窗口状态向量的平滑后验分布施加近似值，利用变分贝叶斯算法可以将噪声协方差矩阵的后验分布解析更新为逆 Wishart 分布，从而避免了定点迭代，实现了良好的计算效率。仿真比较表明，与现有的前沿算法相比，新提出的算法具有更好的滤波精度和一致性。

---

### 25-A Computationally Efficient Variational Adaptive Kalman Filter for Transfer Alignment

* **标题翻译**：《针对转移对齐的计算效率变异自适应卡尔曼滤波器》【[程序](https://www.researchgate.net/publication/342510711_Implementation_codes_for_the_paper_A_Computationally_Efficient_Variational_Adaptive_Kalman_Filter_for_Transfer_Alignment?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：To better solve the filtering problem of transfer alignment with an inaccurate measurement noise covariance matrix, a novel computationally efficient version of existing variational adaptive Kalman filter is proposed in this paper, in which an equivalent variational iteration process of the measurement noise covariance matrix is derived. The proposed filter is identical to the existing variational adaptive Kalman filter, but the total computational complexity of algorithm implementation is significantly reduced, which facilitates the application of variational adaptive Kalman filter to transfer alignment. Simulation and experiment results of transfer alignment demonstrate that the computational complexity of the proposed filter is reduced by 55.4% as compared with existing variational adaptive Kalman filter.
* **摘要翻译**：为了更好地解决量测噪声协方差矩阵不准确时的转移对准滤波问题，本文提出了现有变异自适应卡尔曼滤波器的新型高效计算版本，其中导出了量测噪声协方差矩阵的等效变异迭代过程。新提出的滤波器与现有的变分自适应卡尔曼滤波器相同，但算法实现的总计算复杂度大大降低，这为变分自适应卡尔曼滤波器在转移对齐中的应用提供了便利。转移对齐的仿真和实验结果表明，与现有的变分自适应卡尔曼滤波器相比，新提出的滤波器的计算复杂度降低了 55.4%。

---

### 26-A Novel Outlier-Robust Kalman Filtering Framework based on Statistical Similarity Measure

* **标题翻译**：《基于统计相似性量测的新型离群稳健卡尔曼滤波框架》【[程序](https://www.researchgate.net/publication/345176096_Matlab_codes_for_the_paper_A_Novel_Outlier-Robust_Kalman_Filtering_Framework_based_on_Statistical_Similarity_Measure%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a statistical similarity measure is introduced to quantify the similarity between two random vectors. The measure is then employed to develop a novel outlier-robust Kalman filtering framework. The approximation errors and the stability of the proposed filter are analyzed and discussed. To implement the filter, a fixed-point iterative algorithm and a separate iterative algorithm are given, and their local convergent conditions are also provided, and their comparisons have been made. In addition, selection of the similarity function is considered, and four exemplary similarity functions are established, from which the relations between our new method and existing outlier-robust Kalman filters are revealed. Simulation examples are used to illustrate the effectiveness and potential of the new filtering scheme.
* **摘要翻译**：本文引入了一种统计相似性度量，用于量化两个随机向量之间的相似性。然后，利用该量度推导了一种新型的离群值稳健卡尔曼滤波框架。本文对新提出的滤波器的近似误差和稳定性进行了分析和讨论。为了实现该滤波器，给出了一种定点迭代算法和一种单独的迭代算法，还提供了它们的局部收敛条件，并对它们进行了比较。此外，还考虑了相似性函数的选择，并建立了四个示范性相似性函数，从中揭示了我们的新算法与现有的离群值稳健卡尔曼滤波器之间的关系。仿真实例证明了新滤波方案的有效性和巨大潜力。

---

### 27-A novel multiple-outlier-robust Kalman filter

* **标题翻译**：《新型多异常值稳健卡尔曼滤波器》【[程序](https://www.researchgate.net/publication/351831347_A_demo_code_for_the_paper_%27%27A_novel_multiple-outlier-robust_Kalman_filter%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：This paper presents a novel multiple-outlier-robust Kalman filter(MORKF)for linear stochastic discretetime systems.A new multiple statistical similarity measure is first proposed to evaluate the similarity between two random vectors from dimension to dimension.Then,the proposed MORKF is derived via maximizing a multiple statistical similarity measure based cost function.The MORKF guarantees the convergence of iterations in mild conditions,and the boundedness of the approximation errors is analyzed theoretically.The selection strategy for the similarity function and comparisons with existing robust methods are presented.Simulation results show the advantages of the proposed filter.
* **摘要翻译**：本文首先提出了一种新的多重统计相似度量来评估两个随机向量在不同维度之间的相似性，然后通过最大化基于多重统计相似度量的成本函数推导出了新提出的MORKF。 仿真结果表明了新滤波器的优势。

---

### 28-A robust fixed-interval smoother for nonlinear systems with non-stationary heavy-tailed state and measurement noises

* **标题翻译**：《具有非稳态重尾状态和量测噪声的非线性系统的稳健固定间隔平滑器》【[程序](https://www.researchgate.net/publication/351831280_A_demo_code_for_the_paper_%27%27A_robust_fixed-interval_smoother_for_nonlinear_systems_with_non-stationary_heavy-tailed_state_and_measurement_noises%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：We propose a robust fixed-interval smoother for nonlinear systems with non-stationary heavy-tailed state and measurement noises, in which the state and measurement noises are modelled as Gaussian-Student's t mixture distributions. The variational Bayesian technique is utilized to deduce the smoother approximately. The standard cubature Kalman smoother (CKS) and the robust Gaussian approximate smoother (RGAS) with fixed scale matrices and dof parameters are two particular cases of the proposed smoother. Numerical simulation and target tracking example show the merits of the proposed smoother.
* **摘要翻译**：我们为具有非平稳重尾状态和量测噪声的非线性系统提出了一种稳健的固定区间平滑器，其中状态和量测噪声被建模为高斯-Student's t 混合分布。利用变分贝叶斯技术近似推导出平滑器。标准容积卡尔曼平滑器（CKS）和具有固定比例矩阵和 dof 参数的抗差高斯近似平滑器（RGAS）是新平滑器的两种特殊情况。数值模拟和目标跟踪示例显示了新提出的平滑器的优点。

---

### 29-A Novel Heavy-Tailed Mixture Distribution Based Robust Kalman Filter for Cooperative Localization

* **标题翻译**：《基于重尾混杂分布的新型抗差卡尔曼滤波器用于协同定位》【[程序](https://www.researchgate.net/publication/351831052_A_demo_code_for_the_paper_%27%27A_Novel_Heavy-Tailed_Mixture_Distribution_Based_Robust_Kalman_Filter_for_Cooperative_Localization%27%27?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In cooperative localization for autonomous underwater vehicles (AUVs), the practical stochastic noise may be heavy-tailed and non-stationary distributed because of acoustic speed variation, multi-path effect of acoustic channel, and changeable underwater environment. To address such noise, a novel heavy-tailed mixture (HTM) distribution is firstly proposed and then expressed as a hierarchical Gaussian form by employing a categorical distributed auxiliary vector. Based on that, a novel HTM distribution based robust Kalman filter is proposed, where the one-step prediction and measurement likelihood probability density functions are, respectively, modelled as a HTM distribution and a Normal-Gamma-inverse Wishart distribution. The proposed filter is verified by a lake experiment about cooperative localization for AUVs. Compared with the cutting-edge filter, the proposed filter has been improved by 50.27% in localization error but no more than twice computational time is required.
* **摘要翻译**：在自主水下航行器（AUV）的协同定位中，由于声速变化、声道的多路径效应以及多变的水下环境，实际随机噪声可能是重尾和非稳态分布的。针对这种噪声，首先提出了一种新的重尾混合（HTM）分布，然后通过使用分类分布的辅助向量将其表示为分层高斯形式。在此基础上，提出了一种基于 HTM 分布的新型抗差卡尔曼滤波器，其中一步预测和量测似然概率密度函数分别被模拟为 HTM 分布和正态-伽马-逆 Wishart 分布。提出的滤波器通过 AUV 协同定位的湖泊实验得到了验证。与前沿滤波器相比，新提出的滤波器在定位误差方面提高了 50.27%，但所需计算时间不超过两倍。

---

### 30-A Sliding Window Variational Outlier-Robust Kalman Filter based on Student's t Noise Modelling

* **标题翻译**：《基于 Student's t 噪声建模的滑动窗口变异离群稳健卡尔曼滤波器》【[程序](https://www.researchgate.net/publication/363567877_Matlab_code_of_A_Sliding_Window_Variational_Outlier-Robust_Kalman_Filter_based_on_Student%27s_t_Noise_Modelling?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In cooperative localization for autonomous underwater vehicles (AUVs), the practical stochastic noise may be heavy-tailed and non-stationary distributed because of acoustic speed variation, multi-path effect of acoustic channel, and changeable underwater environment. To address such noise, a novel heavy-tailed mixture (HTM) distribution is firstly proposed and then expressed as a hierarchical Gaussian form by employing a categorical distributed auxiliary vector. Based on that, a novel HTM distribution based robust Kalman filter is proposed, where the one-step prediction and measurement likelihood probability density functions are, respectively, modelled as a HTM distribution and a Normal-Gamma-inverse Wishart distribution. The proposed filter is verified by a lake experiment about cooperative localization for AUVs. Compared with the cutting-edge filter, the proposed filter has been improved by 50.27% in localization error but no more than twice computational time is required.
* **摘要翻译**：在自主水下航行器（AUV）的协同定位中，由于声速变化、声道的多路径效应以及多变的水下环境，实际随机噪声可能是重尾和非稳态分布的。针对这种噪声，首先提出了一种新的重尾混合（HTM）分布，然后通过使用分类分布的辅助向量将其表示为分层高斯形式。在此基础上，提出了一种基于 HTM 分布的新型抗差卡尔曼滤波器，其中一步预测和量测似然概率密度函数分别被模拟为 HTM 分布和正态-伽马-逆 Wishart 分布。提出的滤波器通过 AUV 协同定位的湖泊实验得到了验证。与前沿滤波器相比，新提出的滤波器在定位误差方面提高了 50.27%，但所需计算时间不超过两倍。

---

### 31-Design of sigma-point Kalman filter with recursive updated measurement

* **标题翻译**：《具有递归量测更新的西格玛点卡尔曼滤波》【[程序](https://www.researchgate.net/publication/319551179_Matlab_codes_for_three_papers_about_Gaussian_approximate_filter_with_recursive_measurement_update?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this study, the authors focus on improving measurement update of existing nonlinear Kalman approximation filter and propose a new sigma-point Kalman filter with recursive measurement update. Statistical linearization technique based on sigma transformation is utilized in the proposed filter to linearize the nonlinear measurement function, and linear measurement update is applied gradually and repeatedly based on the statistically linearized measurement equation. The total measurement update of the proposed filter is nonlinear, and the proposed filter can extract state information from nonlinear measurement better than existing nonlinear filters. Simulation results show that the proposed method has higher estimation accuracy than existing methods.
* **摘要翻译**：在这项研究中，作者重点改进了现有非线性卡尔曼近似滤波器的量测更新，并提出了一种具有递归量测更新功能的新型西格玛点卡尔曼滤波器。该滤波器利用基于西格玛变换的统计线性化技术将非线性量测函数线性化，并根据统计线性化量测方程逐步重复应用线性量测更新。新提出的滤波器的总量测更新是非线性的，与现有的非线性滤波器相比，新提出的滤波器能更好地从非线性量测中提取状态信息。仿真结果表明，新提出的算法比现有算法具有更高的估计精度。

---

### 32-Gaussian approximate filter with progressive measurement update

* **标题翻译**：《渐进式量测更新的高斯近似滤波器》【[程序](https://www.researchgate.net/publication/319551179_Matlab_codes_for_three_papers_about_Gaussian_approximate_filter_with_recursive_measurement_update?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：This paper is concerned with recursively estimating the internal state sequence of a discrete-time dynamic system by processing a sequence of noisy measurements taken from the system output. Recursive processing requires some kind of sufficient statistic for representing the information collected up to a certain time step. For this purpose, the probability density functions of the state are especially well suited. Once they are available, almost any type of point estimate, e.g. mean, mode, or median, can be derived. In the case of continuous states, however, the exact probability density functions characterizing the state estimate are in general either not feasible or not well suited for recursive processing. Hence, approximations of the true densities are generally inevitable, where Gaussian mixture approximations are convenient for a number of reasons. However, calculating appropriate mixture parameters that minimize a global measure of deviation from the true density is a tough optimization task. Here, we propose a new approximation method that minimizes the squared integral deviation between the true density and its mixture approximation. Rather than trying to solve the original problem, it is converted into a corresponding system of explicit ordinary first-order differential equations. This system of differential equations is then solved over a finite "time" interval, which is an efficient way of calculating the desired optimal parameter values. We focus on the measurement update in the important case of vector states and scalar measurements. In addition, approximation densities with separable kernels are assumed. It will be shown, that if the measurement nonlinearities are also separable, the required multidimensional integrals can be reduced to the product of one-dimensional integrals. For several important types of measurement functions including polynomial measurement nonlinearities, closed-form analytic expressions for the coefficients of the system of differential equations are available.
* **摘要翻译**：本文涉及通过处理从系统输出中获取的噪声量测序列，递归估计离散时间动态系统的内部状态序列。递归处理需要某种充分的统计量来表示截至某一时间步收集到的信息。为此，状态的概率密度函数尤其适用。一旦有了概率密度函数，几乎任何类型的点估计（如平均值、模式或中位数）都可以得出。然而，对于连续状态，精确描述状态估计值的概率密度函数一般要么不可行，要么不适合递归处理。因此，对真实密度的近似通常是不可避免的，而高斯混合近似由于多种原因是比较方便的。然而，计算适当的混合参数，使偏离真实密度的全局度量最小化，是一项艰巨的优化任务。在这里，我们提出了一种新的近似算法，它能最大限度地减小真实密度与其混合近似值之间的平方积分偏差。我们不是试图求解原始问题，而是将其转换为相应的显式普通一阶微分方程系统。然后在有限的 "时间 "区间内求解这个微分方程系，这是计算所需最优参数值的有效算法。我们重点关注矢量状态和标量量测这一重要情况下的量测更新。此外，我们还假设了具有可分离核的近似密度。我们将证明，如果量测非线性也是可分离的，那么所需的多维积分就可以简化为一维积分的乘积。对于几种重要的量测函数类型，包括多项式量测非线性，微分方程系的系数都有闭式解析表达式。

---

### 33-An improved nonlinear Kalman filter with recursive measurement update

* **标题翻译**：《具有递归量测更新功能的改进型非线性卡尔曼滤波器》【[程序](https://www.researchgate.net/publication/319551179_Matlab_codes_for_three_papers_about_Gaussian_approximate_filter_with_recursive_measurement_update?_tp=eyJjb250ZXh0Ijp7ImZpcnN0UGFnZSI6InByb2ZpbGUiLCJwYWdlIjoicHJvZmlsZSJ9fQ)】
* **摘要原文**：In this paper, a new recursive measurement update strategy is proposed. For linear measurement case, it is identical to standard linear measurement update approach. On the other hand, for nonlinear measurement case, it is superior to existing recursive measurement update strategy because the statistics of measurement noises are updated recursively. An improved nonlinear Kalman filter with recursive measurement update is developed based on the proposed recursion strategy, in which statistical linear regression technique is utilized to statistically linearize the nonlinear measurement function, and linear measurement update is applied gradually and repeatedly based on the linearized measurement equation. The simulations of univariate non-stationary growth model and bearing only tracking show that the proposed method has higher estimation accuracy than existing methods.
* **摘要翻译**：本文提出了一种新的递归量测更新策略。对于线性量测情况，它与标准的线性量测更新算法相同。另一方面，对于非线性量测情况，由于量测噪声的统计量是递归更新的，因此它优于现有的递归量测更新策略。根据新提出的递归策略，利用统计线性回归技术对非线性量测函数进行统计线性化，并根据线性化的量测方程逐步、反复地应用线性量测更新，从而推导出一种具有递归量测更新功能的改进型非线性卡尔曼滤波器。单变量非平稳增长模型和仅轴承跟踪的模拟结果表明，与现有算法相比，新提出的算法具有更高的估计精度。
