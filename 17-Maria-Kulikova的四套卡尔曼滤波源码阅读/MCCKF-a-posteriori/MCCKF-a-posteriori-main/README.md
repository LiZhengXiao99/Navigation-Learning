## Discrete-time MCC-KF and IMCC-KF implementation methods
This repository contains MATLAB functions with various implementation methods of the Maximum Correntropy Criterion Kalman Filter (MCC-KF) by Izanloo et.al. (2016)[^1] and improved MCC-KF (IMCC-KF) by Kulikova (2017)[^2] with a scalar adjusting parameters. They are given in a posteriori form, i.e., no data are assumed to be known at the initial step and, hence, the time update comes first.

#### References
Each code (implementation method) includes the exact reference where the particular algorithm was published. 
If you use these codes in your research, please, cite the corresponding articles mentioned in the codes or in the list below.  

#### Remark
The codes have been presented here for their instructional value only. They have been tested with care but are not guaranteed to be free of error and, hence, they should not be relied on as the sole basis to solve problems. 

### Steps to reproduce
- `Test_MCCKFs` is the script that performs Monte Carlo runs for solving filtering problem by various MCC-KF implementations.
- `Test_IMCCKFs` is the script that performs Monte Carlo runs for solving filtering problem by various IMCC-KF implementations.
- `Test_PI` is the script for calculating the Performance Index (Baram Proximity Measure) by various filtering algorithms. 
- `Illustrate_XP` is the script that illustrates the obtained estimates and the diagonal entries of the error covariance matrix (over time). You can find its call at the end of the script above, which is commented. Just delete this comment sign.
- `Illustrate_PI` is the script that illustrates the Performance Index (Baram Proximity Measure) calculated by various filtering algorithms. 
- `Simulate_Measurements` stands for simulating the state-space model and generating the measurements for the filtering methods.

When the state is estimated, the resulted errors of the MCC-KF implementation methods should be the same because they are mathematically equivalent to each other. Their numerical properties differ, but the ill-conditioned test examples are not given here. Similarly, the resulted errors of the IMCC-KF implementation methods should be the same because they are mathematically equivalent to each other. 

### List of the MCC-KF implementation methods 
**Riccati recursion-based MCC-KF implementation methods:**
| Function | Description |
| ---: | :--- |
| `Riccati_MCCKF` | Conventional implementation, original method[^1]|
| `Riccati_MCCKF_SRCF_QR` | Square-Root Covariance Filter (SRCF) with upper triangular factors[^3]|
| `Riccati_MCCKF_SRCF_QL` | SRCF with lower triangular factors[^4]|
| `Riccati_MCCKF_rSRCF_QL` | Robust SRCF with lower triangular factors[^4]|
| `Riccati_MCCKF_SVD` | SVD-based mixed-type Filter[^3]|
| `Riccati_MCCKF_rSVD` | Robust SVD-based Covariance Filter[^3]|

### List of the IMCC-KF implementation methods 
**Riccati recursion-based IMCC-KF implementation methods:**
| Function | Description |
| ---: | :--- |
| `Riccati_IMCCKF` | Conventional implementation, original method[^2]|
| `Riccati_IMCCKF_seq` |  Sequential (component-wise measurement update) method[^5]|
| `Riccati_IMCCKF_SRCF_QR` | Square-Root Covariance Filter (SRCF) with upper triangular factors[^2]|
| `Riccati_IMCCKF_eSRCF_QR` | Extended SRCF with upper triangular factors[^2]|
| `Riccati_IMCCKF_SVD` | SVD-based mixed-type Filter[^3]|

[^1]: Izanloo, R. and Fakoorian, S.A. and Yazdi, H.S. and Simon D. (2016) Kalman filtering based on the maximum correntropy criterion in the presence of non-Gaussian noise, in: 2016 Annual Conference on Information Science and Systems (CISS), 2016, pp. 500-505. <a href="https://doi.org/10.1109/CISS.2016.7460553">DOI</a>
[^2]: Kulikova, M.V. (2017) Square-root algorithms for maximum correntropy estimation of linear discrete-time systems in presence of non-Gaussian noise. Systems and Control Letters, 108: 8-15. <a href="https://doi.org/10.1016/j.sysconle.2017.07.016">DOI</a>, <a href="https://arxiv.org/abs/1611.03686">PDF on ArXiv</a>
[^3]: Kulikova, M.V. (2019) Factored-form Kalman-like implementations under maximum correntropy criterion. Signal Processing. 160:328-38.  <a href="https://doi.org/10.1016/j.sigpro.2019.03.003">DOI</a>, <a href="https://arxiv.org/pdf/2311.02440">PDF on ArXiv</a>
[^4]: Kulikova, M.V. (2020) On the stable Cholesky factorization-based method for the maximum correntropy criterion Kalman filtering. IFAC-PapersOnLine. 53(2):482-7. <a href="https://doi.org/10.1016/j.ifacol.2020.12.264">DOI</a>, <a href="https://arxiv.org/pdf/2311.02438">PDF on ArXiv</a>
[^5]: Kulikova, M.V. (2020) Sequential maximum correntropy Kalman filtering. Asian Journal of Control. 22(1):25-33. <a href="https://doi.org/10.1002/asjc.1865">DOI</a>, <a href="https://onlinelibrary.wiley.com/doi/pdf/10.1002/asjc.1865">PDF on Wiley</a>

