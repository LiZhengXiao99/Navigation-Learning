## Condensed Kalman filter implementation methods 
This repository contains MATLAB functions for various discrete-time Kalman filter (KF) implementation methods. They are given in a priori form (the predicted form), i.e., the first measurement is available at the initial step and, hence, the measurement update stage comes first. All such methods can be written in the so-called condensed form, i.e., without division on the time and measurement updates. Thus, only condensed algorithms are mentioned in this repository. Two-stage implementations can be easily obtained from the algorithms presented <a href="https://github.com/Maria-Kulikova/KF-a-posteriori">here</a>.   

#### References
Each code (implementation method) includes the exact reference where the particular algorithm was published. 
If you use these codes in your research, please, cite the corresponding articles mentioned in the codes or in the list below.  

#### Remark
The codes have been presented here for their instructional value only. They have been tested with care but are not guaranteed to be free of error and, hence, they should not be relied on as the sole basis to solve problems. 

### Steps to reproduce
- `Test_KFs` is the script that performs Monte Carlo runs for solving filtering problem by various KF implementations.
- `Test_LLF` is the script for calculating the negative log LF by various filtering algorithms. 
- `Illustrate_XP` is the script that illustrates the obtained estimates and the diagonal entries of the error covariance matrix (over time). You can find its call at the end of the script above, which is commented. Just delete this comment sign.
- `Illustrate_LLF` is the script that illustrates the negative log LF calculated by various filtering algorithms. 
- `Simulate_Measurements` stands for simulating the state-space model and generating the measurements for the filtering methods.

When the state is estimated, the resulted errors should be the same for all implementation methods because they are mathematically equivalent to each other. Their numerical properties differ, but the ill-conditioned test examples are not given here. 

### List of the KF implementation methods
**Riccati recursion-based KF implementation methods:**
| Function | Description |
| ---: | :--- |
| `Riccati_KF_standard` | Conventional implementation in one-step condensed form|
| `Riccati_KF_SRCF_QL` | Square-Root Covariance Filter (SRCF) with lower triangular factors[^1]|
| `Riccati_KF_SRCF_QR` | SRCF with upper triangular factors[^1]|
| `Riccati_KF_eSRCF_QL` | Extended SRCF with lower triangular factors[^1]|
| `Riccati_KF_eSRCF_QR` | Extended SRCF with upper triangular factors[^1]|
   
**Chandrasekhar recursion-based KF implementation methods:**
| Function | Description |
| ---: | :--- |
| `Chandrasekhar_KF1` | Conventional implementation[^2]|
| `Chandrasekhar_KF2` | Conventional implementation[^2]|
| `Chandrasekhar_KF3` | Conventional implementation[^2]|
| `Chandrasekhar_KF4` | Conventional implementation[^2]|
  
[^1]: Park, P. and Kailath, T. (1995) New square-root algorithms for Kalman filtering. IEEE Transactions on Automatic Control. 40(5):895-9.  <a href="http://doi.org/10.1109/9.384225">DOI</a> 
[^2]: Morf, M. and Sidhu, G. and Kailath, T. (1974) Some new algorithms for recursive estimation in constant, linear, discrete-time systems. IEEE Transactions on Automatic Control. 19(4):315-23. <a href="http://doi.org/10.1109/TAC.1974.1100576">DOI</a>
