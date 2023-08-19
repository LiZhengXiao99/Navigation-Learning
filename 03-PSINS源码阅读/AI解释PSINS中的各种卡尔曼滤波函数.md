PSINS 中的 Kalman 滤波代码都在百行以内，没调用什么函数，而且通用性很强，拿去让 AI 解释，效果挺好。

### 一、RLS

RLS（递归最小二乘估计）是一种常用的自适应滤波算法，用于估计线性系统的参数1。

RLS估计的基本思想是通过对输入信号进行迭代最小化误差平方和，来逐步调整估计器的增益系数，从而不断优化估计器的性能。具体来说，RLS估计的步骤包括1：

1. 初始化估计器的增益系数和误差方差。
2. 对于每个输入信号采样，计算输出信号的预测值和实际值之间的误差。
3. 根据误差和当前增益系数，计算出新的误差方差和增益系数。
4. 重复执行步骤2和3，进行迭代优化，直到满足收敛条件为止。

```matlab
function kf = RLS(kf, zk)
% Recursive Least Square filter.
%
% Prototype: kf = RLS(kf, zk)
% Inputs: kf - filter structure array
%         zk - measurement vector
% Output: kf - filter structure array after filtering
%
% See also  kfupdate, RLSPot, RLSUD.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/09/2013
    if ~isfield(kf, 'Rk') 
        kf.Rk = eye(size(kf.Hk,1));
    end
    kf.Pxzk = kf.Pxk*kf.Hk';  
    kf.Pzk = kf.Hk*kf.Pxzk + kf.Rk;  
    kf.Kk = kf.Pxzk*kf.Pzk^-1;
    kf.xk = kf.xk + kf.Kk*(zk-kf.Hk*kf.xk);
    kf.Pxk = kf.Pxk - kf.Kk*kf.Pxzk';
    kf.Pxk = (kf.Pxk+kf.Pxk')*0.5;
```

这是一个递归最小二乘（Recursive Least Square，RLS）滤波器的函数。它用于更新滤波器结构体数组 kf，并根据测量向量 zk 进行滤波操作。

函数的输入参数有 kf 和 zk，其中 kf 是滤波器的结构体数组，zk 是测量向量。函数的输出参数也是 kf，表示经过滤波操作后的滤波器结构体数组。

函数首先判断 kf 结构体数组中是否存在 Rk 字段，如果不存在，则初始化 Rk 为单位矩阵。然后计算 Pxzk = Pxk * Hk'，Pzk = Hk * Pxzk + Rk。接着计算增益矩阵 Kk = Pxzk * Pzk^-1。然后更新状态向量 xk = xk + Kk * (zk - Hk * xk)。再更新 Pxk = Pxk - Kk * Pxzk'。最后对 Pxk 进行对称化处理，使其成为对称矩阵。

这段代码的作用是实现递归最小二乘滤波器的更新过程，用于估计系统状态。

#### RLSUD：UD分解RLS

```matlab
function [U, D, K, X] = RLSUD(U, D, H, R, Z, X)
% Recursive Least Square filter using UD decomposition.
%
% Prototype: [U, D, K, X] = RLSUD(U, D, H, R, Z, X)
% Inputs: U,D - factors of UD decomposition
%         H,R - 1-row measurement matrix / measurement var
%         X,Z - state / measurement
% Outputs: U,D,X - as input after update
%          K - gain
%
% See also  KFUD, RLS, kfupdate.

% Copyright(c) 2009-2023, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/01/2023
    if isstruct(U)  % kf = RLSUD(kf, z);
        [U.Uk, U.Dk, ~, U.xk] = RLSUD(U.Uk, U.Dk, U.Hk, U.Rk, D, U.xk);
        return;
    end
    if nargin<4, R=1; end
    n = length(D);
    f = (H*U)';  g = D.*f;  afa = f'*g+R;
    for j=n:-1:1
        afa0 = afa - f(j)*g(j); lambda = -f(j)/afa0;
        D(j) = afa0/afa*D(j);   afa = afa0;
        for i=(j-1):-1:1
            s = (i+1):(j-1);
            U(i,j) = U(i,j) + lambda*(g(i)+U(i,s)*g(s));
        end
    end
    if nargout>2
        K = U*(D.*(H*U)')/R;
        if nargout>3, X = X + K*(Z-H*X); end
    end
```

该代码是一个递归最小二乘滤波器的实现，使用了UD分解技术。以下是其功能的解释：

1. 输入参数：
   - `U`：UD分解中的左奇异矩阵。
   - `D`：UD分解中的右奇异矩阵。
   - `H`：1行测量矩阵。
   - `R`：测量噪声的方差。
   - `Z`：测量值。
   - `X`：状态值。
2. 输出参数： 
   * `U`：更新后的左奇异矩阵。
   * `D`：更新后的右奇异矩阵。
   * `X`：更新后的状态值（仅在输出参数中存在）。
   * `K`：增益（仅在输出参数中存在）。
3. 功能解释： 
   1. 首先，代码检查输入的参数是否为结构体。如果是结构体，则表示调用函数的方式为 `kf = RLSUD(kf, z);`，即用当前的状态值 `kf.xk` 和测量值 `z` 更新结构体变量 `kf`。在这种情况下，函数直接返回更新后的结构体变量 `kf`，不进行其他计算。
   2. 如果输入参数不是结构体，则继续执行下面的代码。首先，根据输入的参数计算中间变量 `f`、`g` 和 `afa`。其中，`f = (H*U)'` 表示将 `H` 与 `U` 进行矩阵乘积的转置，`g = D.*f` 表示将 `D` 与 `f` 进行逐元素相乘，`afa = f'*g+R` 表示将 `f'` 与 `g` 进行逐元素相乘，并将结果加上 `R`。
   3. 接下来，通过循环逐个处理矩阵的列，从最后一列开始逆序迭代。在每一列处理过程中，首先计算新的增益系数 `lambda`，根据递推公式计算新的对角线元素 `D(j)`，并更新上三角矩阵 `U` 的相应元素。具体的计算过程如下：
      1. 首先，计算临时变量 `afa0 = afa - f(j)*g(j)`，表示将 `afa` 减去与当前列的乘积项。
      2. 然后，计算增益系数 `lambda = -f(j)/afa0`，其中负号表示使用最小二乘法。
      3. 接下来，根据递推公式计算新的对角线元素 `D(j) = afa0/afa*D(j)`。
      4. 最后，使用增益系数 `lambda` 和中间变量 `g(i)`、`U(i,s)` 更新上三角矩阵 `U` 的相应元素。
   4. 在完成所有列的处理后，如果输出参数要求计算增益 `K`，则根据公式 `K = U*(D.(H*U)')/R` 计算增益，其中 `.` 表示矩阵乘积的转置。
   5. 如果输出参数要求更新状态值 `X`，则根据公式 `X = X + K*(Z-H*X)` 更新状态值。其中，`Z-H*X` 表示预测误差。
4. 总结：该代码实现了一个递归最小二乘滤波器，利用UD分解技术对矩阵进行更新和计算增益。它通过对矩阵逐列处理的方式，逐步更新滤波器的状态和增益系数，以实现滤波器的滤波功能。

#### UDUT

```matlab
function [U, D] = UDUT(P)
% UDUT decomposition, so that P = U*diag(D)*U'.
%
% Prototype: [U, D] = UDUT(P)
% Input: P - nonnegative define symmetic matrix
% Outputs: U - unit upper-triangular matrix 
%          D - vector representation of diagonal matrix
%
% See also  chol1, qrmgs.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/11/2016
    n = length(P);
    U = eye(n); D = zeros(n,1);  trPn = trace(P)/n*1e-40;
    for j=n:-1:1
        k = (j+1):n;
        D(j) = P(j,j) - (U(j,k).^2)*D(k);
        if D(j)<=trPn, continue; end
        for i=(j-1):-1:1
            U(i,j) = (P(i,j)-(U(i,k).*U(j,k))*D(k)) / D(j);
        end
    end
```

该代码实现了UDUT分解（Upper-Triangular Decomposition with Diagonal Similarity Transform），将一个非负定对称矩阵P分解为U * diag(D) * U'的形式。 2. 从最后一行开始，依次向上处理每一行。 3. 首先计算当前行j的对角线元素D(j)，通过用当前行的上三角元素U(j,k)和D(k)的平方进行插值，减去对角线元素P(j,j)。 4. 如果D(j)小于等于trPn，说明该元素非常小，对分解结果影响不大，因此跳过对该行的处理。 5. 如果D(j)大于trPn，则需要计算当前行j和之前行i的元素U(i,j)，通过用当前行的元素P(i,j)和之前的上三角元素U(i,k)和U(j,k)以及D(k)进行插值，除以D(j)。 6. 循环处理完所有行后，输出U和D即为UDUT分解的结果。

这段代码的目的是将一个非负定对称矩阵进行UDUT分解，以便进行后续的计算和分析。UDUT分解可以用于求解线性方程组、计算矩阵的行列式、计算矩阵的特征值和特征向量等应用中。

### 二、kalman滤波

卡尔曼滤波（Kalman filter）是一种高效率的递归滤波器（自回归滤波器），它能够从一系列的不完全及包含噪声的测量中，估计动态系统的状态1。

卡尔曼滤波的主要思想是利用状态转移矩阵和观测矩阵，结合当前状态和上一时刻的状态，来估计当前的最优状态1。具体来说，卡尔曼滤波的步骤包括1：

1. 初始化滤波器的状态向量和协方差矩阵。
2. 对于每个时间步长，执行以下步骤：
   1. 利用状态转移矩阵和当前状态向量，计算出下一时刻的状态向量和协方差矩阵。
   2. 结合观测矩阵和当前观测向量，计算出当前时刻的状态估计值和协方差矩阵。
   3. 根据新的状态估计值和协方差矩阵，更新滤波器的状态向量和协方差矩阵。
3. 重复执行步骤2，进行迭代优化，直到满足收敛条件为止。

与传统的滤波器相比，卡尔曼滤波具有更好的数值稳定性和计算效率，能够更准确地估计状态变量的值。

```matlab
function [Xk, Pxk, Xkk_1, Pxkk_1] = kalman(Phikk_1, Gammak, Qk, Xk_1, Pxk_1, Hk, Rk, Zk, s)
% A simple Kalman filter By Yan Gongmin
    if nargin<9, s=1; end
    Xkk_1 = Phikk_1*Xk_1;
    Pxkk_1 = Phikk_1*s*Pxk_1*Phikk_1' + Gammak*Qk*Gammak';
    Pxykk_1 = Pxkk_1*Hk';
    Pykk_1 = Hk*Pxykk_1 + Rk;
    Kk = Pxykk_1*Pykk_1^-1;
    Xk = Xkk_1 + Kk*(Zk-Hk*Xkk_1);
    Pxk = Pxkk_1 - Kk*Pykk_1*Kk';
    Pxk = (Pxk+Pxk')/2;
```

这段代码是一个简单的卡尔曼滤波器的实现。卡尔曼滤波器是一种用于估计动态系统状态的方法，通过结合先前的估计和最新的观测数据来更新当前状态的估计。

以下是代码的功能解释：

1. 输入参数：
   - `Phikk_1`：状态转移矩阵，描述系统状态的转移方式。
   - `Gammak`：过程噪声协方差矩阵，描述系统过程中引入的噪声。
   - `Qk`：过程噪声协方差矩阵，描述系统过程中引入的噪声。
   - `Xk_1`：先前的状态估计。
   - `Zk`：观测数据。
   - `s`：可选参数，用于调整卡尔曼增益的平滑因子，默认为1。
2. 初始化：
   - `Xkk_1`：根据状态转移矩阵和先前的状态估计计算得到当前状态的预测值。
   - `Pxkk_1`：根据状态转移矩阵、过程噪声协方差矩阵和先前的状态估计误差协方差矩阵计算得到当前状态预测值的误差协方差矩阵。
3. 预测：
   - `Pxykk_1`：将当前状态预测值的误差协方假矩阵转换为预测的状态和观测之间的协方差矩阵。
   - `Pykk_1`：根据观测矩阵、预测的状态和观测之间的协方差矩阵以及观测噪声协方差矩阵计算得到预测的观测噪声协方差矩阵。
4. 更新：
   - `Kk`：通过预测的状态和观测之间的协方差矩阵以及预测的观测噪声协方差矩阵的逆计算得到卡尔曼增益。
   - `Xk`：根据当前状态预测值和观测数据的卡尔曼增益，对当前状态进行更新。
   - `Pxk`：根据当前状态更新后的误差协方差矩阵，通过减去卡尔曼增益与预测的观测噪声协方差矩阵的乘积的转置，得到更新后的误差协方差矩阵。
   - `Pxkk_1`：当前状态预测值误差协方差矩阵。

该代码实现了一个简单的卡尔曼滤波器，可以根据输入的参数进行状态估计和预测，并输出当前状态的估计值以及误差协方差矩阵等相关信息。

#### KFUD：UD分解Kalman滤波

```matlab
function [U, D, K] = KFUD(U, D, Phi, Tau, Q, H, R)
% UDUT square root Kalman filter.
%
% Prototype: [U, D, K] = KFUD(U, D, Phi, Tau, Q, H, R)
% Inputs: U - unit upper-triangular matrix
%         D - vector representation of diagonal matrix
%         Phi, Tau, Q - system matrix, processing noise distribusion matrix
%              & noise variance (vector representation)
%         H, R - measurement matrix, measurement noise variance
% Outputs: U - unit upper-triangular matrix
%          D - vector representation of diagonal matrix
%          K - Kalman filter gain
%
% See also  RLSUD, UDUT, chol1, qrmgs.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 12/11/2016
    n = length(U);
    W = [Phi*U, Tau];   D1 = [D; Q];  % NOTE: D,Q are both vector
    meanD = mean(D)/n*1e-40;
    for j=n:-1:1   % time updating
        D(j) = (W(j,:).*W(j,:))*D1;
        for i=1:(j-1)
            if D(j)<=meanD, U(i,j) = 0;
            else           U(i,j) = (W(i,:).*W(j,:))*D1/D(j);  end
            W(i,:) = W(i,:) - U(i,j)*W(j,:);
        end
    end
    if ~exist('R', 'var'), return; end
    f = (H*U)';  g = D.*f;  afa = f'*g+R;
    for j=n:-1:1   % measurement updating
        afa0 = afa - f(j)*g(j); lambda = -f(j)/afa0;
        D(j) = afa0/afa*D(j);   afa = afa0;
        for i=1:(j-1)
            s = (i+1):(j-1);
            U(i,j) = U(i,j) + lambda*(g(i)+U(i,s)*g(s));
        end
    end
    K = U*(D.*(H*U)')/R;
```

这个函数实现了一种称为通用对角线卡尔曼滤波（UDUT square root Kalman filter）算法。UD分解卡尔曼滤波是一种扩展的状态估计算法，适用于处理非线性系统。

函数接受以下输入参数：

- `U`：单位上三角矩阵，用于表示状态转移矩阵。
- `D`：一个向量，表示对角线矩阵的元素。
- `Phi`：系统矩阵，表示系统的动态模型。
- `Tau`：一个向量，表示处理噪声的分布矩阵（也称为过程噪声）。
- `Q`：一个向量，表示过程噪声的方差（以向量形式表示）。
- `H`：测量矩阵，表示如何从状态中获取测量值。
- `R`：测量噪声的方差。

函数返回以下输出参数：

- `U`：更新后的单位上三角矩阵。
- `D`：更新后的对角线矩阵的元素向量。
- `K`：卡尔曼滤波增益。

算法的主要步骤如下：

1. 初始化：根据输入参数，初始化一些变量，包括计算过程中的临时变量。
2. 时间更新：从后向前遍历每一个时间步，根据系统的动态模型和过程噪声计算每个时间步的状态预测和对角线矩阵的元素。如果预测的对角线元素小于一个阈值（`meanD`），则将其设为零，以避免过小的数值引起数值不稳定。
3. 测量更新：如果存在测量噪声方差（即`R`不为零），则进行测量更新步骤。首先计算预测的测量残差（`f`）和残差的权重（`g`）。然后从后向前遍历每个时间步，根据测量残差和权重更新对角线矩阵的元素和对角线元素上的权重。接着根据更新的对角线元素更新卡尔曼滤波增益（`K`）。
4. 返回输出：返回更新后的单位上三角矩阵、对角线矩阵的元素向量和卡尔曼滤波增益。

这个函数的输出可以用于进一步的状态估计或控制应用中。

#### kfc2d：连续时间Kalman滤波离散化

```matlab
function [Phikk_1, Qk] = kfc2d(Ft, Qt, Ts, n)
% For Kalman filter system differential equation, convert continuous-time
% model to discrete-time model: Ft->Phikk_1, Qt->Qk.
%
% Prototype: [Phikk_1, Qk] = kfc2d(Ft, Qt, Ts, n)
% Inputs: Ft - continous-time system transition matrix
%         Qt - continous-time system noise variance matrix
%         Ts - discretization time interval
%         n - the order for Taylor expansion
% Outputs: Phikk_1 - discrete-time transition matrix
%          Qk - discrete-time noise variance matrix
% Alogrithm notes:
%    Phikk_1 = I + Ts*Ft + Ts^2/2*Ft^2 + Ts^3/6*Ft^3 
%                + Ts^4/24*Ft^4 + Ts^5/120*Ft^5 + ...
%    Qk = M1*Ts + M2*Ts^2/2 + M3*Ts^3/6 + M4*Ts^4/24 + M5*Ts^5/120 + ...
%    where M1 = Qt; M2 = Ft*M1+(Ft*M1)'; M3 = Ft*M2+(Ft*M2)';
%          M4 = Ft*M3+(Ft*M3)'; M5 = Ft*M4+(Ft*M4)'; ...
%
% See also  kfinit, kffk, kfupdate, kffeedback.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/08/2012
    Phikk_1 = eye(size(Ft)) + Ts*Ft;
    Qk = Qt*Ts;
    if nargin<4 % n==1
        return;
    end
    Tsi = Ts; facti = 1; Fti = Ft; Mi = Qt;
    for i=2:1:n
        Tsi = Tsi*Ts;        
        facti = facti*i;
        Fti = Fti*Ft;
        Phikk_1 = Phikk_1 + Tsi/facti*Fti;  % Phikk_1
        FtMi = Ft*Mi;
        Mi = FtMi + FtMi';
        Qk = Qk + Tsi/facti*Mi;  % Qk
    end    
```

这段代码是一个用于将连续时间Kalman滤波器的系统差分方程转换为离散时间模型的函数。函数名为`kfc2d`，其输入参数包括连续时间系统的转移矩阵`Ft`、系统噪声方差矩阵`Qt`、离散化时间间隔`Ts`和泰勒级数展开的阶数`n`。函数的输出包括离散时间的转移矩阵`Phikk_1`和噪声方差矩阵`Qk`。

代码的主要逻辑如下：

1. 首先初始化离散时间的转移矩阵`Phikk_1`为单位矩阵加上第一阶近似项`Ts*Ft`。
2. 初始化噪声方差矩阵`Qk`为第一阶近似项`Qt*Ts`。
3. 如果输入的`n`小于等于1（默认值），则直接返回。
4. 进入循环，从第二阶开始迭代计算更高阶的近似项。
5. 在每一次迭代中，更新时间间隔`Tsi`为当前时间间隔的幂次，即`Tsi = Tsi*Ts`。
6. 更新阶数因子`facti`为当前阶数因子的幂次，即`facti = facti*i`。
7. 更新连续时间系统的转移矩阵`Fti`为当前转移矩阵的幂次，即`Fti = Fti*Ft`。
8. 将当前时间间隔和阶数因子的乘积与当前转移矩阵相加，得到离散时间的转移矩阵，即`Phikk_1 = Phikk_1 + Tsi/facti*Fti`。
9. 更新噪声方差矩阵的计算，通过当前时间间隔和当前噪声方差矩阵的乘积以及转置运算得到新的噪声方差矩阵，即`Mi = FtMi + FtMi'`。
10. 将当前时间间隔和阶数因子的乘积与新的噪声方差矩阵相加，得到离散时间的噪声方差矩阵，即`Qk = Qk + Tsi/facti*Mi`。
11. 循环结束后，返回离散时间的转移矩阵`Phikk_1`和噪声方差矩阵`Qk`。

该代码主要用于根据连续时间模型的差分方程，通过泰勒级数展开的方式，计算得到离散时间的转移矩阵和噪声方差矩阵，以便在离散时间环境下应用Kalman滤波器进行状态估计和预测。

### 三、ekf：扩展卡尔曼滤波

EKF（Extended Kalman Filter，简称扩展卡尔曼滤波器）是一种高效率的递归滤波器（自回归滤波器），它能够从一系列的不完全及包含噪声的测量中，估计动态系统的状态1。

EKF滤波的主要思想是利用状态转移矩阵和观测矩阵，结合当前状态和上一时刻的状态，来估计当前的最优状态1。具体来说，EKF滤波的步骤包括1：

1. 初始化滤波器的状态向量和协方差矩阵。
2. 对于每个时间步长，执行以下步骤：

a. 利用状态转移矩阵和当前状态向量，计算出下一时刻的状态向量和协方差矩阵。

b. 结合观测矩阵和当前观测向量，计算出当前时刻的状态估计值和协方差矩阵。

c. 根据新的状态估计值和协方差矩阵，更新滤波器的状态向量和协方差矩阵。 3. 重复执行步骤2，进行迭代优化，直到满足收敛条件为止。

与传统的滤波器相比，EKF滤波具有更好的数值稳定性和计算效率，能够更准确地估计状态变量的值，并且在高维度和非线性较强的系统中表现更好。

```matlab
function kf = ekf(kf, yk, TimeMeasBoth)
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state Jacobian matrix
            [kf.Phikk_1, kf.xkk_1] = ekfJcb(kf.fx, kf.xk, kf.px);
            if isempty(kf.xkk_1), kf.xkk_1 = kf.Phikk_1*kf.xk; end
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
        end
        kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end

    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement Jacobian matrix
            [kf.Hk, kf.ykk_1] = ekfJcb(kf.hx, kf.xkk_1, kf.py);
            if isempty(kf.ykk_1), kf.ykk_1 = kf.Hk*kf.xkk_1; end
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
        end
        kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
```

这段代码是一个扩展卡尔曼滤波器（Extended Kalman Filter, EKF）的实现。下面是对代码功能的解释： 

1. 函数 `ekf` 接受三个输入参数：`kf`（扩展卡尔曼滤波器的状态），`yk`（测量值），`TimeMeasBoth`（一个字符，表示时间测量的类型）。

2. 根据输入参数的数量，确定 `TimeMeasBoth` 的值。如果输入参数数量为1，则将 `TimeMeasBoth` 设置为字符 'T'；如果输入参数数量为2，则将 `TimeMeasBoth` 设置为字符 'B'。

3. 根据 `TimeMeasBoth` 的值，执行以下操作：

   1. 如果 TimeMeasBoth 是字符 'T' 或者字符 'B'：

      1. 如果 `kf` 中存在字段 'fx'（非线性状态雅可比矩阵），则调用函数 `ekfJcb` 计算 `kf` 的 'Phikk_1' 和 'xkk_1'（分别是状态转移矩阵和下一时刻的状态估计值）。

         ```matlab
         function [Jcb, y] = ekfJcb(hfx, x, tpara)
             [Jcb, y] = feval(hfx, x, tpara);
         ```

      2. 如果 `xkk_1` 是空的，则将其设置为 `Phikk_1` 乘以当前状态 `xk`。

      3. 如果 `fx` 不存在，则将 `xkk_1` 设置为 'Phikk_1'乘以当前状态 `xk`。

      4. 计算 `Pxkk_1`（下一时刻的协方差矩阵估计值），使用 `Phikk_1`、`Pxk`、`Gammak`、`Qk` 和 `Gammak` 的值进行计算。

      5. 如果 `TimeMeasBoth` 是字符 `T`，则只进行时间更新，将下一时刻的状态估计值 `xkk_1` 和协方差矩阵 `Pxkk_1` 赋值给当前状态 `xk` 和协方差矩阵 `Pxk`，然后返回。

   2. 如果 `TimeMeasBoth` 是字符 `B`，则继续执行下面的操作： 

      1. 根据下一时刻的状态估计值 `xkk_1` 和协方差矩阵 `Pxkk_1`，以及测量值 `yk` 和噪声协方差矩阵 `Rk`，计算卡尔曼增益（Kalman gain）和更新后的状态估计值 `xk` 和协方差矩阵 `Pxk`。
      2. 将更新后的状态估计值 `xk` 和协方差矩阵 `Pxk` 赋值给当前状态 `xk` 和协方差矩阵 `Pxk`。

4. 函数执行完毕，输出更新后的扩展卡尔曼滤波器状态 `kf`。

总体而言，这段代码实现了扩展卡尔曼滤波器的时间更新和测量更新过程，根据输入的状态、测量值和时间测量类型，更新滤波器的状态和协方差矩阵。 

### 四、UKF：无迹卡尔曼滤波

UKF（Unscented Kalman Filter），中文释义是无迹卡尔曼滤波、无迹卡尔曼滤波或者去芳香卡尔曼滤波，是一种用于处理非线性非高斯系统的状态估计的滤波算法1。

UKF滤波的主要思想是利用Unscented变换来将非线性系统方程转化为线性假设下的标准Kalman滤波体系。具体来说，UKF滤波的步骤包括：

1. 选择一组Unscented变换的参数，计算Unscented变换中的权重和均值。
2. 对系统模型和观测模型进行一阶Taylor展开，得到非线性的残差项。
3. 利用这些残差项来更新状态估计值。
4. 重复执行步骤2和3，进行迭代优化，直到满足收敛条件为止。

与传统的卡尔曼滤波相比，UKF滤波具有更好的数值稳定性和计算效率，能够更准确地估计状态变量的值，并且在高维度和非线性较强的系统中表现更好。

需要注意的是，在实际应用中，UKF滤波算法的实现和参数设置需要根据具体的系统模型和观测模型进行调整，并进行实验验证和优化。同时，对于大规模或实时性要求高的应用场景，还需要考虑算法的效率和可扩展性。

```matlab
function kf = ukf(kf, yk, TimeMeasBoth)
% Unscented Kalman filter for nonlinear system.
%
% Prototype: kf = ukf(kf, yk, TimeMeasBoth)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - filter structure array after time/meas updating
%
% See also  ukfUT, ckf, ssukf, ekf, kfupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2012
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if ~isfield(kf, 'alpha')
        kf.alpha = 1e-3; kf.beta = 2; kf.kappa = 0;
    end
    
    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state propagation
            [kf.xkk_1, kf.Pxkk_1] = ukfUT(kf.xk, kf.Pxk, kf.fx, kf.px, kf.alpha, kf.beta, kf.kappa);
            kf.Pxkk_1 = kf.Pxkk_1 + kf.Gammak*kf.Qk*kf.Gammak';
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        end
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end
    
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement propagation
            [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = ukfUT(kf.xkk_1, kf.Pxkk_1, kf.hx, kf.py, kf.alpha, kf.beta, kf.kappa);
            kf.Pykk_1 = kf.Pykk_1 + kf.Rk;
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
            kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        end
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
```

这段代码是一个 UKF 的实现。下面是对代码功能的解释：

1. 输入参数：
   - `kf`：一个结构体数组，包含卡尔曼滤波器的状态和参数信息，例如状态向量 `xk`、状态协方差矩阵 `Pxk`、时间更新系数 `fx`、测量更新系数 `hx` 等。
   - `yk`：测量向量，即系统的观测值。
   - `TimeMeasBoth`：一个字符串，用于指定时间更新和测量更新的方式，可选值为 `'T'`（只进行时间更新）、`'M'`（只进行测量更新）和 `'B'`（同时进行时间和测量更新）。
2. 根据输入参数确定更新方式：
   - 如果输入参数 `TimeMeasBoth` 为 `'T'` 或者没有指定，则执行时间更新。
   - 如果输入参数 `TimeMeasBoth` 为 `'M'` 或者没有指定，则执行测量更新。
3. 时间更新： 
   1. 如果结构体数组 `kf` 中没有 `alpha`、`beta` 和 `kappa` 字段，则给它们赋默认值。
   2. 如果结构体数组 `kf` 中有 `fx` 字段，表示存在非线性状态转移函数，此时需要根据输入的观测值和滤波器参数执行时间更新。
      1. 首先，通过调用函数 `ukfUT`（也是该代码文件中的函数）计算状态转移后的状态向量和状态协方差矩阵。
      2. 然后，根据滤波器参数计算状态协方差矩阵的修正项，并将其加到状态协方差矩阵中。
   3. 如果结构体数组 `kf` 中没有 `fx` 字段，表示状态转移是线性的，直接使用预测的状态向量和协方差矩阵。
4. 测量更新： 
   1. 如果执行的是测量更新，首先判断是否只有测量更新。
   2. 如果只有测量更新，则将预测的状态向量和协方差矩阵分别赋值给当前的状态向量和协方差矩阵。
   3. 如果同时进行时间和测量更新，则根据滤波器参数执行测量更新。
      1. 如果结构体数组 `kf` 中有 `hx` 字段，表示存在非线性测量转移函数，此时需要根据输入的观测值和滤波器参数执行测量更新。 
         1. 首先，通过调用函数 `ukfUT` 计算测量转移后的测量向量、测量协方差矩阵和交叉协方差矩阵。
         2. 然后，根据滤波器参数计算测量协方差矩阵的修正项，并将其加到测量协方差矩阵中。
      2. 如果结构体数组 `kf` 中没有 `hx` 字段，表示测量转移是线性的，直接使用预测的测量向量和协方差矩阵。
5. 返回结果： 更新后的卡尔曼滤波器结构体数组 `kf` 将作为输出返回。

总体来说，这段代码实现了非线性系统的卡尔曼滤波器的时间更新和测量更新过程。它根据输入的观测值和滤波器参数，对滤波器的状态向量和协方差矩阵进行更新，以实现滤波器的状态估计功能。 

#### ukfUT

```matlab
function [y, Pyy, Pxy, X, Y] = ukfUT(x, Pxx, hfx, tpara, alpha, beta, kappa)
% Unscented transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = ukfUT(x, Pxx, hfx, tpara, alpha, beta, kappa)
% Inputs: x, Pxx - state vector and its variance matrix
%         hfx - a handle for nonlinear state equation
%         tpara - some time-variant parameter pass to hfx
%         alpha, beta, kappa - parameters for UT transformation
% Outputs: y, Pyy - state vector and its variance matrix after UT
%          Pxy - covariance matrix between x & y
%          X, Y - Sigma-point vectors before & after UT
%
% See also  ukf, ckfCT, SSUT.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/09/2012
    n = length(x);
    lambda = alpha^2*(n+kappa) - n;
    gamma = sqrt(n+lambda);
    Wm = [lambda/gamma^2; repmat(1/(2*gamma^2),2*n,1)];  
    Wc = [Wm(1)+(1-alpha^2+beta); Wm(2:end)];
    sPxx = gamma*chol(Pxx)';    % Choleskey decomposition
    xn = repmat(x,1,n); 
    X = [x, xn+sPxx, xn-sPxx];
    Y(:,1) = feval(hfx, X(:,1), tpara); m=length(Y); y = Wm(1)*Y(:,1);
    Y = repmat(Y,1,2*n+1);
    for k=2:1:2*n+1     % Sigma points nolinear propagation
        Y(:,k) = feval(hfx, X(:,k), tpara);
        y = y + Wm(k)*Y(:,k);
    end
    Pyy = zeros(m); Pxy = zeros(n,m);
    for k=1:1:2*n+1
        yerr = Y(:,k)-y;
        Pyy = Pyy + Wc(k)*(yerr*yerr');  % variance
        xerr = X(:,k)-x;
        Pxy = Pxy + Wc(k)*xerr*yerr';  % covariance
    end
```

这段代码实现了Unscented变换（Unscented Transformation，UT）的功能。UT是一种用于非线性系统状态估计的方法，它通过对非线性方程进行一系列采样和传播，得到状态变量的估计结果。

该函数的输入参数包括：

- `x`：状态向量（state vector）。
- `Pxx`：状态向量的方差矩阵（variance matrix）。
- `hfx`：一个处理非线性状态方程的句柄（handle）。
- `tpara`：一些时间变化的参数，用于传递给`hfx`。
- `alpha`、`beta`、`kappa`：UT变换的参数。

该函数的输出参数包括：

- `y`：经过UT变换后的状态向量。
- `Pyy`：经过UT变换后的状态向量的方差矩阵。
- `Pxy`：状态向量和经过UT变换后的状态向量之间的协方差矩阵。
- `X`、`Y`：在UT变换之前和之后的Sigma点向量。

代码的主要步骤如下：

1. 根据输入的参数计算一些中间变量，包括`n`（状态向量的长度）、`lambda`、`gamma`、`Wm`和`Wc`。
2. 进行Cholesky分解，将输入的状态方差矩阵`Pxx`转换为下三角矩阵`sPxx`。
3. 构造Sigma点向量`X`，包括三个部分的点，分别是状态向量本身、状态向量加上和减去状态方差矩阵的Cholesky分解后的矩阵。
4. 对每个Sigma点进行非线性传播，通过调用句柄函数`hfx`计算每个点的输出值，并将结果存储在矩阵`Y`中。
5. 根据UT变换的公式计算估计的状态向量`y`和方差矩阵`Pyy`，以及状态向量和估计状态向量之间的协方差矩阵`Pxy`。
6. 返回估计的状态向量、方差矩阵、协方差矩阵以及在UT变换之前和之后的Sigma点向量。

需要注意的是，该代码中使用了MATLAB的函数feval来调用句柄函数hfx，并在循环中进行了多次调用。因此，在使用该代码时，需要确保句柄函数hfx的正确实现和可调用性。

#### utpoint

```matlab
function [U, wm, wc] = utpoint(n, afa, beta, kappa)
% Calculate 3th or 5th-order cubature points.
%
% Prototype: [U, wm, wc] = utpoint(n, afa, beta, kappa)
% Inputs: n - dimension
%         afa,beta,kappa - parameters
% Outputs: U - UT points
%          wm,wc - weights
%
% See also  cubpoint, ghpoint, ukf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/08/2022
    if nargin<4, kappa=0; end
    if nargin<3, beta=2; end
    if nargin<2, afa=1e-3; end
    lambda = afa^2*(n+kappa)-n;  gamma = sqrt(n+lambda);
    U = gamma*[eye(n), -eye(n), zeros(n,1)];
    wm = [repmat(1/(2*gamma^2),1,2*n), lambda/gamma^2];
    wc = wm; wc(end) = wc(end)+(1-afa^2+beta);
```

这段代码是一个MATLAB函数，用于计算3阶或5阶的 cubature 点的权重。下面是代码的功能解释：

1. 函数的输入参数为：
   - `n`：维度（整数）
   - `afa`、`beta`、`kappa`：参数（可以是标量、向量或矩阵）
2. 函数的输出参数为：
   - `U`：cubature 点的向量（大小为 `(3*n, 1)` 的列向量）
   - `wm`：与 `U` 中的每个点对应的权重（大小为 `(3*n, 1)` 的列向量）
   - `wc`：与 `U` 中的每个点对应的权重（大小为 `(3*n, 1)` 的列向量）
3. 代码逻辑解释：
   - 首先，检查输入参数的数量，如果小于4个，将 `kappa` 设置为0；如果小于3个，将 `beta` 设置为2；如果小于2个，将 `afa` 设置为1e-3。
   - 接下来，根据参数计算变量 `lambda` 和 `gamma` 的值。
   - 然后，根据 `gamma` 和其他参数计算 cubature 点的矩阵 `U`。
   - 最后，根据参数和计算结果计算权重向量 `wm` 和 `wc`。其中，`wm` 的前两个元素是相同的，大小为 `(2*n, 1)` 的矩阵，每个元素都是 `1/(2*gamma^2)`；第三个元素是 `lambda/gamma^2`。`wc` 的前两个元素与 `wm` 相同，最后一个元素是 `(3*n, 1)` 的列向量，值为 `(1-afa^2+beta)`。

这个函数的作用是通过输入的参数计算 cubature 点的位置和权重。 cubature 点是一种用于数值积分的方法，通过在低维度空间中计算一些点并将它们的权重相加以估计高维度空间的积分值。

### 五、ssukf

SSUKF是一种非线性滤波算法，全称为Unscented Particle Filtering with Square-Root Unscented Transform。它结合了Unscented变换和粒子滤波的优点，用于处理非线性非高斯系统的状态估计问题。SSUKF的主要步骤包括：

1. 选择一组Unscented变换的参数，计算Unscented变换中的权重和均值。
2. 对系统模型和观测模型进行一阶Taylor展开，得到非线性的残差项。
3. 使用粒子滤波的方法，通过对残差项进行加权和估计，得到新的状态估计值。
4. 重复执行步骤2和3，进行迭代优化，直到满足收敛条件为止。同时，对于大规模或实时性要求高的应用场景，还需要考虑算法的效率和可扩展性。

> UKF 和 SSUKF 区别：
>
> 1. Unscented变换的方式不同：UKF使用标准的Unscented变换，而SSUKF使用Square-Root Unscented变换，这种变换可以更快地计算权重和均值，提高了算法的计算效率。
> 2. 粒子滤波的方式不同：在UKF中，使用标准粒子滤波来估计状态变量，而在SSUKF中，使用一种称为序贯 Importance Sampling的粒子滤波方法来估计状态变量。这种滤波方法可以更好地处理非线性较强的系统，并且可以提高算法的数值稳定性。
> 3. 适用场景不同：UKF适用于处理高维度的非线性系统，而SSUKF适用于处理低维度的非线性系统，尤其是对于一些实时性要求较高的应用场景。
>
> 需要注意的是，无论是UKF还是SSUKF，它们都是一种相对较新的滤波算法，在实际应用中还需要根据具体的系统模型和观测模型进行调整和优化，同时考虑算法的效率和可扩展性1。

```matlab
function kf = ssukf(kf, yk, TimeMeasBoth)
% Unscented Kalman filter for nonlinear system.
%
% Prototype: kf = ssukf(kf, yk, TimeMeasBoth)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - filter structure array after time/meas updating
%
% See also  ukf, SSUT, ckf, ekf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/06/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state propagation
            [kf.xkk_1, kf.Pxkk_1] = SSUT(kf.xk, kf.Pxk, kf.fx, kf.px, 0);
            kf.Pxkk_1 = kf.Pxkk_1 + kf.Gammak*kf.Qk*kf.Gammak';
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        end
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end
    
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement propagation
            [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = SSUT(kf.xkk_1, kf.Pxkk_1, kf.hx, kf.py, 0);
            kf.Pykk_1 = kf.Pykk_1 + kf.Rk;
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
            kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        end
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
```

这段代码是一个用于非线性系统的扩展卡尔曼滤波（Unscented Kalman Filter）的函数。它接受一个初始的卡尔曼滤波器结构数组`kf`，一个测量向量`yk`，以及一个表示时间更新和测量更新的标志`TimeMeasBoth`。

代码的主要逻辑如下：

1. 根据输入参数的数量，确定`TimeMeasBoth`的值，如果只有一个输入参数，则`TimeMeasBoth`被设置为'T'，表示只进行时间更新；如果有两个输入参数，则`TimeMeasBoth`被设置为'B'，表示同时进行时间更新和测量更新。
2. 如果`TimeMeasBoth`为'T'或'B'，则执行时间更新步骤。如果卡尔曼滤波器结构数组`kf`中存在非线性状态转移函数`fx`，则使用SSUT函数进行状态预测，并更新状态和状态协方差矩阵。如果只有时间更新，则直接返回更新后的`kf`。
3. 如果`TimeMeasBoth`为'M'或'B'，则执行测量更新步骤。如果只有测量更新，则将状态和状态协方差矩阵设置为先前的值。如果卡尔曼滤波器结构数组`kf`中存在非线性测量转移函数`hx`，则使用SSUT函数进行测量预测，并更新测量和测量协方差矩阵。
4. 执行滤波步骤。计算卡尔曼增益（Kalman gain），并使用测量和预测的状态和协方差矩阵更新状态估计值和状态协方差矩阵。

最终，函数返回更新后的卡尔曼滤波器结构数组`kf`。

需要注意的是，该代码依赖于其他函数`SSUT`，这些函数可能需要在其他地方定义或提供。此外，该代码还依赖于一些变量的定义，例如`Phikk_1`、`Gammak`、`Qk`、`Hk`等，这些变量应该在输入的卡尔曼滤波器结构数组`kf`中提供。

如果您要使用该函数，请确保提供正确的输入参数和正确的函数依赖关系。

#### SSUT

```matlab
function [y, Pyy, Pxy, X, Y] = SSUT(x, Pxx, hfx, tpara, w0)
% Spherical Simplex Unscented Transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = SSUT(x, Pxx, hfx, tpara, w0)
% Inputs: x, Pxx - state vector and its variance matrix
%         hfx - a handle for nonlinear state equation
%         tpara - some time-variant parameter pass to hfx
%         w0 - weight0
% Outputs: y, Pyy - state vector and its variance matrix after UT
%          Pxy - covariance matrix between x & y
%          X, Y - Sigma-point vectors before & after UT
%
% See also  ssukf, ukfUT, ckfCT.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/06/2022
global SSUT_s  SSUT_w
    if isempty(SSUT_w), SSUT_w(1)=0; end
    if nargin<5, w0=0; end
    n = length(x);
    if n~=size(SSUT_s,1) || SSUT_w(1)~=w0
        w1 = (1-w0)/(n+1);
        s = [0, -1/sqrt(2), 1/sqrt(2)];
        for j=2:n
            s1 = [s(:,1); 0];
            for i=1:j
                s1(:,i+1) = [s(:,i+1); -1/sqrt(j*(j+1))];
            end
            s1(:,j+2) = [zeros(j-1,1);j/sqrt(j*(j+1))];
            s = s1;
        end
        SSUT_s = s/sqrt(w1);  SSUT_w = [w0, repmat(w1,1,n+1)];
    end
    %%
    X = repmat(x,1,n+2) + chol(Pxx)'*SSUT_s;
    y = SSUT_w(1)*feval(hfx, X(:,1), tpara);  m = length(y);
    Y = repmat(y,1,2*n);
    for k=2:1:n+2
        Y(:,k) = feval(hfx, X(:,k), tpara);
        y = y + SSUT_w(k)*Y(:,k);
    end
    Pyy = zeros(m); Pxy = zeros(n,m);
    for k=1:1:n+2
        yerr = Y(:,k)-y;
        Pyy = Pyy + SSUT_w(k)*(yerr*yerr');  % variance
        xerr = X(:,k)-x;
        Pxy = Pxy + SSUT_w(k)*xerr*yerr';  % covariance
    end
```

这是一个实现了球面简单卡尔曼滤波（Spherical Simplex Unscented Transformation, SSUT）的 MATLAB 函数。SSUT 是一种非线性滤波方法，用于对非线性系统进行状态估计和预测。

函数输入参数包括：

- `x`：状态向量（一个列向量）。
- `Pxx`：状态向量的方差矩阵。
- `hfx`：一个用于描述非线性状态方程的函数句柄。
- `tpara`：一个时间变参数向量，用于传递给 `hfx` 函数。
- `w0`：权重参数。

函数输出参数包括：

- `y`：经过 UT 变换后的状态向量（一个列向量）。
- `Pyy`：经过 UT 变换后的状态向量的方差矩阵。
- `Pxy`：状态向量和经过 UT 变换后的状态向量之间的协方差矩阵。
- `X` 和 `Y`：在 UT 变换前后生成的 Sigma 点向量。

函数首先检查是否已经初始化了一些全局变量 `SSUT_s` 和 `SSUT_w`，如果没有则根据默认值进行初始化。然后根据输入的参数 `x` 和 `Pxx` 生成相应的 Sigma 点。接着调用非线性状态方程 `hfx` 来计算这些 Sigma 点对应的输出值，并将这些输出值存储在 `Y` 中。然后根据球面简单卡尔曼滤波的算法计算经过 UT 变换后的状态向量 `y` 和其方差矩阵 `Pyy`，以及状态向量和经过 UT 变换后的状态向量之间的协方差矩阵 `Pxy`。

最后，函数返回经过 UT 变换后的状态向量 `y` 和其方差矩阵 `Pyy`，以及状态向量和经过 UT 变换后的状态向量之间的协方差矩阵 `Pxy`，以及在 UT 变换前后生成的 Sigma 点向量 `X` 和 `Y`。

该函数是 SSUT 的实现，可以用于非线性系统的状态估计和预测。

### 六、CKF：容积卡尔曼滤波

容积卡尔曼滤波（Cube-based Radial Basis Function，简称CKF）是一种用于处理非线性非高斯系统的状态估计的滤波算法。

CKF滤波的核心思想是利用容积准则来近似状态的后验均值和协方差，以保证在理论上以三阶多项式逼近任何非线性高斯状态的后验均值和方差。

与传统的卡尔曼滤波相比，CKF滤波具有更好的数值稳定性和计算效率，能够更准确地估计状态变量的值，并且在高维度和非线性较强的系统中表现更好。

```matlab
function kf = ckf(kf, yk, TimeMeasBoth)
% Cubature transformation KF.
%
% Prototype: kf = ckf(kf, yk, TimeMeasBoth)
% Inputs: kf - filter structure array
%         yk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Output: kf - filter structure array after time/meas updating
%
% See also  ckfCT, ukf, ekf, kfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        if isfield(kf, 'fx')  % nonliear state propagation
            [kf.xkk_1, kf.Pxkk_1] = ckfCT(kf.xk, kf.Pxk, kf.fx, kf.px);
            kf.Pxkk_1 = kf.Pxkk_1 + kf.Gammak*kf.Qk*kf.Gammak';
        else
            kf.xkk_1 = kf.Phikk_1*kf.xk;
            kf.Pxkk_1 = kf.Phikk_1*kf.Pxk*kf.Phikk_1' + kf.Gammak*kf.Qk*kf.Gammak';
        end
        if TimeMeasBoth=='T'    % time updating only
            kf.xk = kf.xkk_1; kf.Pxk = kf.Pxkk_1;
            return;
        end
    end
    
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        if TimeMeasBoth=='M'    % meas updating only
            kf.xkk_1 = kf.xk; kf.Pxkk_1 = kf.Pxk;
        end
        if isfield(kf, 'hx')  % nonliear measurement propagation
            [kf.ykk_1, kf.Pykk_1, kf.Pxykk_1] = ckfCT(kf.xkk_1, kf.Pxkk_1, kf.hx, kf.py);
            kf.Pykk_1 = kf.Pykk_1 + kf.Rk;
        else
            kf.ykk_1 = kf.Hk*kf.xkk_1;
            kf.Pxykk_1 = kf.Pxkk_1*kf.Hk';    kf.Pykk_1 = kf.Hk*kf.Pxykk_1 + kf.Rk;
        end
        % filtering
        kf.Kk = kf.Pxykk_1*kf.Pykk_1^-1;
        kf.xk = kf.xkk_1 + kf.Kk*(yk-kf.ykk_1);
        kf.Pxk = kf.Pxkk_1 - kf.Kk*kf.Pykk_1*kf.Kk';  kf.Pxk = (kf.Pxk+kf.Pxk')/2;
    end
```

这段代码是一个实现Cubature Kalman Filter（CKF）的函数。CKF是一种非线性滤波方法，用于估计动态系统的状态。

函数`ckf`的作用是根据输入的滤波器结构数组`kf`、测量向量`yk`和时间/测量更新标志`TimeMeasBoth`，对滤波器进行时间更新或测量更新。

代码的主要逻辑如下：

1. 根据输入的参数确定时间/测量更新标志`TimeMeasBoth`。如果只有一个输入参数，则默认为时间更新标志为'T'；如果有两个输入参数，则根据第二个参数确定更新标志。
2. 如果时间更新标志为'T'或'B'（表示需要进行时间更新或同时进行时间更新和测量更新），执行以下操作：
   1. 如果滤波器结构数组`kf`中存在非线性状态传播函数`fx`，则使用`ckfCT`函数进行状态预测和协方差预测。
   2. 如果时间更新标志为'T'，则更新滤波器的状态和协方差，并返回结果。
3. 如果测量更新标志为'M'或'B'（表示需要进行测量更新或同时进行时间更新和测量更新），执行以下操作： 
   1. 如果测量更新标志为'M'，则将滤波器的状态和协方差设置为当前状态和协方差。
   2. 如果滤波器结构数组`kf`中存在非线性测量传播函数`hx`，则使用`ckfCT`函数进行测量预测和协方差预测。
   3. 进行滤波，计算卡尔曼增益、状态估计和协方差估计。

最后，函数返回更新后的滤波器结构数组`kf`。

需要注意的是，这段代码中的一些函数（如`ckfCT`、`ukf`、`ekf`、`kfupdate`）并没有在代码中定义，可能是在其他地方定义的辅助函数或子函数。这段代码只展示了实现CKF的主要逻辑，而没有提供完整的实现细节。

#### ckfCT

```matlab
function [y, Pyy, Pxy, X, Y] = ckfCT(x, Pxx, hfx, tpara)
% Spherical-Radial Cubature transformation.
%
% Prototype: [y, Pyy, Pxy, X, Y] = ckfCT(x, Pxx, hfx, tpara)
% Inputs: x, Pxx - state vector and its variance matrix
%         hfx - a handle for nonlinear state equation
%         tpara - some time-variant parameter pass to hfx
% Outputs: y, Pyy - state vector and its variance matrix after UT
%          Pxy - covariance matrix between x & y
%          X, Y - Sigma-point vectors before & after UT
%
% See also  ckf, ukfUT, SSUT.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    n = length(x);
    sPxx = sqrt(n)*chol(Pxx)';    % Choleskey decomposition
    xn = repmat(x,1,n); 
    X = [xn+sPxx, xn-sPxx];
    y = feval(hfx, X(:,1), tpara);
    Y = repmat(y,1,2*n);
    Pyy = Y(:,1)*Y(:,1)'; Pxy = X(:,1)*Y(:,1)';
    for k=2:1:2*n
        Y(:,k) = feval(hfx, X(:,k), tpara);
        y = y + Y(:,k);
        Pyy = Pyy + Y(:,k)*Y(:,k)';
        Pxy = Pxy + X(:,k)*Y(:,k)';
    end
    y = 1/(2*n)*y;  % y mean
    Pyy = 1/(2*n)*Pyy - y*y';  Pxy = 1/(2*n)*Pxy - x*y';
 
%     Y(:,1) = feval(hfx, X(:,1), tpara); m=length(Y); y = Y(:,1);
%     Y = repmat(Y,1,2*n);
%     Pyy = zeros(m); Pxy = zeros(n,m);
%     for k=2:1:2*n     % Sigma points nolinear propagation
%         Y(:,k) = feval(hfx, X(:,k), tpara);
%         y = y + Y(:,k);
%     end
%     y = 1/(2*n)*y;
%     for k=1:1:2*n
%         yerr = Y(:,k)-y;
%         Pyy = Pyy + (yerr*yerr');  % variance
%         xerr = X(:,k)-x;
%         Pxy = Pxy + xerr*yerr';  % covariance
%     end
%     Pyy = 1/(2*n)*Pyy; Pxy = 1/(2*n)*Pxy;
```

This code implements the Spherical-Radial Cubature transformation (CKF-CT) for nonlinear state estimation.下面是代码的功能解释：

1. 代码输入：
   - `x`：当前状态的向量。
   - `Pxx`：当前状态向量的方差矩阵。
   - `hfx`：非线性状态方程的句柄。
   - `tpara`：传递给`hfx`的一些时间变化参数。
2. 代码输出：
   - `y`：经过Unscented变换后的状态向量。
   - `Pyy`：经过Unscented变换后的状态向量的方差矩阵。
   - `Pxy`：经过Unscented变换后的状态向量和初态向量之间的协方差矩阵。
   - `X`：在进行Unscented变换之前的Sigma点向量。
   - `Y`：在进行Unscented变换之后的Sigma点向量。
3. 代码实现：
   - 首先，通过Cholesky分解将输入的方差矩阵`Pxx`转换为上三角矩阵`sPxx`。
   - 然后，根据当前的状态向量`x`和上三角矩阵`sPxx`生成两个Sigma点，即`X`矩阵。
   - 接下来，通过调用非线性状态方程的句柄`hfx`，计算初始的Sigma点对应的输出向量`Y`。
   - 然后，使用循环计算每个Sigma点的输出，并将它们累加到总输出向量`y`和协方差矩阵`Pyy`中。同时，计算初始状态向量和总输出向量之间的协方差矩阵`Pxy`。
   - 最后，根据总输出向量和协方差矩阵计算最终的状态向量和协方差矩阵。
4. 注意事项：
   - 该代码使用了MATLAB中的函数feval，用于调用非线性状态方程的句柄`hfx`进行计算。确保在使用该代码之前，已经定义并正确设置了非线性状态方程的句柄`hfx`和相关参数`tpara`。
   - 代码中的注释提供了一些说明和参考，可以帮助理解代码的功能和实现方法。

总体而言，该代码实现了Spherical-Radial Cubature变换（CKF-CT）用于非线性状态估计，通过Unscented变换（UT）对状态进行估计，并计算估计结果的方差和协方差。

### 七、PF：粒子滤波

粒子滤波（Particle Filter）是一种非参数化的贝叶斯滤波方法，可以用于对随机过程进行估计和预测。它通过使用一组带有权值的粒子（即样本）来表示随机过程的状态，从而实现对随机过程的估计和预测。

粒子滤波的主要思想是利用已知的先验概率分布和观测数据，通过迭代的方式更新粒子的权值和状态，从而得到后验概率分布。具体来说，粒子滤波包括以下步骤：

1. 初始化：根据先验概率分布，生成一组初始粒子，并赋予相应的权值。
2. 预测：根据已知的动力学模型，将每个粒子向前推进一个时间步长，得到下一时刻的粒子集合。
3. 重采样：根据每个粒子的权值，对粒子进行重新采样，使得权值较大的粒子在采样中出现的概率更大。
4. 输出：根据重采样后的粒子集合，可以得到后验概率分布的估计值。

粒子滤波在许多领域都有应用，例如目标跟踪、姿态估计、导航、语音识别、图像处理等。然而，粒子滤波也存在一些问题，例如粒子退化、噪声敏感等，需要进行进一步的研究和改进。

```matlab
function [pf, Pxk] = pfupdate(pf, Zk, TimeMeasBoth)
% Particle filter updating for additive normal distribution process&measure noise.
%
% Prototype: [pf, Pxk] = pfupdate(pf, Zk, TimeMeasBoth)
% Inputs: pf - particle filter structure array
%         Zk - measurement vector
%         TimeMeasBoth - described as follows,
%            TimeMeasBoth='T' (or nargin==1) for time updating only, 
%            TimeMeasBoth='M' for measurement updating only, 
%            TimeMeasBoth='B' (or nargin==2) for both time and 
%                             measurement updating.
% Outputs: pf - particle filter structure array after time/meas updating
%          Pxk - for full covariance (not diagnal)
%
% see also  pfinit, kfupdate, ckf, ukf.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2022
    if nargin==1;
        TimeMeasBoth = 'T';
    elseif nargin==2
        TimeMeasBoth = 'B';
    end

    if TimeMeasBoth=='T' || TimeMeasBoth=='B'
        sQ = chol(pf.Qk+eps*eye(pf.n));
    end
	if TimeMeasBoth=='M' || TimeMeasBoth=='B'
        vRv = zeros(pf.Npts, 1);  invR = pf.Rk^-1;
    end
    for k=1:pf.Npts
        if TimeMeasBoth=='T' || TimeMeasBoth=='B'
            if isfield(pf,'fx')
                pf.particles(:,k) = feval(pf.fx, pf.particles(:,k), pf.tpara) + sQ*randn(pf.n,1);
            else
                pf.particles(:,k) = pf.Phikk_1*pf.particles(:,k) + sQ*randn(pf.n,1);
            end
        end
        if TimeMeasBoth=='M' || TimeMeasBoth=='B'
            if isfield(pf,'hx')
                v = Zk - feval(pf.hx, pf.particles(:,k), pf.tpara)';
            else
                v = Zk - pf.Hk*pf.particles(:,k);
            end
            vRv(k) = v'*invR*v;
        end
    end
    if TimeMeasBoth=='M' || TimeMeasBoth=='B'
%         [vRv, idx] = sort(vRv);  pf.particles = pf.particles(:,idx);
        weight = exp(-vRv/2);%/sqrt((2*pi)^pf.m*det(pf.Rk)); % Normal distribution
        cumw = cumsum(weight+0.01/pf.Npts);  cumw = cumw/cumw(end);
%         idx = interp1(cumw, (1:pf.Npts)', linspace(cumw(1),cumw(end),pf.Npts+10), 'nearest');  % resampling index
        idx = interp1(cumw, (1:pf.Npts)', cumw(1)+(cumw(end)-cumw(1))*rand(pf.Npts+10,1), 'nearest');  % resampling index
        pf.particles = pf.particles(:,idx(3:pf.Npts+2)); % myfig,hist(pf.particles(2,:));
    end
    pf.xk = mean(pf.particles,2);
    xk = pf.particles - repmat(pf.xk,1,pf.Npts);
	pf.Pxk = diag(sum(xk.^2,2)/pf.Npts);
%     for k=1:pf.n
%         pf.particles(k,:) = pf.xk(k)+sqrt(pf.Pxk(k,k))*randn(1,pf.Npts);
%     end
    if nargout==2  % if using full matrix Pxk
        pf.Pxk = zeros(pf.n);
        for k=1:pf.Npts
            pf.Pxk = pf.Pxk + xk(:,k)*xk(:,k)';
        end
        pf.Pxk = pf.Pxk/pf.Npts;
        Pxk = pf.Pxk;
    end
```

```matlab
function pf = pfinit(x0, P0, Qk, Rk, N)
% Particle filter structure initialization.
%
% Prototype: pf = pfinit(x0, P0, Qk, Rk, N)
% Inputs: x0,P0 - initial state & covariance;
%         Qk,Rk - process & measure covariance
%         N - particle number
% Output: pf - particle filter structure array
%
% see also  pfupdate, kfinit.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/04/2022
    if size(P0,2)>1, P0 = diag(P0); end
    pf.n = length(x0);  pf.m = length(Rk);
    for k=1:pf.n
        pf.particles(k,:) = x0(k)+sqrt(P0(k))*randn(1,N);
    end
    pf.Npts = N;
	pf.Pxk = P0; pf.Qk = Qk; pf.Rk = Rk;
```

### 八、akfupdate：自适应滤波

这段代码是一个自适应卡尔曼滤波器的实现。下面是代码的功能解释：

1. 函数 `akfupdate` 接受五个输入参数：
   - `kf`：一个包含卡尔曼滤波器结构信息的数组。
   - `yk`：一个测量向量。
   - `TimeMeasBoth`：一个描述时间更新和测量更新的标志字符，可以取以下三个值：
     - `'T'`：只进行时间更新。
     - `'M'`：只进行测量更新。
     - `'B'`：同时进行时间更新和测量更新。
   - `kftype`：自适应卡尔曼滤波器的类型。
   - `para`：一些参数。
2. 判断输入参数的数量，根据不同的标志字符执行相应的时间更新或测量更新。
3. 如果 `TimeMeasBoth` 是 `'T'`，则执行时间更新：
   1. 用 `kf.Phikk_1` 乘以 `kf.xk`，更新状态估计值 `kf.xk`。
   2. 用 `kf.Phikk_1` 乘以 `kf.Pxk`，再乘以 `kf.Phikk_1'`，并加上 `kf.Gammak * kf.Qk * kf.Gammak'`，更新状态估计误差协方差矩阵 `kf.Pxk`。
4. 如果 `TimeMeasBoth` 是 `'M'`，则执行测量更新： 
   - 保存当前的状态估计值和误差协方差矩阵到 `kf.xkk_1` 和 `kf.Pxkk_1`。
5. 如果 `TimeMeasBoth` 是 `'B'`，则执行时间更新和测量更新： 
   1. 保存当前的状态估计值和误差协方差矩阵到 `kf.xkk_1` 和 `kf.Pxkk_1`。
   2. 计算 `kf.Pxykk_1`，即当前估计值和测量值之间的协方差矩阵。
   3. 计算 `kf.Py0`，即测量噪声的协方差矩阵乘以 `Hk` 的转置。
   4. 计算残差 `kf.rk`，即实际测量值减去预测的测量值。
6. 根据 `kftype` 的类型执行不同的自适应滤波算法： 
   1. 如果 `kftype` 是字符串 `'KF'` 或 `'MSHAKF'`，则根据提供的参数计算一些变量，如 `b` 和 `s`。
   2. 如果 `kftype` 是数字 `2`，则执行 MCKF（Minimum Curvature Filter）算法，其中使用了一个参数 `sigma`。
   3. 如果 `kftype` 是数字 `3`，则执行 RSTKF（Residual Square Total Least Squares Filter）算法，其中使用了一个参数 `v`。
   4. 如果 `kftype` 是数字 `4`，则执行 SSMKF（Square Root Split-Operator Matrix Kalman Filter）算法。
7. 将更新后的状态估计值、状态估计误差协方差矩阵和其他相关信息存储到输入的结构数组 `kf` 中。
8. 输出更新后的卡尔曼滤波器结构数组 `kf`。

该代码实现了自适应卡尔曼滤波器的不同算法，并根据输入的标志字符选择相应的时间更新或测量更新操作。 

#### shamn ：Sage-Husa自适应滤波 

```matlab
function [Rk, beta, maxflag] = shamn(Rk, r2, Rmin, Rmax, beta, b, s)
% Sage-Husa adaptive measurement noise variance 'Rk'. 
%
% Prototype: [Rk, beta] = shamn(Rk, r2, Rmin, Rmax, beta, b, s)
% Inputs: Rk - measurement noise variance
%         r2 - =rk^2-Py0;
%         Rmin,Rmax - min & max variance bound.
%         beta,b - forgettiing factors
%         s - if b==0, s is enlarge factor
% Output: Rk, beta - same as above
%         maxflag - if r2(k)>Rmax(k) then set the max flag
%
% See also  akfupdate.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 26/03/2022
    m = length(r2);  maxflag = zeros(m,1);
    beta = beta./(beta+b);
    for k=1:m
        if b(k)==0,
            r2(k) = r2(k) / s(k)^2;
            if r2(k)<Rmin(k), Rk(k,k)=Rmin(k); else, Rk(k,k)=r2(k);  end
        else
            if r2(k)<Rmin(k), r2(k)=Rmin(k);
            elseif r2(k)>Rmax(k), r2(k)=Rmax(k); beta(k)=1; maxflag(k)=1; end
            Rk(k,k) = (1-beta(k))*Rk(k,k) + beta(k)*r2(k);
        end
    end
```

这段代码是一个Sage-Husa自适应滤波器的实现。下面是代码的功能解释：

1. 输入参数：
   - `Rk`：测量噪声方差，一个列向量。
   - `r2`：`Rk`的平方减去初始残差平方的平均值，也是一个列向量。
   - `Rmin`和`Rmax`：分别是测量噪声方差的下限和上限，两个列向量。
   - `beta`和`b`：两个列向量，分别表示遗忘因子和增益因子。
   - `s`：当`b`等于0时使用的放大因子，一个列向量。
2. 输出参数： 
   - `Rk`：更新后的测量噪声方差，一个列向量。
   - `beta`：更新后的遗忘因子，一个列向量。
   - `maxflag`：当r2(k)大于Rmax(k)时的最大标志，一个列向量。
3. 代码逻辑： 
   - 首先，根据输入参数的长度，初始化一个长度为m的maxflag向量，全部元素设为0。
   - 然后，将beta除以(beta+b)，得到新的beta值。这是根据遗忘因子的公式进行的更新。
   - 对于每个k值，进行以下操作：
     * 如果b(k)等于0，表示使用增益因子s(k)对r2(k)进行放大。此时，如果r2(k)小于Rmin(k)，则将Rk(k,k)设为Rmin(k)；否则将Rk(k,k)设为r2(k)。
     * 如果b(k)不等于0，表示使用遗忘因子beta(k)对Rk(k,k)进行更新。此时，如果r2(k)小于Rmin(k)，则将r2(k)设为Rmin(k)；如果r2(k)大于Rmax(k)，则将r2(k)设为Rmax(k)，并将beta(k)设为1，同时将maxflag(k)设为1。然后根据遗忘因子的公式，更新Rk(k,k)。
4. 最后，更新完成后的Rk、beta和maxflag作为输出结果。

总体来说，这段代码实现了Sage-Husa自适应滤波器的核心功能，根据输入的参数和当前状态，自适应地调整测量噪声方差的值，并监控是否超出上限。 

### 九、抗差等价权函数

#### igg1

```
function w = igg1(err, k0, k1)
% 'Institute of Geodesy & Geophysics' picewise method to calculate weight.
% Ref. IGG抗差估计在高程拟合中的应用研究_李广来,2021
%
% Prototype: gamma = igg1(err, k0, k1)
% Inputs: err - normalized measurement error
%         k0, k1 - picewise points 
% Outputs: w - weight
%
% Example:
%    figure, err=-100:0.1:100; plot(err,igg1(err,3,10)); grid on
%    figure, err=-100:0.1:100; plot(err,igg1(err,3,inf)); grid on
%
% See also  igg3.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/06/2022
    if nargin==1, k0=1.0; k1=3.0; end  % w = igg1(err)
    if nargin==2, k1=k0(2); k0=k0(1); end  % w = igg1(err, k0k1)
    if k1<2*k0, k1=2*k0; end
    err = abs(err);  w = err;
    for k=1:length(err)
        if err(k)<=k0,    w(k)=1;
        elseif err(k)>k1, w(k)=0;
        else,             w(k)=k0/err(k);  end
    end
```

这段代码是一个用于计算权重的函数，采用了Institute of Geodesy & Geophysics（地球测量与地球物理研究所）提出的一种分段函数方法。该函数接受两个输入参数：误差的归一化测量值（err）和两个分段点（k0和k1）。输出为权重值（w）。

代码首先检查输入参数的数量。如果只有一个输入参数（err），则将k0和k1分别设置为1.0和3.0。如果输入参数有两个（err和k0或k1），则将k1设置为k0的第二个元素，将k0设置为k0的第一个元素。

接下来，代码将误差值abs(err)赋给变量w。然后，通过一个循环遍历err中的每个元素。在循环中，根据误差值的大小，使用条件语句来计算相应的权重值。

如果误差值小于等于k0，权重值设为1.0；如果误差值大于k1，权重值设为0.0。对于介于k0和k1之间的误差值，权重值通过将k0除以误差值来计算。

最后，函数返回计算得到的权重值w。

代码中还提供了一些示例绘图语句，用于展示在不同参数设置下，权重如何随着误差值的变化而变化。

#### igg3

```matlab
function w = igg3(err, k0, k1)
% 'Institute of Geodesy & Geophysics' picewise method to calculate weight.
% Ref. IGG抗差估计在高程拟合中的应用研究_李广来,2021
%
% Prototype: gamma = igg3(err, k0, k1)
% Inputs: err - normalized measurement error
%         k0, k1 - picewise points 
% Outputs: w - weight
%
% Example:
%    figure, err=-10:0.1:10; plot(err,igg3(err,3,8)); grid on
%
% See also  igg1.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 20/06/2022
    if nargin==1, k0=2.0; k1=6.0; end  % w = igg3(err)
    if nargin==2, k1=k0(2); k0=k0(1); end  % w = igg3(err, k0k1)
    if k1<2*k0, k1=2*k0; end
    err = abs(err);  w = err;
    for k=1:length(err)
        if err(k)<=k0,    w(k)=1;
        elseif err(k)>k1, w(k)=0;
        else,             w(k)=k0/err(k) * ((k1-err(k))/(k1-k0))^2;  end
    end
```

这段代码是一个MATLAB函数，名为`igg3`。它用于根据给定的测量误差和两个参数`k0`和`k1`计算权重。

函数的输入参数为：

- `err`：归一化测量误差的值。
- `k0`：分段点之一。
- `k1`：分段点之二。

函数的输出为：

- `w`：计算得到的权重。

该函数使用了'Institute of Geodesy & Geophysics'（大地测量与地球物理学研究所）的picewise方法来计算权重。该方法根据测量误差的大小，在两个分段点之间使用不同的计算方式来得到权重值。

函数首先检查输入参数的数量。如果只有一个输入参数`err`，则默认设置`k0`为2.0，`k1`为6.0。根据不同的误差范围，使用不同的公式计算权重值：

- 如果误差小于等于`k0`，则权重为1。
- 如果误差大于`k1`，则权重为0。
- 否则，使用以下公式计算权重值：`w(k) = k0/err(k) * ((k1-err(k))/(k1-k0))^2`

最后，返回计算得到的权重向量作为函数的输出结果。

这个函数主要用于在高程拟合中进行抗差估计，通过根据测量误差的大小调整权重来减小误差对拟合结果的影响。

### 十、数据融合

#### POSFusion

```matlab
function psf = POSFusion(rf, xpf, rr, xpr, ratio)
% POS data fusion for forward and backward results.
%
% Prototype: psf = POSFusion(rf, xpf, rr, xpr, ratio)
% Inputs: rf - forward avp
%         xpf - forward state estimation and covariance
%         rr - backward avp
%         xpr - backward state estimation and covariance
%         ratio - the ratio of state estimation used to modify avp.
% Output: the fields in psf are
%         rf, pf - avp & coveriance after fusion
%         r1, p1 - forward avp & coveriance
%         r2, p2 - backward avp & coveriance
%
% See also  insupdate, kfupdate, POSSmooth, POSProcessing, posplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014
    if nargin<5
        ratio = 1;
    end
    [t, i1, i2] = intersect(rf(:,end), rr(:,end));
    n = size(xpf,2)-1;  n2 = n/2;
    r1 = rf(i1,1:end-1); x1 = xpf(i1,1:n2); p1 = xpf(i1,n2+1:end-1);
    r2 = rr(i2,1:end-1); x2 = xpr(i2,1:n2); p2 = xpr(i2,n2+1:end-1);
    x1(:,1:9) = x1(:,1:9)*ratio; x2(:,1:9) = x2(:,1:9)*ratio;  % ratio
    %  common fusion
    r10 = [r1(:,1:9)-x1(:,1:9),r1(:,10:end)+x1(:,10:end)];
    r20 = [r2(:,1:9)-x2(:,1:9),r2(:,10:end)+x2(:,10:end)];
%     r = ( r10.*p2 + r20.*p1 )./(p1+p2);  
%     p = p1.*p2./(p1+p2);
    [r, p] = fusion(r10, p1, r20, p2);
    % attitude fusion
    for k=1:length(t)
    	r10(k,1:3) = q2att(qdelphi(a2qua(r1(k,1:3)'),x1(k,1:3)'))';
    	r20(k,1:3) = q2att(qdelphi(a2qua(r2(k,1:3)'),x2(k,1:3)'))';
    end
    r(:,1:3) = attfusion(r10(:,1:3), p1(:,1:3), r20(:,1:3), p2(:,1:3));
    rf = [r,t]; r1 = [r10,t]; r2 = [r20,t];
    pf = [p,t]; p1 = [p1,t];  p2 = [p2,t];
    rf(isnan(rf)) = 0;
    pf(isnan(pf)) = 0;
    psf = varpack(rf, pf, r1, p1, r2, p2);
```

这段代码是一个用于进行POS（位置和姿态）数据融合的函数。它接受五个输入参数：rf（正向AVP（伪距））、xpf（正向状态估计和协方差）、rr（反向AVP）、xpr（反向状态估计和协方差）和ratio（用于修改状态估计的比重）。该函数的输出是一个包含以下字段的结构体psf：

- rf和pf：经过融合后的AVP和协方差
- r1和p1：正向AVP和协方差
- r2和p2：反向AVP和协方差

代码的功能如下：

1. 检查输入参数的数量，如果小于5个，则将ratio设置为1。
2. 通过找到rf和rr在最后一个维度上的交集，确定共同的时刻t、索引i1和i2。
3. 提取r1、x1、p1、r2、x2和p2，分别对应于正向和反向的AVP、状态估计和协方差。
4. 根据ratio的值，对x1和x2的前9个字段进行比例调整。
5. 执行融合操作，生成融合后的AVP和协方差r和p。这里使用了名为fusion的函数，具体的融合方法可能在该函数中实现。
6. 将rf和pf中的NaN值替换为0。
7. 将psf作为输出返回。

需要注意的是，这段代码中的一些函数（例如insupdate、kfupdate、POSSmooth、POSPlatform、posplot等）并没有在代码中给出实现，因此无法确定这些函数的具体功能。此外，该代码的实现可能依赖于其他未包含在此代码片段中的函数或库。

#### POSSmooth

```matlab
function vpOut = POSSmooth(vp, gnss, nSec, isfig)
% POS data smooth via INS-VelPos & GNSS-VelPos.
%
% Prototype: vpOut = POSSmooth(vp, gnss, nSec, isfig)
% Inputs: vp - INS Vel&Pos input array = [vn,pos,t]
%         gnss - GNSS Vel&Pos input array = [vn,pos,t]
%         nSen - smooth span in second
%         isfig - figure flag
% Output: vpOut - INS Vel&Pos output after smooth
%
% See also  POSFusion, POSProcessing, smoothol, fusion.

% Copyright(c) 2009-2021, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 18/10/2021
    if nargin<4, isfig=1; end
    if nargin<3, nSec=10; end
	err = gnss(:,end-1) - interp1(vp(:,end), vp(:,end-1), gnss(:,end));
	gnss = gnss(~isnan(err),:);  err = gnss;  errs = err;
    nSec = fix(nSec/(gnss(end,end)-gnss(1,end))*length(gnss));  % smooth points
    for k=1:size(gnss,2)-1
        err(:,end-k) = gnss(:,end-k) - interp1(vp(:,end), vp(:,end-k), gnss(:,end));
        [~, errs(:,end-k)] = smoothol(err(:,end-k), nSec, 2, 0);
    end
    idx = ~isnan(interp1(errs(:,end), errs(:,end-k), vp(:,end)));
    vpOut = vp;
    for k=1:size(gnss,2)-1
        vpOut(idx,end-k) = vp(idx,end-k) + interp1(errs(:,end), errs(:,end-k), vp(idx,end), 'spline');
    end
    if isfig==1
        eth = earth(gnss(1,end-3:end-1)');
        myfig
        if size(gnss,2)==7, subplot(211); end
        plot(err(:,end), [err(:,end-3)*eth.RMh, err(:,end-2)*eth.clRNh, err(:,end-1)]); xygo('dP');
        plot(errs(:,end), [errs(:,end-3)*eth.RMh, errs(:,end-2)*eth.clRNh, errs(:,end-1)],'-.','linewidth',2);
        if size(gnss,2)==7
            subplot(212); 
            plot(err(:,end), err(:,end-6:end-4)); xygo('V'); plot(errs(:,end), errs(:,end-6:end-4), '-.','linewidth',2)
            avpcmpplot(gnss, vpOut(:,end-6:end), 'vp');
        else
            avpcmpplot(gnss, vpOut(:,end-3:end), 'p');
        end
    end
```

该函数是一个用于通过INS-VelPos和GNSS-VelPos数据进行位置平滑的函数。它的输入包括INS Vel&Pos数据数组vp、GNSS Vel&Pos数据数组gnss、平滑时间跨度nSec和一个图形标志isfig。它的输出是经过平滑处理后的INS Vel&Pos数据数组vpOut。

以下是该函数的具体实现步骤：

1. 首先，根据输入参数判断是否指定了图形标志isfig和平滑时间跨度nSec。如果没有指定，则默认isfig为1，nSec为10。
2. 接下来，计算GNSS数据和INS数据之间的误差err。这个误差是通过在INS数据的最后一个点和GNSS数据的最后一个点之间进行线性插值来计算的。然后将gnss数组中的NaN值替换为err数组，并将errs数组初始化为err数组。
3. 根据平滑时间跨度nSec，计算要平滑的点的数量。这是通过将nSec转换为gnss数组中点的数量来完成的。
4. 对于每个要平滑的点k，计算err(:,end-k)的值。这是通过在INS数据的最后一个点和gnss数据的前一个点之间进行线性插值来计算的。然后使用smoothol函数对err(:,end-k)进行平滑处理，得到errs(:,end-k)。
5. 根据interp1函数计算出一个指示索引idx，它用于确定哪些点的位置需要进行平滑。
6. 初始化vpOut数组为原始的vp数组。
7. 对于每个要平滑的点k，根据errs(:,end-k)和vp(idx,end)的值，使用interp1函数进行插值计算，并将结果加到vpOut(idx,end-k)中，以得到平滑后的位置数据。
8. 如果isfig等于1，则绘制平滑结果的图形。其中包括误差曲线和平滑后的位置数据曲线。

需要注意的是，该函数使用了外部库中的一些函数，例如interp1、smoothol和eth。这些函数可能需要在系统上进行相应的设置和配置才能正常运行。

#### POSProcessing

```matlab
function [ps, psf] = POSProcessing(kf, ins, imu, vpGPS, fbstr, ifbstr)
% POS forward and backward data processing.
% States: phi(3), dvn(3), dpos(3), eb(3), db(3), lever(3), dT(1), 
%         dKg(9), dKa(6). (total states 6*3+1+9+6=34)
%
% Prototype: ps = POSProcessing(kf, ins, imu, posGPS, fbstr, ifbstr)
% Inputs: kf - Kalman filter structure array from 'kfinit'
%         ins - SINS structure array from 'insinit'
%         imu - SIMU data including [wm, vm, t]
%         vpGPS - GPS velocity & position [lat, lon, heigth, tag, tSecond]
%                 or [vE, vN, vU, lat, lon, heigth, tag, tSecond], where
%                 'tag' may be omitted.
%         fbstr,ifbstr - Kalman filter feedback string indicator for
%               forward and backward processing. if ifbstr=0 then stop
%               backward processing
% Output: ps - a structure array, its fields are
%             avp,xkpk  - forward processing avp, state estimation and
%                         variance
%             iavp,ixkpk  - backward processing avp, state estimation and
%                         variance
%
% Example:
%     psinstypedef(346);
%     davp0 = avperrset([30;-30;30], [0.01;0.01;0.03], [0.01;0.01;0.03]);
%     lever = [0.; 0; 0]; dT = 0.0; r0 = davp0(4:9)';
%     imuerr = imuerrset(0.01,100,0.001,1, 0,0,0,0, [0;0;1000],0,[0;0;0;0;10;10],0);
%     ins = insinit([att; pos], ts);
%     kf = kfinit(ins, davp0, imuerr, lever, dT, r0);
%     ps = POSProcessing(kf, ins, imu, vpGPS, 'avped', 'avp');
%     psf = POSFusion(ps.avp, ps.xkpk, ps.iavp, ps.ixkpk);
%     POSplot(psf);
% 
% See also  sinsgps, insupdate, kfupdate, POSFusion, posplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014, 08/02/2015
    if ~exist('fbstr','var'),  fbstr = 'avp';  end
    if ~exist('ifbstr','var'),  ifbstr = fbstr;  end
    len = length(imu); [lenGPS,col] = size(vpGPS);
    if col==4||col==7, vpGPS = [vpGPS(:,1:end-1),ones(lenGPS,1),vpGPS(:,end)]; end % add tag
    imugpssyn(imu(:,7), vpGPS(:,end));
    ts = ins.ts; nts = kf.nts; nn = round(nts/ts);
    dKga = zeros(15,1);
    %% forward navigation
	[avp, xkpk, zkrk] = prealloc(ceil(len/nn), kf.n+1, 2*kf.n+1, 2*kf.m+1); ki = 1;
    Qk = zeros(length(vpGPS), kf.n+1);  Rk = zeros(length(vpGPS), 8);  kki = 1;  zk = zeros(size(kf.Hk,1),1);
    timebar(nn, len, 'SINS/GPS POS forward processing.');
    for k=1:nn:(len-nn+1)
        k1 = k+nn-1; wvm = imu(k:k1,1:6); t = imu(k1,7);
        ins = insupdate(ins, wvm);  ins = inslever(ins);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        [kgps, dt] = imugpssyn(k, k1, 'F');
        if kgps>0 
            if vpGPS(kgps,end-1)>=1 % tag OK
                kf.Hk = kfhk(ins);
                if size(kf.Hk,1)==6
                    zk = [ins.vnL-ins.an*dt; ins.posL-ins.Mpvvn*dt]-vpGPS(kgps,1:6)';
                else
                    zk = ins.posL-ins.Mpvvn*dt-vpGPS(kgps,1:3)';
                end
            	kf = kfupdate(kf, zk, 'M');
            end
            Qk(kki,:) = [diag(kf.Qk); t]';
            Rk(kki,:) = [diag(kf.Rk); kf.beta; t]';  kki = kki+1;
        end
        [kf, ins] = kffeedback(kf, ins, nts, fbstr);
        dKg = ins.Kg-eye(3); dKa = ins.Ka-eye(3);
        dKga = [dKg(:,1);dKg(:,2);dKg(:,3); dKa(:,1);dKa(2:3,2);dKa(3,3)];
        avp(ki,:) = [ins.avpL; ins.eb; ins.db; ins.lever; ins.tDelay; dKga; t]';
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        zkrk(ki,:) = [zk; diag(kf.Rk); t]'; ki = ki+1;
        timebar;
    end
    avp(ki:end,:)=[]; xkpk(ki:end,:)=[]; zkrk(ki:end,:)=[]; Qk(kki:end,:)=[];  Rk(kki:end,:)=[];
    ps.avp = avp; ps.xkpk = xkpk; ps.zkrk = zkrk; ps.Qk = [sqrt(Qk(:,1:end-1)),Qk(:,end)];  ps.Rk = [sqrt(Rk(:,1:6)),Rk(:,7:8)];
    if ~ischar(ifbstr), return; end
    %% reverse navigation
    [ikf, iins, idx] = POSReverse(kf, ins);
    [iavp, ixkpk, izkrk] = prealloc(ceil(len/nn), kf.n+1, 2*kf.n+1, 2*kf.m+1); ki = 1;
    timebar(nn, len, 'SINS/GPS POS reverse processing.');
    for k=k1:-nn:(1+nn)
        ik1 = k-nn+1; wvm = imu(k:-1:ik1,1:6); wvm(:,1:3) = -wvm(:,1:3); t = imu(ik1-1,7);
        iins = insupdate(iins, wvm);  iins = inslever(iins);
        ikf.Phikk_1 = kffk(iins);
        ikf = kfupdate(ikf);
        [kgps, dt] = imugpssyn(ik1, k, 'B');
        if kgps>0 
            if vpGPS(kgps,end-1)>=1 % tag OK
                ikf.Hk = kfhk(iins);
                if size(ikf.Hk,1)==6
                    zk = [iins.vnL-iins.an*dt; iins.posL-iins.Mpvvn*dt]-[-vpGPS(kgps,1:3),vpGPS(kgps,4:6)]';
                else
                    zk = iins.posL-iins.Mpvvn*dt-vpGPS(kgps,1:3)';
                end
            	ikf = kfupdate(ikf, zk, 'M');
            end
        end
        [ikf, iins] = kffeedback(ikf, iins, nts, ifbstr);
        dKg = iins.Kg-eye(3); dKa = iins.Ka-eye(3);
        dKga = [dKg(:,1);dKg(:,2);dKg(:,3); dKa(:,1);dKa(2:3,2);dKa(3,3)];
        iavp(ki,:) = [iins.avpL; iins.eb; iins.db; iins.lever; iins.tDelay; dKga; t]';
        ixkpk(ki,:) = [ikf.xk; diag(ikf.Pxk); t]';
        izkrk(ki,:) = [zk; diag(ikf.Rk); t]';     ki = ki+1;
        timebar;
    end
    iavp(ki:end,:)=[]; ixkpk(ki:end,:)=[]; izkrk(ki:end,:)=[];  
    iavp = flipud(iavp); ixkpk = flipud(ixkpk); izkrk = flipud(izkrk); % reverse inverse sequence
    iavp(:,idx) = -iavp(:,idx);  ixkpk(:,idx) = -ixkpk(:,idx);
    ps.iavp = iavp; ps.ixkpk = ixkpk; ps.izkrk = izkrk;
    if nargout==2
        psf = POSFusion(ps.avp, ps.xkpk, ps.iavp, ps.ixkpk);
    end
    
function [ikf, iins, idx] = POSReverse(kf, ins)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn; iins.eb = -iins.eb; iins.tDelay = -iins.tDelay;
    ikf = kf;
    Pd=diag(ikf.Pxk); ikf.Pxk = diag([10*Pd(1:19);Pd(20:end)]);
    idx = [4:6,10:12,19];   % vn,eb,dT  (dKg no reverse!)
    ikf.xk(idx) = -ikf.xk(idx); % !!!
```

#### POSProcessing_ifk_test

```matlab
function POSProcessing_ifk_test()
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 27/12/2019
global glv
    ts = 1; nts = 1; nn = 1; T = 5000; len = T/ts;
    psinstypedef(186);
    %% forward navigation
    ts = 0.01;
    avp0 = avpset([1;10;30]*glv.deg, [100;200;10], [34;106;100]);
    davp0 = avperrset([60;-60;300], [0.1;0.1;0.3], [1;1;3]);
    ins = insinit(avpadderr(avp0,[[30;-30;30]; [1;1;3]; [1;1;3]]), ts); ins.nts = nts;
    Ft = etm(ins);
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn;
    iins.eth = ethupdate(iins.eth, iins.pos, iins.vn);
    iFt = etm(iins);
    xk = [davp0; [1;1;1]*glv.dph; [100;100;100]*glv.ug];
    xk1 = (eye(15)+Ft*ts)*xk;
    ixk = xk1; ixk([4:6,10:12]) = -ixk([4:6,10:12]);
    ixk1 = (eye(15)+iFt*ts)*ixk;
    ixk2 = (eye(15)-Ft*ts)*ixk;
    [ixk1,ixk2,xk]
    
    avp0 = avpset([1;10;30]*glv.deg, [0;0;0], [34;106;100]);
    davp0 = avperrset([1;-1;30], [0.1;0.1;0.3], [0.01;0.01;0.03]);
    imuerr = imuerrset(0, 0, 0, 0);
    imu = imustatic(avp0, ts, len, imuerr);
    ins = insinit(avpadderr(avp0,[[30;-30;30]; [0.1;0.1;0.3]; [0.01;0.01;0.03]]), ts); ins.nts = nts;
    kf = kfinit(ins, davp0, imuerr, [0;0;0], 0, [0;0;0]);
    kf.xk(1:9) = davp0;
	err = prealloc(ceil(len/nn), 19); ki = 1;
    timebar(nn, len, 'SINS/GPS POS forward processing.');
    for k=1:nn:(len-nn+1)
        k1 = k+nn-1; wvm = imu(k:k1,1:6); t = imu(k1,7);
        ins = insupdate(ins, wvm);  ins.vn(3) = 0; kf.xk(6) = 0;
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
        err(ki,:) = [aa2phi(ins.att,avp0(1:3));(ins.avp(4:9)-avp0(4:9)); kf.xk(1:9); t];  ki = ki+1;
        timebar;
    end
    err(ki:end,:)=[];
    %% reverse navigation
    [ikf, iins, idx] = POSReverse(kf, ins);
	ierr = prealloc(ceil(len/nn), 19); ki = 1;
    timebar(nn, len, 'SINS/GPS POS reverse processing.');
    for k=k1:-nn:(1+nn)
        ik1 = k-nn+1; wvm = imu(k:-1:ik1,1:6); wvm(:,1:3) = -wvm(:,1:3); t = imu(ik1-1,7);
        iins = insupdate(iins, wvm);  iins.vn(3) = 0; ikf.xk(6) = 0;
        ikf.Phikk_1 = kffk(iins);
        ikf = kfupdate(ikf);
        ierr(ki,:) = [aa2phi(iins.att,avp0(1:3));(iins.avp(4:9)-avp0(4:9)); ikf.xk(1:9); t-0.5];  ki = ki+1;
        timebar;
    end
    ierr(ki:end,:)=[];
%     ierr = flipud(ierr);  
    ierr(:,[4:6]) = -ierr(:,[4:6]);  ierr(:,9+[4:6]) = -ierr(:,9+[4:6]);
    err = [err;ierr];
    errr = avpcmpplot(err(:,[1:9,end]), err(:,10:end));
%     avpcmpplot(ierr(:,[1:9,end]), ierr(:,10:end));
    function [ikf, iins, idx] = POSReverse(kf, ins)
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 22/01/2014
    iins = ins;
    iins.eth.wie = -iins.eth.wie;
    iins.vn = -iins.vn; iins.eb = -iins.eb; iins.tDelay = -iins.tDelay;
    ikf = kf;
    ikf.Pxk = 10*diag(diag(ikf.Pxk));
    idx = [4:6,10:12];   % vn,eb,dT,dvn  (dKg no reverse!)
    ikf.xk(idx) = -ikf.xk(idx); % !!!
```

### 十一：模拟

#### htwn：产生高斯白噪声

```matlab
function [wn, s] = htwn(wp, s, len)
% Generate Heavy-tailed white noise.
%
% Prototype: wn = htwn(wp, enlarge, len)
% Inputs: wp - with probability
%         s - std enlarge with probability 'wp' 
%         len - element number 
% Outputs: wn - noise output
%          s - enlarge number for each wn
%
% Example1:
%    figure, plot(htwn(0.01, 100, 1000)); grid on
%
% Example2:
%    x=-30:0.1:30; n=1000; figure, plot(x,histc([randn(10000,1),htwn(0.3,10,10000)],x)/n); grid on
%
% See also  rstd.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 01/03/2022
    if length(wp)>1  % [wn, s] = htwn([wp1;wp2;...], [s1;s2;...], [len1;len2;...]);
        s1 = s;
        wn = []; s = [];
        for k=1:length(wp)
            [wnk,sk] = htwn(wp(k), s1(k), len(k));
            wn = [wn; wnk];  s = [s; sk];
        end
        return;
    end
    if nargin<3, len=1; end
    if nargin<2, s=100; end
    if nargin<1, wp=0.01; end
    if wp>1, len=wp; wp=0.01; end  % wn = htwn(len)
    wn = randn(len,1);
    r01 = rand(len,1);
    idx = r01 < wp;
    wn(idx) = wn(idx)*s;
    s(idx,1) = s;  s(~idx,1) = 1;
```

这段代码是一个用于生成heavy-tailed白噪声的函数。下面是代码的功能解释：

1. 函数的输入参数包括：
   - `wp`：一个表示概率的向量或标量，用于确定是否对生成的噪声进行放大。
   - `s`：一个表示放大数量的向量或标量，用于对生成的噪声进行放大。
   - `len`：一个表示生成噪声的元素数量的向量或标量。
2. 函数的输出包括： 
   * `wn`：生成的噪声的向量或矩阵，其中包含了heavy-tailed分布的随机值。
   * `s`：每个噪声元素的放大数量。
3. 代码中的主要逻辑如下：
   - 首先，通过判断输入参数的长度，支持同时生成多个噪声序列。如果`wp`或`s`或`len`的长度大于1，则进入循环，逐个生成噪声序列并返回结果。
   - 然后，根据输入参数检查是否给出了足够的参数。如果没有给出`len`参数，默认为1；如果没有给出`s`参数，默认为100；如果没有给出`wp`参数，默认为0.01。
   - 接下来，生成一个长度为`len`的随机噪声向量`wn`，其中包含了符合标准正态分布的随机值。
   - 然后，生成一个长度与`wn`相同的随机向量`r01`，其中的值在0和1之间。
   - 使用生成的随机向量`r01`和一个阈值`wp`，确定哪些噪声元素需要进行放大。将满足概率小于`wp`的元素乘以放大数量`s`，并更新相应的放大数量`s`。
4. 最后，将生成的噪声向量和放大数量返回作为函数的输出。

总体而言，这段代码通过使用随机生成和放大的方法来生成具有heavy-tailed分布特征的噪声数据。

#### vfbfx：模拟垂直下落物体的状态方程

```matlab
function [X1, Phik, D] = vfbfx(X0, tpara)
% Vertically falling body state equation f(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, state equations:
%   dx1/dt = -x2
%   dx2/dt = g-exp(-gamma*x1)*x2^2*x3
%   dx3/dt = 0
%
% See also  vfbhx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    %     Phik = eye(3) + [0,1,0; 0,0,1; 0,0,0]*tpara.ts; D{1}=zeros(3); D{2}=zeros(3); D{3}=zeros(3);  % linear model test
    %     X1 = Phik*X0;
    %     return;  % for linear test
    gamma = tpara.gamma;  g = tpara.g;  Ts = tpara.Ts;
    x1 = X0(1); x2 = X0(2); x3 = X0(3);
    %% X0->X1
    egx1 = exp(-gamma*x1);
    X1 = [ x1 - x2*Ts;   % Euler1 discretization
           x2 + (g-egx1*x2^2*x3)*Ts;
           x3 ];
%     k1 = [-X0(2);  g-exp(-gamma*X0(1))*X0(2)^2*X0(3);    0];  X01 = X0+k1*Ts/2;  % RK4
%     k2 = [-X01(2); g-exp(-gamma*X01(1))*X01(2)^2*X01(3); 0];  X02 = X0+k2*Ts/2;
%     k3 = [-X02(2); g-exp(-gamma*X02(1))*X02(2)^2*X02(3); 0];  X03 = X0+k3*Ts;
%     k4 = [-X03(2); g-exp(-gamma*X03(1))*X03(2)^2*X03(3); 0];
%     X1 = X0 + Ts/6*(k1+2*(k2+k3)+k4);
    %% Jacobian Phik
    if nargout>1
        Phik = [ 0,            -1,    0;
                [gamma*x2*x3,  -2*x3, -x2]*egx1*x2;
                 0,            0,     0 ];
        Phik = eye(size(Phik)) + Phik*Ts;   % Phi = expm(Phi*ts);
    end
    %% Hessian D
    if nargout>2
        D{1} = zeros(3);
        D{2} = [ [-gamma*x2*x3, 2*x3,       x2]*gamma*egx1*x2;
                  0,            -2*egx1*x3, -2*egx1*x2
                  0,            0,          0 ]*Ts;
        D{2}(2,1)=D{2}(1,2); D{2}(3,1)=D{2}(1,3); D{2}(3,2)=D{2}(2,3); % symmetric
        D{3} = zeros(3);
    end
```

这段代码是一个用于模拟垂直下落物体的状态方程的函数。它接受一个初始状态向量X0和一个包含参数的向量tparana作为输入，并返回预测的状态向量X1、雅可比矩阵Phik和海森矩阵D。

下面是代码的功能解释：

1. 定义参数：
   - `gamma`：阻力系数
   - `g`：重力加速度
   - `Ts`：离散时间步长
2. 初始化变量： 从X0中提取x1、x2和x3的值，分别存储在这些变量中。
3. 预测状态向量X1： 使用Euler离散化方法对状态方程进行预测，得到下一个时刻的状态向量X1。
4. 计算雅可比矩阵Phik： 如果函数有超过一个输出参数，则计算雅可比矩阵Phik。Phik用于线性化状态方程，其在数值上近似于状态方程的导数。
5. 计算海森矩阵D： 如果函数有超过两个输出参数，则计算海森矩阵D。D是用于线性化状态方程的雅可比矩阵的导数，即D是对状态方程的二阶导数。

在这段代码中，使用了Euler离散化方法对状态方程进行预测，并计算了状态方程的雅可比矩阵和海森矩阵。这些矩阵对于进行状态估计和参数辨识等任务非常重要。

#### vfbhx：模拟垂直下落的物体的测量方程

```matlab
function [Z, Hk, D] = vfbhx(X, tpara, rk)
% Vertically falling body measurement equation h(x).
% Ref: Athans, M. 'Suboptimal state estimation for continuous-time nonlinear systems 
% from discrete noisy measurements'. IEEE Transactions on Automatic Control,1968.
% [x1,x2,x3]: altitude,velocity,ballistic-parameter, measurement equation:
%   Z = sqrt(M^2+(x3-H)^2) + V
%
% See also  vfbfx.

% Copyright(c) 2009-2022, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 08/08/2022
    %     Z = X(1);  Hk = [1, 0, 0];  D{1} = zeros(3);   % linear model test
    %     if nargin==3, Z=Z+V*randn(1); end
    %     return;  % for linear test
    M = tpara.M; H = tpara.H;
    %% Z
    Z = sqrt(M^2+(X(1)-H)^2);
    %% Jacobian Hk
    if nargout>1,  Hk = [(X(1)-H)/Z, 0, 0];  end
    %% Hessian D
    if nargout>2,  D{1} = [M^2/Z^3, 0, 0; 0, 0, 0; 0, 0, 0];  end
    %% Measurement noise
    if nargin==3, Z=Z+rk*randn(1); end
```

这段代码是一个MATLAB函数，用于计算垂直下落的物体的测量方程。下面是代码的功能解释：

1. 函数名：`vfbhx`，它接受三个输入参数：`X`，`tpara`和`rk`。
2. `X`是一个列向量，包含了三个元素：高度、速度和弹道参数。
3. `tpara`是一个包含两个元素的向量，第一个元素是质量参数`M`，第二个元素是参考高度`H`。
4. `rk`是测量噪声的倍数，用于模拟真实测量中的噪声。
5. 代码开头的注释提供了一些参考信息，包括参考文献和版权声明。
6. 代码的第一个注释行是一个测试用的线性模型，如果函数有四个输入参数（即`nargin==3`），则在计算结果之前向`Z`中添加一个乘以`V`的随机噪声。
7. `Z`的计算是根据测量方程进行的，使用了高度和弹道参数，并考虑了距离的影响。
8. 如果函数需要输出雅可比矩阵`Hk`（`nargout==2`），则根据测量方程计算出雅可比矩阵的值。
9. 如果函数需要输出海森矩阵`D`（`nargout==3`），则根据测量方程计算出海森矩阵的元素值。
10. 最后，如果输入参数中包含第三个元素`rk`，则在计算结果之前向`Z`中添加一个乘以`rk`的随机噪声，以模拟测量中的噪声。
11. 函数的最后一行是可选的，它用于在命令行中显示函数的输出结果。

总结来说，这段代码实现了一个用于计算垂直下落的物体的测量方程的函数。它根据输入的高度、速度、弹道参数和测量噪声，计算出测量结果，并可以输出雅可比矩阵和海森矩阵（如果需要）。















