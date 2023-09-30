> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、捷联惯导更新

![image-20230816151634166](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230816151634166.png)

### 1、insinit()：初始化 ins 结构体





### 2、ethupdate()：地球自转角速度和牵连角速度更新

主要是根据位置 pos 和速度 vn 计算比力方程中有害加速度相关项。

```matlab
function eth = ethupdate(eth, pos, vn)
% Update the Earth related parameters, much faster than 'earth'.
%
% Prototype: eth = ethupdate(eth, pos, vn)
% Inputs: eth - input earth structure array
%         pos - geographic position [lat;lon;hgt]
%         vn - velocity
% Outputs: eth - parameter structure array
%
% See also  ethinit, earth.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/05/2014
    % 如果没输入速度，就设为 0
    if nargin==2,  vn = [0; 0; 0];  end  
     % 将输入的位置速度赋值给 eth 结构体
    eth.pos = pos;  eth.vn = vn;   
    % 计算 sin(纬度)、cos(纬度)、tan(纬度)、sin(纬度)平方、sin(纬度)四次方
    eth.sl = sin(pos(1));  eth.cl = cos(pos(1));  eth.tl = eth.sl/eth.cl;   
    eth.sl2 = eth.sl*eth.sl;  sl4 = eth.sl2*eth.sl2;
    % 计算计算子午圈半径 RM、卯酉圈半径 RN
    sq = 1-eth.e2*eth.sl2;  RN = eth.Re/sqrt(sq); 
    eth.RNh = RN+pos(3);  eth.clRNh = eth.cl*eth.RNh;
    eth.RMh = RN*(1-eth.e2)/sq+pos(3);
    % 计算地球自转引起的导航系旋转 wie_n、载体在地球表面移动因地球曲率引起的导航系旋转 wen_n
    % 它们的和 wnin
%     eth.wnie = [0; eth.wie*eth.cl; eth.wie*eth.sl];
    eth.wnie(2) = eth.wie*eth.cl; eth.wnie(3) = eth.wie*eth.sl;
%     eth.wnen = [-vn(2)/eth.RMh; vn(1)/eth.RNh; vn(1)/eth.RNh*eth.tl];
    eth.wnen(1) = -vn(2)/eth.RMh; eth.wnen(2) = vn(1)/eth.RNh; eth.wnen(3) = eth.wnen(2)*eth.tl;
%     eth.wnin = eth.wnie + eth.wnen;
    eth.wnin(1) = eth.wnie(1) + eth.wnen(1); eth.wnin(2) = eth.wnie(2) + eth.wnen(2); eth.wnin(3) = eth.wnie(3) + eth.wnen(3); 
%     eth.wnien = eth.wnie + eth.wnin;
    eth.wnien(1) = eth.wnie(1) + eth.wnin(1); eth.wnien(2) = eth.wnie(2) + eth.wnin(2); eth.wnien(3) = eth.wnie(3) + eth.wnin(3);
    % 计算重力
%     eth.gn = [0;0;-eth.g];
    eth.g = eth.g0*(1+5.27094e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3); % grs80
    eth.gn(3) = -eth.g;
%     eth.gcc = eth.gn - cros(eth.wnien,vn); % Gravitational/Coriolis/Centripetal acceleration
%     eth.gcc =  [ eth.wnien(3)*vn(2)-eth.wnien(2)*vn(3);  % faster than previous line
%                  eth.wnien(1)*vn(3)-eth.wnien(3)*vn(1);
%                  eth.wnien(2)*vn(1)-eth.wnien(1)*vn(2)+eth.gn(3) ];
    % 计算有害加速度
    eth.gcc(1) = eth.wnien(3)*vn(2)-eth.wnien(2)*vn(3);
    eth.gcc(2) = eth.wnien(1)*vn(3)-eth.wnien(3)*vn(1);
    eth.gcc(3) = eth.wnien(2)*vn(1)-eth.wnien(1)*vn(2)+eth.gn(3);
    if isfield(eth, 'dgn')
        while eth.dgnt>eth.dgn(eth.dgnk,end) && eth.dgnk<eth.dgnlen, eth.dgnk=eth.dgnk+1; end
        eth.gcc = eth.gcc + eth.dgn(eth.dgnk,1:3)';
        eth.gn = eth.gn + eth.dgn(eth.dgnk,1:3)';
    end
```



将输入的位置速度赋值给 eth 结构体，如果没输入速度，就设为 0：

```matlab
if nargin==2,  vn = [0; 0; 0];  end
eth.pos = pos;  eth.vn = vn; 
```

计算 sin(纬度)、cos(纬度)、tan(纬度)、sin(纬度)平方、sin(纬度)四次方：

```matlab
eth.sl = sin(pos(1));  eth.cl = cos(pos(1));  eth.tl = eth.sl/eth.cl;   
eth.sl2 = eth.sl*eth.sl;  sl4 = eth.sl2*eth.sl2;
```

计算计算子午圈半径 RM、卯酉圈半径 RN：
$$
R_{M}=\frac{R_{e}\left(1-e^{2}\right)}{\left(1-e^{2} \sin ^{2} L\right)^{3 / 2}}、R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```matlab
sq = 1-eth.e2*eth.sl2;  RN = eth.Re/sqrt(sq); 
eth.RNh = RN+pos(3);  eth.clRNh = eth.cl*eth.RNh;
eth.RMh = RN*(1-eth.e2)/sq+pos(3); 
```

计算地球自转引起的导航系旋转 wie_n、载体在地球表面移动因地球曲率引起的导航系旋转 wen_n，以及它们的和 wnin：



```matlab
%     eth.wnie = [0; eth.wie*eth.cl; eth.wie*eth.sl];
    eth.wnie(2) = eth.wie*eth.cl; eth.wnie(3) = eth.wie*eth.sl;
%     eth.wnen = [-vn(2)/eth.RMh; vn(1)/eth.RNh; vn(1)/eth.RNh*eth.tl];
    eth.wnen(1) = -vn(2)/eth.RMh; eth.wnen(2) = vn(1)/eth.RNh; eth.wnen(3) = eth.wnen(2)*eth.tl;
%     eth.wnin = eth.wnie + eth.wnen;
    eth.wnin(1) = eth.wnie(1) + eth.wnen(1); eth.wnin(2) = eth.wnie(2) + eth.wnen(2); eth.wnin(3) = eth.wnie(3) + eth.wnen(3); 
%     eth.wnien = eth.wnie + eth.wnin;
    eth.wnien(1) = eth.wnie(1) + eth.wnin(1); eth.wnien(2) = eth.wnie(2) + eth.wnin(2); eth.wnien(3) = eth.wnie(3) + eth.wnin(3);
```

计算重力、和重力在 n 系投影 gn：



```matlab
eth.g = eth.g0*(1+5.27094e-3*eth.sl2+2.32718e-5*sl4)-3.086e-6*pos(3); % grs80
eth.gn(3) = -eth.g;
```

计算有害加速度积分项 gcc：



```matlab
eth.gcc(1) = eth.wnien(3)*vn(2)-eth.wnien(2)*vn(3);
eth.gcc(2) = eth.wnien(1)*vn(3)-eth.wnien(3)*vn(1);
eth.gcc(3) = eth.wnien(2)*vn(1)-eth.wnien(1)*vn(2)+eth.gn(3);
```





### 3、insupdate()：捷联惯导更新

先进行不可交换误差补偿、再进行传感器标定、再计算地球相关参数，然后先速度更新、再位置更新、最后姿态更新。

先进行不可交换误差补偿，或者先传感器标定都可以，都是小量。PSINS 中没有直接对角增量标定，而是在不可交换误差补偿之后，直接对等效旋转矢量标定；因为如果是四子样，对角增量标定要四次，旋转矢量只要一次。

标定主要就是，扣除零偏，除以比例，乘以不正交安装误差。



速度更新用的是上一时刻姿态，不需要当前姿态，所以放在前面。



## 二、不可交换（圆锥/划桨）误差补偿

### 1、多子样不可交换误差补偿 cnscl

有四种方法：

* **优化不可交换误差补偿**：适用圆锥运动条件，每个相同间隔的角增量得到乘积是相等的。

* **一般方法不可交换误差补偿**：适用多项式条件，调用 `conepolyn` 函数进行补偿。
* **扩展形式不可交换补偿**：调用 `coneuncomp` ，
* **高阶误差补偿**：调用 `conehighorder` ，其它方法都只用两两叉乘加系数的补偿，此方法有三阶、四阶的叉乘。



传入参数：

* `imu`：惯导数据，七列，前三列是陀螺角增量、后三列是加速度计的比力增量，最后一列是时间

* `coneoptimal`：优化模式，

  * `0`：**优化不可交换误差补偿**：基于圆锥运动假设，直接算，用 `glv.cs` 中的补偿系数与角增量相乘，得到补偿量。
  * `1`：**一般方法不可交换误差补偿**，基于多项式条件，调用 `conepolyn` 函数，用补偿系数与角增量相乘，得到补偿量。
  * `2`：**扩展形式不可交换补偿**：调用 `coneuncomp`，推导比较麻烦，一二子样系数与多项式条件差不多。
  * `3`：**高阶误差补偿**：调用 `conehighorder` ，系数保存在 `highordercoef.mat` 中，有二三四阶补偿。

  

  





#### 1. glv.cs ：圆锥/划桨补偿系数（二到六子样）

$$
\begin{array}{c|c|c|c|c|c|c}
\hline N & k_1 & k_2 & k_3 & k_4 & k_5 & \rho_N \\
\hline 1 & - & & & & & 1 / 12 \\
\hline 2 & 2 / 3 & & & & & 1 / 960 \\
\hline 3 & 27 / 20 & 9 / 20 & & & & 1 / 204120 \\
\hline 4 & 214 / 105 & 92 / 105 & 54 / 105 & & & 1 / 82575360 \\
\hline 5 & 1375 / 504 & 650 / 504 & 525 / 504 & 250 / 504 & & 1 / 54140625000 \\
\hline 6 & 15797 / 4620 & 7834 / 4620 & 7296 / 4620 & 4558 / 4620 & 2315 / 4620 & 1 / 52295018840064 \\
\hline
\end{array}
$$

```matlab
glv.cs = [    % coning & sculling compensation coefficients 圆锥/划桨补偿系数
    [2,    0,    0,    0,    0    ]/3
    [9,    27,   0,    0,    0    ]/20
    [54,   92,   214,  0,    0    ]/105
    [250,  525,  650,  1375, 0    ]/504 
    [2315, 4558, 7296, 7834, 15797]/4620 ];
```




### 2、多项式角运动条件下的精确数值解法

`cnscl` 里的算法都是使用固定的系数，解算的模式固定，

输入多子样角增量采样，先采用 `wm2wtcoef` 函数求取角速度多项式系数，再采用如下方法之一求解精确数值解：

* `qpicard`：四元数毕卡迭代法
* `btzpicard`/`rodpicard`：等效旋转矢量/罗德里格参数迭代法
* `qtaylor`/`dcmtaylor`：四元数/方向余弦阵泰勒级数法

> 暂未实现基于切比雪夫多项式的罗德里格参数迭代算法







### 3、仿真举例

#### 1.  conesimu()：纯圆锥运动仿真

*  afa：半锥角
* f：频率
* ts：采样周期
* T：仿真时长



#### 2. highmansimu()：多项式角速度大机动仿真

* procef：多项式系数
* ts：采样周期
* T：仿真时长
* iffig：是否画图



`demos\test_attitude_update_methods_compare.m` 给出了各种误差补偿算法之间精度比较的例子：











