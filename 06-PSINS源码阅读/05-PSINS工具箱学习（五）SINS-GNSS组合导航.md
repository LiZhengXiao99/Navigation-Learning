> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]



imuerrset()：设置 IMU 各种误差参数

* eb：陀螺仪随机常值漂移
* db：加速度计随机常值漂移
* web：陀螺仪角随机游走
* wdb：加速度计速度随机游走
* sqrtROG、TauG：陀螺仪马尔科夫过程方差、相关时间
* sqrtROA、TauA：加速度计马尔科夫过程方差、相关时间
* dKGii：陀螺仪对角线标度系数误差
* dKAii：加速度计对角线标度系数误差
* dKGij：陀螺仪不正交安装误差角
* dKAij：加速度计不正交安装误差角
* KA2：加速度计二次非线性误差（正负不对称性误差）
* rxyz：加速度计杆臂误差（陀螺仪没有杆臂误差）
* dtGA：陀螺仪、加速度计之间的时间不同步



imuadder()：设置误差参数后，向理想的 IMU 数据注入误差







psinstypedef(nnm)：KF 状态维数 nn/ 量测维数m定义（一般可直接用于SINS/GNSS组合），或用户自定义字符串，在 kfinit/kffk/kfhk/kfplot 等函数中作为类型区分标识

kfinit：滤波器主要参数初始化，再用 kfinit0 更多的参数初始化

kfsetting：直接设置几种特定类型 IMU 的滤波器参数

kffk/kfhk：计算状态转移矩阵/量测矩阵，etm 惯导误差传播函数

kfupdate：KF 时间/量测更新，采用序贯量测自适应/方差限制方法

kffeedback：滤波状态反馈，可用部分反馈方法

kfstat：滤波误差分配与可观测度分析

kfplot/xpplot/rvpplot：滤波结果绘图

sinsgps：典型的193或196维 SINS/GNSS 松组合函数

POSProcessing：343或346维 SINS/GNSS 双向滤波松组合函数

POSFusion：对双向滤波结果做加权平均，

posplot：作图







sinsgps：





















