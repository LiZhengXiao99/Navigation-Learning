## 一、简介

POSGO，全称 POSition based on Graph Optimization，是由武汉大学 GNSS 定轨中心李政开源的一套图优化 GNSS 伪距定位程序。该程序目前主要侧重于以伪距为核心的 SPP（单点定位）和 RTD（实时伪距差分定位）解算，并兼容 EKF（扩展卡尔曼滤波器）和 GO（图优化）解算方法。整个项目涵盖了约 8000行的 C++ 代码和 700 行的 Python 脚本，C++ 部分的代码在 RTKLIB 的基础上，进行了面向对象的改编和优化；Python 部分则包含了 Analyze 和 AnalyzeStatic 两个结果分析脚本。POSGO 使用了来自城市道路的动态车辆手机实验数据进行测试。实验结果显示，在遮挡严重的区域，GO解算方法相较于传统的 LS（最小二乘法）和 EKF 解算方法，展现出了更高的定位精度和更强的稳健性。POSGO 未来计划引入对载波相位的支持，以进一步提升定位的精准度，并计划加入对多传感器融合的支持。 

![image-20240701205722084](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240701205722084.png)





|      文件       | 说明 | File | Code |
| :-------------: | :--: | :--: | :--: |
|   **common**    |      |  98  | 3916 |
|    **debug**    |      |  8   | 215  |
|  **ephmeris**   |      |  17  | 434  |
|    **gnss**     |      |  71  | 2271 |
| **gnss_ins_lc** |      |  4   | 201  |
| **gnss_ins_tc** |      |  41  | 1895 |
|     **gui**     |      |  15  | 1047 |
|     **ins**     |      |  12  | 307  |
|  **main_func**  |      |  6   | 266  |
|    **plot**     |      |  9   | 513  |
|  **read_file**  |      |  38  | 2144 |
|    **tides**    |      |  6   | 134  |





算法特点：

* 在进行正式解算之前，会先进行多普勒定速计算；
* 支持 IGG3 和 HUber 抗差核函数；
* 每个历元解算结束，进行卡方、PDOP、抗差阈值检验；
* 







![a7209a9b6cc409103ee3365c63230e8d](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/a7209a9b6cc409103ee3365c63230e8d.png)









