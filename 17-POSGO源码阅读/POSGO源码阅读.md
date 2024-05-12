## 一、简介

POSGO，全称 ，是武大GNSS定轨中心李政开源一套图优化 GNSS 定位程序，目前支持 SPP 和 RTD 解算， 

程序

8000 行 C++ 和 700 行 Python





测试数据采集自华为 Mate20 和 Mate 40







包含 Analyze 和 AnalyzeStatic 两个结果分析脚本





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









