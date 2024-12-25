[TOC]

## 一、GINav 简介

GINav 是 2020 年发布在 GPS Solution 上开源 GNSS/INS 紧组合工具箱，支持多模（GPS/GLONASS/GALILEO/BDS/QZSS）多频（单频到三频）GNSS 数据处理，支持多种处理模式 ，包括 SPP、PPD、PPK、PPS  GNSS/INS 组合导航算法开发与测试。软件采用纯 MATLAB 编写，src 目录下共 325 个 .m 文件，约 12000 行代码；GNSS 部分的代码是移植 RTKLIB 和 GAMP，IMU 递推算法移植自 PSINS，使用过 RTKLIB 的朋友应该很容易就能上手。

与 RTKLIB、Ginan、IGNav 等 C/C++ 编写的导航定位解算程序相比，MATLAB 编写的 GINav 可移植性更差，执行效率低，PPK 解算比 RTKLIB 慢 20 倍；GINav 的优点是简单，调试方便，在变量区可以直接看到矩阵的值，MATLAB 提供的数据处理和分析功能更丰富，方便进行算法的学习和研究。

与同为 MATLAB 实现的 PSINS 捷联惯导工具箱相比，GINav 更侧重 GNSS 和紧组合算法，实现的惯导和滤波算法远没有 PSINS 丰富。与同为 MATLAB 实现的 goGPS 相比，GINav 的源码和软件使用方式都更简单，采用纯面向过程风格，很多的结构体和函数都与 RTKLIB 和 GAMP 一致，更容易上手。



<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5a44608fabc3fb9fd64d728ef29a3ff2.png" alt="5a44608fabc3fb9fd64d728ef29a3ff2" style="zoom: 40%;" />

算法特点包括：

* 通过 TDCP 测速定姿，实现 GNSS/INS 初始对准对准；
* 对于 GNSS/INS 组合模式，支持惯导辅助的周跳探测和抗差估计；





|     文件夹      |                             说明                             | File | Code |
| :-------------: | :----------------------------------------------------------: | :--: | :--: |
|   **common**    | 类似 rtkcmn，存一些基础函数，包括：时间转换、坐标转换、卫星选择、对流层电离层模型、天线改正等 |  98  | 3916 |
|    **debug**    |                                                              |  8   | 215  |
|  **ephmeris**   |                                                              |  17  | 434  |
|    **gnss**     |                                                              |  71  | 2271 |
| **gnss_ins_lc** |                                                              |  4   | 201  |
| **gnss_ins_tc** |                                                              |  41  | 1895 |
|     **gui**     |                                                              |  15  | 1047 |
|     **ins**     |                                                              |  12  | 307  |
|  **main_func**  |                                                              |  6   | 266  |
|    **plot**     |                                                              |  9   | 513  |
|  **read_file**  |                                                              |  38  | 2144 |
|    **tides**    |                                                              |  6   | 134  |



## 二、GINav 使用

### 1、两个界面工具

* **GINavExe**：选择数据文件和配置文件，然后进行导航定位解算，支持模式如下：
  * **纯 GNSS**： SPP、PPD、PPK、PPS、PPP
  * **GNSS/INS 松组合**：PPD_LC、PPK_LC、PPP_LC、SPP_LC
  * **GNSS/INS 紧组合**：SPP_TC、PPD_TC、PPK_TC、PPP_TC
* **Plot_Analysis**：结果绘图分析，支持三种模式：
  * **Polt**：根据 GINavExe 生成的结果文件绘制：
  * **Error plot**：将 GINavExe 生成的结果与参考结果进行比较，绘制 PVA 误差曲线；
  * **PPP plot**：根据 GINavExe 生成的结果和 sinex 文件绘制 PPP 东北天方向的误差曲线；

使用 GINavExe 注意事项：

* 下载 Lambda-3.0，放到 3rd 目录下，用于模糊度固定（[网盘链接](https://pan.baidu.com/s/1Hr7J9NHivYSiwOFKkypPtQ?pwd=aust)）；
* 解压 data 目录下的数据文件，否则无法选择数据文件；
* 在 GINavExe 界面选择配置文件的时候总是报错，把 `GINavCfg.m` 165 行 的 `matchinfile` 注释了就好；
* cpt 数据支持的定位模式最全，但其默认配置大多都是单系统或者 GC、GR 双系统；
* 结果文件默认存储在 result 路径下，默认命名是测站名+解算模式，会覆盖旧的结果；
* GINav 的结果文件相比 RTKLIB 拓展了姿态几列，用 RTKPLOT 绘制速度更快，但是看不了姿态；
* 我写了一个脚本，比较各组定位结果精度，根据参考，计算 RMSE，绘制误差曲线，放在 [Navigation-Script](https://github.com/LiZhengXiao99/Navigation-Script) 仓库；



### 2、cpt 测试数据

GINav 程序提供了四组示例数据：

> 需要注意的是：程序下载下来之后，data 文件夹下各种数据都是压缩包形式存的，使用前记得先解压。

* **cpt**：
* **cu**：
* **mgex**：
* **tokyo**：



其中 cpt 数据，

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240512195507718.png" alt="image-20240512195507718" style="zoom: 50%;" />

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240512204240029.png" alt="image-20240512204240029" style="zoom:50%;" />

本文主要以 cpt 这组数据

![cpt_RTKLIB观测数据分析](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/cpt_RTKLIB%E8%A7%82%E6%B5%8B%E6%95%B0%E6%8D%AE%E5%88%86%E6%9E%90.png)



<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240512204339263.png" alt="image-20240512204339263" style="zoom: 50%;" />



### 3、RTK/RTK-INS 解算结果比较

比较了 RTKLIB、RTKLIB-demo5-b43i、PSINS 的松组合、GINav 的 PPK、RTK/INS-LC、RTK/INS-TC 解如下



各种配置选项尽量保持一致，模糊度固定采用 Fix and Hold 配置模式，



通过绘图比较各种解算的效果如下：



计算得出各种数据 ENU 方向的 RMSE 如下：



### 4、PPP/PPP-INS 解算结果比较





用我的轻薄本（i7-1165G7），对 cpt 各种模式解算时间统计：

|      解算模式       | SPP  | PPP  | PPD  | PPK  |
| :-----------------: | :--: | :--: | :--: | :--: |
|     **纯 GNSS**     |      |      |      |      |
| **GNSS/INS 松组合** |      |      |      |      |
| **GNSS/INS 紧组合** |      |      |      |      |



用 Plot_Analysis GUI 工具可以对结果进行绘图分析



结果文件和 RTKLIB ，拓展了姿态一列，可以直接用 RTKPLOT 进行分析，





## 三、GINavExe 程序结构



<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240515083706867.png" alt="image-20240515083706867" style="zoom:50%;" />



### 2、数据结构

#### 1. 全局变量 glc：存常量





#### 2. 全局变量 gls：存中间变量





#### 3. cfg：存配置选项



#### 4. nav：存 GNSS 星历数据



#### 5. obsb、obsr：存 GNSS 观测值数据



#### 6. imu：存 IMU 量测值





关键函数如下：

* **GINavExe**：
  * **global_variable**：
  * **GINavCfg**：
* **exepos**：
  * **read_infile**：
  * **initoutfile**：
  * **adjobs**、**adjnav**：
  * **baserefpos**：
  * **setpcv**：
* **gnss_processor**：
  * **initrtk**：
  * **searchobsr**、**searchobsb**：
  * **exclude_sat**：
* **gnss_solver**：
  * **scan_obs_spp**：
  * **scan_obs_ppp**：
  * **bds2mp_corr**：
  * **clkrepair**
* **sppos**：
  * **satposs**：
  * **estpos**：
  * **rescode**：
  * **least_square**：
  * **estvel**：
  * **resdop**：
  * **raim_FDE**
* **plot_trajectory_kine**：
* **gi_processor**：
  * **searchimu**
  * **matchobs**
  * **ins_mech**：
  * **earth_update**：
  * **ins_time_updata**：
  * **ins2sol**
  * **gi_initrtk**
* **ins_align**
* **gi_Loose**
* **gi_Tight**



## 四、RTK/INS-LC 执行流程





PPK：

|       待估状态量        | 维数 | 随机模型 |             其它             |
| :---------------------: | :--: | :------: | :--------------------------: |
|      位置参数 $r$       |  3   |  白噪声  |      单点定位解做先验值      |
| 单差模糊度 $N_{rb,j}^m$ |      | 随机游走 | 继承上一历元结果，周跳时重置 |



消电离层 PPP：

| 待估状态量StateVariables | 维数Dimension | 随机模型StochasticModel |               其它 Others                |
| :----------------------: | :-----------: | :---------------------: | :--------------------------------------: |
|       位置参数 $r$       |       3       |         白噪声          | 对于动态模式，建模白噪声，每个历元位置使 |
|    接收机钟差 $dT_r$     |               |                         |                                          |
|  系统间偏差 $dt_{ISB}$   |               |                         |                                          |
|  天顶对流层湿延迟 $Z_w$  |               |                         |                                          |
|  IF 组合模糊度 $N_{IF}$  |               |                         |                                          |





## 五、PPP/INS-TC 执行流程







## 六、在 GINav 基础上拓展











