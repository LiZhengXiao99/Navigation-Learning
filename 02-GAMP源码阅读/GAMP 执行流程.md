

## 零、前言

之前写过一个 RTKLIB 的专栏：

* [RTKLIB学习总结（一）VS配置RTKLIB、manual、矩阵、最小二乘和Kalman滤波基本函数](https://lizhengxiao.blog.csdn.net/article/details/129339677)
* [RTKLIB学习总结（二）时间系统、坐标系统](https://lizhengxiao.blog.csdn.net/article/details/129472515)
* [RTKLIB学习总结（三）RTKGET、RTKCONV、RTKPLOT、RTKPOST、STRSVR的使用](https://lizhengxiao.blog.csdn.net/article/details/129569948)
* [RTKLIB学习总结（四）rnx2rtkp.c、Option文件读取、Trace](https://lizhengxiao.blog.csdn.net/article/details/129617353)
* [RTKLIB学习总结（五）后处理函数调用流程、postpos、execses_b、execses_r、execses、procpos、rtkpos](https://lizhengxiao.blog.csdn.net/article/details/129887413)
* [RTKLIB学习总结（六）导航电文、卫星位置计算](https://lizhengxiao.blog.csdn.net/article/details/129986385)
* [RTKLIB学习总结（七）GNSS观测量、Rinex文件读取](https://lizhengxiao.blog.csdn.net/article/details/130083589)
* [RTKLIB学习总结（八）伪距单点定位SPP](https://lizhengxiao.blog.csdn.net/article/details/130359930)
* [RTKLIB学习总结（九）RTK算法学习](https://lizhengxiao.blog.csdn.net/article/details/130674816)



## 一、VS2022调试GMAP







## 二、数据组织结构







## 二、PPP 函数调用流程

![1688082358362](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1688082358362.png)

### 1、文件读取、预处理

> 参考：[RTKLIB学习总结（四）rnx2rtkp.c、Option文件读取、Trace](https://lizhengxiao.blog.csdn.net/article/details/129617353)、[RTKLIB学习总结（五）后处理函数调用流程、postpos、execses_b、execses_r、execses、procpos、rtkpos](https://lizhengxiao.blog.csdn.net/article/details/129887413)、[RTKLIB学习总结（七）GNSS观测量、Rinex文件读取](https://lizhengxiao.blog.csdn.net/article/details/130083589)

#### main()

程序从 `main.c` 的 `main()` 函数开始执行，整个程序都在 `t1=clock()` 和 `t2=clock()` 中执行，求得`t2-t1`为程序执行时间。`main()` 函数接收传入的命令行参数即 `gamp.cfg` 的文件路径，如果传入了参数，**调用 `proccfgfile()` 进行下一步处理**。

> * VS 中：在 项目属性-调试-命令行参数 中指定命令行参数。
> * Windows 的文件路径中一般用 `\`，且为了避免转移需要写成 `\\`。linux一般用 `/`。

#### proccfgfile()

`proccfgfile()` 函数先将  `PPP_Glo` 结构体初始化，将处理方式，输入输出文件路径赋空值。打开传入的 `gamp.cfg` 文件。获取观测值文件路径和处理方式，根据观测文件的数量调用对应的函数，单个观测文件**调用 `procOneFile()` 进行下一步处理**；如果有多个文件调用 `batchProc()` 进行批量处理，`batchProc()` 会打开文件夹，循环查找文件夹中的观测值O文件，**调用 `procOneFile()` 进行下一步处理**。

> 观测值`O`文件的后缀有两种，一种是直接 `.O` 结尾，一种是 `.ddO` 结尾。

#### procOneFile()

先调用 `preProc()` 预处理：通过调用 `initGlobal()` 初始化 `PPP_Glo` 结构体；调用 `getObsInfo()`  读取观测O文件的一部分，获取起止时间、文件版本、天线种类等基础信息；为 `filopt.inf`、`filopt.outf` 开辟内存空间。

调用 `readcfgFile()` 读取整个配置文件，通过 `strstr(line,"start_time")` 匹配处理选项，存储到 `prcOpt_Ex()`、`prcopt()`。

调用 `getFopt_auto()` ，通过调用 `findClkFile()`、`findNavFile()`，根据后缀名自动查找各种 PPP 解算所需的文件，将文件路径存到 `fopt->inf` 中。

**调用 `gampPos()` 进行下一步处理**；处理结束，调用 `postProc()` 释放 `filopt.inf`、`filopt.outf` 内存空间。

#### gampPos()

先调用 `outhead()` 写输出文件的文件头。调用 `setcodepri()` 设置观测值优先级。调用 `readdcb()`、`readobsnav()`、`readpreceph()` 等函数读取文件。

文件读取完之后，**调用 `execses()` 进行下一步处理**。处理完之后调用 `freeobsnav()`、`freepreceph()` 释放内存空间。

> 详细的文件读取方式可以看我之前的博客：[RTKLIB学习总结（七）GNSS观测量、Rinex文件读取](https://lizhengxiao.blog.csdn.net/article/details/130083589)

#### excses()

先调用 `sampledetermine()` 获取观测值采用间隔（解算频率），调用 `calCsThres()` 获取周跳检测的阈值，调用 `rtkinit()` 初始化 `rtk` 结构体。**调用 `procpos()` 进行下一步处理**。处理完之后调用 `rtkfree()` 释放 `rtk` 结构体。

#### procpos()

循环调用 `inputobs()` 传入一个历元的观测值。调用 `obsScan_SPP()` 观测值检测、调用 `BDmultipathCorr()` 修正北斗伪距。**调用 `rtkpos()` 进行下一步处理**。处理完之后调用 `outResult()`、`outsol()` 输出结果。

#### rtkpos()

至此已经读完了文件，开始进行逐历元解算，先**调用 `spp()` 进行 SPP 解算**，调用 `obsScan_PPP()` 观测值检测，调用 `clkRepair()` 修复钟跳，**调用 `pppos()` 进行 PPP 解算**，调用 `calDop()` 计算各种 DOP 值，调用 `keepEpInfo()` 存储当前历元的信息，其中会调用 `gfmeas()`、`wlAmbMeas()`。

### 2、卫星位置钟差计算

> 与 RTKLIB 中基本一致，我之前的博客有详细解析：[RTKLIB学习总结（六）导航电文、卫星位置计算](https://lizhengxiao.blog.csdn.net/article/details/129986385)

#### satposs_rtklib()

遍历观测数据，找伪距观测值，除以光速得到信号传播时间，用数据接收时刻减去伪距信号传播时间得到信号发射时刻。

调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，不作为最终计算的钟差。信号发射时刻减去钟差 `dt`，得到 GPS 时间下的卫星信号发射时刻。

**调用 `satpos()` 对此观测值进行下一步卫星位置钟差的计算**；`satpos()` 函数对星历计算选项进行判断，**广播星历模式调用 `ephpos()`**，**精密星历模式调用 `peph2pos()`**。最后检测钟差值，如果没有精密星历，则调用 `ephclk()` 用广播星历计算钟差。

#### ephclk()

单观测值卫星钟差计算。由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2clk()` 根据广播星历参数 $a_0$、$a_1$、$a_2$ 计算卫星钟差（迭代 3 次）；

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2clk()` 根据广播星历参数 $t_aun$、$g_aun$  计算卫星钟差（迭代 3 次）。

#### ephpos()

与 `ephclk()` 同理，由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2pos()` 根据广播星历中的开普勒轨道参数和摄动改正计算卫星位置（对北斗 MEO、IGSO 卫星会进行特殊处理）、校正卫星钟差的相对论效应、调用 `var_uraeph()` 用 URA 值来标定方差。

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2pos()` 根据广播星历中 PZ-90 坐标系下卫星状态向量四阶龙格库塔迭代计算卫星位置。

计算完一次位置之后，加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟漂。

#### peph2pos()

调用 `pephpos()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密星历，然后地球自转改正，调用 `interppol()` 内维尔插值获取卫星位置、线性插值获取钟差，最后计算标准差。

调用 `pephclk()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密钟差，再线性插值获取钟差、计算标准差。

计算相对论效应改正量，调用 `satantoff()` 计算卫星天线相位偏差改正。加上改正量得到卫星位置钟差。

加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度中飘。

### 3、SPP 解算

#### spp()

默认使用广播星历计算卫星位置、钟差，使用克罗布歇模型通过广播星历中的参数计算电离层延迟，使用 Saastamoinen 模型计算对流层延迟。**调用 `satposs_rtklib()` 计算卫星位置、卫星钟差**，**调用 `estpos()` 计算接收机位置。**











### 4、PPP 解算

























