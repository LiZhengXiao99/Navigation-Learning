

# 推荐国内的三款开源 GNSS 数据和产品下载软件：GAMP-GOOD、FAST、GDDS

数据下载是困扰 GNSS 初学者入门的一大麻烦事，在此推荐国内的三款开源软件（GAMP-GOOD、FAST、GDDS），希望能帮助大家解决 GNSS 数据下载的烦恼。

三款软件都非常优秀，支持界面操作，支持下载 PPP 的所需的各种产品，支持下载 IGS、MGEX 的全球测站数据和 CUT、HK、NGS 等区域测站的数据，还可以在下载数据之后帮咱解压。

三款软件各有千秋，GAMP-GOOD 精致，FAST 支持下载的数据最多，GDDS 功能最强大，可以都尝试尝试，然后选一个作为数据下载的主力软件，另外两个作为补充，发挥出这些工具最大的效用。

GAMP-GOOD 是三款软件中开源最早的，一直在不断地优化与迭代，周锋老师做过三次直播分享，都有录播（一次在 Geososo 公众号，两次在 B 站）。之前版本只有命令行程序，用户需要通过修改配置文件来选择要下载的数据，较为繁琐；前段时间，周锋老师找我给它做了套界面，最新的 GAMP-GOOD4.0 版本由核心代码库 Libgood、命令行可执行程序 Good_Cui 和 Qt 界面可执行程序 Good_Gui 三部分组成。

界面版程序的易用性相比命令行有了极大的提升，上手难度更低，选择数据更容易，还有更多可选择的文件结构；支持预览下载完成的数据，检查数据是否完整、观测数据有哪些频点；支持根据观测文件，下载对应的导航电文和 PPP 文件；日志输出中的文件路径、链接，可以直接选中跳转。

但是受限于时间和能力，还有一些小 BUG 没有解决，暂时不支持中文路径，不支持下载快速和超快速的精密星历、精密钟差、电离层产品，文件下载失败时日志输出不完整。

FAST 支持的数据种类最多，程序简单，易于拓展，武大近期开源的 GREAT-PVT 里面数据下载就是基于 FAST 实现的。它的命令行程序引导着用户一步步选择要下载的数据，上手容易；最新的程序支持数据质量分析和 SPP，支持批处理，而且绘图美观。

GDDS 开源较晚，目前知名度不如前两款，但它的功能是极其强大的，下载完成试用了之后，我感到惊为天人，实现了很多我在给 GAMP-GOOD 做界面的时候，想做但是因为编程技术和时间的限制而没有实现的功能。最令我感到惊艳的功能是，用户还可以自定义链接格式，拼接出 ftp、http、https 链接，来下载给定的产品类型外的数据。











|                 |                          GAMP-GOOD                           |                             FAST                             |                             GDDS                             |
| :-------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
|    **作者**     |                周锋（山科）、李郑骁（安理工）                |                   常春涛（武大 GNSS 中心）                   |                      卢立果（东华理工）                      |
|  **最新版本**   |                  4.0（Github 只更新到 3.1）                  |                 3.0（Github 只更新到 2.11）                  |                             1.2                              |
|   **Github**    |         https://github.com/zhouforme0318/GAMPII-GOOD         |             https://github.com/ChangChuntao/FAST             |                https://github.com/LECUT/GDDS                 |
| **GPS ToolBox** |                             在投                             |                           准备投稿                           |       https://geodesy.noaa.gov/gps-toolbox/gdds.shtml        |
|  **编程语言**   |                           C++、Qt5                           |                        Python、PyQt5                         |                        Python、PyQt5                         |
|  **软件形式**   |                        命令行 + 界面                         |                        命令行 + 界面                         |                          仅界面程序                          |
|  **数据中心**   |                       CDDIS、IGN、WHU                        |                             WHU                              |               WHU、IGN、ESA、KASI、SIO、CDDIS                |
|  **数据类型**   | OBS、NAV、ORB、CLK、OBX、DSB、OSB、SNX、EOP、ATX、ROTI、IONO、TROP | OBS、NAV、ORB、CLK、OBX、DSB、OSB、SNX、EOP、ATX、IONO、VMF、TROP、Meteorology、CNES_AR、SLR、LEO、Time_series、Velocity_Fields、Panda、GAMIT（支持数据种类最多） | OBS、NAV、ORB、CLK、OBX、DSB、OSB、SNX、EOP、ATX、IONO、TROP、Time_series（除此以外，还可以添加自定义链接格式） |
|  **区域CORS**   |   HK、CUT、NGS、PBO、CHI、GA（GA 数据通过 Linux 脚本下载）   |            NGS、HK、GA（无法下载近三年 GA 数据）             |            NGS、EPN、SPAIN、JPN、HK、CUT、ARPREF             |
|  **测站选择**   |                ①输入测站名；②上传测站列表文件                | ①输入测站名；②上传测站列表文件；③地图筛选指定卫星系统、天线名、经纬度范围内的数据； |                    在给定的测站列表里选择                    |
|  **测站地图**   |                 只能点击按钮跳转测站地图链接                 |                 可以在地图上绘制已选择的测站                 |              调用百度地图 API 显示已选择的测站               |
|  **日期选择**   |                  ①起止日期、②起始日期+天数                   |                           起止日期                           |                           起止日期                           |
|  **下载路径**   | 不支持中文路径（考完研我来改进）、目标路径不存在可以自动创建 |                支持中文路径、目标路径必须存在                |           支持中文路径、目标路径不存在可以自动创建           |
|  **信息输出**   |                     日志窗口 + 日志文件                      |                           日志窗口                           |                      日志文件 + 进度条                       |
|  **使用教程**   | README + Manual + PPT  + 三场直播/录播讲解 + 控件提示 + 热心网友的博客 |                   README + 热心网友的博客                    |                        Manual + 论文                         |
|  **其它亮点**   | ①可以同时下载多种数据，支持多层次的文件结构；②支持下载前后天的精密星历钟差；③日志输出层次清晰，不同级别用不同的颜色；命令行程序支持输出 wget 信息；④支持根据观测文件，下载对应的导航电文和 PPP 文件；⑤支持在程序中预览下载完成的数据，可以看看数据全不全，观测数据有哪些频点；⑥日志输出中的文件路径、链接，可以直接跳转； | ①支持并行下载数据，但是每次只能选择一种类型的数据，并行下载的功能大大受限；②最新的程序支持数据质量分析和 SPP，支持批处理，而且绘图美观；③支持的数据种类最多，而且各种数据类型通过下拉框来展示，拓展较为容易；④命令行程上手容易，引导着用户一步步选择要下载的数据； | ①支持并行下载数据，比 FAST 更好的一点在于 GDDS 可以同时选择星历和观测、或者同时选择各种产品下载；②同种产品可选择的细分数据类型全面；③除了给定的产品类型外，用户还可以自定义链接格式，拼接出 ftp、http、https 链接；④源码注释完善，每个文件、每个代码段前面都有简要注释，拼接产品链接的那行代码前，还会在注释里给出一个链接的示例； |
|  **其它缺陷**   | ①部分产品下载失败的时候，无法正常输出日志；②暂时无法下载快速、超快速的 ORB、CLK、IONO；③同种产品可选择的细分数据类型少，（比如某些产品 FIN 和 RAP 只支持其一，比如默认下载周解 SNX，不存在时才能下载天解）；④界面程序的部分功能不直观，得看介绍才能明白要怎么用；⑤C++ 程序修改和拓展起来比 Python 更困难； | ①日期只能年月日分开输，不能通过日历选择；想输入年积日和 GPS 周，必须先通过左侧工具转为年月日； | ①下载完成之后不能自动解压；②点击下载界面就卡住无响应了，可能是因为没有开新的线程来执行下载操作，导致下载的线程阻塞了界面响应；③异常处理做的不充分，程序易崩溃； |





## GAMP-GOOD

GOOD 是三款软件中开源最早的，历经 15 个版本的迭代，一直在不断地优化迭代，最新的 GAMP-GOOD4.0 版本由核心代码库 Libgood、命令行可执行程序 Good_Cui 和 Qt 界面可执行程序 Good_Gui 三部分组成，实现了常用 GNSS 观测值、广播星历、精密卫星轨道和钟差、地球定向参数、卫星姿态ORBEX、DCB/DSB/OSB、SINEX周解、电离层和对流层延迟产品、天线相位中心等产品下载。



|                 数据类型                 |                   用处                    |
| :--------------------------------------: | :---------------------------------------: |
|    IGS观测数据 (RINEX 2.xx, 短文件名)    |    多用于GPS定轨、估钟、PPP算法验证等     |
|   MGEX观测数据 (RINEX 3.xx, 长文件名)    | 多用于多系统GNSS定轨、估钟、PPP算法验证等 |
|      科廷科技大学短基线 CORS 观测值      |          多用于短基线RTK算法验证          |
|      澳大利亚地球科学局 (GA) 观测值      |   多用于BDS-2/3 UPD估计、PPP算法验证等    |
|   香港CORS观测值(RINEX 3.xx, 长文件名)   |  多用于海潮、GNSS水汽、电离层延迟建模等   |
|        广播星历 (yyg/yyn/yyp 等)         | 多用于SPP、RTK解算中的卫星坐标与钟差计算  |
| IGS 与 MGEX 各分析中心精密轨道和钟差产品 |    多用于PPP解算中的卫星轨道和钟差计算    |
|               地球自转参数               |            多用于计算极潮改正             |
|            IGS SINEX周解/天解            |       多用作定位算法验证的参考坐标        |
|           CODE或MGEX差分码偏差           |          多用于改正伪距硬件延迟           |
|            全球电离层图 (GIM)            |            全球电离层延迟产品             |
|          全球电离层闪烁 (ROTI)           |      多用于电离层不规则体和闪烁研究       |
|  CNES离线实时轨道/钟差/相位小数偏差产品  | 用于实时PPP算法验证、实时PPP模糊度固定等  |
|       IGS天线相位中心改正 (ANTEX)        |     用于卫星和接收机天线相位中心改正      |







## 附录：常用 GNSS 数据下载链接

![PPP 数据模型](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/PPP%2520%25E6%2595%25B0%25E6%258D%25AE%25E6%25A8%25A1%25E5%259E%258B.png)

### 1、数据格式官方文档

```
- [Rinex2.11]    : https://files.igs.org/pub/data/format/rinex211.txt
- [Rinex3.05]    : https://files.igs.org/pub/data/format/rinex305.pdf
- [Rinex4.01]    : https://files.igs.org/pub/data/format/rinex_4.01.pdf
- [DCB/BSX]      : https://files.igs.org/pub/data/format/sinex_bias_100.pdf
- [SP3]          : https://files.igs.org/pub/data/format/sp3d.pdf
- [CLK]          : https://files.igs.org/pub/data/format/rinex_clock304.txt
- [ERP]          : https://files.igs.org/pub/data/format/erp.txt
- [SINEX]        : https://www.iers.org/SharedDocs/Publikationen/EN/IERS/Documents/ac/sinex/sinex_v202_pdf
- [Trop]         : https://files.igs.org/pub/data/format/sinex_tro_v2.00.pdf
- [GIM/Roti]     : https://files.igs.org/pub/data/format/ionex1.pdf
- [ANTEX]        : https://files.igs.org/pub/data/format/antex14.txt
```

### 2、 IGS 数据中心

```
- [WHU (China)] 	: ftp://igs.gnsswhu.cn
- [IGN (France)] 	: ftp://igs.ign.fr
- [ESA (Europe)] 	: ftp://gssc.esa.int
- [KASI (Korea)] 	: ftp://nfs.kasi.re.kr
- [SIO (USA)] 		: ftp://lox.ucsd.edu
- [CDDIS (USA)] 	: ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss
```

### 3、 区域 CORS

```
- [USA CORS] 			: https://geodesy.noaa.gov
- [Europe EPN] 			: ftp://ftp.epncb.oma.be
- [Spain CORS] 			: http://ftp.itacyl.es
- [Japan JPN] 			: ftp://mgmds01.tksc.jaxa.jp
- [Hong Kong CORS] 		: ftp://ftp.geodetic.gov.hk
- [Curtin University] 	: http://gnss.curtin.edu.au
- [Australia APREF] 	: ftp://ftp.data.gnss.ga.gov.au
- [pbo] 				: https://data.unavco.org/archive/gnss/highrate
- [chi] 				: https://gps.csn.uchile.cl/data/
```

### 4、 CDDIS 下载链接

```
- [IGS daily observation (30s) files]     : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/daily
- [IGS hourly observation (30s) files]    : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/hourly
- [IGS high-rate observation (1s) files]  : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/highrate        
- [MGEX daily observation (30s) files]    : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/daily
- [MGEX hourly observation (30s) files]   : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/hourly
- [MGEX high-rate observation (1s) files] : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/highrate
- [broadcast ephemeris files]             : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/daily
- [IGS SP3 files]                         : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products
- [IGS CLK files]                         : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products
- [IGS EOP files]                         : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products
- [IGS weekly SINEX files]                : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products
- [MGEX SP3 files]                        : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex
- [MGEX CLK files]                        : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex
- [MGEX ORBEX files]                      : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex
- [MGEX DSB files]                        : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/bias
- [MGEX OSB files]                        : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex
- [global ionosphere map (GIM) files]     : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/ionex
- [Rate of TEC index (ROTI) files]        : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/ionex
- [IGS final tropospheric product files]  : ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/troposphere/zpd
```

### 5、 IGN 下载链接

```
- [IGS daily observation (30s) files]     : ftp://igs.ign.fr/pub/igs/data
- [IGS hourly observation (30s) files]    : ftp://igs.ign.fr/pub/igs/data/hourly
- [IGS high-rate observation (1s) files]  : ftp://igs.ign.fr/pub/igs/data/highrate
- [MGEX daily observation (30s) files]    : ftp://igs.ign.fr/pub/igs/data
- [MGEX hourly observation (30s) files]   : ftp://igs.ign.fr/pub/igs/data/hourly
- [MGEX high-rate observation (1s) files] : ftp://igs.ign.fr/pub/igs/data/highrate
- [broadcast ephemeris files]             : ftp://igs.ign.fr/pub/igs/data
- [IGS SP3 files]                         : ftp://igs.ign.fr/pub/igs/products
- [IGS CLK files]                         : ftp://igs.ign.fr/pub/igs/products
- [IGS EOP files]                         : ftp://igs.ign.fr/pub/igs/products
- [IGS weekly SINEX files]                : ftp://igs.ign.fr/pub/igs/products
- [MGEX SP3 files]                        : ftp://igs.ign.fr/pub/igs/products/mgex
- [MGEX CLK files]                        : ftp://igs.ign.fr/pub/igs/products/mgex
- [MGEX ORBEX files]                      : ftp://igs.ign.fr/pub/igs/products/mgex
- [MGEX DSB files]                        : ftp://igs.ign.fr/pub/igs/products/mgex/dcb
- [MGEX OSB files]                        : ftp://igs.ign.fr/pub/igs/products/mgex     
- [global ionosphere map (GIM) files]     : ftp://igs.ign.fr/pub/igs/products/ionosphere
- [Rate of TEC index (ROTI) files]        : ftp://igs.ign.fr/pub/igs/products/ionosphere
- [IGS final tropospheric product files]  : ftp://igs.ign.fr/pub/igs/products/troposphere
```

### 6、 WHU 下载链接

```
- [IGS daily observation (30s) files]     : ftp://igs.gnsswhu.cn/pub/gps/data/daily
- [IGS hourly observation (30s) files]    : ftp://igs.gnsswhu.cn/pub/gps/data/hourly
- [IGS high-rate observation (1s) files]  : ftp://igs.gnsswhu.cn/pub/highrate
- [MGEX daily observation (30s) files]    : ftp://igs.gnsswhu.cn/pub/gps/data/daily
- [MGEX hourly observation (30s) files]   : ftp://igs.gnsswhu.cn/pub/gps/data/hourly
- [MGEX high-rate observation (1s) files] : ftp://igs.gnsswhu.cn/pub/highrate               
- [broadcast ephemeris files]             : ftp://igs.gnsswhu.cn/pub/gps/data/daily        
- [IGS SP3 files]                         : ftp://igs.gnsswhu.cn/pub/gps/products
- [IGS CLK files]                         : ftp://igs.gnsswhu.cn/pub/gps/products
- [IGS EOP files]                         : ftp://igs.gnsswhu.cn/pub/gps/products   
- [IGS weekly SINEX files]                : ftp://igs.gnsswhu.cn/pub/gps/products
- [MGEX SP3 files]                        : ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- [MGEX CLK files]                        : ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- [MGEX ORBEX files]                      : ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- [MGEX DSB files]                        : ftp://igs.gnsswhu.cn/pub/gps/products/mgex/dcb     
- [MGEX OSB files]                        : ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- [global ionosphere map (GIM) files]     : ftp://igs.gnsswhu.cn/pub/gps/products/ionex
- [Rate of TEC index (ROTI) files]        : ftp://igs.gnsswhu.cn/pub/gps/products/ionex
- [IGS final tropospheric product files]  : ftp://igs.gnsswhu.cn/pub/gps/products/troposphere/new
```

### 7、 各种产品下载链接

```
- 【Obs】
  - 【igs/mgex/igm】
    - <Daily (30 s)>
      - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/daily
      - ftp://igs.ign.fr/pub/igs/data
      - ftp://igs.gnsswhu.cn/pub/gps/data/daily
    - <Hourly (30 s)>
      - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/hourly
      - ftp://igs.ign.fr/pub/igs/data/hourly
      - ftp://igs.gnsswhu.cn/pub/gps/data/hourly
    - <High-rate (1 s)>
      - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/highrate
      - ftp://igs.ign.fr/pub/igs/data/highrate
      - ftp://igs.gnsswhu.cn/pub/highrate
  - 【cut】<Daily)>: http://saegnss2.curtin.edu/ldc/rinex3/daily
  - 【hk】 <30s / 5s/ 1s>: https://rinex.geodetic.gov.hk
  - 【ngs】<Daily)>: https://noaa-cors-pds.s3.amazonaws.com/rinex
  - 【epn】<Daily>: ftp://ftp.epncb.oma.be/pub/obs
  - 【pbo】<Daily>: https://data.unavco.org/archive/gnss/highrate
  - 【chi】<Daily>: https://gps.csn.uchile.cl/data/
- 【Nav】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/data/daily 
  - ftp://igs.ign.fr/pub/igs/data 
  - ftp://igs.gnsswhu.cn/pub/gps/data/daily 
  - ftp://ftp.pecny.cz/LDC/orbits_brd/gop3 
  - https://igs.bkg.bund.de/root_ftp/IGS/BRDC
- 【Orb/Clk】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products  
  - ftp://igs.ign.fr/pub/igs/products  
  - ftp://igs.gnsswhu.cn/pub/gps/products  
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex 
  - ftp://igs.ign.fr/pub/igs/products/mgex 
  - ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- 【Obx】
- 【Dsb / Osb】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products  
  - ftp://igs.ign.fr/pub/igs/products  
  - ftp://igs.gnsswhu.cn/pub/gps/products  
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex 
  - ftp://igs.ign.fr/pub/igs/products/mgex 
  - ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- 【Snx】
- 【Iono】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/ionex
  - ftp://igs.ign.fr/pub/igs/products/ionosphere
  - ftp://igs.gnsswhu.cn/pub/gps/products/ionex
- 【Trop】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/tropo sphere/zpd
  - ftp://igs.ign.fr/pub/igs/products/troposphere
  - ftp://igs.gnsswhu.cn/pub/gps/products/troposphere/new 
  - ftp://ftp.aiub.unibe.ch/CODE
- 【Eop】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products 
  - ftp://igs.ign.fr/pub/igs/products  
  - ftp://igs.gnsswhu.cn/pub/gps/products 
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/mgex
  - ftp://igs.ign.fr/pub/igs/products/mgex
  - ftp://igs.gnsswhu.cn/pub/gps/products/mgex
- 【Atx】
  - https://files.igs.org/pub/station/general/
- 【Roti】
  - ftps://gdc.cddis.eosdis.nasa.gov/pub/gnss/products/ionex
  - ftp://igs.ign.fr/pub/igs/products/ionosphere
  - ftp://igs.gnsswhu.cn/pub/gps/products/ionex
```

