<div align="center">
    <a name="Top"></a>
	<h1>GAMP-GOOD 快速上手及源码解析</h1>
</div>


[TOC]

## 一、GAMP-GOOD 快速上手

GAMP-GOOD 由核心代码库 Libgood、命令行可执行程序 Good_Cui 和 Qt界面可执行程序 Good_Gui 三部分组成，自2021年4月1.0版本至今已历经 15 个版本迭代，目前已实现常用 GNSS 观测值、广播星历、精密卫星轨道和钟差(实时、超快、快速、最终)、地球定向参数、卫星姿态ORBEX、DCB/DSB/OSB、SINEX周解、电离层和对流层延迟产品、天线相位中心等产品下载。

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

### 1、Good-Gui：Qt 界面数据下载程序

![image-20240831165507638](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165507638.png)

![image-20240831165542898](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165542898.png)

![image-20240831165628437](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165628437.png)

### 2、Good-Cui：命令行数据下载程序

> Windows：`good_cui.exe` + `yaml 配置文件路径`
>
> Linux：`good_cui` + `yaml 配置文件路径`









### 3、YAML 格式配置文件说明

YAML（YAML Ain't Markup Language）是一种轻量级的数据序列化格式，可以用于配置文件、数据交换、API 请求等多种场景。它是一种简单易用的数据序列化格式，使得数据可以以人类易读的方式进行存储和传输。语法非常简单，使用缩进和符号来表示数据结构，需要注意以下激光问题：

> 1. 大小写敏感；
> 2. 缩进不允许使用 Tab，只允许空格；
> 3. 缩进的空格数不重要，只要相同层级的元素左对齐即可；
> 4. 键和冒号之间无空格，冒号和值之间要加空格；
> 5. `#` 后面是注释；

配置文件以 **数组 + 键值对** 的方式组织，每一个键值对都是一个配置项，GAMP-GOOD 配置文件包含以下配置项：

1. 目录：

   > * 各类数据下载的子目录（Obs/Nav/Orb等）都继承自主目录（填相对路径）；
   > * wget/gzip/crx2rnx 的目录以及 Obs/Nav/Trop 测站列表文件可以继承自主目录（填相对路径），也可以独立于主目录之外（填绝对路径）。

2. `procTime`：数据下载的开始时间，支持三种形式：

   ```cpp
   - 1 年/月/日    : 1 2024 8 3
   - 2 年/年积日	: 2 2024 216
   - 3 GPS周/周内天 : 3 2327 7
   ```

3. `minusAdd1day`：【0 : 关、1 : 开】选择是否额外下载前一天和后一天的精密星历和精密钟差。

   > 部分 PPP 程序需要用到前天和后一天的精密星历和精密钟差进行插值。

4. `merge_sp3files`：【0 : 关、1 : 开】选择是否合并下载的三天精密星历和钟差。

   > 仅在 minusAdd1day 开启的时候生效。

5. `printInfoWget`：【0 : 关、1 : 开】选择是否在终端显示 Wegt 输出的信息。

   > Qt 界面程序中无效。

6. `ftpDownloading`：

   1. `opt4ftp`：【0 : 关、1 : 开】数据下载的总开关。

   2. `ftpArch`：【cddis、ign、whu】选择主要的数据中心，支持 cddis、ign、whu：

      ```cpp
      - 美国 cddis : ftps://gdc.cddis.eosdis.nasa.gov/pub/
      - 法国 ign   : ftp://igs.ign.fr/pub/
      - 中国 whu   : ftp://igs.gnsswhu.cn/pub/
      ```

      > * 绝大多数的数据都在选择的数据中心里下载，有部分例外，详见 [GAMP-GOOD 支持下载的产品链接.md](GAMP-GOOD 支持下载的产品链接.md)。
      > * 一般来说美国的 cddis 数据最全，国内用 whu 下载数据最快。

7. `getObs`：

   1. `opt4obs`：【0 : 关、1 : 开】观测数据下载的总开关。
   2. `obsType`：选择观测数据下载的种类
   3. `obsFrom`：
   4. `obsList`：
   5. `sHH4obs`：
   6. `nHH4obs`：
   7. `l2s4obs`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

8. `getNav`：
   1. `opt4nav`：【0 : 关、1 : 开】
   2. `navType`：
   3. `navSys`：
   4. `navFrom`：
   5. `navList`：
   6. `sHH4nav`：
   7. `nHH4nav`：
   8. `l2s4nav`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

9. `getOrbClk`：
   1. `opt4oc`：【0 : 关、1 : 开】
   2. `ocFrom`：
   3. `sHH4oc`：
   4. `nHH4oc`：
   5. `l2s4oc`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

10. `getEop`：

   11. `opt4eop`：【0 : 关、1 : 开】

   12. `eopFrom`：

   13. `sHH4eop`：

   14. `nHH4eop`：

   15. `l2s4eop`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

16. `getObx`：
    1. `opt4obx`：【0 : 关、1 : 开】
    2. `obxFrom`：

17. `getDsb`：

    1. `opt4dsb`：【0 : 关、1 : 开】
    2. `dsbFrom`：

18. `getOsb`：

    1. `opt4osb`：【0 : 关、1 : 开】
    2. `osbFrom`：

19. `getSnx`：
    1. `opt4snx`：【0 : 关、1 : 开】
    2. `l2s4snx`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

20. `getIon`：
    1. `opt4ion`：
    2. `ionFrom`：
    3. `l2s4ion`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

21. `getRoti`
    1. `opt4rot`：【0 : 关、1 : 开】

22. `getTrp`：

    1. `opt4trp`：【0 : 关、1 : 开】
    2. `trpFrom`：
    3. `trpList`：
    4. `l2s4trp`：

23. `getAtx`：
    
    1. `opt4atx`：

---



```yaml
# The root/main directory of GNSS observations and products  -------------------
mainDir       : D:\Projects\proj_VScode\UNIQ\GOOD\dataset_Win
# The sub-directories of GNSS observations and products, which needs to inherit the path of 'mainDir'
#   i.e., 'orbDir' = 'mainDir' + 'orbDir', which is 'D:\Projects\proj_VScode\UNIQ\GOOD\dataset\orb'
obsDir        : obs               # The sub-directory of RINEX format observation files
navDir        : nav               # The sub-directory of RINEX format broadcast ephemeris files
orbDir        : orb               # The sub-directory of SP3 format precise ephemeris files
clkDir        : clk               # The sub-directory of RINEX format precise clock files
eopDir        : eop               # The sub-directory of earth rotation/orientation parameter (EOP) files
obxDir        : obx               # The sub-directory of MGEX final/rapid and/or CNES real-time ORBEX (ORBit EXchange format) files
biaDir        : bia               # The sub-directory of CODE/MGEX differential code/signal bias (DCB/DSB), MGEX observable-specific signal bias (OSB), and/or CNES real-time OSB files
snxDir        : snx               # The sub-directory of SINEX format IGS weekly solution files
ionDir        : ion               # The sub-directory of CODE/IGS global ionosphere map (GIM) files
ztdDir        : ztd               # The sub-directory of CODE/IGS tropospheric product files
tblDir        : tbl               # The sub-directory of table files (i.e., ANTEX, ocean tide loading files, etc.) for processing
logDir        : log               # The sub-directory of log file
3partyDir     : thirdparty_Win  # The sub-directory where third-party softwares (i.e., 'wget', 'gzip', 'crx2rnx' etc) are stored

# Time settings ----------------------------------------------------------------
procTime      : 2  2024  1  1   # The setting of start time for processing, which should be set to '1 year month day ndays' or '2 year doy ndays' or '3 week dow ndays'. NOTE: doy = day of year; week = GPS week; dow = day within week

# Settings of FTP downloading --------------------------------------------------
minusAdd1day  : 1                 # (0: off  1: on) The setting of the day before and after the current day for precise satellite orbit and clock products downloading
merge_sp3files: 1                 # (0: off  1: on) to merge three consecutive sp3 files into one file
printInfoWget : 1                 # (0: off  1: on) Printing the information generated by 'wget'

# Handling of FTP downloading --------------------------------------------------
ftpDownloading:                   # The setting of the master switch for data downloading
  opt4ftp: 1                      #   1st: (0:off  1:on);
  ftpArch: whu                    #   2nd: the FTP archive, i.e., cddis, ign, or whu.
getObs:                           # GNSS observation data downloading option
  opt4obs: 0                      #   1st(opt4obs): (0: off  1: on);
  obsType: daily                  #   2nd(obsType): 'daily', 'hourly', 'highrate', '30s', '5s', or '1s';
  obsFrom: mgex                   #   3rd: 'igs', 'mgex', 'igm', 'cut', 'hk', 'ngs', 'epn', 'pbo', or 'chi';
  obsList: all                    #   4th: 'all' (observation files downloaded in the whole directory) or the site list file name (observation files downloaded site-by-site according to the site list file, which needs to inherit the path of 'mainDir');
  sHH4obs: 01                     #   5th: Start hour (00, 01, 02, ...);
  nHH4obs: 1                      #   6th: The consecutive hours, i.e., '01  3' denotes 01, 02, and 03;
  l2s4obs: 1                      #   7th: Valid only for the observation files with long name, 0: long name, 1: short name, 2: long and short name
                                  #   NOTE: The 5th and 6th items are valid ONLY when the 2nd item 'hourly', 'highrate', '5s', or '1s' is set.
                                  #   NOTE: If the 3rd item is 'igs', 'mgex', 'igm', the 2nd item can be 'daily', 'hourly', or 'highrate';
                                  #         If the 3rd item is 'cut', 'ngs', 'epn', or 'pbo', the 2nd item should be 'daily';
                                  #         If the 3rd item is 'hk', the 2nd item can be 1) '30s', '5s', or '1s' 2) '30s', '05s', or '01s'. However, '30 s', '5 s', or '1 s' is NOT allowed.
                                  #   NOTE: If the 3rd item is 'hk' 'ngs', or 'pbo', the 4th item should ONLY be the full path of site list.
                                  #   INFO: The 2nd item 'igs' is for IGS observation (RINEX version 2.xx, short name 'd');
                                  #         The 2nd item 'mgex' is for MGEX observation (RINEX version 3.xx, long name 'crx');
                                  #         The 2nd item 'igm' is for the union of IGS and MGEX (IGS + MGEX, while the priority of MGEX sites is higher) observation with respect to the site name;
                                  #         The 2nd item 'cut' is for Curtin University of Technology (CUT) observation (RINEX version 3.xx, long name 'crx');
                                  #         The 2nd item 'hk' is for Hong Kong CORS observation (RINEX version 3.xx, long name 'crx');
                                  #         The 2nd item 'ngs' is for NGS/NOAA CORS observation (RINEX version 2.xx, short name 'd');
                                  #         The 2nd item 'epn' is for EUREF Permanent Network (EPN) observation (RINEX version 3.xx, long name 'crx' and RINEX version 2.xx, short name 'd');
                                  #         The 2nd item 'pbo' is for Plate Boundary Observatory (PBO) observation (RINEX version 3.xx, long name 'crx');
                                  #         The 2nd item 'chi' is for Centro Sismologico Nacional of Universidad de Chile observation (RINEX version 2.xx, short name "d")
getNav:                           # Various broadcast ephemeris downloading option
  opt4nav: 0                      #   1st: (0: off  1: on);
  navType: daily                  #   2nd: 'daily' or 'hourly';
  navSys : mixed3                 #   3rd: 'gps', 'glo', 'bds', 'gal', 'qzs', 'irn', 'mixed3', 'mixed4', or 'all';
  navFrom: igs                    #   4th: Analysis center (i.e., 'igs', 'dlr', 'ign', 'gop', or 'wrd') that carries out the combination of broadcast ephemeris for mixed navigation data. From CDDIS or WHU FTP, 'igs' and 'dlr' can be downloaded, and from IGN, 'igs' and 'ign' can be downloaded. The downloading of 'gop' and/or 'wrd' is via the other FTP addresses;
  navList: site_mgex.list         #   5th: ONLY the site list file name is valid if the 2nd item is 'hourly';
  sHH4nav: 01                     #   6th: Start hour (00, 01, 02, ...);
  nHH4nav: 2                      #   7th: The consecutive hours, i.e., '01  3' denotes 01, 02, and 03;
  l2s4nav: 2                      #   8th: Valid only for the navigation files with long name, 0: long name, 1: short name, 2: long and short name
                                  #   NOTE: The 5th, 6th, and 7th items are valid ONLY when the 2nd item 'hourly' is set.
                                  #   NOTE: If the 2nd item is 'daily', the 3rd item should be 'gps', 'glo', 'mixed3', or 'mixed4';
                                  #         If the 2nd item is 'hourly', the 3rd item can be 'gps', 'glo', 'bds', 'gal', 'qzs', 'irn', 'mixed', or 'all'.
                                  #   NOTE: The 4th item is valid ONLY when the 3rd item 'mixed3' is set.
                                  #   NOTE: The 3rd item 'mixed3' is for RINEX 3.xx, while 'mixed4' is for RINEX 4.xx.
getOrbClk:                        # Satellite final/rapid/ultra-rapid precise orbit and clock downloading option
  opt4oc : 1                      #   1st: (0: off  1: on);
  ocFrom : igs+gfz_m                    #   2nd: Analysis center (i.e., IGS final: 'cod', 'emr', 'esa', 'gfz', 'grg', 'igs', 'jgx', 'jpl', 'mit', 'all', 'cod+igs', 'cod+gfz+igs', ...; MGEX final: 'cod_m', 'gfz_m', 'grg_m', 'iac_m', 'jax_m', 'sha_m', 'whu_m', 'all_m', 'cod_m+gfz_m', 'grg_m+whu_m', ...; rapid: 'cod_r', 'emr_r', 'esa_r', 'gfz_r', 'igs_r'; ultra-rapid: 'esa_u', 'gfz_u', 'igs_u', 'whu_u'; real-time: 'cnt'). NOTE: The option of 'cnt' is for real-time precise orbit and clock products from CNES offline files;
  sHH4oc : 01                     #   3rd: Start hour (00, 06, 12, or 18 for esa_u and igs_u; 00, 03, 06, ... for gfz_u; 01, 02, 03, ... for whu_u).
  nHH4oc : 2                      #   4th: The consecutive sessions, i.e., '00  3' denotes 00, 06, and 12 for esa_u and/or igs_u, 00, 03, and 06 for gfz_u, while 00, 01, and 02 for whu_u.
  l2s4oc : 2                      #   5th: Valid only for the precise orbit and clock files with long name, 0: long name, 1: short name, 2: long and short name
getEop:                           # Earth rotation/orientation parameter (ERP/EOP) downloading option
  opt4eop: 0                      #   1st: (0: off  1: on);
  eopFrom: igs                    #   2nd: Analysis center (i.e., final: 'cod', 'emr', 'esa', 'gfz', 'grg', 'igs', 'jgx', 'jpl', 'mit'; ultra-rapid: 'esa_u', 'gfz_u', 'igs_u').
  sHH4eop: 01                     #   3rd: Valid ONLY when  the 2nd item 'esa_u', 'gfz_u', or 'igs_u' is set.
  nHH4eop: 4                      #   4th: Valid ONLY when  the 2nd item 'esa_u', 'gfz_u', or 'igs_u' is set.
  l2s4eop: 2                      #   5th: Valid only for EOP file with long name, 0: long name, 1: short name, 2: long and short name
getObx:                           # ORBEX (ORBit EXchange format) for satellite attitude information downloading option
  opt4obx: 0                      #   1st: (0: off  1: on);
  obxFrom: all                    #   2nd: Analysis center (i.e., final/rapid: 'cod', 'gfz', 'grg', 'whu', 'all'; real-time: 'cnt'). NOTE: The option of 'cnt' is for real-time ORBEX from CNES offline files.
getDsb:                           # Differential code/signal bias (DCB/DSB) downloading option
  opt4dsb: 0                      #   1st: (0: off  1: on);
  dsbFrom: all                    #   2nd: Analysis center (i.e., 'cod', 'cas', 'all'). NOTE: DCBs from CODE are for GPS and GLONASS, while DSBs from CAS are for multiple GNSS.
getOsb:                           # Observable-specific signal bias (OSB) downloading option
  opt4osb: 0                      #   1st: (0: off  1: on);
  osbFrom: cas                    #   2nd: Analysis center (i.e., final/rapid: 'cas', 'cod', 'gfz', 'grg', 'whu', 'all'; real-time: 'cnt'). NOTE: The option of 'cnt' is for real-time OSBs from CNES offline files.
getSnx:                           # IGS weekly SINEX downloading option
  opt4snx: 0                      #   1st: (0: off  1: on)
  l2s4snx: 2                      #   2nd: Valid only for IGS weekly SINEX file with long name, 0: long name, 1: short name, 2: long and short name
getIon:                           # Global ionosphere map (GIM) downloading option
  opt4ion: 0                      #   1st: (0: off  1: on);
  ionFrom: cod                    #   2nd: Analysis center (i.e., final: 'cas', 'cod', 'emr', 'esa', 'igs', 'jpl', 'upc', 'all', 'cas+cod', 'cas+cod+igs', ...;)
  l2s4ion: 2                      #   3rd: Valid only for the ionosphere files with long name, 0: long name, 1: short name, 2: long and short name
getRoti:                          # Rate of TEC index (ROTI) downloading option
  opt4rot: 0                      # (0: off  1: on)
getTrp:                           # CODE/IGS tropospheric product downloading option
  opt4trp: 0                      #   1st: (0:off  1:on);
  trpFrom: cod                    #   2nd: Analysis center (i.e., 'igs' or 'cod');
  trpList: site_trp.list          #   3rd: the file name of site.list. NOTE: It is valid ONLY when the 2nd item 'igs' is set.
  l2s4trp: 2                      #   4th: Valid only for the troposphere files with long name, 0: long name, 1: short name, 2: long and short name
getAtx:                           # ANTEX format antenna phase center correction downloading option
  opt4atx: 0                      # (0:off  1:on)
```







### 4、GA 数据下载脚本使用







### 5、GAMP-GOOD 不支持下载的数据

1. 潮汐文件：

   > 

2. 武汉大学李星星老师团队的 UPD 文件

   > 

3. GAMIT、PANDA、Bernese  表文件

   > 



### 6、已知存在的问题

1. 无法下载快速/超快速的精密星历、精密钟差、电离层产品

   > 代码暂未实现，将在后续版本更新。

2. 部分小时广播星历下载存在问题

   > 

3. 部分 IGS 测站无法下载对流层数据

   > 服务器上没有这些测站的数据。

4. ATX 和 DCB 文件有时候下载很慢

   > * Atx.20

4. grg 的

   > 

5. Roti、Eop、Snx 经常无法下载

   > 



### 7、出现异常情况时的解决方法







## 二、GAMP-GOOD 源码解析

### 1、程序概述

* **Libgood：程序核心代码库**
  * **时间转换函数 (TimeUtil.cpp)**：定义时间结构体gtime_t{int mjd; double sod}作为中间媒介进行时间转换和传递，一般无需改动；
  * **字符串操作函数 (StringUtil.cpp)**：字符串截取、转换等操作，一般无需改动；
  * **配置文件读写函数 (PreProcess.cpp)**：全局控制变量 fopt_t/popt_t、FTP下载选项的初始化，配置文件读取和输出等；
  * **日志输出模块 (logger.h)**：；
  * **数据下载核心模块 (FtpUtil.cpp)** ：其它为相应的子程序实现不同GNSS数据下载；
* **Good_cui：命令行可执行程序**
  * 导入 YAML 配置文件到 fopt、popt 结构体；
  * 调用  FtpUtil::FtpDownload 根据配置下载数据；
* **Good_gui：Qt界面可执行程序**
  * 在界面上进行配置，



### 2、文件结构

![image-20241005223229079](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241005223229079.png)

### 3、CmakeList.txt

![image-20241005225943841](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241005225943841.png)

![image-20241005225959750](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241005225959750.png)

### 4、第三方命令行工具

#### 1. wget 数据下载

> 官网：https://www.gnu.org/software/wget/

`wget` 是一个在命令行环境下工作的免费网络工具，用于从网络上下载文件。它支持 HTTP、HTTPS 和 FTP 协议，并且能够递归下载，即下载整个网站或目录结构。常用命令如下：

* 下载单个文件：
* 递归下载整个目录：



#### 2. gzip 解压 gz 文件

> 官网：https://www.gzip.org/

`gzip` 是一个广泛使用的文件压缩工具，它使用 DEFLATE 算法来压缩文件，以减小文件的大小。`gzip` 通常用于 Unix 和 Linux 系统，但它也可以在其他操作系统上使用。压缩后的文件通常具有 `.gz` 或者 `.z` 扩展名。常用命令如下：

```cpp
- 压缩单个文件              : gzip <filename>
- 解压单个文件              : gzip -d <filename.gz>
- 保留原始文件并压缩         : gzip -c <filename> > <filename.gz>
- 保留原始文件并解压         : gzip -cd <filename.gz> > <filename>
- 查看压缩文件内容无需解压    : zcat <filename.gz>
```

#### 3. crx2rnx 解压 RINEX 文件

> 官网：https://terras.gsi.go.jp/ja/crx2rnx.html
>
> 论文：https://www.gsi.go.jp/common/000045517.pdf

`crx2rnx` 是一个用于将 Trimble 公司的压缩 RINEX 文件（通常具有 `.crx` 扩展名）转换为标准 RINEX 文件（具有 `.rnx` 或 `.obs` 扩展名）的工具。常用命令如下：

```cpp

```



#### 4. lftp

如果无法连接，可以在 lftp 的命令最后加上 `-d` ，使用调试模式





### 4、数据下载一般流程（以 GetRoti 为例）

1. 切换工作目录到数据下载的子文件夹：

   ```cpp
       /* change directory */
   #ifdef _WIN32   /* for Windows */
       _chdir(dir.c_str());
   #else           /* for Linux or Mac */
       chdir(dir.c_str());
   #endif
   ```

2. 生成年/年积日/GPS周/周内天，字符串，用于后续拼接出文件名：

   ```cpp
   /* compute day of year */
   int yyyy, doy;
   GTime::time2yrdoy(ts, yyyy, doy);
   int yy = GTime::yyyy2yy(yyyy);
   std::string syyyy = CString::int2str(yyyy, 4);
   std::string syy = CString::int2str(yy, 2);
   std::string sdoy = CString::int2str(doy, 3);
   ```

3. 生成文件名 `rotfile`，判断文件是否已经存在：

   ```cpp
   std::string rotfile = "roti" + sdoy + "0." + syy + "f";
   if (access(rotfile.c_str(), 0) == -1)
   ```

4. 判断文件是否存在，如果文件已经存在，则无需下载，输出 `文件已经存在` 的信息到终端和日志文件之后直接结束程序；如果文件不存在，则进入正常下载流程：

   ```cpp
   if (access(rotfile.c_str(), 0) == -1){
       ...
   }
   else{
       TRACE(TINFO, "ROTI file " + rotfile + " has existed!");
       TRACEF(fplog_, TINFO, "ROTI file " + dir + std::string(1,FILEPATHSEP) + rotfile + " has existed!");
   }
   ```

5. 从 `ftparchive_` 中取对应的下载地址，拼接上 `年/年积日` 得到待下载文件所在的文件夹路径 `url`：

   ```cpp
   std::string wgetfull = fopt->wgetfull, gzipfull = fopt->gzipfull, qr = fopt->qr;
   std::string url, cutdirs = " --cut-dirs=6 ";
   if (ftpname == "CDDIS") url = ftparchive_.CDDIS[IDX_ROTI] + "/" +
       syyyy + "/" + sdoy;
   else if (ftpname == "IGN") url = ftparchive_.IGN[IDX_ROTI] + "/" +
       syyyy + "/" + sdoy;
   else if (ftpname == "WHU") url = ftparchive_.WHU[IDX_ROTI] + "/" +
        syyyy + "/" + sdoy;
   else url = ftparchive_.CDDIS[IDX_ROTI] + "/" + syyyy + "/" + sdoy;
   ```

6. 调用 wget，下载 `url` 目录中，名为 `rotfile` 的文件，后缀名不限：

   ```cpp
   /* it is OK for '*.Z' or '*.gz' format */
   std::string rotxfile = rotfile + ".*";
   std::string cmd = wgetfull + " " + qr + " -nH -A " + rotxfile + cutdirs + url;
   std::system(cmd.c_str());
   ```

7. 拼接出下载到本地的文件路径 `localfile`：

   ```cpp
   std::string sep;
   sep.push_back((char)FILEPATHSEP);
   std::string localfile = dir + sep + rotfile, url0;
   CString::GetFile(dir, rotfile, rotxfile);
   ```

8. 判断文件是否下载成功，下载失败输出错误，下载成功调用 `gzip` 解压，有的还会进行长短文件名转换：

   ```cpp
   if (access(rotxfile.c_str(), 0) == -1)
   {
       TRACE(TWARNING, "failed to download ROTI file " + rotfile);
       url0 = url + '/' + rotxfile;
       TRACEFP(fplog_, url0, localfile, false);
   }
   else
   {
       /* extract '*.gz' or '*.Z' */
       cmd = gzipfull + " -d -f " + rotxfile;
       std::system(cmd.c_str());
   
       if (access(rotfile.c_str(), 0) == 0)
       {
           TRACE(TINFO, "successfully download ROTI file " + rotfile);
           url0 = url + '/' + rotxfile;
           TRACEFP(fplog_, url0, localfile, true);
       }
   }
   ```

### 5、YAML 配置文件读写





### 6、日志输出



