<div align="center">
    <a name="Top"></a>
    <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/Uniq.png" alt="Uniq" style="zoom: 20%;" />
	<h1>GNSS 数据下载程序 GOOD 4.0 说明</h1>
    <p><strong>安徽理工大学-李郑骁  2024年9月2日</strong></p>
</div>

[TOC]

## 前言

近日，UNIQ导航实验室(Ubiquitous Navigation & Integrated positioning lab in Quest)开源了(Gnss Observations and prOducts Downloader)。相应的源代码、配置文件、样例及用户手册详见https://github.com/zhouforme0318/GAMPII-GOOD。



## 一、GAMP-GOOD 快速上手

GAMP-GOOD 由核心代码库 Libgood、命令行可执行程序 Good_cui 和 Qt界面可执行程序 Good_gui 三部分组成，自2021年4月1.0版本至今已历经15个版本迭代，目前已实现常用GNSS观测值、广播星历、精密卫星轨道和钟差(实时、超快、快速、最终)、地球定向参数、卫星姿态ORBEX、DCB/DSB/OSB、SINEX周解、电离层和对流层延迟产品、天线相位中心等产品下载。

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









### 2、GOOD-cui：命令行数据下载程序









### 3、YAML 格式配置文件说明

YAML（YAML Ain't Markup Language）是一种轻量级的数据序列化格式，可以用于配置文件、数据交换、API 请求等多种场景。它是一种简单易用的数据序列化格式，使得数据可以以人类易读的方式进行存储和传输。语法非常简单，使用缩进和符号来表示数据结构，YAML 使用的时候需要注意：

> - 大小写敏感；
> - 缩进不允许使用 Tab，只允许空格；
> - 缩进的空格数不重要，只要相同层级的元素左对齐即可；
> - 键和冒号之间无空格，冒号和值之间要加空格；
> - `#` 后面是注释；

配置文件以 **数组 + 键值对** 的方式组织，每一个键值对都是一个配置项，



1. 目录：

   > * 各类数据下载的子目录都继承自主目录（填相对路径）；
   >
   > * wget/gzip/crx2rnx 的目录以及 Obs/Nav/Trop 测站列表文件可以继承自主目录（填相对路径），也可以独立于主目录之外（填绝对路径）。

2. `procTime`：数据下载的开始时间，支持三种形式：

   ```cpp
   - 年/月/日    : 1 2024 8 3
   - 年/年积日	   : 2 2024 216
   - GPS周/周内天 : 3 2327 7
   ```

3. `minusAdd1day`：【0 : 关、1 : 开】

4. `merge_sp3files`：【0 : 关、1 : 开】

5. `printInfoWget`：【0 : 关、1 : 开】

6. `ftpDownloading`：

   1. `opt4ftp`：【0 : 关、1 : 开】

   2. `ftpArch`：【cddis、ign、whu】选择主要的数据中心；

      ```cpp
      - 美国 cddis : ftps://gdc.cddis.eosdis.nasa.gov/pub/
      - 法国 ign   : ftp://igs.ign.fr/pub/
      - 中国 whu   : ftp://igs.gnsswhu.cn/pub/
      ```

      > 并非所有数据都在选择的数据中心里下载。

7. `getObs`：

   1. `opt4obs`：【0 : 关、1 : 开】
   2. `obsType`：
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
   1. `opt4eop`：【0 : 关、1 : 开】
   2. `eopFrom`：
   3. `sHH4eop`：
   4. `nHH4eop`：
   5. `l2s4eop`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

11. `getObx`：
    1. `opt4obx`：【0 : 关、1 : 开】
    2. `obxFrom`：

12. `getDsb`：

    1. `opt4dsb`：【0 : 关、1 : 开】
    2. `dsbFrom`：

13. `getOsb`：

    1. `opt4osb`：【0 : 关、1 : 开】
    2. `osbFrom`：

14. `getSnx`：
    1. `opt4snx`：【0 : 关、1 : 开】
    2. `l2s4snx`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

15. `getIon`：
    1. `opt4ion`：
    2. `ionFrom`：
    3. `l2s4ion`：【0 : 仅长文件名、1 : 仅短文件名、2 : 长文件名 + 短文件名】

16. `getRoti`
    1. `opt4rot`：【0 : 关、1 : 开】

17. `getTrp`：

    1. `opt4trp`：【0 : 关、1 : 开】
    2. `trpFrom`：
    3. `trpList`：
    4. `l2s4trp`：

18. `getAtx`：
    1. `opt4atx`：



### 4、GA 数据下载





### 5、GAMP-GOOD 不支持下载的数据





### 6、用 RTKPOST 处理 GAMP-GOOD 下载的数据示例





### 7、用 GAMP 处理 GAMP-GOOD 数据示例









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





### 3、CmakeList.txt





### 4、第三方工具



#### 1. wget 数据下载

> 官网：https://www.gnu.org/software/wget/

`wget` 是一个在命令行环境下工作的免费网络工具，用于从网络上下载文件。它支持 HTTP、HTTPS 和 FTP 协议，并且能够递归下载，即下载整个网站或目录结构。常用命令如下：

* 下载单个文件：
* 递归下载整个目录：



#### 2. gzip 解压 gz

> 官网：https://www.gzip.org/

`gzip` 是一个广泛使用的文件压缩工具，它使用 DEFLATE 算法来压缩文件，以减小文件的大小。`gzip` 通常用于 Unix 和 Linux 系统，但它也可以在其他操作系统上使用。压缩后的文件通常具有 `.gz` 或者 `.z` 扩展名。常用命令如下：

```cpp
- 压缩单个文件              : gzip <filename>
- 解压单个文件              : gzip -d <filename.gz>
- 保留原始文件并压缩         : gzip -c <filename> > <filename.gz>
- 保留原始文件并解压         : gzip -cd <filename.gz> > <filename>
- 查看压缩文件内容无需解压    : zcat <filename.gz>
```

#### 3. crx2rnx 解压 RINEX

> 官网：https://terras.gsi.go.jp/ja/crx2rnx.html
>
> 论文：https://www.gsi.go.jp/common/000045517.pdf

`crx2rnx` 是一个用于将 Trimble 公司的压缩 RINEX 文件（通常具有 `.crx` 扩展名）转换为标准 RINEX 文件（具有 `.rnx` 或 `.obs` 扩展名）的工具。常用命令如下：

```cpp
```





#### 4. lftp



如果无法连接，可以在 lftp 的命令最后加上 `-d` ，使用调试模式





### 4、数据下载一般流程（以 GetRoti 为例）

1. 切换工作目录到数据下载的子文件夹；

   ```cpp
       /* change directory */
   #ifdef _WIN32   /* for Windows */
       _chdir(dir.c_str());
   #else           /* for Linux or Mac */
       chdir(dir.c_str());
   #endif
   ```

2. 生成年/年积日/GPS周/周内天，字符串，用于

   ```cpp
   /* compute day of year */
   int yyyy, doy;
   GTime::time2yrdoy(ts, yyyy, doy);
   int yy = GTime::yyyy2yy(yyyy);
   std::string syyyy = CString::int2str(yyyy, 4);
   std::string syy = CString::int2str(yy, 2);
   std::string sdoy = CString::int2str(doy, 3);
   ```

3. 生成文件名，判断文件是否已经存在：

   ```cpp
   std::string rotfile = "roti" + sdoy + "0." + syy + "f";
   if (access(rotfile.c_str(), 0) == -1)
   ```

4. 如果文件已经存在，则无需下载，输出“文件已经存在”的信息到终端和日志文件之后直接结束程序：

   ```cpp
   if (access(rotfile.c_str(), 0) == -1){
       ...
   }
   else{
       TRACE(TINFO, "ROTI file " + rotfile + " has existed!");
       TRACEF(fplog_, TINFO, "ROTI file " + dir + std::string(1,FILEPATHSEP) + rotfile + " has existed!");
   }
   ```

5. 







### 5、YAML 配置文件读写





## 三、GNSS 数据类型和格式说明

### 1、GNSS 数据分类

![PPP 数据模型](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/PPP%20%E6%95%B0%E6%8D%AE%E6%A8%A1%E5%9E%8B.png)



#### 1. 观测值文件（yy*o、*yyd、crx）

> 格式说明：
>
> * https://files.igs.org/pub/data/format/rinex211.txt
> * https://files.igs.org/pub/data/format/rinex305.pdf
> * https://files.igs.org/pub/data/format/rinex_4.01.pdf



![GNSS观测值](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GNSS%E8%A7%82%E6%B5%8B%E5%80%BC.png)







#### 2. 广播星历 （yyg、yyn、yyp、rnx）

> 格式说明：
>
> * https://files.igs.org/pub/data/format/rinex211.txt
> * https://files.igs.org/pub/data/format/rinex305.pdf
> * https://files.igs.org/pub/data/format/rinex_4.01.pdf





#### 3. 精密星历（sp3、eph）

> 格式说明：https://files.igs.org/pub/data/format/sp3d.pdf





#### 4. 精密钟差（clk、clk_05s、clk_30s）

> 格式说明：https://files.igs.org/pub/data/format/rinex_clock304.txt





#### 5. 地球自转参数 (EOP)

>  格式说明：https://files.igs.org/pub/data/format/erp.txt



#### 6. 码偏差（DCB、OSB、BSX）

> 格式说明：https://files.igs.org/pub/data/format/sinex_bias_100.pdf



|        机构        | 卫星系统 |    产品形式    |                           下载网站                           |
| :----------------: | :------: | :------------: | :----------------------------------------------------------: |
| **武大张小红团队** |   GREC   |  宽巷+窄巷UPD  | https://github.com/FCB-SGG/FCB.FILES<br/>http://igmas.users.sgg.whu.edu.cn/home |
| **武大耿江辉团队** |   GREC   |      OSB       |            ftp://igs.gnsswhu.cn/pub/whu/phaseblas            |
|      **CAS**       |   GREC   |      OSB       |           ftp://ftp.gipp.org.cn/product/dcb/mgex/            |
|      **GFZ**       |   GREC   | 宽巷UPD+相位种 |            ftp://igs.ign.fr/pub/igs/products/mgex            |
|      **CNES**      |    GR    | 宽巷UPD+整数钟 |                ftp://ftpsedr.cls.fr/pub/igsac                |
|      **CODE**      |    GR    |      OSB       |                http://ftp.aiub.unibe.ch/CODE/                |





#### 7. IGS SINEX解（snx）

> 格式说明：https://www.iers.org/IERS/EN/Organization/AnalysisCoordinator/SinexFormat/sinex.html





#### 8. 对流层延迟（yyzpd、TRO）

> 格式说明：https://files.igs.org/pub/data/format/sinex_tro_v2.00.pdf





#### 9. 电离层延迟（yyi、yyI）

> 格式说明：https://files.igs.org/pub/data/format/ionex1.pdf





#### 10. 电离层TEC变化率指数（ROTI）

> 格式说明：https://files.igs.org/pub/data/format/ionex1.pdf





#### 11. 天线相位中心改正（atx）

> 格式说明：https://files.igs.org/pub/data/format/antex14.txt



### 2、CORS 网

#### 1. IGS/MGEX





#### 2. 科廷大学短基线数据

> http://saegnss2.curtin.edu.au/ldc/ 





#### 3. 香港 CORS 网

> **HTTP**：https://www.geodetic.gov.hk/sc/rinex/downv.aspx
>
> **FTP**：[ftp://ftp.geodetic.gov.hk](ftp://ftp.geodetic.gov.hk/)[/](ftp://ftp.geodetic.gov.hk/)

香港卫星定位参考站网由18个平均分布于全港各处的连续运行参考站 (CORS) 组成（包括16个参考站及2个完整性监测站），可免费下载使用，数据分辨率有1s、5s、30s三种。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831164958231.png" alt="image-20240831164958231" style="zoom:50%;" />

#### 4.  美国国家大地测量CORS观测数据

> **官网介绍**：https://geodesy.noaa.gov/CORS/
>
> **测站地图图**：https://arcg.is/18fWq8
>
> **AWS**：https://noaa-cors-pds.s3.amazonaws.com/index.html
>
> **HTTP**：https://geodesy.noaa.gov/corsdata/

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165057899.png" alt="image-20240831165057899" style="zoom:50%;" />





#### 5. 欧洲CORS观测数据

> **官网介绍**：https://epncb.oma.be/ 
>
> **论文**：https://epncb.oma.be/_documentation/papers/Bruyninx_EPNCB_2019.pdf
>
> **测站地图**：https://epncb.oma.be/_networkdata/stationmaps.php
>
> **FTP**：[ftp://ftp.epncb.oma.be/pub/obs](ftp://ftp.epncb.oma.be/pub/obs) 





#### 6. 美国国家科学基金会地球观测计划的板块边界观测站观测数据

> **官网介绍**：https://www.unavco.org/projects/past-projects/pbo/pbo.html 
>
> **PDF**：https://www.unavco.org/projects/past-projects/pbo/lib/docs/dms_cdr.pdf 

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165144842.png" alt="image-20240831165144842" style="zoom:50%;" />





#### 7. 智利观测数据

> 官网介绍：[https://gps.csn.uchile.cl](https://gps.csn.uchile.cl/)[/](https://gps.csn.uchile.cl/) 
>
> 数据下载：[https://gps.csn.uchile.cl/data](https://gps.csn.uchile.cl/data/)[/](https://gps.csn.uchile.cl/data/) 

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165157213.png" alt="image-20240831165157213" style="zoom: 50%;" />



#### 8. Geoscience Australia 







#### 9. 其它 CORS 数据介绍

* 德国 
* 日本







### 3、常见问题

#### 1. 长文件名 or 短文件名





#### 2. IGS or MGEX





#### 3. ATX14 or atx20





#### 4. DCB or DSB or OSB





#### 5. GFZ or GBM





#### 6. ZTD or ZPD





#### 7. BINEX or RINEX











