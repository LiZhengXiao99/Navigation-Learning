### 1、GNSS 数据分类

![PPP 数据模型](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/PPP%2520%25E6%2595%25B0%25E6%258D%25AE%25E6%25A8%25A1%25E5%259E%258B.png)



#### 1. 观测值文件（yy*o、*yyd、crx）

> 格式说明：
>
> * https://files.igs.org/pub/data/format/rinex211.txt
> * https://files.igs.org/pub/data/format/rinex305.pdf
> * https://files.igs.org/pub/data/format/rinex_4.01.pdf

![GNSS观测值](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GNSS%25E8%25A7%2582%25E6%25B5%258B%25E5%2580%25BC.png)







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





#### 7. IGS SINEX 解（snx）

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



MGEX 数据才支持北斗



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

官网的测站图是调用谷歌地图的接口来实现的，所以国内直连无法

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240831165157213.png" alt="image-20240831165157213" style="zoom: 50%;" />



#### 8. Geoscience Australia