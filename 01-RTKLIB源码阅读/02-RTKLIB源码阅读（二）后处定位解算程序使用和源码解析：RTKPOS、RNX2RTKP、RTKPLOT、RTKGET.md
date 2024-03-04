

[TOC]

## 一、后处理程序使用

> 再次推荐一下 B 站赵老师的[视频讲解](https://space.bilibili.com/479790048?spm_id_from=333.337.search-card.all.click)，看视频学软件操作更直观。我主要是用 RTKLIB 的代码库，自带的程序除了 rtkplot 之外我都没咋用过，下面写的内容是看赵老师视频的时候做的笔记。

### 1、常见后处理操作

* 在进行解算前就可以用 RTKPLOT 可以对数据进行分析，最新的 b34 版本的 RTKPLOT 好像有 bug，可以去下载一个别的版本的。

* 通过 RTKGET 可以下载 IGS 观测数据和各种改正产品（我没咋用过）。





### 2、后处理数据获取



### 3、RTKGET 数据下载

> 视频里的下载地址有些不能用了，可以参考[GNSS观测数据及各种产品下载网址分享](https://blog.csdn.net/qq_38607471/article/details/129952202)

1. 观测值下载

   * 选择下载数据的时间：起始时间，结束时间

   * options 设置 URL_LIST，可以用 RTKLIB 默认配置，选择 rtklib 中 data 文件夹下 URL_LIST.txt 文件（我在 bin 版的 rtklib 里没找到，用的源码版的 rtklib 里的文件），加载进来，左边就有了两列内容。

   * OBS 为观测值文件，NAV 为导航电文，EPH 为精密轨道，CLK 精密钟差、ATX 天线文件

   * 做相对定位要下观测值
     * 先选择分析中心，IGS、MGES 等。

     * 后在右边选测站，点...把测站加上去，ALIC、KARR

     * 把测站名点选，再点 Download，理论上就可把数据下到指定目录，但会比较慢

       * 可复制 FTP 路径直接进网页下载

       * 要在 Linux 下大量处理，可写脚本

   ![bf7b9a35f8b2c5f149215023bfd25831](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/bf7b9a35f8b2c5f149215023bfd25831.png)

2. 用 RTKGET 做时间转换：输入年月日时分秒，点问号 ？，就可看各种时间，下载观测值需要年积日 DOY，改链接的日期就可下载对应的观测值文件

   ![image-20231025204151284](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025204151284.png)

3. 数据命名格式：**测站名（4位）+机构信息+年+年积日+采样间隔**，crx 是压缩格式，gz 也是压缩格式，还有一种是 o.z 结尾只进行一次压缩

4. 广播星历文件和精密星历文件：也可用 rtkget 和 ftp下载

### 3、RTKCONV 数据转换

1. 为啥要介绍此模块：用接收机采集的数据一般是二进制文件（RTCM或者接收机原始数据格式），转为 RINEX 方便看到里面具体内容，方便进行后处理解算。

2. ublox 通过串口导出的二进制文件，COM3 开头，.ublox 结尾，除了原始观测数据之外还有 NMEA 数据。通过 notepad++ 打开查看，开头乱码是二进制数据，后面是 NMEA 文本格式。

3. ublox 数据还可用 ucenter 接收机配置软件查看

4. RTKCONV 使用

   * 先选择需要转换数据的起止时间，采样率 Interval。
   * 输入原始数据地址。
   * 选格式，u-blox、RINEX、RTCM3...，不知道格式可选自动 Auto。
   * 勾选选输出数据，一般得要obs观测数据，如果实时数据从网上不能在网上下导航电文，需要转换出的 nav 文件。
   * 配置信息：RINEX 版本号、测站 ID 可以不写、RunBy 可以写自己、天线类型接收机类型有需要可以写，近视坐标，加哪些改正信息，输出哪些系统、观测值类型可都勾上，观测频率，信号通道。
   * 点 Convert 转换。 

   ![19792c3d48403c733efcf69677e4863b](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/19792c3d48403c733efcf69677e4863b.png)

   * 点 Plot 可以直观展示卫星数据质量
     * Sat Vis：卫星可见性，选频率，颜色代表信噪比 SNR。
     * Skyplot：卫星天空视图，站心地平级坐标系，可看出低高度角卫星信号差
     * DOP：上面是可视卫星数，下面是 DOP 值
     * SNR：载噪比、多路径，可选某一颗卫星指定频率，横坐标可选时间、高度角

### 4、RTKPOST 数据后处理

1. 主界面

   * 设置解算起止时间，解算间隔
   * 加载 RINEX OBS 数据：Rover 流动站、Base 基准站，右上角点天空图标开 RTKPLOT 看数据状态。基准站整天的数据非常大，截取流动站对应部分即可。
   * 加载其它数据：NAV、CLK、SP3 等。每个接收机输出的 NAV 只有它能观测到的卫星星历，从网上可下全部所有卫星所有系统的导航电文。
   * 输出默认在流动站文件路径，后缀为 .pos。 

   ![image-20231026163551141](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231026163551141.png)

2. Options设置

   * **定位模式**

     * Single：伪距单点定位
     * DGPS/DGNSS：伪距差分
     * Kinematic：载波动态相对定位，动态RTK，假设流动站是移动的，可以做车载定位
     * Static：载波静态相对定位，静态RTK，两站都是静止的，可以得到很高的精度
     * Static-Start：（demo5 才有）冷启动：先在比较开阔的地方，进行短时期的静态定位，模糊度固定，再动起来
     * Moving-Base：两站都动，主要用来定姿
     * Fixed：固定坐标，解算模糊度、对流层、电离层等参数
     * PPP Kinematic、PPP-Static、PPP Fixed

   * **频率**：可选不同频率组合，如L1+L2

   * **滤波**：前向（后面结果更可靠）、后向（可使刚开始的时候有高的精度）、Combind（正向一个结果，反向一个结果，根据方差加权平均），RTKLIB里除了SPP都用卡尔曼滤波，滤波有一个收敛的过程，后面更准。

   * **设置截止高度角**：可以看天空视图，如果低高度角数据很差，可设置更高的截止高度角。质量好可以不管，有残差检验也可剔除一些数据。

   * **设置截止信噪比**：做工程一般环境都不会很好，想做的序列稳定，要设置截止信噪比。RTK有很多算法，但其实传统算法效果已经很好了，算法不用做的太复杂，把数据质量控制做好就行，RTK就不会有太大的问题。

   * **Rec Dynamics**：动力学模式，选 ON 会估计速度加速度参数，，选 OFF 就只估算动态坐标参数

     * Kinematic 动态模式：把位置参数当白噪声估计。
     * Dynamics 动力学模型：建立 CA 模型，同时估计位置、速度、加速度。

   * **RCV、潮汐改正等**：PPP 才用的到

   * **电离层、对流层改正**：双差已经可以消除部分电离层对流层误差，可以关闭此改正，也可以直接采用广播星历的模型改正。RTKLIB 做 RTK 最好用非组合模式，短基线电离层可以关闭，对流层可以用 saastamoinen 模型直接修正。

   * **卫星星历**：RTK 相对定位距离近可以直接用广播星历，长距离相对定位可选精密星历。

     * SSR APC：参考天线相位中心
     * SSR CoM：参考质心，还需要天线相位中心改正

   * **剔除卫星**：写卫星号，空格隔开，如：C01 C02

   * **RAIM FDE完好性检验**：算法不是很稳健，不选

   * **模糊度固定模式 ARMODE**

     * **OFF**：浮点解，不固定

     * **Continues**：认为模糊度是连续解，通过前面历元的解算结果滤波提高后续历元模糊度固定精度。

     * **Instantaneous**：瞬时模糊度固定，单历元模糊度固定，每个历元都初始化一个参数，这个历元和上个历元模糊度不相关。

     * **Fix and Hold**：先 Continues，在不发生周跳情况下都采用之前模糊度固定的结果作为约束，也有问题：固定错了，时间序列会一直飘，到一定程度变成浮点解，会重置模糊度重新算。

       > 做工程可做两套，Instantaneous 和 Fix and Hold，发现 Fix and Hold 错了，就用 Instantaneous 的解把它替换掉，相当于把模糊度和方差初始化了一次，避免漂移和模糊度重新收敛的过程。

     * **PPP-AR**：PPP 时固定模糊度，不支持，需要额外产品。

   * **Ratio值**：用于检验模糊度是否固定成功，设为 3 即可。

   * **最小LOCK**：连续锁定这颗卫星几次，才用于计算模糊度固定。

   * **用于模糊度固定的最低高度角设置**：可设 15°

   * **最小Fix**：这个历元最少固定多少个模糊度才认为模糊度是固定的，可设 10，现在卫星系统多了，而且组合模式，双频一颗卫星就 2 个模糊度，5 颗卫星固定就能凑 10 个。

   * **Fix hold**：选择哪些模糊度固定结果用于约束后续。

   * **输出结果**：可选  LLH、XYZ、ENU、NMEA

   * **输出解算状态**：可选 OFF、Residuals 残差、State

   * **Debug Trace等级**：1-5级，level 越高输出越多

   * **基准站坐标**：可输入、也可选伪距单点定位

   * **天线类型**：选*，自动获取 O 文件里的

3. 算完之后

   * **Plot**：对解算结果可视化分析，黄色没固定，绿色固定
   * **view**：查看解算结果，类似记事本
   * **KML**：转为 GoogleXML 可把地图展示到地图上

> 建议：下静态数据，找动态车载数据，分别处理静态相对定位和处理动态相对定位，设置不同处理模式，分析定位结果的差异。

4. PPP 数据处理
   * 实时PPP：IGS/MGEX 分析中心播发的实时卫星轨道和钟差产品，结合广播星历
   * 事后或近实时：下载精密星历、钟差产品，结合其它精密改正信息实现定位
   * RTKLIB 使用必须给广播星历，因为解算前都会先进行一次伪距单点定位





## 二、后处理解算

### 1、使用 rnx2rtkp 后处理解算

rnx2rtkp 全称 RINEX to RTK pos，通过原始 RINEX 文件，输出 RTKLIB 的定位坐标，下图可以很好的表示整个程序的逻辑：

![image-20231016123911526](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016123911526.png)

#### 1. 使用方式

* 使用方式：`rnx2rtkp [option]... file file [...] `

* 读取 RINEX：OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。

* 对于相对定位，第一个 OBS 观测值文件需含接收机、流动站观测值，第二个 OBS 文件需含基准站观测值。

* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。

* 想用 SP3 精密星历文件，需提供 .sp3/.eph 文件的路径。

* 输入文件路径可包含通配符 *，为了防止与命令行命令冲突，要用 `"..."`  括起带通配符符路径。

* 输命令行参数的几种方式：
  * **VS**：
  * **VScode**：
  * **Clion**：
  * **Windows 终端**：
  * **Linux 终端**：

#### 2. 命令行参数

1. **-？**：打印 help
2. **-k** file：配置文件的输入选项，默认值是 [off]
3. **-o** file：输出文件选项，默认值是 [stdout]
4. **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是 [obs start time] 
5. **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是 [obs end time] 
6. **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
7. **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是 [2]
8. **-m** mask：设置截止高度角，`(deg) `,默认值是 [15]
9. **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是 [G|R] ，想用除 GPS 以外的系统，还得加宏定义 ENAGLO、ENACMP、ENAGAL
10. **-f** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是 [2]
11. **-v** thres：设置整周模糊度 Ratio 值，写 0.0 为不固定整周模糊度，默认值是 [3.0] 
12. **-b**：后向滤波
13. **-c**：前后向滤波组合
14. **-i**：单历元模糊度固定 instantaneous 
15. **-h**：fix and hold 模糊度固定
16. **-e**：输出 XYZ-ecef 坐标
17. **-a**：输出 ENU-baseline
18. **-n**：输出 NMEA-0183 GGA
19. **-g**：输出经纬度格式为 ddd mm ss.ss ，默认为 [ddd.ddd] 
20. **-t**：输出时间格式为 yyyy/mm/dd hh:mm:ss.ss ，默认为 [sssss.ss] 
21. **-u**：输出为 UTC 时间，默认为 [gpst] 
22. **-d** col：设置时间的小数位数，默认为 [3] 
23. **-s** sep：设置文件分隔符，要写在单引号中，默认为 [' '] 
24. **-r** x y z：基站位置 ECEF-XYZ (m)，默认 [average of single pos] ，流动站位置用于 fixed 模式
25. **-l** lat lon hgt：基站位置 LLH (deg/m)，默认 [average of single pos]，流动站位置用于 fixed模式
26. **-y** level：输出结果信息 (**0**:off,**1**:states,**2**:residuals) ，默认为 [0] 
27. **-x** level：输出 debug trace 等级，默认为 [0] 

#### 3. 数据下载





RTKGET 可以批量下载数据，本文下一章会介绍。

#### 4. 错误解决

rnx2rtkp 的命令行参数很复杂，一不下心就会出错，这时候可以去看 trace 文件，重点看出现 error 的部分，复制 error 信息的前半部分，去程序里搜索，定位到出现错误的地方，在附近设几个断点，看看到的是咋错的。

比如我的朋友按照吴桐的[博客](https://zhuanlan.zhihu.com/p/528855325)运行 rnx2rtkp，出错了来问我：

![image-20231015211515575](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231015211515575.png)

* 首先看左边的 trace 输出，有两行里显示 error，说是打开观测文件和星历文件时候出错了。

* 复制前面的 ”rinex file error“，在代码中全局搜索，发现只有在 `readrnxfile()` 函数里有可能输出这段信息。

* 可以明显看出来是 `fopen()` 根据路径打开文件的时刻出错了，那就在 `fopen()` 的那一行设断点，看看打开文件路径到的是什么。

* 最后调试发现，路径中 `\` 都是连续出现四个。推测分析是因为朋友的 VS 版本，输完的命令行参数，会自动把其中的 `\` 换成 `\\`，自己输文件路径的时候只要一个 `\` 就行了。

  ```
  -x 5 -p 0 -m 15 -n -o D:\\source\\RTKLIB-rtklib_2.4.3\\rtklib\\out.pos D:\\source\\RTKLIB-rtklib_2.4.3\\test\\data\\rinex\\07590920.05o D:\\source\\RTKLIB-rtklib_2.4.3\\test\\data\\rinex\\07590920.05n
  ```

  要改成：

  ```
  -x 5 -p 0 -m 15 -n -o D:\source\RTKLIB-rtklib_2.4.3\rtklib\out.pos D:\source\RTKLIB-rtklib_2.4.3\test\data\rinex\07590920.05o D:\source\RTKLIB-rtklib_2.4.3\test\data\rinex\07590920.05n
  ```

再比如，程序运行结束看不到结果输出，可以在 `outhead()` 下面位置设断点，看看 `outfile` 变量里存没存文件路径，那个位置有没有输出了文件头的结果文件。

![image-20231028150122875](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028150122875.png)

### 2、rnx2rtkp 主函数

在文章的最后，对 rnx2rtkp 的主函数做一个介绍：

* 读取配置文件过程：
  * 循环判断参数是否有 `-k`，如果有就代表传入了配置文件，需要读取进来
  * 创建 `porcopt_t`、`solopt_t`、`filopt_t` 变量用于接受读取到的配置
  * 调用 `resetsysopts()` ，重置配置为默认值。
  * 调用 `loadopts()`，从文件中读取配置，存到 `opt_t` 类型的 `sysopt` 中。
  * 调用 `getsysopts()`，把 `opt_t` 类型的 `sysopt` 转到 `porcopt_t`、`solopt_t`、`filopt_t` 变量中，会调用 `buff2sysopts()`。
* 读其它参数：
  * 循环内，`if-else if`，判断参数，根据参数赋值
  * 若都不是参数，最后一个到 `else if`，认为是文件路径，用 `infile` 数组接收
* 最后调用 `postpos()` 后处理解算

我根据自己的理解给代码加了中文注释，代码如下：

```c
int main(int argc, char **argv)
{
    prcopt_t prcopt=prcopt_default;     // 定位处理模式
    solopt_t solopt=solopt_default;     // 结果输出形式
    filopt_t filopt={""};               // 文件路径选项
    gtime_t ts={0},te={0};              // ts开始时间、te结束时间
    double tint=0.0,
            es[]={2000,1,1,0,0,0},
            ee[]={2000,12,31,23,59,59},
            pos[3];
    int i,      // for循环的计数
        j,      // 嵌套的for循环计数
        n,      // 记录读入文件数
        ret;    // 接受postpos的返回值
    char *infile[MAXFILE],  // 读入文件，默认最多16个，可改MAXFILE定义
         *outfile="",       // 输出文件
         *p;                // 指向字符串的指针，用于循环指向各main函数参数
    
    prcopt.mode  =PMODE_KINEMA;     // 定位模式默认动态相对定位Kinematic
    prcopt.navsys=0;                // 卫星系统，先设置无
    prcopt.refpos=1;                // 基准站坐标，先设为由SPP平均解得到
    prcopt.glomodear=1;             // GLONASS AR mode,先设on
    solopt.timef=0;                 // 输出时间格式，先设sssss.s
    sprintf(solopt.prog ,"%s ver.%s %s",PROGNAME,VER_RTKLIB,PATCH_LEVEL);   // 项目名称
    sprintf(filopt.trace,"%s.trace",PROGNAME);
    
    /* load options from configuration file */
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-k")&&i+1<argc) {          // 如果有-k和配置文件输入
            resetsysopts();                             // 先重置所有配置
            if (!loadopts(argv[++i],sysopts)) return -1;// 再读取配置文件内容，存入opt_t的sysopt中
            getsysopts(&prcopt,&solopt,&filopt);        // opt_t转到porcopt_t/solopt_t/filopt_t，
        }   
    }          
    // for 循环判断 main 函数参数
    for (i=1,n=0;i<argc;i++) {  
        if      (!strcmp(argv[i],"-o")&&i+1<argc) outfile=argv[++i];//读取输出文件路径，赋值给outfile
        else if (!strcmp(argv[i],"-ts")&&i+2<argc) {    // 读取开始解算时间   
            sscanf(argv[++i],"%lf/%lf/%lf",es,es+1,es+2);
            sscanf(argv[++i],"%lf:%lf:%lf",es+3,es+4,es+5);
            ts=epoch2time(es);      // 转为gtime_t
        }
        else if (!strcmp(argv[i],"-te")&&i+2<argc) {    // 读取结束解算时间
            sscanf(argv[++i],"%lf/%lf/%lf",ee,ee+1,ee+2);   
            sscanf(argv[++i],"%lf:%lf:%lf",ee+3,ee+4,ee+5);
            te=epoch2time(ee);  // 转为gtime_t
        }
        else if (!strcmp(argv[i],"-ti")&&i+1<argc) tint=atof(argv[++i]);        // 读取解算时间间隔频率
        else if (!strcmp(argv[i],"-k")&&i+1<argc) {++i; continue;}              // 有-k，跳过
        else if (!strcmp(argv[i],"-p")&&i+1<argc) prcopt.mode=atoi(argv[++i]);  // 读取解算模式
        else if (!strcmp(argv[i],"-f")&&i+1<argc) prcopt.nf=atoi(argv[++i]);    // 读取用于计算的频率
        else if (!strcmp(argv[i],"-sys")&&i+1<argc) {       // 读取用于计算的导航系统
            for (p=argv[++i];*p;p++) {      
                switch (*p) {                           //有对应导航系统，就把它的码做与运算加上                 
                    case 'G': prcopt.navsys|=SYS_GPS;
                    case 'R': prcopt.navsys|=SYS_GLO;
                    case 'E': prcopt.navsys|=SYS_GAL;
                    case 'J': prcopt.navsys|=SYS_QZS;
                    case 'C': prcopt.navsys|=SYS_CMP;
                    case 'I': prcopt.navsys|=SYS_IRN;
                }
                if (!(p=strchr(p,','))) break;  
            }
        }
        else if (!strcmp(argv[i],"-m")&&i+1<argc) prcopt.elmin=atof(argv[++i])*D2R;     // 设置截止高度角     
        else if (!strcmp(argv[i],"-v")&&i+1<argc) prcopt.thresar[0]=atof(argv[++i]);    // 设置整周模糊度Ratio值
        else if (!strcmp(argv[i],"-s")&&i+1<argc) strcpy(solopt.sep,argv[++i]);         // 设置文件路径分隔符
        else if (!strcmp(argv[i],"-d")&&i+1<argc) solopt.timeu=atoi(argv[++i]);         // 设置时间小数位数
        else if (!strcmp(argv[i],"-b")) prcopt.soltype=1;           // 后向滤波
        else if (!strcmp(argv[i],"-c")) prcopt.soltype=2;           // 前后向滤波组合
        else if (!strcmp(argv[i],"-i")) prcopt.modear=2;            // 单历元模糊度固定
        else if (!strcmp(argv[i],"-h")) prcopt.modear=3;            // fix and hold 模糊度固定
        else if (!strcmp(argv[i],"-t")) solopt.timef=1;             // 输出时间格式为 yyyy/mm/dd hh:mm:ss.ss
        else if (!strcmp(argv[i],"-u")) solopt.times=TIMES_UTC;     // 输出为 UTC 时间
        else if (!strcmp(argv[i],"-e")) solopt.posf=SOLF_XYZ;       // 输出 XYZ-ecef 坐标
        else if (!strcmp(argv[i],"-a")) solopt.posf=SOLF_ENU;       // 输出 ENU-baseline
        else if (!strcmp(argv[i],"-n")) solopt.posf=SOLF_NMEA;      // 输出 NMEA-0183 GGA
        else if (!strcmp(argv[i],"-g")) solopt.degf=1;              // 输出经纬度格式为 ddd mm ss.ss
        else if (!strcmp(argv[i],"-r")&&i+3<argc) {                 // 基站位置E CEF-XYZ (m)              
            prcopt.refpos=prcopt.rovpos=0;                  // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) prcopt.rb[j]=atof(argv[++i]); // 循环存入基准站坐标
            matcpy(prcopt.ru,prcopt.rb,3,1);    
        }
        else if (!strcmp(argv[i],"-l")&&i+3<argc) {     // 循环存入基站位置基站位置LLH (deg/m)
            prcopt.refpos=prcopt.rovpos=0;              // 基准站和流动站位置都先设0
            for (j=0;j<3;j++) pos[j]=atof(argv[++i]);   
            for (j=0;j<2;j++) pos[j]*=D2R;              // 角度转弧度   
            pos2ecef(pos,prcopt.rb);                    // LLH 转 XYZ
            matcpy(prcopt.ru,prcopt.rb,3,1);
        }
        else if (!strcmp(argv[i],"-y")&&i+1<argc) solopt.sstat=atoi(argv[++i]); //输出结果信息
        else if (!strcmp(argv[i],"-x")&&i+1<argc) solopt.trace=atoi(argv[++i]); //输出debug trace等级
        else if (*argv[i]=='-') printhelp();        //输入-，打印帮助
        else if (n<MAXFILE) infile[n++]=argv[i];    //循环判断完一遍参数之后，认为参数是文件路径，用infile数组接收
    }
    if (!prcopt.navsys) {               //如果没设卫星系统，默认为GPS、GLONASS
        prcopt.navsys=SYS_GPS|SYS_GLO;
    }
    if (n<=0) {         //如果读入文件数为0,报错，-2退出
        showmsg("error : no input file");
        return -2;
    }

    
    //   gtime_t ts       I   processing start time (ts.time==0: no limit)
    //   gtime_t te       I   processing end time   (te.time==0: no limit)
    //   double ti        I   processing interval  (s) (0:all)
    //   double tu        I   processing unit time (s) (0:all)
    //   prcopt_t *popt   I   processing options
    //   solopt_t *sopt   I   solution options
    //   filopt_t *fopt   I   file options
    //   char   **infile  I   input files (see below)
    //   int    n         I   number of input files
    //   char   *outfile  I   output file ("":stdout, see below)
    //   char   *rov      I   rover id list        (separated by " ")
    //   char   *base     I   base station id list (separated by " ")
    //后处理定位解算
    ret=postpos(ts,te,tint,0.0,&prcopt,&solopt,&filopt,infile,n,outfile,"",""); 
    
    if (!ret) fprintf(stderr,"%40s\r","");
    return ret;
}
```

### 3、自己写后处理主函数

> 参考：https://www.cnblogs.com/ahulh/p/15584998.html

rnx2rtkp 程序比较复杂，要传入很多命令行参数，可以自己写主函数。再比如说要写个松组合程序，可以先调用 `postpos()` 通过GNSS原始原始数据算出定位解，然后与 INS 组合。

rtklib 后处理主函数需要做的主要就是：读取配置文件（可省）、设置处理选项、传入文件路径、调用 postpos()；下面我将分别对这几部分的写法做介绍：

#### 1. 实现用户自定义函数 showmsg、settspan、settime

* **showmsg**：
* **settspan**：
* **settime**：

#### 2. 配置文件读取



读取配置文件流程：

* 先调用 `resetsysopts()` 重置所有配置为默认。
* 调用 `loadopts()` 读取配置文件内容，存入 `opt_t` 的 `sysopt` 中。
* 最后调用 `getsysopts()` 将 `opt_t` 转到 `porcopt_t`/`solopt_t`/`filopt_t`。



#### 3. postpos() 参数

先来看看需要传入的参数：

```c
gtime_t ts       I   处理的起始时间，写0表示不限制
gtime_t te       I   处理的起始时间，写0表示不限制
double ti        I   处理的间隔时间 (s)，写0表示不限制，全处理
double tu        I   处理的单元时间（s)，写0表示全部做一个单元处理
prcopt_t *popt   I   处理选项结构体
solopt_t *sopt   I   结果选项结构体
filopt_t *fopt   I   文件选项结构体
char   **infile  I   传入文件路径数组首地址
int    n         I   传入文件数量
char   *outfile  I   输出文件的路径，写0表示stdout终端
char   *rov      I   流动站ID列表，空格隔开
char   *base     I   基准站ID列表，空格隔开
```

* `ts`、`te`、`ti`、`tu`：这几个参数用于设置解算时间，可以都填 0，让程序把传入的观测文件全解算了。
* `infile`、`n`：
* `outfile`：
* `rov`、`base`：
* `popt`、`sopt`、`fopt`：这几个选项结构体是设置的重点
  * `filopt_t`：**文件选项**，存结果输出、Trace、各种改正文件路径，不包括星历文件和观测文件。
  * `solopt_t`：**结果选项**，可以设置结果输出形式（ENU、ECEF、NMEA、BLH），小数位数，是否输出文件头，是否输出速度等。
  * `prcopt_t`：**处理选项**，是配置的重头戏，可以先看 [postpos 的用法](https://www.bilibili.com/video/BV1m5411Y7xV) 学习界面程序的配置方式。写代码配置和界面程序需要配置的东西是一样的，只是从在界面上点，换成在了代码里给对应字段赋值。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB%25E9%2585%258D%25E7%25BD%25AE%25E9%2580%2589%25E9%25A1%25B9.png)

## 三、三种输出文件

了解三种输出文件具体内容、默认路径、命名方式，

看懂文件在哪个函数创建，哪个函数输出，

知道怎么去拓展，输出些我们想要的其它信息



后处理过程中，三种输出文件都在 execses() 函数中创建：

![image-20240222200316982](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240222200316982.png)

Trace 信息都是直接输出，结果和中间结果先存在` rtk->sol`、`rtk->ssat` 结构体中，然后每个历元输出一次，







### 1、Trace文件输出





### 2、结果文件输出



，procpos() 函数 while循环中每次解算完调用 outsol() 输出结果，



### 3、中间结果文件







## 四、从 postpos 到 rtkpos 的函数调用过程



* **postpos()**：按解算时间分开处理，替换输入文件时间通配符，读取天线参数文件、大地水准面文件。
* **execses_b()**：分基准站进行定位解算，替换输入文件基准站ID通配符。

* **execses_r()**：对每个流动站进行定位解算，替换输入文件流动站ID通配符。
* **execses()**：创建三种输出文件（Trace、结果文件、中间结果文件），读取 TEC、ERP、OBS、NAV、DCB、PCV、BLQ文件，计算基准站坐标，根据滤波解算顺序配置参数调用 procpos() 解算，释放读取的观测数据和星历数据。
* **procpos()**：先初始化` rtk_t`，然后进入 while 循环，每次循环都通过 `inputobs ()` 函数按滤波顺序读取一个历元的数据，调用 `satsys()` 排除禁用卫星，并调用 `rtkpos()` 函数对该历元的数据进行解算，调用 `outsol()` 输出解释结果。
* 

![请添加图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/25a7656c3b0148f0ba14f6365027cf0e.png)
![请添加图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/c782474785b24b44ab4de4fbd8999af9.png)

### 1、postpos()

#### 1. 功能

后处理定位的主入口函数，根据tu分计算时间段，调用调用`execses_b()`进行下一步解算

输入文件包括观测文件、导航文件、精密星历文件等，postpos在处理输入文件时有两种方法，一种是输入文件可以只包含替换，然后通过函数`reppath()`处理，将关键词用时间、基准站编号、流动站编号等代替，另一种是直接调用输入文件的文件名，postpos主要是来判断是哪一种输入方式，然后调用相应函数。

#### 2. 输入参数：

```c
gtime_t ts       I   处理的起始时间，写0表示不限制
gtime_t te       I   处理的起始时间，写0表示不限制
double ti        I   处理的间隔时间 (s)，写0表示不限制，全处理
double tu        I   处理的单元时间（s)，写0表示全部做一个单元处理
prcopt_t *popt   I   处理选项结构体
solopt_t *sopt   I   结果选项结构体
filopt_t *fopt   I   文件选项结构体
char   **infile  I   传入文件路径数组首地址
int    n         I   传入文件数量
char   *outfile  I   输出文件的路径，写0表示stdout终端
char   *rov      I   流动站ID列表，空格隔开
char   *base     I   基准站ID列表，空格隔开
```

#### 3. 返回值

* 处理一切正常会接收execxes_b()的返回值，失败返回0，内存失败返回-1
* execses_b()正常会接收execses_b()的返回值，失败返回0
* execses_r()正常会接收execses()的返回值，失败返回0，aborts返回1

#### 4. 执行流程

1. 变量定义，`stat`默认为0，`flag`默认为1。

2. 调用`openses()`，开始解算进程，读取天线、大地水准面文件。

3. 判断起始解算时间`ts`、结束解算时间`te`、解算时间单元`tu`，有三种情况：

   > * 为何要判断：拆分时间段解算需要tu值有效、调用reppath需要ts有效，调用reppaths需要ts和te有效。
   > * ifile[]、ofile[]作用：infile[]、ofile[]里的路径替换处理后存到ifile[]、ofile[]，传入`execses_b()`进行之后的解算。
   > * index[]的作用：会传给`execses_b()`，再传给`execses_r()`，再传给`execses()`，再传给`readobsnav() `。如果不需要根据tu分时间段解算，index存的就是0~n，如果需要分时间段解算，index存的是对应时间段内文件的下标。

   ①：**若`ts`、`te`不为0，`tu`大于等于0**：

   * 判断`te`早于`ts`，return
   * 为`ifile[]`数组空间
   * 处理解算时间单元`tu`，0或者时间大于100天，设为100天 
   * 循环处理每个时间单元`tts`到`tte`：
     * 计算解算时间单元的开始`tts`、结束`tte`，判断`tts<ts`则设为`ts`，`tte>te`设为`te`
       * 流动站、基准站名赋空值								

     * 遍历遍历infile[]，`strrchr`找文件后缀名，`strcmp`判断后缀名 ：

       * rtcm3：直接把`infile[j]`中路径赋值到`ifile[]`中
       * 星历文件：精密星历`ttte=tte+一小时`、广播星历`ttte=tte+两小时`，根据`tts`、`ttte`调用`reppaths()`将infile[j]中路径展开到`ifile[nf]`中。

       之后把`infile[]`的下标`j`存到`index[]`中。

     * 调用`reppath()`替换`outfile`的替换符，存到ofile中。

     * 调用`execses_b()`进行下一步解算。

   ②：**若`ts`不为0，`tu`为0或小于0** ：就不考虑`te`、和`tu`

   * 为`ifile[]`开辟空间，循环替换`infile[i]`的替换符到`ifile[i]`中。
   * 调用`reppath`替换outfile的替换符，存到ofile中。
   * 调用`execses_b()`进行下一步解算。

   ③：**若`ts`为0**：直接把把`infile[]`的下标`j`存到`index[]`中，调用`execses_b` 进行下一步解算

4. 调用`closeses()`，释放`openses()`开辟的内存。

```c
extern int postpos(gtime_t ts, gtime_t te, double ti, double tu,
                   const prcopt_t *popt, const solopt_t *sopt,
                   const filopt_t *fopt, char **infile, int n, char *outfile,
                   const char *rov, const char *base)
{
    gtime_t tts,    //解算单元的开始时间
            tte,    //解算单元的结束时间
            ttte;   //读取星历文件的结束时间
    double tunit,   //
           tss;     //
    int i,j,k,      //循环和数组下标控制
            nf,     //文件路径数组下标控制
        stat=0,     //接收返回状态值，为1
        week,       //用于存GPST的周
        flag=1,
        index[MAXINFILE]={0};
    char *ifile[MAXINFILE],
          ofile[1024],
          *ext;
    
    trace(3,"postpos : ti=%.0f tu=%.0f n=%d outfile=%s\n",ti,tu,n,outfile);
    
    /* open processing session */   //开始处理,文件读取，赋值navs、pcvs、pcvsr
    if (!openses(popt,sopt,fopt,&navs,&pcvss,&pcvsr)) return -1;
    
    if (ts.time!=0&&te.time!=0&&tu>=0.0) {  //判断起始时间ts、te、处理单位时间是否大于0
        if (timediff(te,ts)<0.0) {  //结束时间早于开始时间
            showmsg("error : no period");
            closeses(&navs,&pcvss,&pcvsr);  //不合理则关闭处理，释放navs、pcvs、pcvsr
            return 0;
        }
        for (i=0;i<MAXINFILE;i++) {
            if (!(ifile[i]=(char *)malloc(1024))) { //为infile数组malloc开辟空间
                for (;i>=0;i--) free(ifile[i]);     //开辟失败则释放已开辟的空间，关闭处理释放navs、pcvs、pcvsr
                closeses(&navs,&pcvss,&pcvsr);
                return -1;
            }
        }
        if (tu==0.0||tu>86400.0*MAXPRCDAYS) tu=86400.0*MAXPRCDAYS;  //解算处理时间单元处理，0或者时间大于100天，设为100天
        settspan(ts,te);    //设置时间跨度，好像是空函数，需要自己实现
        
        tunit=tu<86400.0?tu:86400.0;    //tunit：如果tu小于一天就为tu；否则为一天
        tss=tunit*(int)floor(time2gpst(ts,&week)/tunit);   //
        
        //根据解算时间单元，分时间段循环处理，算出来tts>te或过程有错误，结束循环
        //很多时候解算单元时间直接设0.0，只循环一次，tts=ts，tte=te
        for (i=0;;i++) { /* for each periods */
            tts=gpst2time(week,tss+i*tu);       //解算单元开始时间，每次循环加上一个i个tu？
            tte=timeadd(tts,tu-DTTOL);          //解算结束时间tte=tu-DTTOL
            if (timediff(tts,te)>0.0) break;   //算出来tts>te结束循环
            if (timediff(tts,ts)<0.0) tts=ts;   //分时间段后tts若早于ts，设为ts
            if (timediff(tte,te)>0.0) tte=te;   //分时间段后tte若早于te，设为te
            
            strcpy(proc_rov ,"");   //流动站、基准站值赋空
            strcpy(proc_base,"");   
            if (checkbrk("reading    : %s",time_str(tts,0))) {
                stat=1;
                break;
            }
            for (j=k=nf=0;j<n;j++) {    //遍历infile[]，根据后缀名
                
                ext=strrchr(infile[j],'.'); //ext：文件路径中.后缀开始的位置
                
                if (ext&&(!strcmp(ext,".rtcm3")||!strcmp(ext,".RTCM3"))) {  //rtcm3文件
                    strcpy(ifile[nf++],infile[j]);
                }
                else {      //星历文件，包括精密星历和广播星历
                    /* include next day precise ephemeris or rinex brdc nav */
                    ttte=tte;
                    if (ext&&(!strcmp(ext,".sp3")||!strcmp(ext,".SP3")||
                              !strcmp(ext,".eph")||!strcmp(ext,".EPH"))) {
                        ttte=timeadd(ttte,3600.0);  //精密星历加一小时
                    }
                    else if (strstr(infile[j],"brdc")) {
                        ttte=timeadd(ttte,7200.0);  //广播星历加两小时
                    }
                    nf+=reppaths(infile[j],ifile+nf,MAXINFILE-nf,tts,ttte,"","");
                }
                while (k<nf) index[k++]=j;
                
                if (nf>=MAXINFILE) {
                    trace(2,"too many input files. trancated\n");
                    break;
                }
            }
            if (!reppath(outfile,ofile,tts,"","")&&i>0) flag=0;
            
            /* execute processing session */
            stat=execses_b(tts,tte,ti,popt,sopt,fopt,flag,ifile,index,nf,ofile,
                           rov,base);
            
            if (stat==1) break;
        }

        for (i=0;i<MAXINFILE;i++) free(ifile[i]);
    }
    else if (ts.time!=0) {  //如果起始时间不为0，结束时间为0或处理单元时间小于0
        for (i=0;i<n&&i<MAXINFILE;i++) {
            if (!(ifile[i]=(char *)malloc(1024))) {
                for (;i>=0;i--) free(ifile[i]);
                return -1;
            }
            reppath(infile[i],ifile[i],ts,"","");
            index[i]=i;
        }
        reppath(outfile,ofile,ts,"","");
        
        /* execute processing session */
        stat=execses_b(ts,te,ti,popt,sopt,fopt,1,ifile,index,n,ofile,rov,
                       base);
        
        for (i=0;i<n&&i<MAXINFILE;i++) free(ifile[i]);
    }
    else {  //如果起始时间为0
        for (i=0;i<n;i++) index[i]=i;
        
        /* execute processing session */
        stat=execses_b(ts,te,ti,popt,sopt,fopt,1,infile,index,n,outfile,rov,
                       base);
    }
    /* close processing session */
    closeses(&navs,&pcvss,&pcvsr);
    
    return stat;
}
```



#### 5. 调用的函数

* **openses()**：开始解算进程，读取天线、大地水准面文件

  * readpcv()：读取天线文件，会调用readantex()、readngspcv()
  * opengeoid()：读取geoid文件，会调用closegeoid()

  ```c
  static int openses(const prcopt_t *popt, const solopt_t *sopt,
                     const filopt_t *fopt, nav_t *nav, pcvs_t *pcvs, pcvs_t *pcvr)
  {
      trace(3,"openses :\n");
      
      /* read satellite antenna parameters */
      if (*fopt->satantp&&!(readpcv(fopt->satantp,pcvs))) {
          showmsg("error : no sat ant pcv in %s",fopt->satantp);
          trace(1,"sat antenna pcv read error: %s\n",fopt->satantp);
          return 0;
      }
      /* read receiver antenna parameters */
      if (*fopt->rcvantp&&!(readpcv(fopt->rcvantp,pcvr))) {
          showmsg("error : no rec ant pcv in %s",fopt->rcvantp);
          trace(1,"rec antenna pcv read error: %s\n",fopt->rcvantp);
          return 0;
      }
      /* open geoid data */
      if (sopt->geoid>0&&*fopt->geoid) {
          if (!opengeoid(sopt->geoid,fopt->geoid)) {
              showmsg("error : no geoid data %s",fopt->geoid);
              trace(2,"no geoid data %s\n",fopt->geoid);
          }
      }
      return 1;
  }
  ```

  

* **closeses()**：结束解算程序，释放天线、geoid、erp、trace、fp_stat 。会调用closegeoid() 、rtkclosestat() 、traceclose() 。

  ```c
  static void closeses(nav_t *nav, pcvs_t *pcvs, pcvs_t *pcvr)
  {
      trace(3,"closeses:\n");
      
      /* free antenna parameters */
      free(pcvs->pcv); pcvs->pcv=NULL; pcvs->n=pcvs->nmax=0;
      free(pcvr->pcv); pcvr->pcv=NULL; pcvr->n=pcvr->nmax=0;
      
      /* close geoid data */
      closegeoid();
      
      /* free erp data */
      free(nav->erp.data); nav->erp.data=NULL; nav->erp.n=nav->erp.nmax=0;
      
      /* close solution statistics and debug trace */
      rtkclosestat();
      traceclose();
  }
  ```

  

* **reppaths()**：根据ts、te分时间段，循环调用reppath()，替换path[]中的替换符，存到repath[]中，返回文件数量

  **reppath()**：如果输入文件（file）中，含有替换符，则 reppath函数的目的就是将文件名中的替换符调用repstr() 进行替换，保存到rpath中 。替换符如下：

  > reppaths()需要ts和te、而reppath只用ts

  ```c
  %Y -> yyyy : year (4 digits) (1900-2099)
  %y -> yy   : year (2 digits) (00-99)
  %m -> mm   : month           (01-12)
  %d -> dd   : day of month    (01-31)
  %h -> hh   : hours           (00-23)
  %M -> mm   : minutes         (00-59)
  %S -> ss   : seconds         (00-59)
  %n -> ddd  : day of year     (001-366)
  %W -> wwww : gps week        (0001-9999)
  %D -> d    : day of gps week (0-6)
  %H -> h    : hour code       (a=0,b=1,c=2,...,x=23)
  %ha-> hh   : 3 hours         (00,03,06,...,21)
  %hb-> hh   : 6 hours         (00,06,12,18)
  %hc-> hh   : 12 hours        (00,12)
  %t -> mm   : 15 minutes      (00,15,30,45)
  %r -> rrrr : rover id
  %b -> bbbb : base station id
  ```

  ```c
  extern int reppath(const char *path, char *rpath, gtime_t time, const char *rov,
                     const char *base)
  {
      double ep[6],ep0[6]={2000,1,1,0,0,0};
      int week,dow,doy,stat=0;
      char rep[64];
      
      strcpy(rpath,path);
      
      if (!strstr(rpath,"%")) return 0;           //找不到%号直接结束
      if (*rov ) stat|=repstr(rpath,"%r",rov );   //如果有，替换基准站、流动站名
      if (*base) stat|=repstr(rpath,"%b",base);
      if (time.time!=0) {
          //把时间从gtime_t转为ep数组、DOW、DOY
          time2epoch(time,ep);    
          ep0[0]=ep[0];
          dow=(int)floor(time2gpst(time,&week)/86400.0);
          doy=(int)floor(timediff(time,epoch2time(ep0))/86400.0)+1;
          //把要替换的内容存到rep中，再用rep替换
          sprintf(rep,"%02d",  ((int)ep[3]/3)*3);   stat|=repstr(rpath,"%ha",rep);
          sprintf(rep,"%02d",  ((int)ep[3]/6)*6);   stat|=repstr(rpath,"%hb",rep);
          sprintf(rep,"%02d",  ((int)ep[3]/12)*12); stat|=repstr(rpath,"%hc",rep);
          sprintf(rep,"%04.0f",ep[0]);              stat|=repstr(rpath,"%Y",rep);
          sprintf(rep,"%02.0f",fmod(ep[0],100.0));  stat|=repstr(rpath,"%y",rep);
          sprintf(rep,"%02.0f",ep[1]);              stat|=repstr(rpath,"%m",rep);
          sprintf(rep,"%02.0f",ep[2]);              stat|=repstr(rpath,"%d",rep);
          sprintf(rep,"%02.0f",ep[3]);              stat|=repstr(rpath,"%h",rep);
          sprintf(rep,"%02.0f",ep[4]);              stat|=repstr(rpath,"%M",rep);
          sprintf(rep,"%02.0f",floor(ep[5]));       stat|=repstr(rpath,"%S",rep);
          sprintf(rep,"%03d",  doy);                stat|=repstr(rpath,"%n",rep);
          sprintf(rep,"%04d",  week);               stat|=repstr(rpath,"%W",rep);
          sprintf(rep,"%d",    dow);                stat|=repstr(rpath,"%D",rep);
          sprintf(rep,"%c",    'a'+(int)ep[3]);     stat|=repstr(rpath,"%H",rep);
          sprintf(rep,"%02d",  ((int)ep[4]/15)*15); stat|=repstr(rpath,"%t",rep);
      }
      else if (strstr(rpath,"%ha")||strstr(rpath,"%hb")||strstr(rpath,"%hc")||
               strstr(rpath,"%Y" )||strstr(rpath,"%y" )||strstr(rpath,"%m" )||
               strstr(rpath,"%d" )||strstr(rpath,"%h" )||strstr(rpath,"%M" )||
               strstr(rpath,"%S" )||strstr(rpath,"%n" )||strstr(rpath,"%W" )||
               strstr(rpath,"%D" )||strstr(rpath,"%H" )||strstr(rpath,"%t" )) {
          return -1; /* no valid time */
      }
      return stat;
  }
  ```

  ```c
  extern int reppaths(const char *path, char *rpath[], int nmax, gtime_t ts,
                      gtime_t te, const char *rov, const char *base)
  {
      gtime_t time;
      double tow,tint=86400.0;
      int i,n=0,week;
      
      trace(3,"reppaths: path =%s nmax=%d rov=%s base=%s\n",path,nmax,rov,base);
      
      if (ts.time==0||te.time==0||timediff(ts,te)>0.0) return 0; //如果起止时间为0，或ts>te，直接return
      
      if (strstr(path,"%S")||strstr(path,"%M")||strstr(path,"%t")) tint=900.0;    //15分钟
      else if (strstr(path,"%h")||strstr(path,"%H")) tint=3600.0;     //一小时
      
      tow=time2gpst(ts,&week);
      time=gpst2time(week,floor(tow/tint)*tint);  
      
      while (timediff(time,te)<=0.0&&n<nmax) {
          reppath(path,rpath[n],time,rov,base);
          if (n==0||strcmp(rpath[n],rpath[n-1])) n++;
          time=timeadd(time,tint);
      }
      for (i=0;i<n;i++) trace(3,"reppaths: rpath=%s\n",rpath[i]);
      return n;
  }
  ```



### 2、execses_b()、execses_r()

#### 1. 功能

	execses_b()和execses_r()函数非常类似，execsec_b()会调用调用`readpreceph()`读取精密星历和SBAS数据，把传入`infile[]`文件中基准站替换符进行替换，之后调用`execses_r()`。`execses_r()`把传入`infile[]`文件中流动站站替换符进行替换，再调用`execses()`

#### 2. 输入参数

```c
gtime_t ts              I   处理的起始时间，写0表示不限制
gtime_t te              I   处理的起始时间，写0表示不限制
double ti               I   处理的间隔时间 (s)，写0表示不限制，全处理
const prcopt_t *popt    I   处理选项结构体
const solopt_t *sopt    I   结果选项结构体
const filopt_t *fopt    I   文件选项结构体
int flag                I   用于控制输出
char **infile           I   传入文件路径数组首地址
const int *index        I   传入文件路径数组首地址
int n                   I   传入文件数量
char *outfile           I   输出文件的路径，写0表示stdout终端
const char *rov         I   流动站ID列表，空格隔开
const char *base        I   基准站ID列表，空格隔开
```

* 参数flag：

  * 传入execses_r()，再传入execses()，用于控制输出，如果值为0,很多不输出；
  * 在postpos函数中赋值传入，替换输出文件替换符出错的时候设为0，其它情况为1

  ```c
   if (flag&&sopt->trace>0) {
          if (*outfile) {
              strcpy(tracefile,outfile);
              strcat(tracefile,".trace");
          }
          else {
              strcpy(tracefile,fopt->trace);
          }
          traceclose();
          traceopen(tracefile);
          tracelevel(sopt->trace);
      }
  ```

  ```c
      if (flag&&sopt->sstat>0) {
          strcpy(statfile,outfile);
          strcat(statfile,".stat");
          rtkclosestat();
          rtkopenstat(statfile,sopt->sstat);
      }
      /* write header to output file */   //写输出结果文件的文件头
      if (flag&&!outhead(outfile,infile,n,&popt_,sopt)) {
          freeobsnav(&obss,&navs);
          return 0;
      }
  ```

  

#### 3. execses_b()执行流程

1. 调用`readpreceph()`读取精密星历和SBAS数据。
2. 遍历`infile[]`，寻找基准站替换符%b：
   * 找不到基准站ID的替换符，直接调用`execses_r()`进行下一步解算 。
   * 找到了`infile[i]`含有基准站ID的替换符，遍历基准站：
     * 将基准站ID赋值给`proc_base`。
     * 循环替换`infile[i]`里的基准站ID的替换符到`ifile[i] `。
     * 替换`outfile`里的基准站ID替换符到ofile。
     * 调用`execses_r()`进行下一步解算 。
3. 调用`freepreceph()`，释放`readpreceph()`开辟的空间。

```c
static int execses_b(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                     const solopt_t *sopt, const filopt_t *fopt, int flag,
                     char **infile, const int *index, int n, char *outfile,
                     const char *rov, const char *base)
{
    gtime_t t0={0};
    int i,stat=0;
    char *ifile[MAXINFILE],ofile[1024], *base_,*p,*q,s[64];
    
    trace(3,"execses_b: n=%d outfile=%s\n",n,outfile);
    
    /* read prec ephemeris and sbas data */
    readpreceph(infile,n,popt,&navs,&sbss); //读取精密星历和SBAS数据
    
    //%b：基准站ID的替换符
    for (i=0;i<n;i++) if (strstr(infile[i],"%b")) break;
    //如果某个infile[i]含有基准站ID的替换符
    if (i<n) { /* include base station keywords */
        //为base_开辟空间，将base赋值给base_
        if (!(base_=(char *)malloc(strlen(base)+1))) {  
            freepreceph(&navs,&sbss);
            return 0;
        }
        strcpy(base_,base); 
        
        for (i=0;i<n;i++) {     //为ifile[]开辟空间
            if (!(ifile[i]=(char *)malloc(1024))) { 
                free(base_); for (;i>=0;i--) free(ifile[i]);
                freepreceph(&navs,&sbss);
                return 0;
            }
        }
        //遍历base_基准站字符串
        for (p=base_;;p=q+1) { /* for each base station */
            if ((q=strchr(p,' '))) *q='\0'; //拆出一个基准站
            
            if (*p) {   
                strcpy(proc_base,p);    //把基准站名赋值给proc_base
                if (ts.time) time2str(ts,s,0); else *s='\0';
                if (checkbrk("reading    : %s",s)) {
                    stat=1;
                    break;
                }
                //循环替换infile[i]里的基准站ID的替换符到ifile[i]
                for (i=0;i<n;i++) reppath(infile[i],ifile[i],t0,"",p);
                //替换outfile里的基准站ID替换符到ofile
                reppath(outfile,ofile,t0,"",p); 
                //调用execses_r()进行下一步解算
                stat=execses_r(ts,te,ti,popt,sopt,fopt,flag,ifile,index,n,ofile,rov);
            }
            if (stat==1||!q) break;
        }
        free(base_); for (i=0;i<n;i++) free(ifile[i]);
    }
    else {  //infile[i]都没有有基准站ID的替换符，直接调用execses_r()进行下一步解算
        stat=execses_r(ts,te,ti,popt,sopt,fopt,flag,infile,index,n,outfile,rov);
    }
    /* free prec ephemeris and sbas data */
    freepreceph(&navs,&sbss);
    
    return stat;
}
```

```c
static int execses_r(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                     const solopt_t *sopt, const filopt_t *fopt, int flag,
                     char **infile, const int *index, int n, char *outfile,
                     const char *rov)
{
    gtime_t t0={0};
    int i,stat=0;
    char *ifile[MAXINFILE],ofile[1024],*rov_,*p,*q,s[64]="";
    
    trace(3,"execses_r: n=%d outfile=%s\n",n,outfile);
    
    for (i=0;i<n;i++) if (strstr(infile[i],"%r")) break;
    
    //如果某个infile[i]含有基准站ID的替换符
    if (i<n) { /* include rover keywords */
        if (!(rov_=(char *)malloc(strlen(rov)+1))) return 0;
        strcpy(rov_,rov);
        
        for (i=0;i<n;i++) {
            if (!(ifile[i]=(char *)malloc(1024))) {
                free(rov_); for (;i>=0;i--) free(ifile[i]);
                return 0;
            }
        }
        for (p=rov_;;p=q+1) { /* for each rover */
            if ((q=strchr(p,' '))) *q='\0';
            
            if (*p) {
                strcpy(proc_rov,p);
                if (ts.time) time2str(ts,s,0); else *s='\0';
                if (checkbrk("reading    : %s",s)) {
                    stat=1;
                    break;
                }
                for (i=0;i<n;i++) reppath(infile[i],ifile[i],t0,p,"");
                reppath(outfile,ofile,t0,p,"");
                
                /* execute processing session */
                stat=execses(ts,te,ti,popt,sopt,fopt,flag,ifile,index,n,ofile);
            }
            if (stat==1||!q) break;
        }
        free(rov_); for (i=0;i<n;i++) free(ifile[i]);
    }
    else {
        /* execute processing session */
        stat=execses(ts,te,ti,popt,sopt,fopt,flag,infile,index,n,outfile);
    }
    return stat;
}
```



#### 4. 调用的函数

* **readpreceph()**：遍历infile[]，判断，调用readsp3()读取精密星历、调用readrnxc() 读取精密钟差，调用sbsreadmsg()读取sbas文件，将RCTM的路径赋值给rtcm_file，调用init_rtcm()初始化rtcm控制结构体。

  ```c
  static void readpreceph(char **infile, int n, const prcopt_t *prcopt,
                          nav_t *nav, sbs_t *sbs)
  {
      seph_t seph0={0};
      int i;
      char *ext;
      
      trace(2,"readpreceph: n=%d\n",n);
      
      nav->ne=nav->nemax=0;
      nav->nc=nav->ncmax=0;
      sbs->n =sbs->nmax =0;
      
      /* read precise ephemeris files */  //读精密星历sp3
      for (i=0;i<n;i++) {
          if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
          readsp3(infile[i],nav,0);
      }
      /* read precise clock files */  //读精密钟差
      for (i=0;i<n;i++) {
          if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
          readrnxc(infile[i],nav);
      }
      /* read sbas message files */   //读sbas文件
      for (i=0;i<n;i++) {
          if (strstr(infile[i],"%r")||strstr(infile[i],"%b")) continue;
          sbsreadmsg(infile[i],prcopt->sbassatsel,sbs);
      }
      /* allocate sbas ephemeris */   //为nav->seph开辟空间
      nav->ns=nav->nsmax=NSATSBS*2;   
      if (!(nav->seph=(seph_t *)malloc(sizeof(seph_t)*nav->ns))) {
           showmsg("error : sbas ephem memory allocation");
           trace(1,"error : sbas ephem memory allocation");
           return;
      }
      for (i=0;i<nav->ns;i++) nav->seph[i]=seph0; 
      
      /* set rtcm file and initialize rtcm struct */
      rtcm_file[0]=rtcm_path[0]='\0'; fp_rtcm=NULL;
      
      //遍历ifile，将后缀为RTCM3的路径赋值到rtcm_file,初始化rtcm控制结构体
      for (i=0;i<n;i++) {
          if ((ext=strrchr(infile[i],'.'))&&      
              (!strcmp(ext,".rtcm3")||!strcmp(ext,".RTCM3"))) {
              strcpy(rtcm_file,infile[i]);
              init_rtcm(&rtcm);
              break;
          }
      }
  }
  ```



### 四、execses()

#### 1. 功能

读取各种文件，并将文件中的内容赋值到程序的结构体内，获取基准站的位置，根据滤波方向调用procpos()进行下一步解算。.trace文件的生成、文件读取相关trace文件内容的生成，均在execses中 。

#### 2. 输入参数

```c
gtime_t ts              I   处理的起始时间，写0表示不限制
gtime_t te              I   处理的起始时间，写0表示不限制
double ti               I   处理的间隔时间 (s)，写0表示不限制，全处理
const prcopt_t *popt    I   处理选项结构体
const solopt_t *sopt    I   结果选项结构体
const filopt_t *fopt    I   文件选项结构体
int flag                I   用于控制输出
char **infile           I   传入文件路径数组首地址
const int *index        I   传入文件路径数组首地址
int n                   I   传入文件数量
char *outfile           I   输出文件的路径，写0表示stdout终端
const char *rov         I   流动站ID列表，空格隔开
const char *base        I   基准站ID列表，空格隔开
```

#### 3. 执行流程

1. 调用`traceclose() `、`traceopen()` 、`tracelevel()`，先关闭原有trace，打开trace文件，并设置trace等级。

2. 调用`readtec()` ，读取电离层TEC文件，TEC:Total electronic content 总电子含量 。

3. 调用`readerp()`，读取地球自转参数ERP文件。

4. 调用`readobsnav() `，读取OBS和NAV文件 。

5. 调用`readdcb()`，读取差分码偏差DCB参数，一种硬件误差 。

6. 调用`setpcv()`，读取天线参数，PCV：天线相位中心变化 。

7. 调用`readotl()`，读取潮汐参数 。

8. FIXED模式，调用`antpos()`得到流动站坐标 。

9. DGPS、KINEMA、STATIC模式，调用`antpos()`得到基准站坐标 。

10. 调用`rtkclosestat() `、`rtkopenstat()`，打开结果统计文件 。

11. 调用`outhead()`，写输出结果文件的文件头 。结果文件的文件尾在`procpos()`内调用`outsol()`输出。

12. 判断滤波类型，用不同的方式调用`procpos()`进行下一步解算：

    > 前向滤波和后向滤波调用procpos函数传参相同，两者区别在于procpos函数内会调用inputobs函数，针对不同的滤波解算类型，inputobs函数内读取文件数据的顺序不同。
    >
    > revs：0：forward；1：backward
    >
    > iobsu：当前流动站观测数据下标
    >
    > iobsr：当前参考站观测数据下标
    >
    > isbs：当前sbas数据下标

    * forward 前向滤波：iobsu=iobsr=isbs=revs 0，直接调用`procpos()`。
    * backward 后向滤波：res=1，iobsu=iobsr=obss.n-1 ，isbs=sbss.n-1 ，再调用`procpos()`。
    * combined ：先算前向滤波的结果，设置revs、iobsu、iobsr、isbs值之后再算后向滤波的结果，最后调用combress()结合。

13. 调用`freeobsnav()`释放obs->data 、nav->eph 、nav->geph 、nav->seph 

```c
static int execses(gtime_t ts, gtime_t te, double ti, const prcopt_t *popt,
                   const solopt_t *sopt, const filopt_t *fopt, int flag,
                   char **infile, const int *index, int n, char *outfile)
{
    FILE *fp;
    prcopt_t popt_=*popt;
    char tracefile[1024],statfile[1024],path[1024],*ext;
    
    trace(3,"execses : n=%d outfile=%s\n",n,outfile);
    
    /* open debug trace */  //打开trace文件，并设置trace等级
    if (flag&&sopt->trace>0) {
        if (*outfile) {
            strcpy(tracefile,outfile);
            strcat(tracefile,".trace");
        }
        else {
            strcpy(tracefile,fopt->trace);
        }
        traceclose();
        traceopen(tracefile);
        tracelevel(sopt->trace);
    }
    /* read ionosphere data file */ //读取电离层TEC文件
    if (*fopt->iono&&(ext=strrchr(fopt->iono,'.'))) {
        if (strlen(ext)==4&&(ext[3]=='i'||ext[3]=='I')) {
            reppath(fopt->iono,path,ts,"","");
            readtec(path,&navs,1);  //TEC:Total electronic content 总电子含量
        }
    }
    /* read erp data */ //读取地球自转参数ERP文件
    if (*fopt->eop) {
        free(navs.erp.data); navs.erp.data=NULL; navs.erp.n=navs.erp.nmax=0;
        reppath(fopt->eop,path,ts,"","");
        if (!readerp(path,&navs.erp)) {
            showmsg("error : no erp data %s",path);
            trace(2,"no erp data %s\n",path);
        }
    }
    /* read obs and nav data */ //读取OBS和NAV文件
    if (!readobsnav(ts,te,ti,infile,index,n,&popt_,&obss,&navs,stas)) return 0;
    
    /* read dcb parameters */   //读取差分码偏差DCB参数，一种硬件误差
    if (*fopt->dcb) {
        reppath(fopt->dcb,path,ts,"","");
        readdcb(path,&navs,stas);
    }
    /* set antenna paramters */ //读取天线参数，PCV：天线相位中心变化
    if (popt_.mode!=PMODE_SINGLE) {
        setpcv(obss.n>0?obss.data[0].time:timeget(),&popt_,&navs,&pcvss,&pcvsr,
               stas);
    }
    /* read ocean tide loading parameters */    //读取潮汐参数
    if (popt_.mode>PMODE_SINGLE&&*fopt->blq) {
        readotl(&popt_,fopt->blq,stas);
    }
    /* rover/reference fixed position */    //FIXED模式，调用antpos()得到流动站坐标
    if (popt_.mode==PMODE_FIXED) {
        if (!antpos(&popt_,1,&obss,&navs,stas,fopt->stapos)) {
            freeobsnav(&obss,&navs);
            return 0;
        }
    }
    else if (PMODE_DGPS<=popt_.mode&&popt_.mode<=PMODE_STATIC) {    //DGPS、KINEMA、STATIC模式，调用antpos()得到基准站坐标
        if (!antpos(&popt_,2,&obss,&navs,stas,fopt->stapos)) {
            freeobsnav(&obss,&navs);
            return 0;
        }
    }
    /* open solution statistics */  //打开结果统计文件
    if (flag&&sopt->sstat>0) {
        strcpy(statfile,outfile);
        strcat(statfile,".stat");
        rtkclosestat();
        rtkopenstat(statfile,sopt->sstat);
    }
    /* write header to output file */   //写输出结果文件的文件头
    if (flag&&!outhead(outfile,infile,n,&popt_,sopt)) {
        freeobsnav(&obss,&navs);
        return 0;
    }
    iobsu=iobsr=isbs=revs=aborts=0;
    
    if (popt_.mode==PMODE_SINGLE||popt_.soltype==0) {
        if ((fp=openfile(outfile))) {
            procpos(fp,&popt_,sopt,0); /* forward */    //前向滤波
            fclose(fp);
        }
    }
    else if (popt_.soltype==1) {
        if ((fp=openfile(outfile))) {
            revs=1; iobsu=iobsr=obss.n-1; isbs=sbss.n-1;
            procpos(fp,&popt_,sopt,0); /* backward */   //后向滤波
            fclose(fp);
        }
    }
    else { /* combined */
        //开辟内存空间
        solf=(sol_t *)malloc(sizeof(sol_t)*nepoch);     //前向结果
        solb=(sol_t *)malloc(sizeof(sol_t)*nepoch);     //后向结果
        rbf=(double *)malloc(sizeof(double)*nepoch*3);  //前向基准站坐标
        rbb=(double *)malloc(sizeof(double)*nepoch*3);  //后向基准站坐标
        
        if (solf&&solb) {   //判断内存开辟成功
            isolf=isolb=0;
            procpos(NULL,&popt_,sopt,1); /* forward */      //前向滤波
            revs=1; iobsu=iobsr=obss.n-1; isbs=sbss.n-1;
            procpos(NULL,&popt_,sopt,1); /* backward */     //后向滤波

            //虽然前向滤波和后向滤波调用procpos函数的源代码相同（如下所示），
            //但是两者最主要的一个区别就是由于procpos函数内会调用inputobs函数，
            //然而针对不同的滤波解算类型，inputobs函数内读取文件数据的顺序不同
            /* combine forward/backward solutions */
            if (!aborts&&(fp=openfile(outfile))) {
                combres(fp,&popt_,sopt);
                fclose(fp);
            }
        }
        else showmsg("error : memory allocation");
        free(solf);
        free(solb);
        free(rbf);
        free(rbb);
    }
    /* free obs and nav data */
    freeobsnav(&obss,&navs);
    
    return aborts?1:0;
}
```

#### 4. 调用的函数

* **antpos()**：得到坐标，参2`rcvno`传1得到流动站坐标，传0得到基准站坐标

  * postype=POSOPT_SINGLE ：调用`avepos()`利用基准站的观测文件计算其SPP定位结果作为基准站的坐标 。

  * postype=POSOPT_FILE ：调用`getstapos()`从pos文件读取基准站坐标 。

  * postype=POSOPT_RINEX ：从rinex头文件中获取测站经过相位中心改正的位置数据。头文件中的测站数据经过读取后已存到stas中。

    ```c
    static sta_t stas[MAXRCV];      /* station infomation */
    ```

    ```c
    typedef struct {        /* station parameter type */
        char name   [MAXANT]; /* marker name */
        char marker [MAXANT]; /* marker number */
        char antdes [MAXANT]; /* antenna descriptor */
        char antsno [MAXANT]; /* antenna serial number */
        char rectype[MAXANT]; /* receiver type descriptor */
        char recver [MAXANT]; /* receiver firmware version */
        char recsno [MAXANT]; /* receiver serial number */
        int antsetup;       /* antenna setup id */
        int itrf;           /* ITRF realization year */
        int deltype;        /* antenna delta type (0:enu,1:xyz) */
        double pos[3];      /* station position (ecef) (m) */
        double del[3];      /* antenna position delta (e/n/u or x/y/z) (m) */
        double hgt;         /* antenna height (m) */
        int glo_cp_align;   /* GLONASS code-phase alignment (0:no,1:yes) */
        double glo_cp_bias[4]; /* GLONASS code-phase biases {1C,1P,2C,2P} (m) */
    } sta_t;
    ```

  ```c
  static int antpos(prcopt_t *opt, int rcvno, const obs_t *obs, const nav_t *nav,
                    const sta_t *sta, const char *posfile)
  {
      double *rr=rcvno==1?opt->ru:opt->rb,
              del[3],pos[3],dr[3]={0};
      int i,
      postype=rcvno==1?opt->rovpos:opt->refpos;
      char *name;
      
      trace(3,"antpos  : rcvno=%d\n",rcvno);
      
      if (postype==POSOPT_SINGLE) { /* average of single position */  //利用基准站的观测文件计算其SPP定位结果作为基准站的坐标
          if (!avepos(rr,rcvno,obs,nav,opt)) {
              showmsg("error : station pos computation");
              return 0;
          }
      }
      else if (postype==POSOPT_FILE) { /* read from position file */  //从pos文件读取基准站坐标
          name=stas[rcvno==1?0:1].name;
          if (!getstapos(posfile,name,rr)) {
              showmsg("error : no position of %s in %s",name,posfile);
              return 0;
          }
      }
      else if (postype==POSOPT_RINEX) { /* get from rinex header */   //从基准站的OBS观测文件的文件头部分读取基准站坐标
          if (norm(stas[rcvno==1?0:1].pos,3)<=0.0) {      //如果没有坐标数据，报错
              showmsg("error : no position in rinex header");
              trace(1,"no position position in rinex header\n");
              return 0;
          }
          //天线相位中心偏差改正
          /* antenna delta */
          if (stas[rcvno==1?0:1].deltype==0) { /* enu */
              for (i=0;i<3;i++) del[i]=stas[rcvno==1?0:1].del[i];
              del[2]+=stas[rcvno==1?0:1].hgt;
              ecef2pos(stas[rcvno==1?0:1].pos,pos);
              enu2ecef(pos,del,dr);
          }
          else { /* xyz */
              for (i=0;i<3;i++) dr[i]=stas[rcvno==1?0:1].del[i];
          }
          for (i=0;i<3;i++) rr[i]=stas[rcvno==1?0:1].pos[i]+dr[i];
      }
      return 1;
  }
  ```

* **avepos()**：通过nav和多个obs单点定位计算位置，存到ra[]中 

  ```c
  static int avepos(double *ra, int rcv, const obs_t *obs, const nav_t *nav,
                    const prcopt_t *opt)
  {
      obsd_t data[MAXOBS];
      gtime_t ts={0};
      sol_t sol={{0}};
      int i,j,n=0,m,iobs;
      char msg[128];
      
      trace(3,"avepos: rcv=%d obs.n=%d\n",rcv,obs->n);
      
      for (i=0;i<3;i++) ra[i]=0.0;    
      
      //遍历obs
      for (iobs=0;(m=nextobsf(obs,&iobs,rcv))>0;iobs+=m) {
          
          for (i=j=0;i<m&&i<MAXOBS;i++) {
              data[j]=obs->data[iobs+i];
              if ((satsys(data[j].sat,NULL)&opt->navsys)&&
                  opt->exsats[data[j].sat-1]!=1) j++;
          }
          if (j<=0||!screent(data[0].time,ts,ts,1.0)) continue; /* only 1 hz */
          
          //单点定位，结果存到sol，再加到ra[]
          if (!pntpos(data,j,nav,opt,&sol,NULL,NULL,msg)) continue;
          for (i=0;i<3;i++) ra[i]+=sol.rr[i];
          n++;
      }
      if (n<=0) {
          trace(1,"no average of base station position\n");
          return 0;
      }
      for (i=0;i<3;i++) ra[i]/=n; // ra/=obs数，得到平均位置
      return 1;
  }
  ```

* **getstapos()**：从pos文件读取基准站坐标

  ```c
  static int getstapos(const char *file, char *name, double *r)
  {
      FILE *fp;
      char buff[256],sname[256],*p,*q;
      double pos[3];
      
      trace(3,"getstapos: file=%s name=%s\n",file,name);
      
      if (!(fp=fopen(file,"r"))) {    //以读的方式打开file
          trace(1,"station position file open error: %s\n",file);
          return 0;
      }
      //循环读取，每次读一行数据，到\n或者256位结束
      while (fgets(buff,sizeof(buff),fp)) {   
          //如果在行中找到%，截断，赋值\0
          if ((p=strchr(buff,'%'))) *p='\0';  
          //格式化读取,测站位置存到pos[3],测站名存到sname
          if (sscanf(buff,"%lf %lf %lf %s",pos,pos+1,pos+2,sname)<4) continue;
          //逐字符转大写比较name、sname
          for (p=sname,q=name;*p&&*q;p++,q++) {
              if (toupper((int)*p)!=toupper((int)*q)) break;
          }
          if (!*p) {
              pos[0]*=D2R;
              pos[1]*=D2R;
              pos2ecef(pos,r);
              fclose(fp);
              return 1;
          }
      }
      fclose(fp);
      trace(1,"no station position: %s %s\n",name,file);
      return 0;
  }
  ```

* **outhead()**：创建输出结果文件，写入文件头

  ```c
  static int outhead(const char *outfile, char **infile, int n,
                     const prcopt_t *popt, const solopt_t *sopt)
  {
      FILE *fp=stdout;    //fp默认初始为stdout
      
      trace(3,"outhead: outfile=%s n=%d\n",outfile,n);
      
      if (*outfile) {
          createdir(outfile); //递归的创建文件夹
          
          if (!(fp=fopen(outfile,"wb"))) {    //wb：以写的方式打开二进制文件
              showmsg("error : open output file %s",outfile);
              return 0;
          }
      }
      /* output header */
      outheader(fp,infile,n,popt,sopt);
      
      if (*outfile) fclose(fp);
      
      return 1;
  }
  ```

* **openfile()**：以追加的方式打开结果文件，返回文件描述符

  ```c
  static FILE *openfile(const char *outfile)
  {
      trace(3,"openfile: outfile=%s\n",outfile);
      
      return !*outfile?stdout:fopen(outfile,"ab");    //ab：以追加的方式打开二进制文件
  }
  ```

* **combres()**：调用smoother()结合前后向滤波的结果

  $Q_s=(Q_f^{-1}+Q_b^{-1})^{-1} $     $ X_s=Q_s*(Q_f^{-1}*X_f+Qb^{-1}*X_b) $

  执行流程：

  * 判断静态模式，处理选项和结果选项都得为静态 
  * 开始大循环，i:从前到后，取前向滤波的结果 ，j:从后到前，取后向滤波的结果 ，判断前后向滤波结果的时间差 tt
  * 时间差大于DTTOL ，sols、rbs取时间早的结果，另一个结果的下标不变，进行下一次循环的判断 
  * 时间差很小，solution status不同，sols、rbs取solution status小的结果 
  * 时间差很小，solution status相同，进行结合
    * sols取前向滤波结果 ，时间取前后向时间的平均 
    * 相对定位模式，若结果为固定解，调用valcomb()检验，如果失败将fix降级为float 
    * 赋值前后向协方差给Qf、Qb ，调用smoother()进行前后向滤波结果结合，位置存在sols.rr[]，方差存在sols.qr[] 
    * 同样的方式，对速度进行结合，位置存在sols.rr[]，方差存在sols.qv[]

  > * 结果状态的#define：
  >
  > ```c
  > #define SOLQ_NONE   0                   /* solution status: no solution */
  > #define SOLQ_FIX    1                   /* solution status: fix */
  > #define SOLQ_FLOAT  2                   /* solution status: float */
  > #define SOLQ_SBAS   3                   /* solution status: SBAS */
  > #define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
  > #define SOLQ_SINGLE 5                   /* solution status: single */
  > #define SOLQ_PPP    6                   /* solution status: PPP */
  > #define SOLQ_DR     7                   /* solution status: dead reconing */
  > #define MAXSOLQ     7                   /* max number of solution status */
  > ```
  >
  > * sol_t结构体：
  >
  > > 因为协方差矩阵是对称的，qr、qv都只用6个元素就可存协方差矩阵，但计算的时候得转成3*3矩阵才行。
  >
  > ```c
  > typedef struct {        /* solution type */
  >     gtime_t time;       /* time (GPST) */
  >     double rr[6];       /* position/velocity (m|m/s) */
  >                         /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
  >     float  qr[6];       /* position variance/covariance (m^2) */
  >                         /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
  >                         /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
  >     float  qv[6];       /* velocity variance/covariance (m^2/s^2) */
  >     double dtr[6];      /* receiver clock bias to time systems (s) */
  >     uint8_t type;       /* type (0:xyz-ecef,1:enu-baseline) */
  >     uint8_t stat;       /* solution status (SOLQ_???) */
  >     uint8_t ns;         /* number of valid satellites */
  >     float age;          /* age of differential (s) */
  >     float ratio;        /* AR ratio factor for valiation */
  >     float thres;        /* AR ratio threshold for valiation */
  > } sol_t;
  > ```

```c
static void combres(FILE *fp, const prcopt_t *popt, const solopt_t *sopt)
{
    gtime_t time={0};
    sol_t sols={{0}},sol={{0}};
    double tt,Qf[9],Qb[9],Qs[9],rbs[3]={0},rb[3]={0},rr_f[3],rr_b[3],rr_s[3];
    int i,j,k,solstatic,pri[]={0,1,2,3,4,5,1,6};
    
    trace(3,"combres : isolf=%d isolb=%d\n",isolf,isolb);
    
    //判断静态模式，处理选项和结果选项都得为静态
    solstatic=sopt->solstatic&&
              (popt->mode==PMODE_STATIC||popt->mode==PMODE_PPP_STATIC);
    
    //i:从前到后，取前向滤波的结果
    //j:从后到前，取后向滤波的结果
    for (i=0,j=isolb-1;i<isolf&&j>=0;i++,j--) {
        //判断前后向滤波结果的时间差，时间差大于DTTOL，
        //sols、rbs取时间早的结果，另一个结果的下标不变，进行下一次循环的判断
        if ((tt=timediff(solf[i].time,solb[j].time))<-DTTOL) {  //如果前向时间迟于后向时间
            sols=solf[i];                                       
            for (k=0;k<3;k++) rbs[k]=rbf[k+i*3];                //把前向基站坐标赋值给rbs[]
            j++;            //j不变
        }
        else if (tt>DTTOL) {                        //如果前向时间早于后向时间
            sols=solb[j];                           
            for (k=0;k<3;k++) rbs[k]=rbb[k+j*3];    //把后向基站坐标赋值给rbs[]
            i--;            //i不变
        }
        //时间差很小，solution status不同，sols、rbs取solution status小的结果
        else if (solf[i].stat<solb[j].stat) {
            sols=solf[i];
            for (k=0;k<3;k++) rbs[k]=rbf[k+i*3];
        }
        else if (solf[i].stat>solb[j].stat) {
            sols=solb[j];
            for (k=0;k<3;k++) rbs[k]=rbb[k+j*3];
        }
        //时间差很小，solution status相同
        else {
            sols=solf[i];   //sols取前向滤波结果
            sols.time=timeadd(sols.time,-tt/2.0);   //时间取前后向时间的平均
            //相对定位模式，若结果为固定解，调用valcomb()检验，如果失败将fix降级为float
            if ((popt->mode==PMODE_KINEMA||popt->mode==PMODE_MOVEB)&&
                sols.stat==SOLQ_FIX) {
                /* degrade fix to float if validation failed */
                if (!valcomb(solf+i,solb+j)) sols.stat=SOLQ_FLOAT;
            }
            //赋值前后向协方差给Qf、Qb，
            for (k=0;k<3;k++) {     //k+k*3是取对角线元素
                Qf[k+k*3]=solf[i].qr[k];
                Qb[k+k*3]=solb[j].qr[k];
            }
            Qf[1]=Qf[3]=solf[i].qr[3];  //赋值非对角线元素
            Qf[5]=Qf[7]=solf[i].qr[4];
            Qf[2]=Qf[6]=solf[i].qr[5];
            Qb[1]=Qb[3]=solb[j].qr[3];
            Qb[5]=Qb[7]=solb[j].qr[4];
            Qb[2]=Qb[6]=solb[j].qr[5];
            
            //调用smoother()进行前后向滤波结果结合，位置存在sols.rr[]，方差存在sols.qr[]
            if (popt->mode==PMODE_MOVEB) {  //如果是移动基线模式
                for (k=0;k<3;k++) rr_f[k]=solf[i].rr[k]-rbf[k+i*3]; //流动站坐标-基准站坐标得到基线
                for (k=0;k<3;k++) rr_b[k]=solb[j].rr[k]-rbb[k+j*3];
                if (smoother(rr_f,Qf,rr_b,Qb,3,rr_s,Qs)) continue;
                for (k=0;k<3;k++) sols.rr[k]=rbs[k]+rr_s[k];
            }
            else {
                if (smoother(solf[i].rr,Qf,solb[j].rr,Qb,3,sols.rr,Qs)) continue;
            }
            sols.qr[0]=(float)Qs[0];
            sols.qr[1]=(float)Qs[4];
            sols.qr[2]=(float)Qs[8];
            sols.qr[3]=(float)Qs[1];
            sols.qr[4]=(float)Qs[5];
            sols.qr[5]=(float)Qs[2];
            
            /* smoother for velocity solution */
            if (popt->dynamics) {
                for (k=0;k<3;k++) {
                    Qf[k+k*3]=solf[i].qv[k];
                    Qb[k+k*3]=solb[j].qv[k];
                }
                Qf[1]=Qf[3]=solf[i].qv[3];
                Qf[5]=Qf[7]=solf[i].qv[4];
                Qf[2]=Qf[6]=solf[i].qv[5];
                Qb[1]=Qb[3]=solb[j].qv[3];
                Qb[5]=Qb[7]=solb[j].qv[4];
                Qb[2]=Qb[6]=solb[j].qv[5];
                if (smoother(solf[i].rr+3,Qf,solb[j].rr+3,Qb,3,sols.rr+3,Qs)) continue;
                sols.qv[0]=(float)Qs[0];
                sols.qv[1]=(float)Qs[4];
                sols.qv[2]=(float)Qs[8];
                sols.qv[3]=(float)Qs[1];
                sols.qv[4]=(float)Qs[5];
                sols.qv[5]=(float)Qs[2];
            }
        }
        if (!solstatic) {
            outsol(fp,&sols,rbs,sopt);
        }
        else if (time.time==0||pri[sols.stat]<=pri[sol.stat]) {
            sol=sols;
            for (k=0;k<3;k++) rb[k]=rbs[k];
            if (time.time==0||timediff(sols.time,time)<0.0) {
                time=sols.time;
            }
        }
    }
    //循环处理完之后，如果是静态模式且时间存在，调用outsol()输出结果
    if (solstatic&&time.time!=0.0) {    
        sol.time=time;
        outsol(fp,&sol,rb,sopt);
    }
}
```

* **valcomb()**：判断combine结果的有效性，ok if in 4-sigma 

  ```c
  static int valcomb(const sol_t *solf, const sol_t *solb)
  {
      double dr[3],var[3];
      int i;
      char tstr[32];
      
      trace(3,"valcomb :\n");
      
      /* compare forward and backward solution */
      for (i=0;i<3;i++) {
          dr[i]=solf->rr[i]-solb->rr[i];  //坐标值差dr为两坐标相减
          var[i]=solf->qr[i]+solb->qr[i]; //方差car为两相加
      }
      //dr在限差4倍标准差之内，就合格return 1，否则return 0
      for (i=0;i<3;i++) {         
          if (dr[i]*dr[i]<=16.0*var[i]) continue; /* ok if in 4-sigma */
          
          time2str(solf->time,tstr,2);
          trace(2,"degrade fix to float: %s dr=%.3f %.3f %.3f std=%.3f %.3f %.3f\n",
                tstr+11,dr[0],dr[1],dr[2],SQRT(var[0]),SQRT(var[1]),SQRT(var[2]));
          return 0;
      }
      return 1;
  }
  ```

  

### 五、procpos()

#### 1. 功能

从这个函数开始正式整个流动站和基准站逐历元处理。每次循环都通过inputobs函数读取一个历元的数据，并调用rtkpos函数对该历元的数据进行解算。 

#### 2. 传入参数

```c
FILE *fp	   		   I/O 输出结果文件指针  
const prcopt_t *popt    I   处理选项结构体
const solopt_t *sopt    I   结果选项结构体
const filopt_t *fopt    I   文件选项结构体
int mode			   I   0：forward/backward、1：combined
```

#### 3. 执行流程

* 判断结果是否为静态,处理选项和结果选项都为静态才算静态 
* 调用`rtkinit()` 初始化`rtk_t `，将popt结构体赋值给rtk的部分成员 
* while大循环，调用`inputobs()`，每次取一个历元的观测数据`obs[]`
* 排除禁用卫星的观测值
* PPP中如果需要，调用`corr_phase_bias_ssr()`相位的小数轴偏差改正
* 调用rtkpos()对当前历元进行解算 
* 根据模式，输出结果，记录当前历元时间

```c
static void procpos(FILE *fp, const prcopt_t *popt, const solopt_t *sopt,
                    int mode)
{
    gtime_t time={0};
    sol_t sol={{0}};
    rtk_t rtk;
    obsd_t obs[MAXOBS*2]; /* for rover and base */
    double rb[3]={0};
    int i,
    nobs,
    n,
    solstatic,
    pri[]={6,1,2,3,4,5,1,6};
    
    trace(3,"procpos : mode=%d\n",mode);
    
    solstatic=sopt->solstatic&&     //先判断结果是否为静态,处理选项和结果选项都为静态才算静态
              (popt->mode==PMODE_STATIC||popt->mode==PMODE_PPP_STATIC); 
    
    rtkinit(&rtk,popt);    //初始化rtk_t，主要将popt结构体赋值给rtk的部分成员
    rtcm_path[0]='\0';
    //对每一个历元进行遍历求解和输出
    //获取当前历元观测值数nobs以及当前历元各观测记录obs[MAXOBS*2]
    while ((nobs=inputobs(obs,rtk.sol.stat,popt))>=0) { 
        /* exclude satellites */
        for (i=n=0;i<nobs;i++) {
            //satsys:传入satellite number，返回卫星系统(SYS_GPS,SYS_GLO,...) ，通过传入的指针prn传出PRN码。
            if ((satsys(obs[i].sat,NULL)&popt->navsys)&&
                popt->exsats[obs[i].sat-1]!=1) obs[n++]=obs[i]; //排除禁用卫星的观测值
        }
        if (n<=0) continue;
        //如果ppp模式设置了fractional cycle bias相位的小数轴偏差
        /* carrier-phase bias correction */
        if (!strstr(popt->pppopt,"-ENA_FCB")) {     
            corr_phase_bias_ssr(obs,n,&navs);   
        }
        //调用rtkpos()进行解算
        if (!rtkpos(&rtk,obs,n,&navs)) continue;
        
        //单forward/backward模式
        if (mode==0) { /* forward/backward */
            if (!solstatic) {   //不是静态模式就直接输出结果
                outsol(fp,&rtk.sol,rtk.rb,sopt);
            }
            else if (time.time==0||pri[rtk.sol.stat]<=pri[sol.stat]) {
                sol=rtk.sol;    
                for (i=0;i<3;i++) rb[i]=rtk.rb[i];
                if (time.time==0||timediff(rtk.sol.time,time)<0.0) {
                    time=rtk.sol.time;      //记录上一历元的时间
                }
            }
        }
        else if (!revs) { /* combined-forward */
            if (isolf>=nepoch) return;
            solf[isolf]=rtk.sol;
            for (i=0;i<3;i++) rbf[i+isolf*3]=rtk.rb[i]; 
            isolf++;
        }
        else { /* combined-backward */
            if (isolb>=nepoch) return;
            solb[isolb]=rtk.sol;
            for (i=0;i<3;i++) rbb[i+isolb*3]=rtk.rb[i];
            isolb++;
        }
    }
    if (mode==0&&solstatic&&time.time!=0.0) {
        sol.time=time;
        outsol(fp,&sol,rb,sopt);
    }
    rtkfree(&rtk);
}
```
#### 4. 调用的函数

* **inputobs()**：取一个历元基准站、流动站的观测数据到OBS数组中；如果需要，调用sbsupdatecorr()、update_rtcm_ssr()进行改正。

  > **涉及的全局变量：**
  >
  > * iobsu ：流动站当前历元索引
  > * iobsr ：基准站当前历元索引
  > * isbs  ：SBAS信息索引
  > * revs  ：0:forward 1:backward

  > * **nextobsf()**：在obs中正向指定接收机查找下一个历元观测数据的下标 ，下标从 i 开始，连续 n 个，之间卫星不同。
  >
  > ```c
  > static int nextobsf(const obs_t *obs, int *i, int rcv)  //正向查找下一个观测数据的下标
  > {
  >     double tt;
  >     int n;
  > //obs->data的元素已经用sortobs(),根据time, rcv, sat 排序、去重了
  >     //一直正向i++，直到obsd的rcv与传入接收机ID相等，找到传入接收机
  >     for (;*i<obs->n;(*i)++) if (obs->data[*i].rcv==rcv) break;  
  >     //在i的基础上加n++，直到流动站变了或时间差大于DTTOL
  >     for (n=0;*i+n<obs->n;n++) {
  >         tt=timediff(obs->data[*i+n].time,obs->data[*i].time);   //求i+n位数据与i数据的时间差tt
  >         if (obs->data[*i+n].rcv!=rcv||tt>DTTOL) break;  //时间不同或rcv不同，则结束循环
  >     }
  >     return n;   //返回在i基础上加的n,n应该是同一接收机同一时间的OBS数，卫星不同,即n为卫星数
  > }
  > ```
  >
  > * **nextobsb()**：在obs反向查找指定接收机下一个历元观测数据的下标 
  >
  > ```c
  > static int nextobsb(const obs_t *obs, int *i, int rcv)  //反向查找下一个观测数据的下标
  > {
  >     double tt;
  >     int n;
  >     //一直反向--i，直到obsd的rcv与传入rcv流动站ID相等，找到传入的流动站
  >     for (;*i>=0;(*i)--) if (obs->data[*i].rcv==rcv) break;
  >     //在i的基础上减n++，直到流动站变了或时间差小于DTTOL
  >     for (n=0;*i-n>=0;n++) {
  >         tt=timediff(obs->data[*i-n].time,obs->data[*i].time);
  >         if (obs->data[*i-n].rcv!=rcv||tt<-DTTOL) break;
  >     }
  >     return n;   //返回在i基础上减的n
  > }
  > ```

  ```c
  static int inputobs(obsd_t *obs, int solq, const prcopt_t *popt)
  {
      gtime_t time={0};
      int i,
      nu,nr,  //nu、nr存同一流动站基准站相同历元的观测值个数，应该是卫星不同
      n=0;    //obs数组下标
      
      //iobsu ：流动站当前历元索引
      //iobsr ：基准站当前历元索引
      //isbs  ：SBAS信息索引
      //revs  ：0:forward 1:backward
  
      trace(3,"infunc  : revs=%d iobsu=%d iobsr=%d isbs=%d\n",revs,iobsu,iobsr,isbs);
      
      if (0<=iobsu&&iobsu<obss.n) {
          //settime:共享库的伪应用程序函数，
          settime((time=obss.data[iobsu].time));  //time赋值为当前流动站的时间
          if (checkbrk("processing : %s Q=%d",time_str(time,0),solq)) {
              aborts=1; showmsg("aborted"); return -1;
          }
      }
      
      if (!revs) { /* input forward data */   //前向滤波
          if ((nu=nextobsf(&obss,&iobsu,1))<=0) return -1;
          if (popt->intpref) {
              for (;(nr=nextobsf(&obss,&iobsr,2))>0;iobsr+=nr)
                  if (timediff(obss.data[iobsr].time,obss.data[iobsu].time)>-DTTOL) break;
          }
          else {
              for (i=iobsr;(nr=nextobsf(&obss,&i,2))>0;iobsr=i,i+=nr)
                  if (timediff(obss.data[i].time,obss.data[iobsu].time)>DTTOL) break;
          }
          nr=nextobsf(&obss,&iobsr,2);
          if (nr<=0) {
              nr=nextobsf(&obss,&iobsr,2);
          }
          for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsu+i]; //循环nu次，把流动站同一时间、接收机不同卫星的数据加入obs[],
          for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsr+i]; //循环nr次，把基准站的数据加入obs[]
          iobsu+=nu;  //流动站当前历元索引
          
          /* update sbas corrections */
          while (isbs<sbss.n) {
              time=gpst2time(sbss.msgs[isbs].week,sbss.msgs[isbs].tow);
              
              if (getbitu(sbss.msgs[isbs].msg,8,6)!=9) { /* except for geo nav */
                  sbsupdatecorr(sbss.msgs+isbs,&navs);
              }
              if (timediff(time,obs[0].time)>-1.0-DTTOL) break;
              isbs++;
          }
          /* update rtcm ssr corrections */
          if (*rtcm_file) {
              update_rtcm_ssr(obs[0].time);
          }
      }
      else { /* input backward data */        //后向滤波
          if ((nu=nextobsb(&obss,&iobsu,1))<=0) return -1;
          if (popt->intpref) {
              for (;(nr=nextobsb(&obss,&iobsr,2))>0;iobsr-=nr)
                  if (timediff(obss.data[iobsr].time,obss.data[iobsu].time)<DTTOL) break;
          }
          else {
              for (i=iobsr;(nr=nextobsb(&obss,&i,2))>0;iobsr=i,i-=nr)
                  if (timediff(obss.data[i].time,obss.data[iobsu].time)<-DTTOL) break;
          }
          nr=nextobsb(&obss,&iobsr,2);
          for (i=0;i<nu&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsu-nu+1+i];
          for (i=0;i<nr&&n<MAXOBS*2;i++) obs[n++]=obss.data[iobsr-nr+1+i];
          iobsu-=nu;
          
          /* update sbas corrections */
          while (isbs>=0) {
              time=gpst2time(sbss.msgs[isbs].week,sbss.msgs[isbs].tow);
              
              if (getbitu(sbss.msgs[isbs].msg,8,6)!=9) { /* except for geo nav */
                  sbsupdatecorr(sbss.msgs+isbs,&navs);    
              }
              if (timediff(time,obs[0].time)<1.0+DTTOL) break;
              isbs--;
          }
      }
      return n;       //返回n：此历元基准站、流动站观测值OBS总数
  }
  ```

  

* **rtkinit()**：初始化rtk_t结构体，

  ```c
  typedef struct {        /* RTK control/result type */
      sol_t  sol;         /* RTK solution */
      double rb[6];       /* base position/velocity (ecef) (m|m/s) */
      int nx,na;          /* number of float states/fixed states */
      double tt;          /* time difference between current and previous (s) */
      double *x, *P;      /* float states and their covariance */
      double *xa,*Pa;     /* fixed states and their covariance */
      int nfix;           /* number of continuous fixes of ambiguity */
      ambc_t ambc[MAXSAT]; /* ambibuity control */
      ssat_t ssat[MAXSAT]; /* satellite status */
      int neb;            /* bytes in error message buffer */
      char errbuf[MAXERRMSG]; /* error message buffer */
      prcopt_t opt;       /* processing options */
  } rtk_t;
  ```


### 六、rtkpos()

#### 1. 功能：

根据观测数据和导航信息，计算接收机的位置、速度和钟差。 设置基准站位置，记录观测值数量。调用 pntpos 进行接收机单点定位。若为单点定位模式，输出，返回。若为 PPP 模式，调用 pppos 进行精密单点定位，输出，返回。若无基准站观测数据，输出，返回。若为移动基站模式，调用 pntpos 进行基站单点定位，并加以时间同步；否则只计算一下差分时间。调用 relpos 进行相对基站的接收机定位，输出，返回。相对定位模式在调用rtkpos之前应该先设置好基站位置，动基线模式除外。

#### 2. 传入参数

```c
rtk_t *rtk			RTK控制结构体
const obsd_t *obs	 观测数据OBS
int n			    观测数据数量
const nav_t *nav	导航电文信息
```

#### 3. 执行流程

```c
extern int rtkpos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;    //这里定义了一个prcopt_t用来储存传入的rtk_t中的prcopt_t
    sol_t solb={{0}};
    gtime_t time;
    int i,nu,nr;
    char msg[128]="";
    
    trace(3,"rtkpos  : time=%s n=%d\n",time_str(obs[0].time,3),n);
    trace(4,"obs=\n"); traceobs(4,obs,n);
    
    //设置rtk内基准站坐标，基准站坐标在execses函数内已经计算了，速度设为0.0
    //这里将配置结构体opt内基准站的坐标赋值给解算结构体rtk内基准站的坐标
    /* set base staion position */  
    if (opt->refpos<=POSOPT_RINEX&&opt->mode!=PMODE_SINGLE&&
        opt->mode!=PMODE_MOVEB) {
        for (i=0;i<6;i++) rtk->rb[i]=i<3?opt->rb[i]:0.0;    //opt内基准站坐标赋值给rtk->rb,速度设为0.0
    }
    /* count rover/base station observations */     //统计基准站OBS个数nu，流动站OBS个数nr，可用于后面判断是否满足差分条件
    for (nu=0;nu   <n&&obs[nu   ].rcv==1;nu++) ;
    for (nr=0;nu+nr<n&&obs[nu+nr].rcv==2;nr++) ;
    
    time=rtk->sol.time; /* previous epoch */
    
    //利用观测值及星历计算流动站的SPP定位结果，作为kalman滤波的近似坐标。需要注意，
    //如果由于流动站SPP定位结果坐标误差过大等原因导致的SPP无解，则不进行rtk运算，当前历元无解。
    /* rover position by single point positioning */
    if (!pntpos(obs,nu,nav,&rtk->opt,&rtk->sol,NULL,rtk->ssat,msg)) {
        errmsg(rtk,"point pos error (%s)\n",msg);
        
        if (!rtk->opt.dynamics) {
            outsolstat(rtk);
            return 0;
        }
    }
    if (time.time!=0) rtk->tt=timediff(rtk->sol.time,time);
    
    /* single point positioning */
    if (opt->mode==PMODE_SINGLE) {  //单点定位模式直接输出刚刚SPP算的坐标
        outsolstat(rtk);
        return 1;
    }
    //如果不是单点模式，抑制单点解的输出，
    /* suppress output of single solution */    
    if (!opt->outsingle) {           
        rtk->sol.stat=SOLQ_NONE;     
    }

    /* precise point positioning */ //精密单点定位
    if (opt->mode>=PMODE_PPP_KINEMA) {
        pppos(rtk,obs,nu,nav);
        outsolstat(rtk);
        return 1;
    }

    //检查该历元流动站观测时间和基准站观测时间是否对应，若无基准站观测数据，return
    /* check number of data of base station and age of differential */
    if (nr==0) {
        errmsg(rtk,"no base station observation data for rtk\n");
        outsolstat(rtk);
        return 1;
    }
    //动基线与其他差分定位方式，动基线的基站坐标需要随时间同步变化，所以需要计算出变化速率,
    //解释了为什么第二步除了单点定位，动基线也不参与基站解算，动基线在这里单独解算
    if (opt->mode==PMODE_MOVEB) { /*  moving baseline */    //若为移动基线模式
        
        /* estimate position/velocity of base station */    //spp计算基准站位置
        if (!pntpos(obs+nu,nr,nav,&rtk->opt,&solb,NULL,NULL,msg)) {
            errmsg(rtk,"base station position error (%s)\n",msg);
            return 0;
        }
        rtk->sol.age=(float)timediff(rtk->sol.time,solb.time);  //计算差分龄期rtk->sol.age
        
        if (fabs(rtk->sol.age)>TTOL_MOVEB) {
            errmsg(rtk,"time sync error for moving-base (age=%.1f)\n",rtk->sol.age);
            return 0;
        }
        for (i=0;i<6;i++) rtk->rb[i]=solb.rr[i];        //把solb.rr赋值给rtk->rb
        
        /* time-synchronized position of base station */    //时间同步
        for (i=0;i<3;i++) rtk->rb[i]+=rtk->rb[i+3]*rtk->sol.age;    //位置+=对应速度*差分龄期
    }
    else {
        rtk->sol.age=(float)timediff(obs[0].time,obs[nu].time);
        
        if (fabs(rtk->sol.age)>opt->maxtdiff) {
            errmsg(rtk,"age of differential error (age=%.1f)\n",rtk->sol.age);
            outsolstat(rtk);
            return 1;
        }
    }

    //上面的步骤只算了相对定位的差分时间和动基线坐标,这里进行相位定位，并输出最终结果，到这里定位步骤全部完成
    //相对定位算法的核心函数
    /* relative potitioning */
    relpos(rtk,obs,nu,nr,nav);
    outsolstat(rtk);
    
    return 1;
}

    //检查该历元流动站观测时间和基准站观测时间是否对应，若无基准站观测数据，return
    /* check number of data of base station and age of differential */
    if (nr==0) {
        errmsg(rtk,"no base station observation data for rtk\n");
        outsolstat(rtk);
        return 1;
    }
    //动基线与其他差分定位方式，动基线的基站坐标需要随时间同步变化，所以需要计算出变化速率,
    //解释了为什么第二步除了单点定位，动基线也不参与基站解算，动基线在这里单独解算
    if (opt->mode==PMODE_MOVEB) { /*  moving baseline */    //若为移动基线模式
        
        /* estimate position/velocity of base station */    //spp计算基准站位置
        if (!pntpos(obs+nu,nr,nav,&rtk->opt,&solb,NULL,NULL,msg)) {
            errmsg(rtk,"base station position error (%s)\n",msg);
            return 0;
        }
        rtk->sol.age=(float)timediff(rtk->sol.time,solb.time);  //计算差分龄期rtk->sol.age
        
        if (fabs(rtk->sol.age)>TTOL_MOVEB) {
            errmsg(rtk,"time sync error for moving-base (age=%.1f)\n",rtk->sol.age);
            return 0;
        }
        for (i=0;i<6;i++) rtk->rb[i]=solb.rr[i];        //把solb.rr赋值给rtk->rb
        
        /* time-synchronized position of base station */    //时间同步
        for (i=0;i<3;i++) rtk->rb[i]+=rtk->rb[i+3]*rtk->sol.age;    //位置+=对应速度*差分龄期
    }
    else {
        rtk->sol.age=(float)timediff(obs[0].time,obs[nu].time);
        
        if (fabs(rtk->sol.age)>opt->maxtdiff) {
            errmsg(rtk,"age of differential error (age=%.1f)\n",rtk->sol.age);
            outsolstat(rtk);
            return 1;
        }
    }

    //上面的步骤只算了相对定位的差分时间和动基线坐标,这里进行相位定位，并输出最终结果，到这里定位步骤全部完成
    //相对定位算法的核心函数
    /* relative potitioning */
    relpos(rtk,obs,nu,nr,nav);
    outsolstat(rtk);
    
    return 1;
}
```



















