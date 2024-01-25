

[TOC]

## 一、后处理程序使用

> 再次推荐一下 B 站赵老师的[视频讲解](https://space.bilibili.com/479790048?spm_id_from=333.337.search-card.all.click)，看视频学软件操作更直观。我主要是用 RTKLIB 的代码库，自带的程序除了 rtkplot 之外我都没咋用过，下面写的内容是看赵老师视频的时候做的笔记。

### 1、

* 在进行解算前就可以用 RTKPLOT 可以对数据进行分析，最新的 b34 版本的 RTKPLOT 好像有 bug，可以去下载一个别的版本的。

* 通过 RTKGET 可以下载 IGS 观测数据和各种改正产品（我没咋用过）。





### 2、后处理数据获取







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

rnx2rtkp 程序比较复杂，要传入很多命令行参数，可以自己写主函数。再比如说我松组合程序，可以先调用 `postpos()` 通过GNSS原始原始数据算出定位解，然后与 INS 组合。

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