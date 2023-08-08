[TOC]

##  一、rnx2rtkp

> 配置选项可以参考本系列[上一篇博客](https://blog.csdn.net/daoge2666/article/details/129569948?spm=1001.2014.3001.5501)中的RTKPOS：

### 1、简介


* 使用方式：`rnx2rtkp [option]... file file [...] `。VS 中加 main 函数参数：在项目的 配置属性-调试-命令参数 中设置。

* 读取RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。

* 对与相对定位，第一个OBS观测值文件需含接收机、流动站观测值，第二个OBS文件需含基准站观测值。

* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。

* 想用SP3精密星历文件，需提供.sp3/.eph文件的路径。

* 输入文件路径可包含通配符*，为了防止与命令行命令冲突，要用\"...\" 括起带通配符符路径。

  

### 2、main函数参数

1. **-？、-**：打印help
2. **-k** file：配置文件的输入选项，默认值是[off]
3. **-o** file：输出文件选项，默认值是[stdout]
4. **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是[obs start time] 
5. **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是[obs end time] 
6. **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
7. **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是[2]
8. **-m** mask：设置截止高度角，`(deg) `,默认值是[15]
9. **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是[G|R] 
10. **-f** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是[2]
11. **-v** thres：设置整周模糊度Ratio值，写0.0为不固定整周模糊度，默认值是[3.0] 
12. **-b**：后向滤波
13. **-c**：前后向滤波组合
14. **-i**：单历元模糊度固定instantaneous 
15. **-h**：fix and hold模糊度固定
16. **-e**：输出XYZ-ecef坐标
17. **-a**：输出ENU-baseline
18. **-n**：输出NMEA-0183 GGA
19. **-g**：输出经纬度格式为ddd mm ss.ss ，默认为[ddd.ddd] 
20. **-t**：输出时间格式为yyyy/mm/dd hh:mm:ss.ss ，默认为[sssss.ss] 
21. **-u**：输出为UTC时间，默认为[gpst] 
22. **-d** col：设置时间的小数位数，默认为[3] 
23. **-s** sep：设置文件分隔符，要写在单引号中，默认为[' '] 
24. **-r** x y z：基站位置ECEF-XYZ (m)，默认[average of single pos] ，流动站位置用于fixed模式
25. **-l** lat lon hgt：基站位置LLH (deg/m)，默认[average of single pos]，流动站位置用于fixed模式
26. **-y** level：输出结果信息(**0**:off,**1**:states,**2**:residuals) ，默认为[0] 
27. **-x** level：输出debug trace等级，默认为[0] 



### 3、main函数题解读

> 我根据自己的理解给代码加了中文注释

* 读取配置文件过程
  * 循环判断参数是否有-k
  * 创建porcopt_t/solopt_t/filopt_t 变量用于接受读取到的配置
  * resetsysopts() ，重置配置为默认
  * loadopts()，从文件中读取配置，存到opt_t类型的sysopt中
  * getsysopts()，把opt_t类型的sysopt转到porcopt_t/solopt_t/filopt_t 变量中，会调用buff2sysopts() 
* 读其它参数
  * 循环内，if-else if，判断参数，根据参数赋值
  * 若都不是参数，最后一个到else if，认为是文件路径，用infile数组接收
* 调用postpos()后处理解算

```c
int main(int argc, char **argv)
{
    prcopt_t prcopt=prcopt_default;     //定位处理模式
    solopt_t solopt=solopt_default;     //结果输出形式
    filopt_t filopt={""};               //文件路径选项
    gtime_t ts={0},te={0};              //ts开始时间、te结束时间
    double tint=0.0,
            es[]={2000,1,1,0,0,0},
            ee[]={2000,12,31,23,59,59},
            pos[3];
    int i,  //for循环的计数
        j,  //嵌套的for循环计数
        n,  //记录读入文件数
        ret;    //接受postpos的返回值
    char *infile[MAXFILE],  //读入文件，默认最多16个，可改MAXFILE定义
         *outfile="",       //输出文件
         *p;                //指向字符串的指针，用于循环指向各main函数参数
    
    prcopt.mode  =PMODE_KINEMA;     //定位模式默认动态相对定位Kinematic
    prcopt.navsys=0;        //卫星系统，先设置无
    prcopt.refpos=1;        //基准站坐标
    prcopt.glomodear=1;     //GLONASS AR mode,先设on
    solopt.timef=0;         //输出时间格式，先设sssss.s
    sprintf(solopt.prog ,"%s ver.%s %s",PROGNAME,VER_RTKLIB,PATCH_LEVEL);   //项目名称
    sprintf(filopt.trace,"%s.trace",PROGNAME);
    
    /* load options from configuration file */
    for (i=1;i<argc;i++) {
        if (!strcmp(argv[i],"-k")&&i+1<argc) {  //如果有-k和配置文件输入
            resetsysopts();         //先重置所有配置
            if (!loadopts(argv[++i],sysopts)) return -1;    //再读取配置文件内容，存入opt_t的sysopt中
            getsysopts(&prcopt,&solopt,&filopt);    //opt_t转到porcopt_t/solopt_t/filopt_t，
        }   
    }                  
    for (i=1,n=0;i<argc;i++) {  //循环判断main函数参数
        if      (!strcmp(argv[i],"-o")&&i+1<argc) outfile=argv[++i];//读取输出文件路径，赋值给outfile
        else if (!strcmp(argv[i],"-ts")&&i+2<argc) {    //读取开始解算时间   
            sscanf(argv[++i],"%lf/%lf/%lf",es,es+1,es+2);
            sscanf(argv[++i],"%lf:%lf:%lf",es+3,es+4,es+5);
            ts=epoch2time(es);      //转为gtime_t
        }
        else if (!strcmp(argv[i],"-te")&&i+2<argc) {    //读取结束解算时间
            sscanf(argv[++i],"%lf/%lf/%lf",ee,ee+1,ee+2);   
            sscanf(argv[++i],"%lf:%lf:%lf",ee+3,ee+4,ee+5);
            te=epoch2time(ee);  //转为gtime_t
        }
        else if (!strcmp(argv[i],"-ti")&&i+1<argc) tint=atof(argv[++i]);    //读取解算时间间隔频率
        else if (!strcmp(argv[i],"-k")&&i+1<argc) {++i; continue;}  //有-k，跳过
        else if (!strcmp(argv[i],"-p")&&i+1<argc) prcopt.mode=atoi(argv[++i]);  //读取解算模式
        else if (!strcmp(argv[i],"-f")&&i+1<argc) prcopt.nf=atoi(argv[++i]);    //读取用于计算的频率
        else if (!strcmp(argv[i],"-sys")&&i+1<argc) {       //读取用于计算的导航系统
            for (p=argv[++i];*p;p++) {      
                switch (*p) {   //有对应导航系统，就把它的码做与运算加上                 
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
        else if (!strcmp(argv[i],"-m")&&i+1<argc) prcopt.elmin=atof(argv[++i])*D2R;     //设置截止高度角     
        else if (!strcmp(argv[i],"-v")&&i+1<argc) prcopt.thresar[0]=atof(argv[++i]);    //设置整周模糊度Ratio值
        else if (!strcmp(argv[i],"-s")&&i+1<argc) strcpy(solopt.sep,argv[++i]);         //设置文件路径分隔符
        else if (!strcmp(argv[i],"-d")&&i+1<argc) solopt.timeu=atoi(argv[++i]);         //设置时间小数位数
        else if (!strcmp(argv[i],"-b")) prcopt.soltype=1;   //后向滤波
        else if (!strcmp(argv[i],"-c")) prcopt.soltype=2;   //前后向滤波组合
        else if (!strcmp(argv[i],"-i")) prcopt.modear=2;    //单历元模糊度固定
        else if (!strcmp(argv[i],"-h")) prcopt.modear=3;    //fix and hold模糊度固定
        else if (!strcmp(argv[i],"-t")) solopt.timef=1;     //输出时间格式为yyyy/mm/dd hh:mm:ss.ss
        else if (!strcmp(argv[i],"-u")) solopt.times=TIMES_UTC;    //输出为UTC时间
        else if (!strcmp(argv[i],"-e")) solopt.posf=SOLF_XYZ;      //输出XYZ-ecef坐标
        else if (!strcmp(argv[i],"-a")) solopt.posf=SOLF_ENU;      //输出ENU-baseline
        else if (!strcmp(argv[i],"-n")) solopt.posf=SOLF_NMEA;     //输出NMEA-0183 GGA
        else if (!strcmp(argv[i],"-g")) solopt.degf=1;             //输出经纬度格式为ddd mm ss.ss
        else if (!strcmp(argv[i],"-r")&&i+3<argc) { //基站位置ECEF-XYZ (m)              
            prcopt.refpos=prcopt.rovpos=0;          //基准站和流动站位置都先设0
            for (j=0;j<3;j++) prcopt.rb[j]=atof(argv[++i]); //循环存入基准站坐标
            matcpy(prcopt.ru,prcopt.rb,3,1);    //
        }
        else if (!strcmp(argv[i],"-l")&&i+3<argc) {     //循环存入基站位置基站位置LLH (deg/m)
            prcopt.refpos=prcopt.rovpos=0;              //基准站和流动站位置都先设0
            for (j=0;j<3;j++) pos[j]=atof(argv[++i]);   
            for (j=0;j<2;j++) pos[j]*=D2R;           //角度转弧度   
            pos2ecef(pos,prcopt.rb);        //LLH转XYZ
            matcpy(prcopt.ru,prcopt.rb,3,1);
        }
        else if (!strcmp(argv[i],"-y")&&i+1<argc) solopt.sstat=atoi(argv[++i]); //输出结果信息
        else if (!strcmp(argv[i],"-x")&&i+1<argc) solopt.trace=atoi(argv[++i]); //输出debug trace等级
        else if (*argv[i]=='-') printhelp();    //输入-，打印帮助
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

## 二、自己写后处理主函数

> 转载一篇写RTKLIB后处理main函数的博客，也推荐去[原网站](https://www.cnblogs.com/ahulh/p/15584998.html)看，页面显示效果特别好。
### 1、精密单点定位

**所选用的数据为[武汉大学IGS数据中心](http://www.igs.gnsswhu.cn/index.php/home/data_product/igs.html)上下载的2020年9月25日长春站（chan）的观测数据，同时下载了当天的广播星历以及精密星历。（chan2690.20o、brdc2690.20n、igs21245.sp3）** 

```c
int main() {
	int i, n, ret;
	double tint = 0.0;       /* 求解时间间隔(0:默认) */
	gtime_t ts = { 0 }, te = { 0 }; /* 历元时段始末控制变量 */
	char *infile[MAXFILE], outfile[MAXSTRPATH] = { '\0' };
	char resultpath[MAXSTRPATH] = "H:\\20211108\\result"; /* 结果输出路径 */
	char sep = (char)FILEPATHSEP;
	prcopt_t prcopt = prcopt_default; /* 默认处理选项设置 */
	solopt_t solopt = solopt_default; /* 默认求解格式设置 */
	filopt_t filopt = { /* 参数文件路径设置 */
		"", /* 卫星天线参数文件 */
		"", /* 接收机天线参数文件 */
		"", /* 测站位置文件 */
		"", /* 扩展大地水准面数据文件 */
		"", /* 电离层数据文件 */
		"", /* DCB数据文件 */
		"", /* 地球自转参数文件 */
		"", /* 海洋潮汐负荷文件 */
	};
	char infile_[MAXFILE][MAXSTRPATH] = {
		"H:\\20211108\\chan2690.20o",
		"H:\\20211108\\brdc2690.20n",
		"H:\\20211108\\igs21245.sp3",
		"",
		"",
		"",
		"",
		""
	};
	long t1, t2;
	double eps[]={2020,9,25,0,0,0},epe[]={2020,9,25,23,0,0}; /* 设置计算的历元时段 */
	ts=epoch2time(eps);te=epoch2time(epe);

	for (i = 0, n = 0; i < MAXFILE; i++)
		if (strcmp(infile_[i], "")) infile[n++] = &infile_[i][0];

	sprintf(outfile, "%s%c", resultpath, sep);//设置输出路径

	/* 自定义求解格式 --------------------------------------------------------*/
	solopt.posf = SOLF_XYZ;   /* 选择输出的坐标格式，经纬度或是XYZ坐标等 */
	solopt.times  =TIMES_UTC; /* 控制输出解的时间系统类型 */
	solopt.degf   =0;         /* 输出经纬度格式(0:°, 1:°′″) */
	solopt.outhead=1;         /* 是否输出头文件(0:否,1:是) */
	solopt.outopt =1;         /* 是否输出prcopt变量(0:否,1:是) */
	solopt.height =1;         /* 高程(0:椭球高,1:大地高) */

	/* 自定义处理选项设置 ----------------------------------------------------*/
	prcopt.mode = PMODE_PPP_KINEMA; /* PPP动态处理 */
	prcopt.modear = 4;     /* 求解模糊度类型 */
	prcopt.sateph = EPHOPT_PREC;      /* 使用精密星历 */
	prcopt.ionoopt = IONOOPT_IFLC;     /* 使用双频消电离层组合模型 */
	prcopt.tropopt = TROPOPT_EST;      /* 使用对流层天顶延迟估计模型 */
	prcopt.tidecorr = 0; /* 地球潮汐改正选项(0:关闭,1:固体潮,2:固体潮+?+极移) */
	prcopt.posopt[0] = 0; /* 卫星天线模型 */
	prcopt.posopt[1] = 0; /* 接收机天线模型 */
	prcopt.posopt[2] = 0; /* 相位缠绕改正 */
	prcopt.posopt[3] = 0; /* 排除掩星 */
	prcopt.posopt[4] = 0; /* 求解接收机坐标出错后的检查选项 */
	prcopt.navsys = SYS_GPS; /* 处理的导航系统 */
	sprintf(outfile, "%s%cChan200925.pos", resultpath, sep); /* 输出结果名称 */
	prcopt.nf       =2;       /* 参与计算的载波频率个数 */
	prcopt.elmin    =10.0*D2R;/* 卫星截止高度角 */
	prcopt.soltype  =0;       /* 求解类型(0:向前滤波,1:向后滤波,2:混合滤波) */

	t1 = clock();
	ret = postpos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infile, n, outfile, "", "");
	t2 = clock();

	if (!ret) fprintf(stderr, "%40s\r", "");

	printf("\n * The total time for running the program: %6.3f seconds\n%c", (double)(t2 - t1) / CLOCKS_PER_SEC, '\0');
	printf("Press any key to exit!\n");
	getchar();
	return ret;
}
```

### 2、差分定位

**所选用的数据为武汉大学IGS数据中心上下载的2020年9月25日长春站（chan）与北京佛山站（bjfs）的观测数据，同时下载了当天的广播星历以及精密星历。（chan2690.20o、bjfs2690.20o、brdc2690.20n、igs21245.sp3）**

> **在差分定位中，需要至少两个站的观测值，在RTKLIB中只有第一个观测值文件会被当做移动站处理，这里笔者将chan作为移动站，bjfs作为基准站**

```c
int main() {
	int i, n, ret;
	double tint = 0.0;       /* 求解时间间隔(0:默认) */
	gtime_t ts = { 0 }, te = { 0 }; /* 历元时段始末控制变量 */
	char *infile[MAXFILE], outfile[MAXSTRPATH] = { '\0' };
	char resultpath[MAXSTRPATH] = "H:\\20211108\\result"; /* 结果输出路径 */
	char sep = (char)FILEPATHSEP;
	prcopt_t prcopt = prcopt_default; /* 默认处理选项设置 */
	solopt_t solopt = solopt_default; /* 默认求解格式设置 */
	filopt_t filopt = { /* 参数文件路径设置 */
		"", /* 卫星天线参数文件 */
		"", /* 接收机天线参数文件 */
		"", /* 测站位置文件 */
		"", /* 扩展大地水准面数据文件 */
		"", /* 电离层数据文件 */
		"", /* DCB数据文件 */
		"", /* 地球自转参数文件 */
		"", /* 海洋潮汐负荷文件 */
	};
	char infile_[MAXFILE][MAXSTRPATH] = { /* 前面观测值为移动站,后面观测值为基准站 */
		"H:\\20211108\\chan2690.20o",
		"H:\\20211108\\brdc2690.20n",
		"H:\\20211108\\igs21245.sp3",
		"H:\\20211108\\bjfs2690.20o",
		"",
		"",
		"",
		""
	};
	long t1, t2;
	double eps[]={2020,9,25,0,0,0},epe[]={2020,9,25,23,0,0}; /* 设置计算的历元时段 */
	ts=epoch2time(eps);te=epoch2time(epe);

	for (i = 0, n = 0; i < MAXFILE; i++)
		if (strcmp(infile_[i], "")) infile[n++] = &infile_[i][0];

	sprintf(outfile, "%s%c", resultpath, sep);

	/* 自定义求解格式 --------------------------------------------------------*/
	solopt.timef = 1;       /* 时间格式(0:sssss.s, 1:yyyy/mm/dd hh:mm:ss.s) */
	solopt.outhead = 1;       /* 是否输出头文件(0:否,1:是) */
	solopt.posf = SOLF_XYZ;  /* 输出的坐标格式 */
	//solopt.sstat =1;         /* 输出状态文件 */
	solopt.times =TIMES_UTC; /* 控制输出解的时间系统类型 */
	//solopt.degf  =0;         /* 输出经纬度格式(0:°, 1:°′″) */
	solopt.outopt=1;         /* 是否输出prcopt变量(0:否,1:是) */
	solopt.height=1;         /* 高程(0:椭球高,1:大地高) */
	//solopt.sstat =1;         /* 输出求解状态 */

	/* 自定义处理选项设置 ----------------------------------------------------*/
	prcopt.mode = PMODE_DGPS; /* 差分GPS处理 */
	prcopt.modear = ARMODE_FIXHOLD;  /* 求解模糊度类型 */
	prcopt.sateph = EPHOPT_BRDC; /* 使用广播星历 */
	prcopt.ionoopt = IONOOPT_BRDC;  /* 使用广播电离层模型 */
	prcopt.tropopt = TROPOPT_SAAS;  /* 使用萨斯坦莫宁模型 */
	prcopt.refpos =3;       /* 相对模式中基站位置获得方式 */
							/* (0:pos in prcopt,  1:average of single pos, */
							/*  2:read from file, 3:rinex header, 4:rtcm pos) */
	//freqindex     =0;       /* 单频计算时设置所用计算的波段 */
	prcopt.nf     =2;       /* 参与计算的载波频率个数 L1+L2*/
	//prcopt.glomodear =2;
	//prcopt.thresar[0]=2;
	//prcopt.posopt[4]=1;     /* 求解接收机坐标出错后的检查选项 */
	//prcopt.intpref=1;
	prcopt.elmin  =10.0*D2R;/* 卫星截止高度角 */
	prcopt.soltype=0;       /* 求解类型(0:向前滤波,1:向后滤波,2:混合滤波) */


	prcopt.navsys = SYS_GPS; /* 处理的导航系统 */
	sprintf(outfile, "%s%cchan200925DGPS.pos", resultpath, sep); /* 输出结果名称 */

	//prcopt.navsys =SYS_GPS|SYS_CMP; /* 处理的导航系统 */
	//sprintf(outfile,"%s%cresult_MIX.txt",resultpath,sep);

	t1 = clock();
	ret = postpos(ts, te, tint, 0.0, &prcopt, &solopt, &filopt, infile, n, outfile, "", "");
	t2 = clock();

	if (!ret) fprintf(stderr, "%40s\r", "");

	printf("\n * The total time for running the program: %6.3f seconds\n%c", (double)(t2 - t1) / CLOCKS_PER_SEC, '\0');
	printf("Press any key to exit!\n");
	getchar();
	return ret;
}
```

## 三、C语言字符串处理、文件IO知识补充

### 1、C语言字符串的存储
C语言没有专门用于存储字符串的数据类型，字符串都存在char类型的数组里，数组由连续的存储单元组成，每个存储单元存一个字符，C语言字符串末尾用`\0`表示字符串的结束。

#### 1.字符串常量
用双引号扩起来的内容，编译时自动在末尾加`\0`

####  2.字符串变量
字符串变量一般有两种方法创建，指针或者是数组 

     ```c
     char a[]="hello world !";   //数组
     char* b=" hello world !";   //指针
     ```

   - **数组形式**：初始化数组把静态存储区的字符串拷贝到数组中，你的数组得到的只是它的副本 。

   - **指针形式**：初始化指针只把字符串的地址拷贝给指针，你得的是静态存储区字符串的地址，**用指针你不能修改字符串字面量** ，如果你打算修改字符串，就不要用指针去初始化。 你用多个指针初始化的字符串字面量是一样的，它们得到的是同一个地址 。

### 2、处理字符串的函数

   > 大多数用到`string.h`头文件

   - **sscanf**和**sprintf**效果很强大，见博客：[sscanf函数和sprintf函数](http://t.csdn.cn/vbkiW)

   - **gets()**：接受从终端输入的一个字符串，以`/0`结束

   - **puts()**：将一个字符串输出到终端，以`/0`结束

   - **strcat()**：字符串连接，结果存在str1中

     ```c
     char str1[10] = "aa";
     char str2[10] = "bb";
     
     strcat(str1,str2);
     puts(str1);
     ```

 - **strcpy()**：字符串复制，将str2内容复制到str1中

    **strncpy()**：复制前n个字符串

    > 注意：str2会覆盖str1中全部的字符，str2长度不能超过str1

    ```c
    char str1[10] = " ";
    char str2[10] = "bb";
    
    strcpy(str1,str2);
    puts(str1);
    ```

 - **strcmp()**：字符串比较，相同返回0，str1>str2返回1，str1<str2返回-1

    **strncmp()**：比较前n个字符

    **stricmp()**：忽略大小写比较

    ```c
    char str1[10] = "abc";
    char str2[10] = "bbb";
    
    int n = strcmp(str1,str2);
    printf("%d\n", n);
    ```

 - **strlen()**：返回字符串长度，不计`/0`

  - **strlow()**：字符串转小写

    **strupr()**：字符串转大写

 - **strstr()**：返回查找str2中str1首次出现的位置，没出现返回NULL

    **strrstr()**：在字符串中反向查找

    **strchr()**：在字符串中找字符

    **strrchr()**：在字符中找字符，成功返回该位置到字符串结尾的所有字符

  - **strtok()**：字符串分割

    > 1. 功能：在字符串中查找指定标记：如在（add@acdd#sd$）中，查找标记：（@#￥），然后把标记隔开的字符打印；
    > 2. 第一个参数是字符串，它包含了0个或者多个由sep字符串中一个或者多个分隔符分隔标记；
    > 3. strtok函数找到str中的标记符，并将它替换为’\0’，并且用一个指针保存这个地址，然后返回这个指针；strtok函数会改变被操作的字符串，所以在使用之前应该拷贝所要操作的字符串，然后传递拷贝的字符串给strtok函数，这样目标字符串就不会被改变；
    > 4. strtok函数第一个参数不为NULL时，函数讲找到str字符串中的第一个标记，strtok函数将保存它在字符串中的位置；
    > 5. strtok函数的第一个参数为NULL时，函数将在同一个字符串中被保存的位置开始，查找下一个标记；
    > 6. 如果字符串中不存在更多的标记，则返回NULL指针；

    ```c
    int main()
    {
    		char arr[] = "zyth@guhdh.tech";
    		char* p ="@ .";
    		char tmp[20] ={0};
    		strcpy(tmp,arr);
    		char* ret = NULL;
    		ret = strtok(tmp,p);
    		printf("%s\n",ret);
    		ret = strtok(NULL,p);
    		printf("%s\n",ret);
    			ret = strtok(NULL,p);
    		printf("%s\n",ret);
    //进阶用法：
    		for(ret = strtok(tmp,p);ret != NULL;ret = strtok(NULL,p))
    		{
    			printf("%s\n",ret);
    		}
    ｝
    ```

* **atoi()**：把字符串转为整型，atoi()函数的功能：将字符串转换成整型数；atoi()会扫描参数nptr字符串，跳过前面的空格字符，直到遇上数字或正负号才开始做转换，而再遇到非数字或字符串时（'\0'）才结束转化，并将结果返回（返回转换后的整型数）。 
* **atof()**：把字符串转为浮点型double，名字来源于 ascii to floating point numbers 的缩写，它会扫描参数str字符串，跳过前面的空白字符（例如空格，tab缩进等，可以通过 [isspace()](http://c.biancheng.net/cpp/html/120.html) 函数来检测），直到遇上数字或正负符号才开始做转换，而再遇到非数字或字符串结束时('\0')才结束转换，并将结果返回。参数str 字符串可包含正负号、小数点或E(e)来表示指数部分，如123. 456 或123e-2 



> 文件IO：
>
> * Linux文件IO参考博客：[c语言系统编程二：文件IO操做](http://t.csdn.cn/FZopQ)
> * Windows文件IO参考博客：[C语言的文件IO操作,非常详细!!](http://t.csdn.cn/KnvfQ)



## 四、Option/Configuration文件读取

### 1、Option文件格式介绍

* 配置文件包含了processing options、solution options、file options三大块，用于RTKNAVI、RTKPOST、RTKRCV、RNX2RTKP。

* 文件中都以**Keyword = Value**形式记录不同的配置项。

* 对于枚举选项，可选值是选项序号(0,1,2,...) 或选项字符串(off, on, ...)。 

* 以#开头的行和行中#之后的文本被视为注释。

  ```python
  # RTKNAVI options (2013/03/01 10:41:04, v.2.4.2)
  
  pos1-posmode       =single     # (0:single,1:dgps,2:kinematic,3:static,4:movingbase,5:fixed,6:ppp-kine,7:ppp-static)
  pos1-frequency     =l1+l2      # (1:l1,2:l1+l2,3:l1+l2+l5)
  pos1-soltype       =forward    # (0:forward,1:backward,2:combined)
  pos1-elmask        =10         # (deg)
  pos1-snrmask_r     =off        # (0:off,1:on)
  pos1-snrmask_b     =off        # (0:off,1:on)
  pos1-snrmask_L1    =0,0,0,0,0,0,0,0,0
  pos1-snrmask_L2    =0,0,0,0,0,0,0,0,0
  pos1-snrmask_L5    =0,0,0,0,0,0,0,0,0
  pos1-dynamics      =off        # (0:off,1:on)
  pos1-tidecorr      =off        # (0:off,1:on)
  pos1-ionoopt       =brdc       # (0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc,7:qzs-lex,8:vtec_sf,9:vtec_ef,10:gtec)
  pos1-tropopt       =saas       # (0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad)
  pos1-sateph        =brdc       # (0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom)
  pos1-posopt1       =on         # (0:off,1:on)
  pos1-posopt2       =on         # (0:off,1:on)
  pos1-posopt3       =on         # (0:off,1:on)
  pos1-posopt4       =on         # (0:off,1:on)
  pos1-posopt5       =off        # (0:off,1:on)
  pos1-exclsats      =           # (prn ...)
  pos1-navsys        =63         # (1:gps+2:sbas+4:glo+8:gal+16:qzs+32:comp)
  pos2-armode        =fix-and-hold # (0:off,1:continuous,2:instantaneous,3:fix-and-hold)
  pos2-gloarmode     =off        # (0:off,1:on,2:autocal)
  pos2-arthres       =3
  pos2-arlockcnt     =0
  pos2-arelmask      =20         # (deg)
  pos2-arminfix      =0
  pos2-elmaskhold    =0          # (deg)
  pos2-aroutcnt      =5
  pos2-maxage        =30         # (s)
  pos2-slipthres     =0.05       # (m)
  pos2-rejionno      =30         # (m)
  pos2-rejgdop       =30
  pos2-niter         =1
  pos2-baselen       =0          # (m)
  pos2-basesig       =0          # (m)
  out-solformat      =llh        # (0:llh,1:xyz,2:enu,3:nmea)
  out-outhead        =off        # (0:off,1:on)
  out-outopt         =off        # (0:off,1:on)
  out-timesys        =gpst       # (0:gpst,1:utc,2:jst)
  out-timeform       =hms        # (0:tow,1:hms)
  out-timendec       =3
  out-degform        =deg        # (0:deg,1:dms)
  out-fieldsep       =
  out-height         =geodetic   # (0:ellipsoidal,1:geodetic)
  out-geoid          =internal   # (0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000)
  out-solstatic      =all        # (0:all,1:single)
  out-nmeaintv1      =1          # (s)
  out-nmeaintv2      =1          # (s)
  out-outstat        =off        # (0:off,1:state,2:residual)
  stats-eratio1      =300
  stats-eratio2      =300
  stats-errphase     =0.003      # (m)
  stats-errphaseel   =0.003      # (m)
  stats-errphasebl   =0          # (m/10km)
  stats-errdoppler   =1          # (Hz)
  stats-stdbias      =30         # (m)
  stats-stdiono      =0.03       # (m)
  stats-stdtrop      =0.3        # (m)
  stats-prnaccelh    =10         # (m/s^2)
  stats-prnaccelv    =10         # (m/s^2)
  stats-prnbias      =0.0001     # (m)
  stats-prniono      =0.001      # (m)
  stats-prntrop      =0.0001     # (m)
  stats-clkstab      =5e-12      # (s/s)
  ant1-postype       =llh        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
  ant1-pos1          =90         # (deg|m)
  ant1-pos2          =0          # (deg|m)
  ant1-pos3          =-6335367.6285 # (m|m)
  ant1-anttype       =NOV702GG
  ant1-antdele       =0          # (m)
  ant1-antdeln       =0          # (m)
  ant1-antdelu       =0          # (m)
  ant2-postype       =rtcm       # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm)
  ant2-pos1          =0          # (deg|m)
  ant2-pos2          =0          # (deg|m)
  ant2-pos3          =0          # (m|m)
  ant2-anttype       =TRM29659.00
  ant2-antdele       =0          # (m)
  ant2-antdeln       =0          # (m)
  ant2-antdelu       =0          # (m)
  misc-timeinterp    =off        # (0:off,1:on)
  misc-sbasatsel     =0          # (0:all)
  misc-rnxopt1       =
  misc-rnxopt2       =
  file-satantfile    =Y:\madoca\data\igs08.atx
  file-rcvantfile    =Y:\madoca\data\igs08.atx
  file-staposfile    =
  file-geoidfile     =
  file-ionofile      =
  file-dcbfile       =Y:\madoca\data\dcb\P1P21201.DCB
  file-eopfile       =
  file-blqfile       =
  file-tempdir       =C:\Temp
  file-geexefile     =
  file-solstatfile   =
  file-tracefile     =
  
  inpstr1-type       =ntripcli   # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http)
  inpstr2-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http)
  inpstr3-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,7:ntripcli,8:ftp,9:http)
  inpstr1-path       =kaiyodai:tuomsat00@mgex.igs-ip.net:2101/CUT07:
  inpstr2-path       =
  inpstr3-path       =
  inpstr1-format     =rtcm3      # (0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,15:sp3)
  inpstr2-format     =rtcm3      # (0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,15:sp3)
  inpstr3-format     =rtcm3      # (0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,15:sp3)
  inpstr2-nmeareq    =off        # (0:off,1:latlon,2:single)
  inpstr2-nmealat    =26.37293571 # (deg)
  inpstr2-nmealon    =127.143649075 # (deg)
  outstr1-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr)
  outstr2-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr)
  outstr1-path       =
  outstr2-path       =
  outstr1-format     =llh        # (0:llh,1:xyz,2:enu,3:nmea)
  outstr2-format     =nmea       # (0:llh,1:xyz,2:enu,3:nmea)
  logstr1-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr)
  logstr2-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr)
  logstr3-type       =off        # (0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr)
  logstr1-path       =
  logstr2-path       =
  logstr3-path       =
  misc-svrcycle      =10         # (ms)
  misc-timeout       =30000      # (ms)
  misc-reconnect     =10000      # (ms)
  misc-nmeacycle     =5000       # (ms)
  misc-buffsize      =32768      # (bytes)
  misc-navmsgsel     =all        # (0:all,1:rover,2:base,3:corr)
  misc-proxyaddr     =
  misc-fswapmargin   =30         # (s)
  ```

  

### 2、存Option的类型

#### 1.prcopt_t结构体：存算法处理选项

  ```c
  typedef struct {        /* processing options type */
      int mode;           /* positioning mode (PMODE_???) */
      int soltype;        /* solution type (0:forward,1:backward,2:combined) */
      int nf;             /* number of frequencies (1:L1,2:L1+L2,3:L1+L2+L5) */
      int navsys;         /* navigation system */
      double elmin;       /* elevation mask angle (rad) */
      snrmask_t snrmask;  /* SNR mask */
      int sateph;         /* satellite ephemeris/clock (EPHOPT_???) */
      int modear;         /* AR mode (0:off,1:continuous,2:instantaneous,3:fix and hold,4:ppp-ar) */
      int glomodear;      /* GLONASS AR mode (0:off,1:on,2:auto cal,3:ext cal) */
      int bdsmodear;      /* BeiDou AR mode (0:off,1:on) */
      int maxout;         /* obs outage count to reset bias */
      int minlock;        /* min lock count to fix ambiguity */
      int minfix;         /* min fix count to hold ambiguity */
      int armaxiter;      /* max iteration to resolve ambiguity */
      int ionoopt;        /* ionosphere option (IONOOPT_???) */
      int tropopt;        /* troposphere option (TROPOPT_???) */
      int dynamics;       /* dynamics model (0:none,1:velociy,2:accel) */
      int tidecorr;       /* earth tide correction (0:off,1:solid,2:solid+otl+pole) */
      int niter;          /* number of filter iteration */
      int codesmooth;     /* code smoothing window size (0:none) */
      int intpref;        /* interpolate reference obs (for post mission) */
      int sbascorr;       /* SBAS correction options */
      int sbassatsel;     /* SBAS satellite selection (0:all) */
      int rovpos;         /* rover position for fixed mode */
      int refpos;         /* base position for relative mode */
                          /* (0:pos in prcopt,  1:average of single pos, */
                          /*  2:read from file, 3:rinex header, 4:rtcm pos) */
      double eratio[NFREQ]; /* code/phase error ratio */
      double err[5];      /* measurement error factor */
                          /* [0]:reserved */
                          /* [1-3]:error factor a/b/c of phase (m) */
                          /* [4]:doppler frequency (hz) */
      double std[3];      /* initial-state std [0]bias,[1]iono [2]trop */
      double prn[6];      /* process-noise std [0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos */
      double sclkstab;    /* satellite clock stability (sec/sec) */
      double thresar[8];  /* AR validation threshold */
      double elmaskar;    /* elevation mask of AR for rising satellite (deg) */
      double elmaskhold;  /* elevation mask to hold ambiguity (deg) */
      double thresslip;   /* slip threshold of geometry-free phase (m) */
      double maxtdiff;    /* max difference of time (sec) */
      double maxinno;     /* reject threshold of innovation (m) */
      double maxgdop;     /* reject threshold of gdop */
      double baseline[2]; /* baseline length constraint {const,sigma} (m) */
      double ru[3];       /* rover position for fixed mode {x,y,z} (ecef) (m) */
      double rb[3];       /* base position for relative mode {x,y,z} (ecef) (m) */
      char anttype[2][MAXANT]; /* antenna types {rover,base} */
      double antdel[2][3]; /* antenna delta {{rov_e,rov_n,rov_u},{ref_e,ref_n,ref_u}} */
      pcv_t pcvr[2];      /* receiver antenna parameters {rov,base} */
      uint8_t exsats[MAXSAT]; /* excluded satellites (1:excluded,2:included) */
      int  maxaveep;      /* max averaging epoches */
      int  initrst;       /* initialize by restart */
      int  outsingle;     /* output single by dgps/float/fix/ppp outage */
      char rnxopt[2][256]; /* rinex options {rover,base} */
      int  posopt[6];     /* positioning options */
      int  syncsol;       /* solution sync mode (0:off,1:on) */
      double odisp[2][6*11]; /* ocean tide loading parameters {rov,base} */
      int  freqopt;       /* disable L2-AR */
      char pppopt[256];   /* ppp option */
  } prcopt_t;
  ```

  

#### 2.solopt_t 结构体：存输出结果设置

  ```c
  typedef struct {        /* solution options type */
      int posf;           /* solution format (SOLF_???) */
      int times;          /* time system (TIMES_???) */
      int timef;          /* time format (0:sssss.s,1:yyyy/mm/dd hh:mm:ss.s) */
      int timeu;          /* time digits under decimal point */
      int degf;           /* latitude/longitude format (0:ddd.ddd,1:ddd mm ss) */
      int outhead;        /* output header (0:no,1:yes) */
      int outopt;         /* output processing options (0:no,1:yes) */
      int outvel;         /* output velocity options (0:no,1:yes) */
      int datum;          /* datum (0:WGS84,1:Tokyo) */
      int height;         /* height (0:ellipsoidal,1:geodetic) */
      int geoid;          /* geoid model (0:EGM96,1:JGD2000) */
      int solstatic;      /* solution of static mode (0:all,1:single) */
      int sstat;          /* solution statistics level (0:off,1:states,2:residuals) */
      int trace;          /* debug trace level (0:off,1-5:debug) */
      double nmeaintv[2]; /* nmea output interval (s) (<0:no,0:all) */
                          /* nmeaintv[0]:gprmc,gpgga,nmeaintv[1]:gpgsv */
      char sep[64];       /* field separator */
      char prog[64];      /* program name */
      double maxsolstd;   /* max std-dev for solution output (m) (0:all) */
  } solopt_t;
  ```

  

#### 3.filopt_t 结构体：存文件设置

  ```c
  typedef struct {        /* file options type */
      char satantp[MAXSTRPATH]; /* satellite antenna parameters file */
      char rcvantp[MAXSTRPATH]; /* receiver antenna parameters file */
      char stapos [MAXSTRPATH]; /* station positions file */
      char geoid  [MAXSTRPATH]; /* external geoid data file */
      char iono   [MAXSTRPATH]; /* ionosphere data file */
      char dcb    [MAXSTRPATH]; /* dcb data file */
      char eop    [MAXSTRPATH]; /* eop data file */
      char blq    [MAXSTRPATH]; /* ocean tide loading blq file */
      char tempdir[MAXSTRPATH]; /* ftp/http temporaly directory */
      char geexe  [MAXSTRPATH]; /* google earth exec file */
      char solstat[MAXSTRPATH]; /* solution statistics file */
      char trace  [MAXSTRPATH]; /* debug trace file */
  } filopt_t;
  ```

  

* 系统配置选项表：系统配置选项序号表，每条都是一个字符串，“选项序号：选项字符串，选项序号：选项字符串...” 

  ```c
  #define SWTOPT  "0:off,1:on"    
  #define MODOPT  "0:single,1:dgps,2:kinematic,3:static,4:movingbase,5:fixed,6:ppp-kine,7:ppp-static,8:ppp-fixed"
  #define FRQOPT  "1:l1,2:l1+2,3:l1+2+3,4:l1+2+3+4,5:l1+2+3+4+5"
  #define TYPOPT  "0:forward,1:backward,2:combined"
  #define IONOPT  "0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc"
  #define TRPOPT  "0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad"
  #define EPHOPT  "0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom"
  #define NAVOPT  "1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds+64:navic"
  #define GAROPT  "0:off,1:on"
  #define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea"
  #define TSYOPT  "0:gpst,1:utc,2:jst"
  #define TFTOPT  "0:tow,1:hms"
  #define DFTOPT  "0:deg,1:dms"
  #define HGTOPT  "0:ellipsoidal,1:geodetic"
  #define GEOOPT  "0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000"
  #define STAOPT  "0:all,1:single"
  #define STSOPT  "0:off,1:state,2:residual"
  #define ARMOPT  "0:off,1:continuous,2:instantaneous,3:fix-and-hold"
  #define POSOPT  "0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw"
  #define TIDEOPT "0:off,1:on,2:otl"
  #define PHWOPT  "0:off,1:on,2:precise"
  ```


#### 4.opt_t结构体：存一条有选项信息的结构体

  **opt_t数组sysopts**：存所有的选项 

  * 第一个值为选项名。
  * 第二个为选项内容格式，0int、1double、2string、3enum。
  * 第三个值为指向存配置选项内容（prcopt_t、solopt_t、filopt_t、antpos_t结构体内的字段）的指针。
  * 第四个值为选项的表示形式，格式、系统配置选项序号表。

  ```c
  typedef struct {        /* option type */
      const char *name;   /* option name */
      int format;         /* option format (0:int,1:double,2:string,3:enum) */
      void *var;          /* pointer to option variable */
      const char *comment; /* option comment/enum labels/unit */
  } opt_t;
  ```

  

* 开头的静态变量，配置选项缓冲区

  ```c
  static prcopt_t prcopt_;
  static solopt_t solopt_;
  static filopt_t filopt_;
  static int antpostype_[2];
  static double elmask_,elmaskar_,elmaskhold_;
  static double antpos_[2][3];
  static char exsats_[1024];
  static char snrmask_[NFREQ][1024];
  ```



### 3、options.c函数

#### 1.chop ()：去除#后的注释，把#替换为\0。

  ```c
  static void chop(char *str)
  {
      char *p;
      if ((p=strchr(str,'#'))) *p='\0'; /* comment */
      for (p=str+strlen(str)-1;p>=str&&!isgraph((int)*p);p--) *p='\0';
  }
  ```



#### 2.eunm2str()：把选项序号转为选项字符串。

#### 3.str2enum()：把选项字符串转为选项序号。

  * 如时间类型选项表：“0:gpst,1:utc,2:jst”，用enum2str()把0转为gpst，用str2enum()把gpst转为0.

  ```c
  static int enum2str(char *s, const char *comment, int val)  
  {
      char str[32],*p,*q;
      int n;
      
      n=sprintf(str,"%d:",val);           //把val选项序号转为字符串，并用n记录长度
      if (!(p=strstr(comment,str))) {     //在系统配置选项序号表查找val序号，P指针移动到对应位置
          return sprintf(s,"%d",val);     //找不到直接把val号转成字符串返回
      }
      if (!(q=strchr(p+n,','))&&!(q=strchr(p+n,')'))) {   //如果后面找不到“，”和“）”，那p+n以后的字符串就是选项序号要转为的选项字符串
          strcpy(s,p+n);              
          return (int)strlen(p+n);
      }
      strncpy(s,p+n,q-p-n); s[q-p-n]='\0';    //在p+n后面找到“，”位置为q，则选项序号要转为的字符串为p+n到q前
      return (int)(q-p-n);
  }
  ```

  ```c
  static int str2enum(const char *str, const char *comment, int *val)
  {
      const char *p;
      char s[32];
      
      for (p=comment;;p++) {
         if (!(p=strstr(p,str))) break;
         if (*(p-1)!=':') continue;
         for (p-=2;'0'<=*p&&*p<='9';p--) ;
         return sscanf(p+1,"%d",val)==1;
      }
      sprintf(s,"%.30s:",str);
      if ((p=strstr(comment,s))) { /* number */
          return sscanf(p,"%d",val)==1;
      }
      return 0;
  }
  ```




#### 4.searchopt()：根据选项名找选项
在opt_t数组中根据配置选项名找对应选项，找到了返回对应指针。

  ```c
  extern opt_t *searchopt(const char *name, const opt_t *opts)
  {
      int i;
      
      trace(3,"searchopt: name=%s\n",name);
      
      for (i=0;*opts[i].name;i++) {
          if (strstr(opts[i].name,name)) return (opt_t *)(opts+i);
      }
      return NULL;
  }
  ```

  

#### 5.str2opt()：把字符串转为对应的opt值
传入字符串，根据选项内容格式，把字符串转为对应的opt值。
#### 6.opt2str()：把opt值转为字符串（value）

  ```c
  extern int str2opt(opt_t *opt, const char *str)
  {
      switch (opt->format) {
          case 0: *(int    *)opt->var=atoi(str); break;
          case 1: *(double *)opt->var=atof(str); break;
          case 2: strcpy((char *)opt->var,str);  break;
          case 3: return str2enum(str,opt->comment,(int *)opt->var);
          default: return 0;
      }
      return 1;
  }
  ```

  

#### 7.opt2buf()：把opt转为字符串（keyword=value # comment）

  ```c
  extern int opt2buf(const opt_t *opt, char *buff)
  {
      char *p=buff;
      int n;
      
      trace(3,"opt2buf : name=%s\n",opt->name);
      
      p+=sprintf(p,"%-18s =",opt->name);
      p+=opt2str(opt,p);
      if (*opt->comment) {
          if ((n=(int)(buff+30-p))>0) p+=sprintf(p,"%*s",n,"");
          p+=sprintf(p," # (%s)",opt->comment);
      }
      return (int)(p-buff);
  }
  ```

  

#### 8.loadopts()：从文件中加载选项信息
之后还要用`getsysopts()`函数。
  ```c
  extern int loadopts(const char *file, opt_t *opts)
  {
      FILE *fp;              //创建文件指针
      opt_t *opt;
      char buff[2048],*p;
      int n=0;
      
      trace(3,"loadopts: file=%s\n",file);
      
      if (!(fp=fopen(file,"r"))) {    //以读的方式打开文件
          trace(1,"loadopts: options file open error (%s)\n",file);
          return 0;
      }
      while (fgets(buff,sizeof(buff),fp)) {   //循环用fgets读取文件，每次读buff-1=2048个字符，到buff中
          n++;
          chop(buff);     //去除fgets带来的/0
          
          if (buff[0]=='\0') continue; //如果没有内容，直接进行下一次循环
          
          if (!(p=strstr(buff,"="))) {    //如果找不到=，就输出错误
              fprintf(stderr,"invalid option %s (%s:%d)\n",buff,file,n);
              continue;
          }
          *p++='\0';
          chop(buff); //去除#后的注释
          if (!(opt=searchopt(buff,opts))) continue;  //在opt_t数组中根据配置选项名找对应选项
          
          if (!str2opt(opt,p)) {  //传入字符串，根据选项内容格式，把字符串转为对应的opt值
              fprintf(stderr,"invalid option value %s (%s:%d)\n",buff,file,n);
              continue;
          }
      }
      fclose(fp);     //关闭文件
      
      return 1;
  }
  ```
#### 9.saveopts()：保存选项信息到文件
之后还要用setsysopts()函数。



  ```c
  extern int saveopts(const char *file, const char *mode, const char *comment,
                      const opt_t *opts)
  {
      FILE *fp;
      char buff[2048];
      int i;
      
      trace(3,"saveopts: file=%s mode=%s\n",file,mode);
      
      if (!(fp=fopen(file,mode))) {
          trace(1,"saveopts: options file open error (%s)\n",file);
          return 0;
      }
      if (comment) fprintf(fp,"# %s\n\n",comment);
      
      for (i=0;*opts[i].name;i++) {
          opt2buf(opts+i,buff);
          fprintf(fp,"%s\n",buff);
      }
      fclose(fp);
      return 1;
  }
  ```

  

#### 10.resetsysopts()：重置选项到默认。

  ```c
  extern void resetsysopts(void)
  {
      int i,j;
      
      trace(3,"resetsysopts:\n");
      
      prcopt_=prcopt_default;
      solopt_=solopt_default;
      filopt_.satantp[0]='\0';
      filopt_.rcvantp[0]='\0';
      filopt_.stapos [0]='\0';
      filopt_.geoid  [0]='\0';
      filopt_.dcb    [0]='\0';
      filopt_.blq    [0]='\0';
      filopt_.solstat[0]='\0';
      filopt_.trace  [0]='\0';
      for (i=0;i<2;i++) antpostype_[i]=0;
      elmask_=15.0;
      elmaskar_=0.0;
      elmaskhold_=0.0;
      for (i=0;i<2;i++) for (j=0;j<3;j++) {
          antpos_[i][j]=0.0;
      }
      exsats_[0] ='\0';
  }
  ```




#### 11.buff2sysopts()：选项缓冲区转选项结构体
把选项缓冲区中`antpostype_ `,`elmask_`,`elmaskar_`,`elmaskhold_` ,`antpos_ `,`exsats_ `,`snrmask_ `中的值转到`antpostype_ `、`prcopt_ `等结构体中。

#### 12.sysopts2buff()：选项结构体转选项缓冲区

  ```c
  static void buff2sysopts(void)
  {
      double pos[3],*rr;
      char buff[1024],*p,*id;
      int i,j,sat,*ps;
      
      prcopt_.elmin     =elmask_    *D2R;
      prcopt_.elmaskar  =elmaskar_  *D2R;
      prcopt_.elmaskhold=elmaskhold_*D2R;
      
      for (i=0;i<2;i++) {
          ps=i==0?&prcopt_.rovpos:&prcopt_.refpos;
          rr=i==0?prcopt_.ru:prcopt_.rb;
          
          if (antpostype_[i]==0) { /* lat/lon/hgt */
              *ps=0;
              pos[0]=antpos_[i][0]*D2R;
              pos[1]=antpos_[i][1]*D2R;
              pos[2]=antpos_[i][2];
              pos2ecef(pos,rr);
          }
          else if (antpostype_[i]==1) { /* xyz-ecef */
              *ps=0;
              rr[0]=antpos_[i][0];
              rr[1]=antpos_[i][1];
              rr[2]=antpos_[i][2];
          }
          else *ps=antpostype_[i]-1;
      }
      /* excluded satellites */
      for (i=0;i<MAXSAT;i++) prcopt_.exsats[i]=0;
      if (exsats_[0]!='\0') {
          strcpy(buff,exsats_);
          for (p=strtok(buff," ");p;p=strtok(NULL," ")) {
              if (*p=='+') id=p+1; else id=p;
              if (!(sat=satid2no(id))) continue;
              prcopt_.exsats[sat-1]=*p=='+'?2:1;
          }
      }
      /* snrmask */
      for (i=0;i<NFREQ;i++) {
          for (j=0;j<9;j++) prcopt_.snrmask.mask[i][j]=0.0;
          strcpy(buff,snrmask_[i]);
          for (p=strtok(buff,","),j=0;p&&j<9;p=strtok(NULL,",")) {
              prcopt_.snrmask.mask[i][j++]=atof(p);
          }
      }
      /* number of frequency (4:L1+L5) */
      if (prcopt_.nf==4) {
          prcopt_.nf=3;
          prcopt_.freqopt=1;
      }
  }
  ```

  ```c
  static void sysopts2buff(void)
  {
      double pos[3],*rr;
      char id[32],*p;
      int i,j,sat,*ps;
      
      elmask_    =prcopt_.elmin     *R2D;
      elmaskar_  =prcopt_.elmaskar  *R2D;
      elmaskhold_=prcopt_.elmaskhold*R2D;
      
      for (i=0;i<2;i++) {
          ps=i==0?&prcopt_.rovpos:&prcopt_.refpos;
          rr=i==0?prcopt_.ru:prcopt_.rb;
          
          if (*ps==0) {
              antpostype_[i]=0;
              ecef2pos(rr,pos);
              antpos_[i][0]=pos[0]*R2D;
              antpos_[i][1]=pos[1]*R2D;
              antpos_[i][2]=pos[2];
          }
          else antpostype_[i]=*ps+1;
      }
      /* excluded satellites */
      exsats_[0]='\0';
      for (sat=1,p=exsats_;sat<=MAXSAT&&p-exsats_<(int)sizeof(exsats_)-32;sat++) {
          if (prcopt_.exsats[sat-1]) {
              satno2id(sat,id);
              p+=sprintf(p,"%s%s%s",p==exsats_?"":" ",
                         prcopt_.exsats[sat-1]==2?"+":"",id);
          }
      }
      /* snrmask */
      for (i=0;i<NFREQ;i++) {
          snrmask_[i][0]='\0';
          p=snrmask_[i];
          for (j=0;j<9;j++) {
              p+=sprintf(p,"%s%.0f",j>0?",":"",prcopt_.snrmask.mask[i][j]);
          }
      }
      /* number of frequency (4:L1+L5) */
      if (prcopt_.nf==3&&prcopt_.freqopt==1) {
          prcopt_.nf=4;
          prcopt_.freqopt=0;
      }
  }
  ```

  


####  13.getsysopts()：获取配置选项
`opt_t`转到`porcopt_t`/`solopt_t`/`filopt_t `，先用`loadopts()`函数从文件中读。

#### 14.setsysopts()：保存配置选项
先用`saveopts()`函数。
  ```c
  extern void getsysopts(prcopt_t *popt, solopt_t *sopt, filopt_t *fopt)
  {
      trace(3,"getsysopts:\n");
      
      buff2sysopts();
      if (popt) *popt=prcopt_;
      if (sopt) *sopt=solopt_;
      if (fopt) *fopt=filopt_;
  }
  ```

  ```c
  extern void setsysopts(const prcopt_t *prcopt, const solopt_t *solopt,
                         const filopt_t *filopt)
  {
      trace(3,"setsysopts:\n");
      
      resetsysopts();
      if (prcopt) prcopt_=*prcopt;
      if (solopt) solopt_=*solopt;
      if (filopt) filopt_=*filopt;
      sysopts2buff();
  }
  ```
## 五、Trace
> 在rtklib.h中加入 #define TRACE，启用trace ，不定义着将trace函数全赋空值，如下：

```c
extern void traceopen(const char *file) {}
extern void traceclose(void) {}
extern void tracelevel(int level) {}
extern void trace   (int level, const char *format, ...) {}
extern void tracet  (int level, const char *format, ...) {}
extern void tracemat(int level, const double *A, int n, int m, int p, int q) {}
extern void traceobs(int level, const obsd_t *obs, int n) {}
extern void tracenav(int level, const nav_t *nav) {}
extern void tracegnav(int level, const nav_t *nav) {}
extern void tracehnav(int level, const nav_t *nav) {}
extern void tracepeph(int level, const nav_t *nav) {}
extern void tracepclk(int level, const nav_t *nav) {}
extern void traceb  (int level, const uint8_t *p, int n) {}
```

### 1、rtkcmn.c关于trace的静态全局变量

  ```c
  static FILE *fp_trace=NULL;     //trace的文件指针
  static char file_trace[1024];   //trace文件名
  static int level_trace=0;       //trace等级（1-5)，等级越高输出的信息越多
  static uint32_t tick_trace=0;   //以毫米计的系统时间，在tracet()中用到：fprintf(fp_trace,"%d %9.3f: ",level,(tickget()-tick_trace)/1000.0);
  static gtime_t time_trace={0};  //打开trace的时间，获取的系统时间，并转为GPST
  static lock_t lock_trace;       //trace的进程锁
  ```
### 2、Trace相关函数
#### 1.trace()：将传入的trace格式化字符串写入trace文件

  ```c
  extern void trace(int level, const char *format, ...)
  {
      va_list ap;
      
      //如果trace等级小于1，写入错误信息到屏幕stderr
      /* print error message to stderr */
      if (level<=1) {
          va_start(ap,format); vfprintf(stderr,format,ap); va_end(ap);
      }
      //如果fp_trace为空，或当前trace操作等级高于设置的level_trace，直接返回
      if (!fp_trace||level>level_trace) return;
  
      traceswap();        //如果需要，分文件
      fprintf(fp_trace,"%d ",level);  //先写入trace等级
      va_start(ap,format); vfprintf(fp_trace,format,ap); va_end(ap);  //再写入传入的trace格式化字符串
      fflush(fp_trace);   //缓冲区内容写入文件,清空文件缓冲区
  }
  ```

#### 2.tracet()：写入带秒数的trace格式字符串
相比于trace多写入了trace开始后的秒数（ms级精度）

  ```c
  extern void tracet(int level, const char *format, ...)
  {
      va_list ap;
      
      if (!fp_trace||level>level_trace) return;
      traceswap();
      fprintf(fp_trace,"%d %9.3f: ",level,(tickget()-tick_trace)/1000.0); //相比于trace，多写入了trace开始后的秒数
      va_start(ap,format); vfprintf(fp_trace,format,ap); va_end(ap);
      fflush(fp_trace);
  }
  ```

#### 3.traceclose()：关闭trace文件描述符，将文件指针置空

  ```c
  extern void traceclose(void)
  {
      if (fp_trace&&fp_trace!=stderr) fclose(fp_trace);   //关闭trace文件描述符
      fp_trace=NULL;      //将文件指针置空
      file_trace[0]='\0';
  }
  ```

#### 4.traceopen()：创建或打开trace文件

1. 调用`utc2gpst(timeget()) `获取系统时间time，赋值给time_trace。
2. 调用`reppath() `替换传入trace路径的替换符。
3. 以读的方式创建trace文件，创建失败就用stderr当trace文件。
4. 调用`tickget() `，获取以毫米计的系统时间赋值给tick_trace，
5. 调用`initlock()`初始化lock_trace 。

  ```c
  extern void traceopen(const char *file)
  {
      gtime_t time=utc2gpst(timeget());   //获取系统时间，并转为GPST
      char path[1024];
      
      reppath(file,path,time,"","");      //替换file替换到path[]
      //以w方式打开文件，文件不存则创建，存在则重写，返回文件描述符fp_trace，失败就指向stderr
      if (!*path||!(fp_trace=fopen(path,"w"))) fp_trace=stderr;      
      strcpy(file_trace,file);
      tick_trace=tickget();
      time_trace=time;
      initlock(&lock_trace);
  }
  ```

#### 5.tracelevel()：将传入的trace等级赋值给level_trace 

  ```c
  extern void tracelevel(int level)
  {
      level_trace=level;
  }
  ```

#### 6.traceswap()：根据时间分trace文件

  ```c
  static void traceswap(void)
  {
      gtime_t time=utc2gpst(timeget());   //获取系统时间
      char path[1024];
      
      lock(&lock_trace);  //上锁
      
      //如果当前系统时间,如果当前时间的周内秒和当前trace文件的time_trace差距小于一天，直接return
      if ((int)(time2gpst(time      ,NULL)/INT_SWAP_TRAC)==
          (int)(time2gpst(time_trace,NULL)/INT_SWAP_TRAC)) {
          unlock(&lock_trace);    //解锁，return
          return;
      }
      //如果差别大，创建一个新的trace文件
      time_trace=time;    
      if (!reppath(file_trace,path,time,"","")) {
          unlock(&lock_trace);
          return;
      }
      if (fp_trace) fclose(fp_trace);
      
      if (!(fp_trace=fopen(path,"w"))) {
          fp_trace=stderr;
      }
      unlock(&lock_trace);    //解锁
  }
  ```

  

#### 7.tracemat()：写入矩阵
调用`matfprint()`，将矩阵写入文件，列优先顺序

  ```c
  extern void matfprint(const double A[], int n, int m, int p, int q, FILE *fp)
  {
      int i,j;
      
      for (i=0;i<n;i++) { //列
          for (j=0;j<m;j++)   //行
              fprintf(fp," %*.*f",p,q,A[i+j*n]);
          fprintf(fp,"\n");   //换行
      }
  }
  ```

#### 8.traceobs()：写入obsd_t
遍历`obsd_t`数组`obs`，输出信息

  ```c
  typedef struct {        /* observation data record */
      gtime_t time;       /* receiver sampling time (GPST) */
      uint8_t sat,rcv;    /* satellite/receiver number */
      uint16_t SNR[NFREQ+NEXOBS]; /* signal strength (0.001 dBHz) */  //信噪比
      uint8_t  LLI[NFREQ+NEXOBS]; /* loss of lock indicator */        //周跳
      uint8_t code[NFREQ+NEXOBS]; /* code indicator (CODE_???) */
      double L[NFREQ+NEXOBS]; /* observation data carrier-phase (cycle) */
      double P[NFREQ+NEXOBS]; /* observation data pseudorange (m) */
      float  D[NFREQ+NEXOBS]; /* observation data doppler frequency (Hz) */
  } obsd_t;
  ```

  ```c
  extern void traceobs(int level, const obsd_t *obs, int n)
  {
      char str[64],id[16];
      int i;
      
      if (!fp_trace||level>level_trace) return;   
      for (i=0;i<n;i++) {
          time2str(obs[i].time,str,3);    //时间
          satno2id(obs[i].sat,id);        //卫星ID（Gnn、Cnn、Rnn。。。）
          fprintf(fp_trace," (%2d) %s %-3s rcv%d %13.3f %13.3f %13.3f %13.3f %d %d %d %d %3.1f %3.1f\n",
                i+1,str,id,obs[i].rcv,obs[i].L[0],obs[i].L[1],obs[i].P[0],
                obs[i].P[1],obs[i].LLI[0],obs[i].LLI[1],obs[i].code[0],
                obs[i].code[1],obs[i].SNR[0]*SNR_UNIT,obs[i].SNR[1]*SNR_UNIT);
      }
      fflush(fp_trace);
  }
  ```

#### 9.tracenav()：写入导航电文
写入`nav->eph`、`nav->ion_gps`/`ion_gal`/`ion_bds`电离层信息、星历数据的的信息。

*   **tracegnav()**:写入nav->geph星历信息。

*  **tracehnav()**:写入nav->seph信息。

 * **tracepeph()** 写入nav->peph 精密星历信息

*  **tracepclk()**:写入nav->pclk 精密钟差信息

```c
typedef struct {        /* navigation data type */
    int n,nmax;         /* number of broadcast ephemeris */
    int ng,ngmax;       /* number of glonass ephemeris */
    int ns,nsmax;       /* number of sbas ephemeris */
    int ne,nemax;       /* number of precise ephemeris */
    int nc,ncmax;       /* number of precise clock */
    int na,namax;       /* number of almanac data */
    int nt,ntmax;       /* number of tec grid data */
    eph_t *eph;         /* GPS/QZS/GAL/BDS/IRN ephemeris */
    geph_t *geph;       /* GLONASS ephemeris */
    seph_t *seph;       /* SBAS ephemeris */
    peph_t *peph;       /* precise ephemeris */
    pclk_t *pclk;       /* precise clock */
    alm_t *alm;         /* almanac data */
    tec_t *tec;         /* tec grid data */
    erp_t  erp;         /* earth rotation parameters */
    double utc_gps[8];  /* GPS delta-UTC parameters {A0,A1,Tot,WNt,dt_LS,WN_LSF,DN,dt_LSF} */
    double utc_glo[8];  /* GLONASS UTC time parameters {tau_C,tau_GPS} */
    double utc_gal[8];  /* Galileo UTC parameters */
    double utc_qzs[8];  /* QZS UTC parameters */
    double utc_cmp[8];  /* BeiDou UTC parameters */
    double utc_irn[9];  /* IRNSS UTC parameters {A0,A1,Tot,...,dt_LSF,A2} */
    double utc_sbs[4];  /* SBAS UTC parameters */
    double ion_gps[8];  /* GPS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_gal[4];  /* Galileo iono model parameters {ai0,ai1,ai2,0} */
    double ion_qzs[8];  /* QZSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_cmp[8];  /* BeiDou iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    double ion_irn[8];  /* IRNSS iono model parameters {a0,a1,a2,a3,b0,b1,b2,b3} */
    int glo_fcn[32];    /* GLONASS FCN + 8 */
    double cbias[MAXSAT][3]; /* satellite DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
    double rbias[MAXRCV][2][3]; /* receiver DCB (0:P1-P2,1:P1-C1,2:P2-C2) (m) */
    pcv_t pcvs[MAXSAT]; /* satellite antenna pcv */
    sbssat_t sbssat;    /* SBAS satellite corrections */
    sbsion_t sbsion[MAXBAND+1]; /* SBAS ionosphere corrections */
    dgps_t dgps[MAXSAT]; /* DGPS corrections */
    ssr_t ssr[MAXSAT];  /* SSR corrections */
} nav_t;
```



#### 10.traceb()：写入buff缓冲区数据
在skytraq.c和ublox.c中被调用。

  ```c
  extern void traceb(int level, const uint8_t *p, int n)
  {
      int i;
      if (!fp_trace||level>level_trace) return;
      for (i=0;i<n;i++) fprintf(fp_trace,"%02X%s",*p++,i%8==7?" ":"");
      fprintf(fp_trace,"\n");
  }
  ```