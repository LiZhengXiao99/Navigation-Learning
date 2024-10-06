[TOC]

## 一、RTK算法数据类型

### 1、rtk_t：rtk控制结构体

存RTK选项，定位结果

```c
sol_t  sol;			//结果结构体                         
double rb[6];        //基准站位置、速度
int nx,na;         	 //na为除模糊度外参数数、nx为加上模糊度参数数
double tt;           //当前历元和先前历元时间差
double *x, *P;       //浮点解和协方差
double *xa,*Pa;      //固定解和协方差
int nfix;            //number of continuous fixes of ambiguity
ambc_t ambc[MAXSAT]; //模糊度控制结构体数组
ssat_t ssat[MAXSAT]; //卫星状态控制结构体数组
int neb;             //错误信息的缓冲区长度
char errbuf[MAXERRMSG];//错误信息缓冲区
prcopt_t opt;        //处理选项
```

### 2、sol_t：结果结构体

```c
gtime_t time;       //GPST时间
double rr[6];       /* 位置、速度结果 (m|m/s) */
                        /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
float  qr[6];       /* 位置估计协方差阵 (m^2) */
                        /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                        /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
float  qv[6];       /* 速度估计协方差阵 (m^2/s^2) */
double dtr[6];      /* receiver clock bias to time systems (s) */
uint8_t type;       /* type (0:xyz-ecef,1:enu-baseline) */
uint8_t stat;       /* solution status (SOLQ_???) */
uint8_t ns;         //有效卫星数
float age;          //差分龄期
float ratio;        //模糊度固定Ratio值
float thres;        //模糊度固定的Ratio阈值
```

### 3、SOLQ_XXX：解的类型状态

```c
#define SOLQ_NONE   0                   /* solution status: no solution */
#define SOLQ_FIX    1                   /* solution status: fix */
#define SOLQ_FLOAT  2                   /* solution status: float */
#define SOLQ_SBAS   3                   /* solution status: SBAS */
#define SOLQ_DGPS   4                   /* solution status: DGPS/DGNSS */
#define SOLQ_SINGLE 5                   /* solution status: single */
#define SOLQ_PPP    6                   /* solution status: PPP */
#define SOLQ_DR     7                   /* solution status: dead reconing */
#define MAXSOLQ     7                   /* max number of solution status */
```

### 4、ambc_t：模糊度固定控制结构体

```c
typedef struct {        /* ambiguity control type */
    gtime_t epoch[4];   /* last epoch */
    int n[4];           /* number of epochs */
    double LC [4];      /* linear combination average */
    double LCv[4];      /* linear combination variance */
    int fixcnt;         /* fix count */
    char flags[MAXSAT]; /* fix flags */
} ambc_t;
```

### 5、ssat_t：卫星状态控制结构体

```c
typedef struct {        /* satellite status type */
    uint8_t sys;         //卫星导航系统
    uint8_t vs;          //有效卫星单一标志
    double azel[2];      //方位角，高度角
    double resp[NFREQ];  //伪距残差
    double resc[NFREQ];  //载波相位残差
    uint8_t vsat[NFREQ]; //有效卫星标志
    uint16_t snr[NFREQ]; //信噪比
    uint8_t fix [NFREQ]; //模糊度的状态，浮点解、固定解
    uint8_t slip[NFREQ]; /* cycle-slip flag */          
    uint8_t half[NFREQ]; /* half-cycle valid flag */
    int lock [NFREQ];   /* lock counter of phase */
    uint32_t outc [NFREQ]; //载波中断计数
    uint32_t slipc[NFREQ]; /* cycle-slip counter */
    uint32_t rejc [NFREQ]; /* reject counter */
    double gf[NFREQ-1]; /* geometry-free phase (m) */
    double mw[NFREQ-1]; /* MW-LC (m) */
    double phw;         /* phase windup (cycle) */
    gtime_t pt[2][NFREQ]; /* previous carrier-phase time */
    double ph[2][NFREQ]; /* previous carrier-phase observable (cycle) */
} ssat_t;
```

### 6、prcopt_t：算法处理选项结构体

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

### 7、obs_t：观测值信息结构体

存一系列的`obsd_t`

```c
typedef struct {        /* observation data */
    int n,nmax;         /* number of obervation data/allocated */
    obsd_t *data;       /* observation data records */
} obs_t;
```

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

- `data`字段是`obsd_t`数组，
- `n`字段表示存着的`obsd_t`数目，
- `nmax`字段表示目前`data`内存空间最大能存的`obsd_t`数目
- `addobsdata()`函数执行向`obs->data[] `中添加`OBS`观测值数据的操作，先检验`nmax`值，不够就`realloc()`
- `readobsnav()`函数中读取完`OBS`数据后，会调用`sortobs()`根据time, rcv, sat ，对`obs->data`的元素进行排序、去重，得到历元数`nepoch`，调用`uniqnav()`,进行星历数据的排序去重。 
- `procpos()`函数while大循环中，调用`inputobs()`每次取一个历元的观测数据用`rtkpos()`处理。

### 8、nav_t：导航电文信息结构体

存全部的星历数据，历书数据、精密星历、TEC格网、广播星历电离层参数、DGPS、SSR改正信息，`nav_t`存`eph`、`geph`、`seph`、`peph`、`pclk`、`alm`、`erp`的方式与`obs_t`存`obs`的方式类似

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

### 9、卫星系统的表示

- 表示卫星系统的字母：G：GPS、R：GLONASS、E：GALILEO、C：BDS、J：QZSS，I：IRNSS、S：SBAS

- 7位二进制码表示，对应位写1表示有对应的系统，做与或算可加系统。

  ```c
  static const int navsys[]={             /* satellite systems */
      SYS_GPS,SYS_GLO,SYS_GAL,SYS_QZS,SYS_SBS,SYS_CMP,SYS_IRN,0
  };
  ```

  ```
  #define SYS_NONE    0x00                /* navigation system: none */
  #define SYS_GPS     0x01                /* navigation system: GPS */
  #define SYS_SBS     0x02                /* navigation system: SBAS */
  #define SYS_GLO     0x04                /* navigation system: GLONASS */
  #define SYS_GAL     0x08                /* navigation system: Galileo */
  #define SYS_QZS     0x10                /* navigation system: QZSS */
  #define SYS_CMP     0x20                /* navigation system: BeiDou */
  #define SYS_IRN     0x40                /* navigation system: IRNS */
  #define SYS_LEO     0x80                /* navigation system: LEO */
  #define SYS_ALL     0xFF                /* navigation system: all */
  ```

### 10、卫星的表示

可以表示为各系统的卫星ID（系统缩写+PRN）：B02、C21，也可表示为连续的satellite number ，各种转换函数如下：

- **satno()**：传入卫星系统(SYS_GPS,SYS_GLO,...) ，和PRN码，转换为连续的satellite number。
- **satsys()**：传入satellite number ，返回卫星系统(SYS_GPS,SYS_GLO,...) ，通过传入的指针prn传出PRN值。
- **satid2no()**：传入卫星ID，返回satellite number。
- **satno2id()**：传入卫星系统，和PRN，返回卫星ID(Gxx,Cxx)
- **sat2code()**：传入satellite number，返回卫星ID(Gxx,Cxx)
- **code2sys()**：传入卫星系统缩写，返回系统二进制码SYS_XXX。
- **satexclude()**：检测某颗卫星在定位时是否需要将其排除

### 11、观测值类型的表示

- C：伪距、D：多普勒、L：载波相位、S：载噪比

```c
  static const char obscodes[]="CLDS";    /* observation type codes */
```

- sigind_t：表示每种卫星系统的载波类型和观测值类型 ，每种类型的系统其实对应的就是一个sigind_t结构体，也就是说只需要建立七个结构体就够了。 

  ```c
  typedef struct {                        /* signal index type */
      int n;                              /* number of index */   //n代表这个卫星系统总的观测值类型，对应的卫星系统标识符后面的数字
      int idx[MAXOBSTYPE];                /* signal freq-index */
      int pos[MAXOBSTYPE];                /* signal index in obs data (-1:no) */
      uint8_t pri [MAXOBSTYPE];           /* signal priority (15-0) */
      uint8_t type[MAXOBSTYPE];           /* type (0:C,1:L,2:D,3:S) */
      uint8_t code[MAXOBSTYPE];           /* obs-code (CODE_L??) */
      double shift[MAXOBSTYPE];           /* phase shift (cycle) */
  } sigind_t;
  
  ```

- **CODE_XXX**：观测值类型定义，用一串连续的数字表示。

  ```c
  #define CODE_NONE   0                   /* obs code: none or unknown */
  #define CODE_L1C    1                   /* obs code: L1C/A,G1C/A,E1C (GPS,GLO,GAL,QZS,SBS) */
  #define CODE_L1P    2                   /* obs code: L1P,G1P,B1P (GPS,GLO,BDS) */
  ......
  #define CODE_L4B    67                  /* obs code: G1aL1OCd   (GLO) */
  #define CODE_L4X    68                  /* obs code: G1al1OCd+p (GLO) */
  #define MAXCODE     68                  /* max number of obs code */
  ```

- **code2obs**()：传入obs code (CODE_???) ，返回code string ("1C","1P","1Y",...)

- **obs2code**()：传入code string ("1C","1P","1Y",...)，返回obs code (CODE_???) 

  ```c
  static char *obscodes[]={       /* observation code strings */
      
      ""  ,"1C","1P","1W","1Y", "1M","1N","1S","1L","1E", /*  0- 9 */
      "1A","1B","1X","1Z","2C", "2D","2S","2L","2X","2P", /* 10-19 */
      "2W","2Y","2M","2N","5I", "5Q","5X","7I","7Q","7X", /* 20-29 */
      "6A","6B","6C","6X","6Z", "6S","6L","8L","8Q","8X", /* 30-39 */
      "2I","2Q","6I","6Q","3I", "3Q","3X","1I","1Q","5A", /* 40-49 */
      "5B","5C","9A","9B","9C", "9X","1D","5D","5P","5Z", /* 50-59 */
      "6E","7D","7P","7Z","8D", "8P","4A","4B","4X",""    /* 60-69 */
  };
  ```

- **code2idx**()：传入obs code (CODE_???) 和卫星系统(SYS_???) ，返回载波频率的下标

  ```c
  frequency index (-1: error)
                        0     1     2     3     4 
             --------------------------------------
              GPS       L1    L2    L5     -     - 
              GLONASS   G1    G2    G3     -     -  (G1=G1,G1a,G2=G2,G2a)
              Galileo   E1    E5b   E5a   E6   E5ab
              QZSS      L1    L2    L5    L6     - 
              SBAS      L1     -    L5     -     -
              BDS       B1    B2    B2a   B3   B2ab (B1=B1I,B1C,B2=B2I,B2b)
              NavIC     L5     S     -     -     - 
  ```

- **code2freq**()：传入obs code (CODE???))和卫星系统(SYS_???)，以及GLONASS的信道，调用code2freq_GPS()、code2freq_GLO()、code2freq_GAL()、code2freq_QZS()、code2freq_SBS()、code2freq_BDS()、code2freq_IRN()，返回对应的载波频率(Hz) 

- **sat2freq()**：传入satellite number和obs code ，返回对应的载波频率(Hz) 

- **setcodepri()**、**getcodepri()**：设置和获取信号优先级。如果输入的观测数据在同一频率内包含多个信号，RTKLIB将按照以下默认优先级选择一个信号进行处理。 

  ```c
  static char codepris[7][MAXFREQ][16]={  /* code priority for each freq-index */
     /*    0         1          2          3         4         5     */
      {"CPYWMNSL","PYWCMNDLSX","IQX"     ,""       ,""       ,""      ,""}, /* GPS */
      {"CPABX"   ,"PCABX"     ,"IQX"     ,""       ,""       ,""      ,""}, /* GLO */
      {"CABXZ"   ,"IQX"       ,"IQX"     ,"ABCXZ"  ,"IQX"    ,""      ,""}, /* GAL */
      {"CLSXZ"   ,"LSX"       ,"IQXDPZ"  ,"LSXEZ"  ,""       ,""      ,""}, /* QZS */
      {"C"       ,"IQX"       ,""        ,""       ,""       ,""      ,""}, /* SBS */
      {"IQXDPAN" ,"IQXDPZ"    ,"DPX"     ,"IQXA"   ,"DPX"    ,""      ,""}, /* BDS */
      {"ABCX"    ,"ABCX"      ,""        ,""       ,""       ,""      ,""}  /* IRN */
  };
  ```

  

## 二、rtkpos()：实现实时动态定位（单历元定位）

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/184e597c45d24b8aa9d77ed71f96dd53.png)


![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/ce409d244fbd4dc0a3da7c4cb5c93c85.png)


### 1、传入参数

```c
rtk_t *rtk			RTK控制结构体
const obsd_t *obs	 观测数据OBS
int n			    观测数据数量
const nav_t *nav	 导航电文信息
```
### 2、执行流程

* 设置rtk内基准站坐标`rtk->rb `，速度设为0.0，基准站坐标在`execses()`函数内调用`antpos()`函数根据选项获取：
  * `postype=POSOPT_SINGLE`：调用`avepos()`利用基准站的观测文件计算其**SPP定位结果平均值**作为基准站的坐标 。
  * `postype=POSOPT_FILE：`调用`getstapos()`从**pos文件读取**基准站坐标 。
  * `postype=POSOPT_RINEX` ：从**rinex头文件中获取**测站经过相位中心改正的位置数据。头文件中的测站数据经过读取后已存到`stas`中。
* 统计基准站OBS个数`nu`，流动站OBS个数`nr`，可用于后面判断是否满足差分条件 
* 赋值先前历元的时间`time=rtk->sol.time `
* 调用`pntpos()`计算流动站坐标，作为kalman滤波的近似坐标，如果由于流动站SPP定位结果坐标误差过大等原因导致的SPP无解，则不进行rtk运算，当前历元无解。 
* 计算当前历元和上一历元时间差`rtk->tt `
* 单点定位模式直接输出刚刚`pntpos()`算的坐标 
* 如果不是单点模式，抑制单点解的输出，`rtk->sol.stat=SOLQ_NONE`
* 如果是PPP模式，调用`pppos()`解算，输出结果，return 1
* 动基线模式：
  * 调用`pntpos()`，传入`obs+nu`基准站观测值计算基准站坐标`solb`。
  * 计算差分龄期`rtk->sol.age `
  * 把`solb.rr`赋值给`rtk->rb `
  * **时间同步**：位置+=对应速度*差分龄期
* 非动基线模式：差分龄期`rtk->sol.age `，等于第一个流动站观测值时间`obs[0].time`减去第一个基准站观测值时间`obs[nu].time `
* 调用`relpos()`进行RTK解算
* 调用`outsolstat()`输出解算结果

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

    //计算当前历元和上一历元时间差rtk->tt，rtk->sol.time是当前历元时间，time是上一历元时间
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
```

### 3、outsolstat()：输出中间结果

#### 1.执行流程

```c
static void outsolstat(rtk_t *rtk)
{
    ssat_t *ssat;
    double tow;
    char buff[MAXSOLMSG+1],id[32];
    int i,j,n,week,nfreq,nf=NF(&rtk->opt);
    
    if (statlevel<=0||!fp_stat||!rtk->sol.stat) return;
    
    trace(3,"outsolstat:\n");
    
    /* swap solution status file */
    swapsolstat();  //根据时间分结果文件
    
    /* write solution status */
    n=rtkoutstat(rtk,buff); buff[n]='\0';
    
    fputs(buff,fp_stat);
    
    //如果解的状态为SOLQ_NONE，或结果输出等级小于等于1，直接return
    if (rtk->sol.stat==SOLQ_NONE||statlevel<=1) return;     
    
    tow=time2gpst(rtk->sol.time,&week);
    nfreq=rtk->opt.mode>=PMODE_DGPS?nf:1;
    
    /* write residuals and status */
    for (i=0;i<MAXSAT;i++) {
        ssat=rtk->ssat+i;
        if (!ssat->vs) continue;
        satno2id(i+1,id);
        for (j=0;j<nfreq;j++) {
            fprintf(fp_stat,"$SAT,%d,%.3f,%s,%d,%.1f,%.1f,%.4f,%.4f,%d,%.1f,%d,%d,%d,%d,%d,%d\n",
                    week,tow,id,j+1,ssat->azel[0]*R2D,ssat->azel[1]*R2D,
                    ssat->resp[j],ssat->resc[j],ssat->vsat[j],
                    ssat->snr[j]*SNR_UNIT,ssat->fix[j],ssat->slip[j]&3,
                    ssat->lock[j],ssat->outc[j],ssat->slipc[j],ssat->rejc[j]);
        }
    }
}
```

#### 2.swapsolstat()：根据时间拆分结果文件

周内秒差距超过一天，就创建一个新文件

```c
static void swapsolstat(void)
{
    gtime_t time=utc2gpst(timeget());   //获取当前系统时间time
    char path[1024];
    //如果当前系统时间周内秒与time_stat差距小于1天，直接return
    if ((int)(time2gpst(time     ,NULL)/INT_SWAP_STAT)==    
        (int)(time2gpst(time_stat,NULL)/INT_SWAP_STAT)) {
        return;
    }
    time_stat=time;
    
    if (!reppath(file_stat,path,time,"","")) {
        return;
    }
    if (fp_stat) fclose(fp_stat);
    
    if (!(fp_stat=fopen(path,"w"))) {
        trace(2,"swapsolstat: file open error path=%s\n",path);
        return;
    }
    trace(3,"swapsolstat: path=%s\n",path);
}
```

#### 3.rtkoutstat()：写中间结果到缓冲区buffer

内容包括：

1. **位置状态参数**

   ```
   $POS,week,tow,stat,posx,posy,posz,posxf,posyf,poszf
   ```

   * `week/tow` : gps week no/time of week (s)
   * `stat` : solution status 定位结果求解状态
   * `posx/posy/posz` : position x/y/z ecef (m) float
   * `posxf/posyf/poszf` : position x/y/z ecef (m) fixed

2. **速度、加速度状态参数**

   ```
   $VELACC,week,tow,stat,vele,veln,velu,acce,accn,accu,velef,velnf,veluf,accef,accnf,accuf
   ```

   * `week/tow` : gps week no/time of week (s)
   * `stat `: solution status
   * `vele/veln/velu` : velocity e/n/u (m/s) float
   * `acce/accn/accu `: acceleration e/n/u (m/s^2) float
   * `velef/velnf/veluf `: velocity e/n/u (m/s) fixed
   * `accef/accnf/accuf` : acceleration e/n/u (m/s^2) fixed

3. **接收机钟差状态参数**

   ```
   $CLK,week,tow,stat,rcv,clk1,clk2,clk3,clk4
   ```

   * `week/tow `: gps week no/time of week (s)
   * `stat` : solution status
   * `rcv` : receiver (1:rover,2:base station)
   * `clk1` : receiver clock bias GPS (ns)
   * `clk2` : receiver clock bias GLONASS (ns)
   * `clk3` : reserved
   * `clk4` : reserved

4. **估计电离层状态参数**

   ```
   $ION,week,tow,stat,sat,az,el,ion,ion‐fixed
   ```

   * `week/tow` : gps week no/time of week (s)
   * `stat` : solution status
   * `sat` : satellite id
   * `az/el` : azimuth/elevation angle(deg)
   * `ion` : vertical ionospheric delay L1 (m) float
   * `ion‐fixed`: vertical ionospheric delay L1 (m) fixed

5. **估计对流层状态参数**

   ```
   $TROP,week,tow,stat,rcv,ztd,ztdf
   ```

   * `week/tow` : gps week no/time of week (s)
   * `stat` : solution status
   * `rcv`: receiver (1:rover,2:base station)
   * `ztd` : zenith total delay (m) float
   * `ztdf` : zenith total delay (m) fixed

6. **估计GLONASS receiver H/W bias difference参数**

   ```
   $HWBIAS,week,tow,stat,frq,bias,biasf
   ```

   * `week/tow` : gps week no/time of week (s)
   * `stat` : solution status
   * `frq` : frequency (1:L1,2:L2,3:L5,...)
   * `bias` : h/w bias coefficient (m/MHz) float
   * `biasf` : h/w bias coefficient (m/MHz) fixed

7. **伪距和载波相位观测量的残差**

   ```
   $SAT,week,tow,sat,frq,az,el,resp,resc,vsat,snr,fix,slip,lock,outc,slipc,rejc
   ```

   * `week/tow` : gps week no/time of week (s)
   * `sat/frq` : satellite id/frequency (1:L1,2:L2,3:L5,...)
   * `az/el` : azimuth/elevation angle (deg)
   * `resp` : pseudorange residual (m)
   * `resc` : carrier‐phase residual (m)
   * `vsat` : valid data flag (0:invalid,1:valid)
   * `snr` : signal strength (dbHz)
   * `fix` : ambiguity flag (0:no data,1:float,2:fixed,3:hold)
   * `slip` : cycle‐slip flag (bit1:slip,bit2:parity unknown)
   * `lock` : carrier‐lock count
   * `outc` : data outage count
   * `slipc` : cycle‐slip count
   * `rejc` : data reject (outlier) count

```c
extern int rtkoutstat(rtk_t *rtk, char *buff)
{
    ssat_t *ssat;
    double tow,pos[3],vel[3],acc[3],vela[3]={0},acca[3]={0},xa[3];
    int i,j,week,est,nfreq,nf=NF(&rtk->opt);
    char id[32],*p=buff;
    
    //如果中间结果为SOLQ_NONE，直接return 0
    if (rtk->sol.stat<=SOLQ_NONE) {
        return 0;
    }

    /* write ppp solution status to buffer */
    if (rtk->opt.mode>=PMODE_PPP_KINEMA) {
        return pppoutstat(rtk,buff);
    }
    est=rtk->opt.mode>=PMODE_DGPS;
    nfreq=est?nf:1;
    tow=time2gpst(rtk->sol.time,&week);
    
    /* receiver position */
    if (est) {
        for (i=0;i<3;i++) xa[i]=i<rtk->na?rtk->xa[i]:0.0;
        p+=sprintf(p,"$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",week,tow,
                   rtk->sol.stat,rtk->x[0],rtk->x[1],rtk->x[2],xa[0],xa[1],
                   xa[2]);
    }
    else {
        p+=sprintf(p,"$POS,%d,%.3f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",week,tow,
                   rtk->sol.stat,rtk->sol.rr[0],rtk->sol.rr[1],rtk->sol.rr[2],
                   0.0,0.0,0.0);
    }
    /* receiver velocity and acceleration */
    if (est&&rtk->opt.dynamics) {
        ecef2pos(rtk->sol.rr,pos);
        ecef2enu(pos,rtk->x+3,vel);
        ecef2enu(pos,rtk->x+6,acc);
        if (rtk->na>=6) ecef2enu(pos,rtk->xa+3,vela);
        if (rtk->na>=9) ecef2enu(pos,rtk->xa+6,acca);
        p+=sprintf(p,"$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                   week,tow,rtk->sol.stat,vel[0],vel[1],vel[2],acc[0],acc[1],
                   acc[2],vela[0],vela[1],vela[2],acca[0],acca[1],acca[2]);
    }
    else {
        ecef2pos(rtk->sol.rr,pos);
        ecef2enu(pos,rtk->sol.rr+3,vel);
        p+=sprintf(p,"$VELACC,%d,%.3f,%d,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.5f,%.5f,%.5f\n",
                   week,tow,rtk->sol.stat,vel[0],vel[1],vel[2],
                   0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
    }
    /* receiver clocks */
    p+=sprintf(p,"$CLK,%d,%.3f,%d,%d,%.3f,%.3f,%.3f,%.3f\n",
               week,tow,rtk->sol.stat,1,rtk->sol.dtr[0]*1E9,rtk->sol.dtr[1]*1E9,
               rtk->sol.dtr[2]*1E9,rtk->sol.dtr[3]*1E9);
    
    /* ionospheric parameters */
    if (est&&rtk->opt.ionoopt==IONOOPT_EST) {
        for (i=0;i<MAXSAT;i++) {
            ssat=rtk->ssat+i;
            if (!ssat->vs) continue;
            satno2id(i+1,id);
            j=II(i+1,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$ION,%d,%.3f,%d,%s,%.1f,%.1f,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,id,ssat->azel[0]*R2D,ssat->azel[1]*R2D,
                       rtk->x[j],xa[0]);
        }
    }
    /* tropospheric parameters */
    if (est&&(rtk->opt.tropopt==TROPOPT_EST||rtk->opt.tropopt==TROPOPT_ESTG)) {
        for (i=0;i<2;i++) {
            j=IT(i,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$TROP,%d,%.3f,%d,%d,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,i+1,rtk->x[j],xa[0]);
        }
    }
    /* receiver h/w bias */
    if (est&&rtk->opt.glomodear==2) {
        for (i=0;i<nfreq;i++) {
            j=IL(i,&rtk->opt);
            xa[0]=j<rtk->na?rtk->xa[j]:0.0;
            p+=sprintf(p,"$HWBIAS,%d,%.3f,%d,%d,%.4f,%.4f\n",week,tow,
                       rtk->sol.stat,i+1,rtk->x[j],xa[0]);
        }
    }
    return (int)(p-buff);
}
```

### 4、rtkpos.c 开头的宏函数

RTKLIB中常用很长的一维数组存信息，为方便找对应数据的数组下标，开头定义了几个宏函数：

|  变量  |                             定义                             |                             说明                             |
| :----: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| **NF** | `define NF(opt) ((opt)->ionoopt==IONOOPT_IFLC?1:(opt)->nf)`  |   频率数，电离层与双频的线性组合时为1，否则为设置的频率数    |
| **NP** |          `define NP(opt) ((opt)->dynamics==0?3:9)`           |       位置参数数量，默认为 3，dynamics 动力学模式为 9        |
| **NI** |   `define NI(opt) ((opt)->ionoopt!=IONOOPT_EST?0:MAXSAT)`    |     Estimate STEC 估算斜电子含量时为最大卫星数，否则为 0     |
| **NT** | `define NT(opt) ((opt)->tropopt<TROPOPT_EST?0:((opt)->tropopt<TROPOPT_ESTG?2:6))` | 对流层参数，不估计时为 0，`TROPOPT_EST`时为 2，`TROPOPT_ESTG`时为 6 |
| **NL** |      `define NL(opt) ((opt)->glomodear!=2?0:NFREQGLO)`       | GLONASS AR 模式，auto cal为0，其它为 GLONASS 的载波频率数 2  |
| **NB** | `define NB(opt) ((opt)->mode<=PMODE_DGPS?0:MAXSAT*NF(opt))`  | 模糊度参数，DGPS 和单点定位模式为 0，其它模式为最大卫星数MAXSAT 乘频率数 NF(opt) |
| **NR** |      `define NR(opt) (NP(opt)+NI(opt)+NT(opt)+NL(opt))`      | 非模糊度参数 = 位置参数 NP(opt)+电离层估计参数 NI(opt) +对流层参数NT(opt)+GLONASS AR 参数NL(opt) |
| **NX** |              `define NX(opt) (NR(opt)+NB(opt))`              |           总参数 = 非模糊度参数 NR+ 模糊度参数 NB            |
| **II** |              `define II(s,opt) (NP(opt)+(s)-1)`              |                电离层参数下标 (s 是卫星编号)                 |
| **IT** |     `define IT(r,opt)   (NP(opt)+NI(opt)+NT(opt)/2*(r))`     |          对流层参数下标 (r：流动站是 0、基准站是 1)          |
| **IL** |      `define IL(f,opt)   (NP(opt)+NI(opt)+NT(opt)+(f))`      |                   GLONASS 接收机 h/w 延迟                    |
| **IB** |       `define IB(s,f,opt) (NR(opt)+MAXSAT*(f)+(s)-1)`        |       整周模糊度参数下标，(s 是卫星编号，f 是频率编号)       |

> * 为啥没有位置参数下标宏？因为在最前面，直接取值就行。
> * 为啥没有钟差参数？因为双差基本消除了钟差，不用计算。

## 三、relpos()：相对定位算法入口函数

> 适用于 DGPS/DGNSS、Kinematic、Static、Moving-Base、Fixed 5 种模式

### 1、RTK算法流程图

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/b1a48cf9b3c245f89a520e2ebef24ea9.png)


![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/b0b8906da35a4f51a3f025b702a4606d.png)


### 2、传入参数

```c
rtk_t    *rtk      IO  rtk控制结构体
obsd_t   *obs      I   观测数据
int      nu        I   接收机观测数据的数量
int      nr        I   基站观测数据的数量
nav_t    *nav      I   导航数据
```
### 3、执行流程

* 计算流动站、基准站间时间差`dt`，等于第一个流动站观测值时间`obs[0].time`减去第一个基准站观测值时间`obs[nu].time `
* 调用`satposs()`计算当前历元下，各卫星的位置速度`rs`、钟差`dts`
* 调用`zdres()`计算基准站的各卫星观测值的**非差残差**（观测值-计算值）及卫星的高度角、方位角、卫星矢量等
* 后处理中，需要时，调用`intpres()`进行**插值** 
* 调用`selsat()`选择基准站和流动站之间的**共视卫星**，进行rtk算法时只需要基线间的同步观测卫星，返回共同观测的卫星个数`ns`，输出卫星号列表`sat`，在接收机观测值中的index值列表`iu`和在基站观测值中的index值列表`ir` 
* **时间更新**：调用`udstate()`更新状态值`rtk->x`及其误差协方差`rtk->P`
* 设置迭代次数`niter`，动基线加2次 
* for循环迭代量测更新`niter`次：
  * 调用`zdres()`计算流动站的**各位卫星观测值非差残差**（观测值-计算值）及卫星的高度角、方位角、卫星矢量等 
  * 调用`ddres()`根据上述计算的基准站和流动站的各卫星观测值非差残差，计算**双差残差**矩阵`v` ，根据流动站非差卫星矢量计算观测方程的双差系数矩阵`H`，根据非差观测值误差计算观测噪声的双差协方差矩阵`R`
  * 调用`filter()`，计算**增益矩阵**并**状态更新**，EKF计算**浮点解**。
* 量测更新完成，再次调用`zdres()`和`ddres()`计算双差相位/码残差，调用`valpos()`进行**浮点解有效性**验证；若通过则更新`rtk->x`以及`rtk->P`，并更新模糊度控制结构体。 
* 调用`resamb_LAMBDA()`，利用lambda算法**固定模糊度**。
  * 模糊度解算成功，调用`zdres()`和`ddres()`根据固定结果计算残差和协方差，并进行调用`valpos()`校验 
  * 固定解验证有效，若为hold模式，需要存模糊度信息调用`holdamb()`
* **保存中间结果**,位置，速度，方差，到`sol.rr`、`sol.qr`、`sol.qv`，固定解数据在`rtk->xa`、`rtk->Pa`，浮点解数据在`rtk->x`、`rtk->P`
* 循环，**存当前历元载波信息**`rtk->ssat[sat[i]-1].pt`存时间、rtk->ssat[sat[i]-1]`.ph`存载波相位观测值，供下次使用
* 循环，**存SNR**信噪比信息到`rtk->ssat[sat[i]-1].snr `
* 循环，存卫星的模糊度固定信息`rtk->ssat[i].fix[j]`及周跳信息，`rtk->ssat[i].slipc[j]`
* 释放资源

```c
static int relpos(rtk_t *rtk, const obsd_t *obs, int nu, int nr,
                  const nav_t *nav)
{
    prcopt_t *opt=&rtk->opt;
    gtime_t time=obs[0].time;
    double *rs,*dts,*var,*y,*e,*azel,*freq,*v,*H,*R,*xp,*Pp,*xa,*bias,dt;
    int i,j,f,n=nu+nr,ns,ny,nv,sat[MAXSAT],iu[MAXSAT],ir[MAXSAT],niter;
    int info,vflg[MAXOBS*NFREQ*2+1],svh[MAXOBS*2];
    int stat=rtk->opt.mode<=PMODE_DGPS?SOLQ_DGPS:SOLQ_FLOAT;
    int nf=opt->ionoopt==IONOOPT_IFLC?1:opt->nf;
    
    trace(3,"relpos  : nx=%d nu=%d nr=%d\n",rtk->nx,nu,nr);
    
    dt=timediff(time,obs[nu].time); //计算流动站，参考站时间差
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); y=mat(nf*2,n); e=mat(3,n);
    azel=zeros(2,n); freq=zeros(nf,n);
    
    for (i=0;i<MAXSAT;i++) {
        rtk->ssat[i].sys=satsys(i+1,NULL);
        for (j=0;j<NFREQ;j++) rtk->ssat[i].vsat[j]=0;
        for (j=1;j<NFREQ;j++) rtk->ssat[i].snr [j]=0;
    }
    
    //根据卫星星历计算当前历元下各卫星的位置、速度、钟差
    /* satellite positions/clocks */
    satposs(time,obs,n,nav,opt->sateph,rs,dts,var,svh);
    
    //计算基准站的各卫星观测值的非差闭合差（观测值-计算值）及卫星的高度角、方位角、卫星矢量等。
    /* UD (undifferenced) residuals for base station */
    if (!zdres(1,obs+nu,nr,rs+nu*6,dts+nu*2,var+nu,svh+nu,nav,rtk->rb,opt,1,
               y+nu*nf*2,e+nu*3,azel+nu*2,freq+nu*nf)) {
        errmsg(rtk,"initial base station position error\n");
        
        free(rs); free(dts); free(var); free(y); free(e); free(azel);
        free(freq);
        return 0;
    }
    //后处理中，需要时，调用 intpres 进行插值
    /* time-interpolation of residuals (for post-processing) */
    if (opt->intpref) {
        dt=intpres(time,obs+nu,nr,nav,rtk,y+nu*nf*2);
    }
    //选择基准站和流动站之间的同步观测卫星。进行rtk算法时只需要基线间的同步观测卫星。
    //返回共同观测的卫星个数，输出卫星号列表sat、在接收机观测值中的index值列表 iu 和在基站观测值中的index值列表 ir。
    /* select common satellites between rover and base-station */
    if ((ns=selsat(obs,azel,nu,nr,opt,sat,iu,ir))<=0) {
        errmsg(rtk,"no common satellite\n");
        
        free(rs); free(dts); free(var); free(y); free(e); free(azel);
        free(freq);
        return 0;
    }
    //调用 udstate 更新状态值 rtk->x 及其误差协方差 rtk->P
    /* temporal update of states */
    udstate(rtk,obs,sat,iu,ir,ns,nav);
    
    //初始化变量内存以及赋初值
    trace(4,"x(0)="); tracemat(4,rtk->x,1,NR(opt),13,4);
    xp=mat(rtk->nx,1); Pp=zeros(rtk->nx,rtk->nx); xa=mat(rtk->nx,1);
    matcpy(xp,rtk->x,rtk->nx,1);
    ny=ns*nf*2+2;
    v=mat(ny,1); H=zeros(rtk->nx,ny); R=mat(ny,ny); bias=mat(rtk->nx,1);
    
    //设置迭代次数niter，动基线加2次
    /* add 2 iterations for baseline-constraint moving-base */
    niter=opt->niter+(opt->mode==PMODE_MOVEB&&opt->baseline[0]>0.0?2:0);
    
    //迭代量测更新niter次
    for (i=0;i<niter;i++) {
        //计算流动站的各位卫星观测值非差闭合差（观测值-计算值）及卫星的高度角、方位角、卫星矢量等。
        /* UD (undifferenced) residuals for rover */
        if (!zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,freq)) {
            errmsg(rtk,"rover initial position error\n");
            stat=SOLQ_NONE;
            break;
        }

        //根据上述计算的基准站和流动站的各卫星观测值非差闭合差，计算双差闭合差矩阵v，
        //并且在这个函数内，根据流动站非差卫星矢量计算观测方程的双差系数矩阵H、
        //根据非差观测值误差计算观测噪声的双差协方差矩阵R。
        /* DD (double-differenced) residuals and partial derivatives */
        if ((nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,H,R,
                      vflg))<1) {
            errmsg(rtk,"no double-differenced residual\n");
            stat=SOLQ_NONE;
            break;
        }

        //进行卡尔曼滤波的第二步——计算增益矩阵并状态更新，利用卡尔曼滤波计算rtk浮点解。
        //在进入filter函数之后需要对状态参数矩阵x、状态噪声协方差矩阵p、
        //观测方程系数阵H进行矩阵缩小简化，去除未用到的卫星，正式计算公式如下：
        //这里H矩阵之所以使用转置的形式，是因为RTKLIB内存储矩阵的规则是按列存储的。
        /* Kalman filter measurement update */
        matcpy(Pp,rtk->P,rtk->nx,rtk->nx);
        if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv))) {
            errmsg(rtk,"filter error (info=%d)\n",info);
            stat=SOLQ_NONE;
            break;
        }
        trace(4,"x(%d)=",i+1); tracemat(4,xp,1,NR(opt),13,4);
    }
    
    //量测更新完成，再次调用zdres和ddres计算双差相位/码残差，调用valpos进行验证，
    //若通过则更新 rtk->x 以及 rtk->P，并更新模糊度控制结构体。
    if (stat!=SOLQ_NONE&&zdres(0,obs,nu,rs,dts,var,svh,nav,xp,opt,0,y,e,azel,
                               freq)) {
        //利用浮点结果计算双差残差和量测噪声
        /* post-fit residuals for float solution */
        nv=ddres(rtk,nav,dt,xp,Pp,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,vflg);
        
        //计算流动站非差、双差闭合差，进行浮点解有效性验证
        /* validation of float solution */
        if (valpos(rtk,v,R,vflg,nv,4.0)) {
            //存储浮点结果
            /* update state and covariance matrix */
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
            //存模糊度相关信息，统计有效卫星数
            /* update ambiguity control struct */
            rtk->sol.ns=0;
            for (i=0;i<ns;i++) for (f=0;f<nf;f++) {
                if (!rtk->ssat[sat[i]-1].vsat[f]) continue;
                rtk->ssat[sat[i]-1].lock[f]++;
                rtk->ssat[sat[i]-1].outc[f]=0;
                if (f==0) rtk->sol.ns++; /* valid satellite count by L1 */
            }
            /* lack of valid satellites */  //检验卫星是否有效
            if (rtk->sol.ns<4) stat=SOLQ_NONE;
        }
        else stat=SOLQ_NONE;
    }

    //利用lambda算法固定模糊度计算rtk固定解。
    /* resolve integer ambiguity by LAMBDA */
    if (stat!=SOLQ_NONE&&resamb_LAMBDA(rtk,bias,xa)>1) {
        //模糊度解算成功，根据固定结果计算残差和协方差，并进行校验
        if (zdres(0,obs,nu,rs,dts,var,svh,nav,xa,opt,0,y,e,azel,freq)) {
            
            /* post-fit reisiduals for fixed solution */
            nv=ddres(rtk,nav,dt,xa,NULL,sat,y,e,azel,freq,iu,ir,ns,v,NULL,R,
                     vflg);


            //计算流动站非差、双差闭合差，进行固定解有效性验证
            /* validation of fixed solution */
            if (valpos(rtk,v,R,vflg,nv,4.0)) {
                //固定解验证有效，若为hold模式，需要存模糊度信息
                /* hold integer ambiguity */
                if (++rtk->nfix>=rtk->opt.minfix&&
                    rtk->opt.modear==ARMODE_FIXHOLD) {
                    holdamb(rtk,xa);
                }
                stat=SOLQ_FIX;
            }
        }
    }
    //保存solution状态,位置，速度，方差
    /* save solution status */
    if (stat==SOLQ_FIX) {   //固定了存固定解，固定解在rtk->xa、rtk->Pa
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->xa[i];
            rtk->sol.qr[i]=(float)rtk->Pa[i+i*rtk->na];
        }
        rtk->sol.qr[3]=(float)rtk->Pa[1];
        rtk->sol.qr[4]=(float)rtk->Pa[1+2*rtk->na];
        rtk->sol.qr[5]=(float)rtk->Pa[2];
        
        if (rtk->opt.dynamics) { /* velocity and covariance */
            for (i=3;i<6;i++) {
                rtk->sol.rr[i]=rtk->xa[i];
                rtk->sol.qv[i-3]=(float)rtk->Pa[i+i*rtk->na];
            }
            rtk->sol.qv[3]=(float)rtk->Pa[4+3*rtk->na];
            rtk->sol.qv[4]=(float)rtk->Pa[5+4*rtk->na];
            rtk->sol.qv[5]=(float)rtk->Pa[5+3*rtk->na];
        }
    }
    else {  //浮点解，浮点解数据在rtk->x、rtk->P
        for (i=0;i<3;i++) {
            rtk->sol.rr[i]=rtk->x[i];
            rtk->sol.qr[i]=(float)rtk->P[i+i*rtk->nx];
        }
        rtk->sol.qr[3]=(float)rtk->P[1];
        rtk->sol.qr[4]=(float)rtk->P[1+2*rtk->nx];
        rtk->sol.qr[5]=(float)rtk->P[2];
        
        if (rtk->opt.dynamics) { /* velocity and covariance */
            for (i=3;i<6;i++) {
                rtk->sol.rr[i]=rtk->x[i];
                rtk->sol.qv[i-3]=(float)rtk->P[i+i*rtk->nx];
            }
            rtk->sol.qv[3]=(float)rtk->P[4+3*rtk->nx];
            rtk->sol.qv[4]=(float)rtk->P[5+4*rtk->nx];
            rtk->sol.qv[5]=(float)rtk->P[5+3*rtk->nx];
        }
        rtk->nfix=0;
    }

    //存当前历元载波信息，供下次使用
    for (i=0;i<n;i++) for (j=0;j<nf;j++) {
        if (obs[i].L[j]==0.0) continue;
        rtk->ssat[obs[i].sat-1].pt[obs[i].rcv-1][j]=obs[i].time;    //先前历元载波相位时间
        rtk->ssat[obs[i].sat-1].ph[obs[i].rcv-1][j]=obs[i].L[j];    //先前历元载波相位观测值
    }

    //存SNR信噪比信息
    for (i=0;i<ns;i++) for (j=0;j<nf;j++) {        
        /* output snr of rover receiver */
        rtk->ssat[sat[i]-1].snr[j]=obs[iu[i]].SNR[j];
    }

    //存卫星的fix信息及周跳信息
    for (i=0;i<MAXSAT;i++) for (j=0;j<nf;j++) {
        if (rtk->ssat[i].fix[j]==2&&stat!=SOLQ_FIX) rtk->ssat[i].fix[j]=1;
        if (rtk->ssat[i].slip[j]&1) rtk->ssat[i].slipc[j]++;
    }
    free(rs); free(dts); free(var); free(y); free(e); free(azel); free(freq);
    free(xp); free(Pp);  free(xa);  free(v); free(H); free(R); free(bias);
    
    if (stat!=SOLQ_NONE) rtk->sol.stat=stat;
    
    return stat!=SOLQ_NONE;
}
```

## 四、zdres()：计算非差残差![image-20241006144531175](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241006144531175.png)

计算流动站（base=0）和基准站（base=1）的非差残差，即非差的相位/伪距残差（Zero-Difference Residuals）

* 残差 = 观测量 - 伪距估计量，伪距估计量在整个过程中的修正包括潮汐修正（可选项）、卫星钟差修正、对流层修正，并计算了接收机天线偏移误差 

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/54ec5a0d997b4af39086535821c75248.png)

### 1、传入参数

```c
int      base      I   0表示接收机，1表示基站
obsd_t   *obs      I   OBS观测数据
int      n         I   OBS的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   NAV导航数据
double   *rr       I   接收机/基站的位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
prcopt_t *opt      I   处理过程选项
int      index     I   0表示接收机，1表示基站，与参数 base 重复了
double   *y        O   相位/码残差
double   *e        O   观测矢量 (ecef)
double   *azel     O   方位角和俯仰角 (rad)
double   *freq     O   载波频率
```
### 2、执行流程

* 若没有接收机位置，return 0 
* 接收机位置（ECEF-XYZ）传给`rr_`
* 若需要**地球潮校正**，调用`tidedisp()`对`rr_`进行校正
* 调用`ecef2pos()`将XYZ的`rr_`转为LLH的`pos`
* for循环遍历所有观测值（卫星）：
  * 调用`geodist()`根据卫星位置和接收机位置计算卫地距`r`，和和接收机到卫星的单位向量`e+i*3`，并进行**地球自转改正**。
  * 调用`satazel()`计算卫星方位角、高度角`azel+i*2`，排除小于截止高度角`opt->elmin`的观测值。
  * 对卫地距进行卫星钟差距离改正`r+=-CLIGHT*dts[i*2]`
  * 调用`tropmodel()`利用**Saastamoinen模型**只改正对流层干延迟`zhd`（由于湿度值传0，高度角传0，只计算了天顶方向干延迟 ），湿分量会在之后的ddres（计算双差残差的函数）函数中进行扣除 ，调用`tropmapf()`计算出干延迟投影系数（即天顶方向到接收机相对卫星观测方向上的对流层延迟投影系数）。`tropmodel()`计算出的**天顶对流层干延迟**与`tropmapf()`计算出**干延迟投影系数**相乘，从而得到接收机相对卫星观测方向上的对流层延迟 ，对卫地距进行干延迟改正`r+=tropmapf(obs[i].time,pos,azel+i*2,NULL)*zhd;`
  * 由于之后会计算双差后的残差，因此在短基线的情况下，大部分电离层误差已经得到了消除，所以未进行电离层误差修正；而对流层误差受基站和移动站之间的高度差影响，因此通常还需要进行考虑。
  * 调用`antmodel()`计算接收机天线相位中心改正值`dant`，对每一个频率都有一个值。相对定位只需要计算**接收机**端的天线相位中心修正值。这是由于，相对定位进行单差时，已经将卫星端的天线误差消除了。
  * 调用`zdres_sat()`观测值减经过上述各项改正后的计算值，得到最终的非差残差存到数组`y[]`。**消电离层组合**也在次函数实现。

```c
static int zdres(int base, const obsd_t *obs, int n, const double *rs,
                 const double *dts, const double *var, const int *svh,
                 const nav_t *nav, const double *rr, const prcopt_t *opt,
                 int index, double *y, double *e, double *azel, double *freq)
{
    double r,rr_[3],pos[3],dant[NFREQ]={0},disp[3];
    double zhd,zazel[]={0.0,90.0*D2R};
    int i,nf=NF(opt);
    
    trace(3,"zdres   : n=%d\n",n);
    
    for (i=0;i<n*nf*2;i++) y[i]=0.0;
    //若没有接收机位置，return 0
    if (norm(rr,3)<=0.0) return 0; /* no receiver position */
    
    for (i=0;i<3;i++) rr_[i]=rr[i]; //接收机位置传给 rr_
    //若需要地球潮校正，调用 tidedisp 对 rr_ 进行校正。地球潮包含固体潮、极潮和海潮负荷
    /* earth tide correction */
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rr_,opt->tidecorr,&nav->erp,
                 opt->odisp[base],disp);
        for (i=0;i<3;i++) rr_[i]+=disp[i];
    }
    ecef2pos(rr_,pos);  //rr_ : XYZ->LLH
    
    //遍历观测量
    for (i=0;i<n;i++) {
        //根据卫星位置和接收机位置计算卫地距，并进行地球自转改正
        /* compute geometric-range and azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr_,e+i*3))<=0.0) continue;
        if (satazel(pos,e+i*3,azel+i*2)<opt->elmin) continue;
        
        /* excluded satellite? */   //排除需要排除的卫星
        if (satexclude(obs[i].sat,var[i],svh[i],opt)) continue;
        
        //对卫地距进行卫星钟差距离改正r
        /* satellite clock-bias */
        r+=-CLIGHT*dts[i*2];
        
        //对卫地距进行对流层延迟改正。对流层延迟可分为大概90%的干延迟和10%的湿延迟，
        //RTKLIB内利用Saastamoinen模型只改正了对流层的干延迟，湿延迟通过随机游走估计的方式进行改正。
        //此处湿度传0，湿延迟计算出的结果就是0
        //当然，建议对这一部分进行算法改进，利用GPT2_w等经验模型改正对流层的总延迟
        /* troposphere delay model (hydrostatic) */
        zhd=tropmodel(obs[0].time,pos,zazel,0.0);
        r+=tropmapf(obs[i].time,pos,azel+i*2,NULL)*zhd;
        
        //接收机天线相位中心改正，调用 antmodel 计算校正值 dant（对每一个频率都有一个值）
        /* receiver antenna phase center correction */
        antmodel(opt->pcvr+index,opt->antdel[index],azel+i*2,opt->posopt[1],
                 dant);

        //观测值减经过上述各项改正后的计算值，
        //得到最终的非差闭合差v。此处涉及到无电离层组合观测值的求解。
        /* UD phase/code residual for satellite */
        zdres_sat(base,r,obs+i,nav,azel+i*2,dant,opt,y+i*nf*2,freq+i*nf);
    }
    trace(4,"rr_=%.3f %.3f %.3f\n",rr_[0],rr_[1],rr_[2]);
    trace(4,"pos=%.9f %.9f %.3f\n",pos[0]*R2D,pos[1]*R2D,pos[2]);
    for (i=0;i<n;i++) {
        trace(4,"sat=%2d %13.3f %13.3f %13.3f %13.10f %6.1f %5.1f\n",
              obs[i].sat,rs[i*6],rs[1+i*6],rs[2+i*6],dts[i*2],azel[i*2]*R2D,
              azel[1+i*2]*R2D);
    }
    trace(4,"y=\n"); tracemat(4,y,nf*2,n,13,3);
    
    return 1;
}
```

### 3、zdres_sat()：计算一个站心非差残差

计算接收机或基站对某一颗卫星的非差残差，在此实现消电离层组合。

**传入参数**：

```c
int      base      I   0表示接收机，1表示基站
double   r         I   经过钟差和对流层校正后的几何距离
obsd_t   *obs      I   OBS观测数据
nav_t    *nav      I   NAV导航数据
double   *azel     I   方位角和俯仰角 (rad)
double   *dant     I   接收机天线校正值
prcopt_t *opt      I   处理过程选项
double   *y        O   非差闭合差
double   *freq	   O   载波频率
```
**执行流程**：

* 如果是消电离层组合`IONOOPT_IFLC`

  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/d96fdfdc1a3c4ed397a5b0a299c81d01.png)


  * 调用`sat2freq()`获取观测值`obs->code[0]`、`obs->code[1]`的频率`freq1`、`freq2`
  * 调用`testsnr()`检测信噪比是否都小于`opt->snrmask`
  * 计算消电离层组合系数`c1`、`c2`
  * 计算天线校正值`dant_if`
  * 计算载波相位残差`y[0]`，伪距残差`y[1]`，残差 = IFLC观测量 - 卫地距 - 天线偏移量 
  * `freq[0]=1.0`

* 不是消电离层组合的情况，for循环遍历`obs`所有频率值，都计算残差
  * 调用`sat2freq()`获取`obs->code[i]`频率`freq[i]`
  * 调用`testsnr()`检测信噪比是否小于`opt->snrmask`
  * 计算载波相位残差`y[i]`，伪距残差`y[i+nf]`，残差 = 观测量 - 卫地距 - 天线偏移量 

```c
static void zdres_sat(int base, double r, const obsd_t *obs, const nav_t *nav,
                      const double *azel, const double *dant,
                      const prcopt_t *opt, double *y, double *freq)
{
    double freq1,freq2,C1,C2,dant_if;
    int i,nf=NF(opt);
    
    //电离层校正模式为 IONOOPT_IFLC 的情况
    if (opt->ionoopt==IONOOPT_IFLC) { /* iono-free linear combination */
        //获取观测值obs->code[0]、obs->code[1]的频率freq1、freq2
        freq1=sat2freq(obs->sat,obs->code[0],nav);  
        freq2=sat2freq(obs->sat,obs->code[1],nav);
        if (freq1==0.0||freq2==0.0) return;
        //检测信噪比
        if (testsnr(base,0,azel[1],obs->SNR[0]*SNR_UNIT,&opt->snrmask)||
            testsnr(base,1,azel[1],obs->SNR[1]*SNR_UNIT,&opt->snrmask)) return;
        //计算消电离层组合系数c1、c2    (E.5.23)
        C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
        C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
        //计算天线校正值 dant_if
        dant_if=C1*dant[0]+C2*dant[1];
        //计算残差，残差 = IFLC观测量 - 卫地距 - 天线偏移量 
        if (obs->L[0]!=0.0&&obs->L[1]!=0.0) {   //载波相位残差  (E.5.22) 
            y[0]=C1*obs->L[0]*CLIGHT/freq1+C2*obs->L[1]*CLIGHT/freq2-r-dant_if;
        }
        if (obs->P[0]!=0.0&&obs->P[1]!=0.0) {   //伪距残差  (E.5.21) 
            y[1]=C1*obs->P[0]+C2*obs->P[1]-r-dant_if;
        }
        freq[0]=1.0;
    }
    //电离层校正模式不为 IONOOPT_IFLC 的情况
    else {
        for (i=0;i<nf;i++) {
            if ((freq[i]=sat2freq(obs->sat,obs->code[i],nav))==0.0) continue;
            //检测信噪比
            /* check SNR mask */
            if (testsnr(base,i,azel[1],obs->SNR[i]*SNR_UNIT,&opt->snrmask)) {
                continue;
            }
            //计算残差，残差 = IFLC观测量 - 卫地距 - 天线偏移量 
            /* residuals = observable - pseudorange */
            if (obs->L[i]!=0.0) y[i   ]=obs->L[i]*CLIGHT/freq[i]-r-dant[i];
            if (obs->P[i]!=0.0) y[i+nf]=obs->P[i]               -r-dant[i];
        }
    }
}
```


### 4、geodist()：计算站心几何距离 

返回值为接收机和卫星的几何距离ECEF（m），参数`e[]`为接收机到卫星的单位向量，并进行地球自转改正

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7f2c6543004541d58ef011c8ac6a5660.png)


```c
extern double geodist(const double *rs, const double *rr, double *e)
{
    double r;
    int i;
    
    if (norm(rs,3)<RE_WGS84) return -1.0;   //检查卫星到 WGS84坐标系原点的距离是否大于基准椭球体的长半径。
    for (i=0;i<3;i++) e[i]=rs[i]-rr[i];     //求卫星和接收机坐标差e[]
    r=norm(e,3);                            //求未经萨格纳克效应改正的距离
    for (i=0;i<3;i++) e[i]/=r;  //接收机到卫星的单位向量e[]     (E.3.9)
    return r+OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT;         //(E.3.8b)
}
```



### 5、tropmodel()：Saastamoinen 模型计算对流层延迟

利用 Saastamoinen 模型只改正对流层干延迟`zhd`（由于湿度值传 0，高度角传 0，只计算了天顶方向干延迟），先计算一些标准大气值，包括总压 `p`、大气温度 `T`、水汽压力 `e`，计算静力学延迟(干延迟) `trph`，然后计算了湿延迟 `trpw`

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/60ccc36567ae4c749cc0d931be4e5536.png)


```c
extern double tropmodel(gtime_t time, const double *pos, const double *azel,
                        double humi)
{
    const double temp0=15.0; /* temparature at sea level */
    double hgt,pres,temp,e,z,trph,trpw;
    
    if (pos[2]<-100.0||1E4<pos[2]||azel[1]<=0) return 0.0;
    
    /* standard atmosphere */
    hgt=pos[2]<0.0?0.0:pos[2];
    
    pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);         //求大气压P (E.5.1)
    temp=temp0-6.5E-3*hgt+273.16;                       //求温度temp (E.5.2)
    e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45)); //求大气水汽压力e (E.5.3)
    
    /* saastamoninen model */
    z=PI/2.0-azel[1];       //求天顶角z 卫星高度角azel[1]的余角
    trph=0.0022768*pres/(1.0-0.00266*cos(2.0*pos[0])-0.00028*hgt/1E3)/cos(z);   //求静力学延迟Th
    trpw=0.002277*(1255.0/temp+0.05)*e/cos(z);              //求湿延迟Tw (E.5.4)
    return trph+trpw;       //Saastamoinen中对流层延迟为静力学延迟Th湿延迟Tw的和
}
```

### 6、tropmapf()：计算干湿延迟投影系数

干投影函数是通过返回值获得的，而湿投影是通过输入/输出参数`mapfw`获得的，有两种投影函数的计算方法，分别是 GMF 和 NMF ，默认使用的是 NMF 方法，也可以通过定义`IERS_MODEL`宏来使用 GMF 方法。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/91ef038974d34520bbfa700e7a935a73.png)


```c
extern double tropmapf(gtime_t time, const double pos[], const double azel[],
                       double *mapfw)
{
#ifdef IERS_MODEL
    const double ep[]={2000,1,1,12,0,0};
    double mjd,lat,lon,hgt,zd,gmfh,gmfw;
#endif
    trace(4,"tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n",
          pos[0]*R2D,pos[1]*R2D,pos[2],azel[0]*R2D,azel[1]*R2D);
    
    if (pos[2]<-1000.0||pos[2]>20000.0) {
        if (mapfw) *mapfw=0.0;
        return 0.0;
    }
#ifdef IERS_MODEL
    mjd=51544.5+(timediff(time,epoch2time(ep)))/86400.0;
    lat=pos[0];
    lon=pos[1];
    hgt=pos[2]-geoidh(pos); /* height in m (mean sea level) */
    zd =PI/2.0-azel[1];
    
    /* call GMF */
    gmf_(&mjd,&lat,&lon,&hgt,&zd,&gmfh,&gmfw);
    
    if (mapfw) *mapfw=gmfw;
    return gmfh;
#else
    return nmf(time,pos,azel,mapfw); /* NMF */
#endif
}
```

* **nmf()**：Neill投影函数

  ```c
  extern double tropmapf(gtime_t time, const double pos[], const double azel[],
                         double *mapfw)
  {
  #ifdef IERS_MODEL
      const double ep[]={2000,1,1,12,0,0};
      double mjd,lat,lon,hgt,zd,gmfh,gmfw;
  #endif
      trace(4,"tropmapf: pos=%10.6f %11.6f %6.1f azel=%5.1f %4.1f\n",
            pos[0]*R2D,pos[1]*R2D,pos[2],azel[0]*R2D,azel[1]*R2D);
      
      if (pos[2]<-1000.0||pos[2]>20000.0) {
          if (mapfw) *mapfw=0.0;
          return 0.0;
      }
  #ifdef IERS_MODEL
      mjd=51544.5+(timediff(time,epoch2time(ep)))/86400.0;
      lat=pos[0];
      lon=pos[1];
      hgt=pos[2]-geoidh(pos); /* height in m (mean sea level) */
      zd =PI/2.0-azel[1];
      
      /* call GMF */
      gmf_(&mjd,&lat,&lon,&hgt,&zd,&gmfh,&gmfw);
      
      if (mapfw) *mapfw=gmfw;
      return gmfh;
  #else
      return nmf(time,pos,azel,mapfw); /* NMF */
  #endif
  }
  
  static double interpc(const double coef[], double lat)
  {
      int i=(int)(lat/15.0);
      if (i<1) return coef[0]; else if (i>4) return coef[4];
      return coef[i-1]*(1.0-lat/15.0+i)+coef[i]*(lat/15.0-i);
  }
  
  static double mapf(double el, double a, double b, double c)
  {
      double sinel=sin(el);
      return (1.0+a/(1.0+b/(1.0+c)))/(sinel+(a/(sinel+b/(sinel+c))));
  }
  ```



## 五、天线相位中心改正

### 1、理论概述

* **天线相位中心**，即天线]接收信号的**电气中心**，其空间位置在出厂时往往不在天线的**几何中心**上。天线所辐射出的电磁波在离开天线一定的距离后，其等相位面会近似为一个球面，该球面的球心即为该天线的等效相位中心，即天线相位中心（Antenna Phase Center ） 

* GNSS观测量是相对于**接收机天线的平均相位中心**而言的，而接收机天线对中是相对于几何中也而言的，这两种中心一般不重合，两者之差就称为**平均相位中心偏差（PCO）**，其大小可达**毫米级或厘米级**。且接收机天线的相位中也会随卫星信号输入的方向和强度的变化而变化，此时观测时刻的瞬时相位中也与平均相位中心的偏差称为**平均相位中心变化（PCV）**，它与卫星的高度角和方位角有关。因此接收机天线相位偏差由接收机天线PCO和PCV两部分组成。

* NGS 提供的 ANTEX 格式天线模型，包含了卫星天线模型以及部分接收机天线模型。使用天线模型的目的包括： 

  * 修正天线参考点和天线相位中心的之间的偏差；
  * 修正和仰角有关的误差；
  * 修正 L1 和 L2 之间的相位中心偏差（这个误差可能对整周模糊度固定造成影响）

* **接收机天线相位中心模型**：一般选取接收机天线底部与天线中轴的交点作为参考点（称天线参考点，ARP）： 

  * ARP 与实际相位中心的几何偏差值称为**天线相位中心偏差**（PCO）
  * 由不同高度角、方位角测得的距离产生系统性的测量偏差为**天线相位中心变化**（PCV）

  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/fc25f5a5d20b4d15ba35ea130701519c.png)


  * RTKLIB 支持 NGS PCV 以及 ANTEX 格式的天线模型，其中包括了 PCO 和 PCV 修正参数。通过手册 E.8 章节可知，接收机天线修正如下： 

    * **PCO修正**通常是当地坐标系ENU参数，因此需要利用转换矩阵转到ECEF坐标系  
      $$
      \boldsymbol{d}_{r, p c o, i}=\boldsymbol{E}_{r}{ }^{T} \boldsymbol{d}_{r, p c o, i, e n u}
      $$

    * **PCV修正**则通过对高度角进行插值得到
      $$
      \boldsymbol{d}_{r, p c o, i}=\boldsymbol{E}_{r}{ }^{T} \boldsymbol{d}_{r, p c o, i, e n u}d_{r, p c v, i}(E l)=\frac{\left(E l-E l_{i}\right) d_{r, p c v, i}\left(E l_{i}\right)+\left(E l_{i+1}-E l\right) d_{r, p c v, i}\left(E l_{i+1}\right)}{E l_{i+1}-E l_{i}}
      $$

* **卫星天线相位中心模型**：

  卫星天线偏移（Satellite Antenna Offsets）是指卫星的天线相位中心与卫星质心的偏移。在PPP中我们需要用到精密星历。IGS 提供的精密星历(卫星位置)是相对卫星质心的，而载波相位、伪距测量值是相对天线相位中心的，因此我们需要计算PCV和PCO值，将所有值统一转换成相对天线相位中心。

  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/04e503cc4c16481c96ebac52430258ad.png)


### 2、pcv文件读取

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/706f1ff966ec4068a57c96ab4463b64b.png)


> readantex() 函数有bug，接收机端同时出现 GPS、GLO 的PCO、PCV时，会用 GLO 系统的值覆盖 GPS

### 3、RTK中的接收机天线相位中心改正：antmodel()

`zdres()`函数:位于rtkpos.c中，调用`antmodel()`函数计算接收机端修正值，在`zdres_sat()`函数中进行修正。相对定位进行单差时，已经将卫星端的天线误差消除了，在处理相对定位中，可采用同一类型的天线且天线指北的方法从而消除接收机天线相位中也偏差的影响，但是在基线两端使用不用的天线时必须对其进行改正。执行流程如下：

* 计算LOS视向量在ENU中的单位矢量`e`
* 频段不同，天线的相位中心偏移(PCO)不同，先计算出每个频段天线在东、北、天三个方向总的偏移,即相位中心偏移`pcv->off`与`del[j]`之和。
* 计算相位中心偏移(PCO)在观测单位矢量`e`上的投影`dot(off,e,3)`
* 计算天线相位中心变化量(PCV)：不同的高度角，相位中心变化不同，因此根据高度角对`pcv->var[i]`进行插值计算。
* PCO和PCV两部分求和为`dant[]`

> `del`为相对天线参考点偏移值
>
> `azel`为方位角和俯仰角，
>
> `pcv->off`为 phase center offset（PCO）
>
> `pcv->var`为 phase center variations （PCV）

```c
extern void antmodel(const pcv_t *pcv, const double *del, const double *azel,
                     int opt, double *dant)
{
    double e[3],off[3],cosel=cos(azel[1]);
    int i,j;
    
    trace(4,"antmodel: azel=%6.1f %4.1f opt=%d\n",azel[0]*R2D,azel[1]*R2D,opt);
    
    //计算LOS视向量在ENU中的单位矢量e
    e[0]=sin(azel[0])*cosel;
    e[1]=cos(azel[0])*cosel;
    e[2]=sin(azel[1]);
    
    //频段不同，天线的相位中心偏移(PCO)不同。
    //先计算出每个频段天线在东、北、天三个方向总的偏移,即相位中心偏移pcv->off[i][j]与del[j]之和
    for (i=0;i<NFREQ;i++) {
        for (j=0;j<3;j++) off[j]=pcv->off[i][j]+del[j]; //相位中心偏移(PCO),pcv->off[i][j]中的值来自于天线PCV文件
        
        //相位中心偏移(PCO)在观测单位矢量e上的投影dot(off,e,3)
        //计算天线相位中心变化量(PCV)：不同的高度角，相位中心变化不同，因此根据高度角对pcv->var[i]进行插值计算。
        //dant[]为上面两部分相加
        dant[i]=-dot(off,e,3)+(opt?interpvar(90.0-azel[1]*R2D,pcv->var[i]):0.0);
    }
    trace(5,"antmodel: dant=%6.3f %6.3f\n",dant[0],dant[1]);
}
```


## 六、selsat()：选择共视卫星

* 返回基准站和流动站之间的同步观测的卫星个数`ns`，输出卫星号列表`sat`，在接收机观测值中的index值列表`iu`和在基站观测值中的index值列表`ir`
* `obs[i]`中：流动站为`obs[0~nu-1]` ，基准站为`obs[nu~nu+nr-1]`

```c
static int selsat(const obsd_t *obs, double *azel, int nu, int nr,
                  const prcopt_t *opt, int *sat, int *iu, int *ir)
{
    int i,j,k=0;
    
    trace(3,"selsat  : nu=%d nr=%d\n",nu,nr);
    
    for (i=0,j=nu;i<nu&&j<nu+nr;i++,j++) {
        if      (obs[i].sat<obs[j].sat) j--;
        else if (obs[i].sat>obs[j].sat) i--;
        else if (azel[1+j*2]>=opt->elmin) { /* elevation at base station */
            sat[k]=obs[i].sat; iu[k]=i; ir[k++]=j;
            trace(4,"(%2d) sat=%3d iu=%2d ir=%2d\n",k-1,obs[i].sat,i,j);
        }
    }
    return k;
}
```



## 七、udstate()：Kalman滤波时间更新

![image-20241006144611825](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241006144611825.png)

kalman滤波的时间更新，更新状态值`rtk->x`及其误差协方差`rtk->P`

### 1、传入参数

```c
rtk_t    *rtk      IO  rtk控制结构体
obsd_t   *obs      I   观测数据
int      sat       I   接收机和基站共同观测的卫星号列表
int      *iu       I   接收机和基站共同观测的卫星在接收机观测值中的index值列表
int      *ir       I   接收机和基站共同观测的卫星在基站观测值中的index值列表
int      ns        I   接收机和基站共同观测的卫星个数
nav_t    *nav      I   导航数据
```
### 2、执行流程

* 调用`udpos()`根据不同模式更新rtk中的位置、速度、加速度值和协方差。

* 电离层模式`>=IONOOPT_EST`，调用`baseline()`计算基线长度`BL`，调用`udion()`根据基线长度`bl`更新状态 rtk->x 中的电离层参数（MAXSAT个）及其协方差。

  > 在基线比较长的情况下（>10km）,由于电离层的误差，通过双差不能完全消除，因此需要考虑电离层的影响。在长基线的情况，可以在配置中选择电离层修正方式为IONOOPT_EST。这种配置下，会将垂直方向的单差电离层延迟添加到卡尔曼滤波状态量中。

* 若对流层模式`>=TROPOPT_EST`，调用`udtrop()`根据基线长度`BL`更新状态 rtk->x 中的对流层参数（2或6个）及其协方差。

  > 调用`udtrop()会传入`电离层模式`>=IONOOPT_EST`，调用`baseline()`计算基线长度`BL`？

* 若为 GLONASS AR模式，调用`udrcvbias()`更新接收机硬件偏移。 

* 若模式`>PMODE_DGPS`，用载波相位定位，调用`udbias`更新载波相位偏移状态值以及其误差协方差。 （周跳检测）

```c
static void udstate(rtk_t *rtk, const obsd_t *obs, const int *sat,
                    const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double tt=rtk->tt,bl,dr[3];
    
    trace(3,"udstate : ns=%d\n",ns);

    //调用 udpos 根据不同模式更新rtk中的位置、速度、加速度值和协方差
    /* temporal update of position/velocity/acceleration */
    udpos(rtk,tt);
    
    //电离层模式>=IONOOPT_EST，调用 udion 更新状态 rtk->x 中的电离层参数（MAXSAT个）及其协方差
    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt>=IONOOPT_EST) {    
        bl=baseline(rtk->x,rtk->rb,dr);
        udion(rtk,tt,bl,sat,ns);
    }

    //若对流层模式>=TROPOPT_EST，调用 udtrop 更新状态 rtk->x 中的对流层参数（2或6个）及其协方差
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt>=TROPOPT_EST) {
        udtrop(rtk,tt,bl);          //也需要用到上面电离层处理时的基线长度bl
    }

    //若为 GLONASS AR模式，调用 udrcvbias 更新接收机硬件偏移。
    /* temporal update of eceiver h/w bias */
    if (rtk->opt.glomodear==2&&(rtk->opt.navsys&SYS_GLO)) {
        udrcvbias(rtk,tt);
    }

    //若 模式>PMODE_DGPS，调用 udbias 更新载波相位偏移状态值以及其误差协方差。
    /* temporal update of phase-bias */
    if (rtk->opt.mode>PMODE_DGPS) {
        udbias(rtk,tt,obs,sat,iu,ir,ns,nav);
    }
}
```
### 3、initx()：初始化状态和协方差 

赋值`xi`、`var`、给`rtk->x[i]`，`rtk->P[i,i]`，`rtk->P[i]`上非对角线元素赋0

```c
static void initx(rtk_t *rtk, double xi, double var, int i)
{
    int j;
    rtk->x[i]=xi;   //赋值rtk->x[i]
    for (j=0;j<rtk->nx;j++) {   //遍历rtk->P[i]，赋值rtk->P[i，i]
        rtk->P[i+j*rtk->nx]=rtk->P[j+i*rtk->nx]=i==j?var:0.0;   //对角线上协方差为方差，对角线外协方差为0
    }
}
```

* `rtk->opt.prn[]`：过程噪声(process-noise std)：[0]bias,[1]iono [2]trop [3]acch [4]accv [5] pos 。
* `rtk->opt.std[]`：初始状态标准差(initial-state std)：[0]bias,[1]iono [2]trop

### 4、udpos()：位置参数时间更新

更新rtk中的位置、速度、加速度值和协方差

**几种模式**：

* **流动站固定模式**（已知流动站坐标）：以流动站坐标为每一历元的时间更新值，并给协方差阵`P`设置一个较小的方差值 (1E-8)

* **流动站静态定位**：以上一历元的解作为这一历元的时间更新值，不需要更新，直接return

* **流动站动态定位（非动力学）**：以这一历元的单点定位解作为位置状态量的时间更新值，方差为默认 (30*30)

* **流动站动态定位（动力学）**：假设历元间保持匀速运动，根据速度、加速度完成时间更新，协方差过大会用伪距、多普勒计算的位置、速度重置，加速度设为极小值1E-6

  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6fb97020960145a78997fe4f6f9fec63.png)

**传入参数**：

```c
rtk_t    *rtk      IO    rtk控制结构体
double   tt        I     本次更新与上次更新的时间差
```

**执行流程**：

* 若为`PMODE_FIXED`模式，直接从选项中取得位置值给`rtk->x`将并其在协方差阵P的方差设置为一个较小的方差值（1E-8） ，然后返回

* 若为第一个历元，用`rtk->sol`中的位置值初始化`rtk->x`,位置状态量初始化为单点定位所得到的位置值，方差设置为VAR_POS

* 若为`PMODE_STATIC`静态模式，上一时刻状态就是当前时刻状态，return

* 若不是 dynamics 模式，用`rtk->sol`中的位置值初始化`rtk->x`，return，下面执行的都是dynamic模式的处理，对速度和加速度也作为参数估计：

  * 检查位置协方差，如果位置状态量的平均方差大于阈值VAR_POS则用`rtk->sol`中的位置值、速度值重置`rtk->x[0~2]`、`rtk->x[3~5]`，1E-6赋值`rtk->x[3~5]`，方差分别设为`VAR_POS`、`VAR_VEL`、`VAR_ACC`。

  * 生成有效状态(非0)下标`ix[]`

  * 用时间差`tt`构建状态转移矩阵`F`

    ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/bf0f0baa95774bcc98ac460a7a64388a.png)


  * `rtk->x`赋值参数阵`x[]`、`rtk->P`赋值方差阵`P[]`

  * 状态更新`rtk->x`，`rtk->P`，$x=Fx$, $P=FPF^T+Q$ ，

  * 给加速度加过程噪声(随机游走噪声)`rtk->opt.prn[3]`，`rtk->opt.prn[4]`，先存到`Q[]`，再赋值给`rtk->P`

  ```c
  static void udpos(rtk_t *rtk, double tt)
  {
      double *F,*P,*FP,*x,*xp,pos[3],Q[9]={0},Qv[9],var=0.0;
      int i,j,*ix,nx;
      
      trace(3,"udpos   : tt=%.3f\n",tt);
      //若为 PMODE_FIXED 模式，直接从选项中取得位置值给rtk->x，然后返回
      /* fixed mode */
      if (rtk->opt.mode==PMODE_FIXED) {
          for (i=0;i<3;i++) initx(rtk,rtk->opt.ru[i],1E-8,i);
          return;
      }
      
      //若为第一个历元，用rtk->sol中的位置值初始化rtk->x
      /* initialize position for first epoch */
      if (norm(rtk->x,3)<=0.0) {
          for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
          if (rtk->opt.dynamics) {
              for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
              for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
          }
      }
      /* static mode */
      if (rtk->opt.mode==PMODE_STATIC) return;    //若为 PMODE_STATIC 模式，return
      
      //若非dynamics模式，用rtk->sol中的位置值初始化rtk->x，return
      /* kinmatic mode without dynamics */
      if (!rtk->opt.dynamics) {
          for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
          return;
      }
  
      //检查位置协方差，若大于阈值VAR_POS则用rtk->sol中的位置值重置rtk->x
      /* check variance of estimated postion */
      for (i=0;i<3;i++) var+=rtk->P[i+i*rtk->nx];
      var/=3.0;
      if (var>VAR_POS) {
          /* reset position with large variance */
          for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);  //位置
          for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);  //速度
          for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);            //加速度
          trace(2,"reset rtk position due to large variance: var=%.3f\n",var);
          return;
      }
  
      //生成有效状态(非0)下标ix[]
      /* generate valid state index */
      ix=imat(rtk->nx,1);
      for (i=nx=0;i<rtk->nx;i++) {
          if (rtk->x[i]!=0.0&&rtk->P[i+i*rtk->nx]>0.0) ix[nx++]=i;
      }
      if (nx<9) {
          free(ix);
          return;
      }
      
      //用时间差tt构建状态转移矩阵F
      /* state transition of position/velocity/acceleration */
      F=eye(nx); P=mat(nx,nx); FP=mat(nx,nx); x=mat(nx,1); xp=mat(nx,1);
      for (i=0;i<6;i++) {
          F[i+(i+3)*nx]=tt;
      }
      for (i=0;i<3;i++) {
          F[i+(i+6)*nx]=SQR(tt)/2.0;
      }
  
      //rtk->x赋值参数阵x[]、rtk->P赋值方差阵P[]
      for (i=0;i<nx;i++) {
          x[i]=rtk->x[ix[i]];
          for (j=0;j<nx;j++) {
              P[i+j*nx]=rtk->P[ix[i]+ix[j]*rtk->nx];
          }
      }
  
      //状态更新
      /* x=F*x, P=F*P*F+Q */
      matmul("NN",nx,1,nx,1.0,F,x,0.0,xp);
      matmul("NN",nx,nx,nx,1.0,F,P,0.0,FP);
      matmul("NT",nx,nx,nx,1.0,FP,F,0.0,P);
      for (i=0;i<nx;i++) {
          rtk->x[ix[i]]=xp[i];
          for (j=0;j<nx;j++) {
              [ix[i]+ix[j]*rtk->nx]=P[i+j*nx];
          }
      }
      
      //给加速度加过程噪声,先存到Q，再赋值给rtk->P
      /* process noise added to only acceleration */
      Q[0]=Q[4]=SQR(rtk->opt.prn[3])*fabs(tt);
      Q[8]=SQR(rtk->opt.prn[4])*fabs(tt);
      ecef2pos(rtk->x,pos);
      covecef(pos,Q,Qv);
      for (i=0;i<3;i++) for (j=0;j<3;j++) {
          rtk->P[i+6+(j+6)*rtk->nx]+=Qv[i+j*3];
      }
      free(ix); free(F); free(P); free(FP); free(x); free(xp);
  }
  ```

* **baseline()**：求基线长度，流动站坐标减基准站坐标的模

```c
static double baseline(const double *ru, const double *rb, double *dr)
{
    int i;
    for (i=0;i<3;i++) dr[i]=ru[i]-rb[i];
    return norm(dr,3);
}
```

### 5、udion()：电离层参数时间更新

> * 长基线电离层误差通过双差不能完全消除，可以配置IONOOPT_EST将垂直方向的单差电离层延迟添加到卡尔曼滤波状态量中 
> * 在整个电离层状态量的时间更新中，实际上仅对电离层状态量为0的那些卫星进行了初始化，其余卫星的电离层状态量并没有变化，仅仅是在协方差阵中加入了过程噪声。

`IONOOPT_EST`模式才会执行此函数，电离层参数数为卫星数

**传入参数**：

```c
IO  rtk_t  	*rtk:     rtk solution structure
I   double 	tt:       当前历元与前一历元的时间差
I   double 	bl:       基线长度
I   int    	*sat:     移动站、基站共视星列表
I   int    	ns:       共视星个数
```

**执行流程**：

* 遍历每颗卫星，如果两个频率，载波中断计数都大于GAP_RESION(120)，电离层参数设为0 
* 遍历共视卫星
  * 如果电离层状态量为0，状态设为`1E-6`，协方差设为`SQR(rtk->opt.std[1]*bl/1E4)`，`bl`为基线长度
  * 电离层状态量不为0，加过程噪声`SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt)`

```c
static void udion(rtk_t *rtk, double tt, double bl, const int *sat, int ns)
{
    double el,fact;
    int i,j;
    
    trace(3,"udion   : tt=%.3f bl=%.0f ns=%d\n",tt,bl,ns);
    
    //遍历每颗卫星
    for (i=1;i<=MAXSAT;i++) {
        j=II(i,&rtk->opt);  //获取电离层参数下标j
        //如果两个频率，载波中断计数都大于GAP_RESION(120)，电离层参数设为0
        if (rtk->x[j]!=0.0&&
            rtk->ssat[i-1].outc[0]>GAP_RESION&&rtk->ssat[i-1].outc[1]>GAP_RESION)
            rtk->x[j]=0.0;
    }
    //遍历共视卫星
    for (i=0;i<ns;i++) {
        j=II(sat[i],&rtk->opt); //获取电离层参数下标
        
        //如果电离层状态量为0，状态设为1E-6，协方差设为SQR(rtk->opt.std[1]*bl/1E4)，bl为基线长度
        if (rtk->x[j]==0.0) {
            initx(rtk,1E-6,SQR(rtk->opt.std[1]*bl/1E4),j);
        }
        //电离层状态量不为0，加过程噪声SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt)
        else {
            /* elevation dependent factor of process noise */
            el=rtk->ssat[sat[i]-1].azel[1];
            fact=cos(el);
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[1]*bl/1E4*fact)*fabs(tt);
        }
    }
}
```

### 6、udtrop()：对流层参数时间更新

> * 在基线比较长（>10km）、或者是基线和移动站之间高度差较大的情况下，由于通过双差不能完全消除对流层误差，因此需要考虑对流层延迟的影响。在长基线的情况，可以在配置中选择对流层修正方式为Estiamte ZTD或者Estimate ZTD+Grad 
> * Estiamte ZTD：将基站、移动站天顶方向的对流层延迟($Z_b$,$Z_r$)加入卡尔曼滤波状态量，即新增2个状态量
> * Estimate ZTD+Grad：除了基线、移动天顶方向对流层延迟外，还会将东向、北向的对流层梯度系数$G_{E,r}$,$G_{N,r}$,$G_{E,b}$,$G_{N,b}$加入卡尔曼滤波状态量，即新增6个状态量

`TROPOPT_ESTG`和`TROPOPT_EST`模式才执行此函数。对流层参数数与卫星数无关，`TROPOPT_EST`模式为2个，`TROPOPT_ESTG`模式为6个。

**传入参数**：

```c
IO  rtk_t  	*rtk:     rtk solution structure
I   double 	tt:       当前历元与前一历元的时间差
I   double 	bl:       基线长度
```

**执行流程**：

* for循环，两次分别处理基站和移动站
  * 如果对流层状态量为0：用`INIT_ZWD`、`SQR(rtk->opt.std[2])`初始化天顶方向对流层延迟，`TROPOPT_ESTG`模式，用`1E-6`、`VAR_GRA`初始化东向、北向的对流层梯度系数
  * 如果对流层状态量不为0：增加过程噪声`SQR(rtk->opt.prn[2])*fabs(tt)`，`TROPOPT_ESTG`模式，给东向、北向的对流层梯度系数加过程噪声`SQR(rtk->opt.prn[2]*0.3)*fabs(tt)`

```c
static void udtrop(rtk_t *rtk, double tt, double bl)
{
    int i,j,k;
    
    trace(3,"udtrop  : tt=%.3f\n",tt);
    
    //处理基站、移动站对流层延迟
    for (i=0;i<2;i++) { 
        j=IT(i,&rtk->opt);  //获取对流层参数下标j
        //如果对流层状态量为0，
        if (rtk->x[j]==0.0) {   
        //用INIT_ZWD、SQR(rtk->opt.std[2])初始化天顶方向对流层延迟
            initx(rtk,INIT_ZWD,SQR(rtk->opt.std[2]),j); 
            //TROPOPT_ESTG模式，用1E-6、VAR_GRA初始化东向、北向的对流层梯度系数
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {   
                for (k=0;k<2;k++) initx(rtk,1E-6,VAR_GRA,++j);
            }
        }
        //对流层状态量不为0
        else {
            //增加过程噪声SQR(rtk->opt.prn[2])*fabs(tt)
            rtk->P[j+j*rtk->nx]+=SQR(rtk->opt.prn[2])*fabs(tt);
            //TROPOPT_ESTG模式，给东向、北向的对流层梯度系数加过程噪声`SQR(rtk->opt.prn[2]*0.3)*fabs(tt)`
            if (rtk->opt.tropopt>=TROPOPT_ESTG) {   
                for (k=0;k<2;k++) {
                    rtk->P[++j*(1+rtk->nx)]+=SQR(rtk->opt.prn[2]*0.3)*fabs(tt);
                }
            }
        }
    }
}
```

### 7、udrcvbias()：GLONASS接收机硬件偏移时间更新

 若为 GLONASS AR模式，调用此函数更新接收机硬件偏移

```c
static void udrcvbias(rtk_t *rtk, double tt)
{
    int i,j;
    
    trace(3,"udrcvbias: tt=%.3f\n",tt);
    
    //遍历GLONASS的频率
    for (i=0;i<NFREQGLO;i++) {
        j=IL(i,&rtk->opt);  //获取下标
        
        //如果接收机硬件偏移参数为0，用1E-6,VAR_HWBIAS初始化
        if (rtk->x[j]==0.0) {
            initx(rtk,1E-6,VAR_HWBIAS,j);
        }
        
        /* hold to fixed solution */
        else if (rtk->nfix>=rtk->opt.minfix&&rtk->sol.ratio>rtk->opt.thresar[0]) {
            initx(rtk,rtk->xa[j],rtk->Pa[j+j*rtk->na],j);
        }
        else {
            rtk->P[j+j*rtk->nx]+=SQR(PRN_HWBIAS)*fabs(tt);  //加过程噪声
        }
    }
}
```

### 8、udbias()：单差相位偏移时间更新（单差整周模糊度 )

#### 1.单差模糊度更新原理

- 先进行**周跳检测**

- 然后利用单差伪距和单差载波相位计算一个**单差相位偏移平均值**，来对状态中已有的单差模糊度进行状态更新如下

  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5f17401838bc4d0db306c105719699de.png)


- 若状态中没有对应卫星的单差模糊度，则直接将该单差相位估计值作为其单差模糊度初值

#### 2.传入参数

```c
rtk_t    *rtk      IO  rtk控制结构体
double   tt        I   本次更新与上次更新的时间差
obsd_t   *obs      I   观测数据
int      sat       I   接收机和基站共同观测的卫星号列表
int      *iu       I   接收机和基站共同观测的卫星在接收机观测值中的index值列表
int      *ir       I   接收机和基站共同观测的卫星在基站观测值中的index值列表
int      ns        I   接收机和基站共同观测的卫星个数
nav_t    *nav      I   导航数据
```

#### 3.执行流程

- 首先是对所有共视星进行**周跳检测**，遍历共视卫星：
  - 调用`detslp_ll()`，根据LLI检查接收机和基站观测数据是否有周跳
  - 调用`detslp_gf()`，利用几何无关组合进行周跳检测 
  - 调用`detslp_dop()`，利用多普勒进行周跳检测（暂未使用）
  - 更新half-cycle valid flag 
- 遍历每一个频率：
- **判断是否需要重置单差相位偏移**：
  - 遍历每一颗卫星，若为`instantaneous`模式，或者卫星载波相位的中断次数`rtk->ssat[i-1].outc`大于配置中所设置的最大次数`rtk->opt.maxout`，重置载波偏移值为0
  - 遍历共视卫星，对单差相位偏移状态量的协方差阵加入过程噪声`rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt)`，如果发现有周跳或者异常值，则单差相位偏移状态量重置为0
- **更新相位偏移**，遍历共视卫星：
  - 如果不是`IONOOPT_IFLC`消电离层组合：
    - 调用`sdobs()`函数，对基准站、流动站观测值作差，求出载波相位单差观测值`cp`、伪距单差观测值`pr`
    - 调用`sat2freq()`函数，获取载波频率
    - 无效观测值，continue
    - 相位偏差`bias[i]` = 单差伪距 - (单差载波相位/光速)：`bias[i]=cp-pr*freqi/CLIGHT;`
  - 如果是消电离层组合：获取两个频率的单差伪距，用消电离层组合的方法计算相位偏差`bias[i]`
  - 如果相位偏移不为0，有效卫星数`j++`，`offset`累加上与原来相位偏移的差
- 上一步累加有效卫星的相位偏差的差值到`offset`，除以有效星的数`j`，求得**单差相位偏移平均值**，所有有效卫星的相位偏差加上此平均值，完成相位偏移更新：`rtk->x[IB(i,k,&rtk->opt)] += offset/j`
- 利用求得的`bias[i]`来对相位偏差参数为0的卫星，进行单差相位偏移状态量的初始化。 

```c
static void udbias(rtk_t *rtk, double tt, const obsd_t *obs, const int *sat,
                   const int *iu, const int *ir, int ns, const nav_t *nav)
{
    double cp,pr,cp1,cp2,pr1,pr2,*bias,offset,freqi,freq1,freq2,C1,C2;
    int i,j,k,slip,reset,nf=NF(&rtk->opt);
    
    trace(3,"udbias  : tt=%.3f ns=%d\n",tt,ns);
    
    //首先是对所有共视星进行周跳检测
    //遍历每一颗卫星
    for (i=0;i<ns;i++) {
        //调用detslp_ll()，根据LLI检查接收机和基站观测数据是否有周跳
        /* detect cycle slip by LLI */  
        for (k=0;k<rtk->opt.nf;k++) rtk->ssat[sat[i]-1].slip[k]&=0xFC;
        detslp_ll(rtk,obs,iu[i],1);
        detslp_ll(rtk,obs,ir[i],2);

        //调用detslp_gf()利用几何无关组合进行周跳检测
        /* detect cycle slip by geometry-free phase jump */ 
        detslp_gf(rtk,obs,iu[i],ir[i],nav);
        
        //调用detslp_dop()，利用多普勒进行周跳检测（但该函数由于时间跳变的原因，暂未使用）。
        /* detect cycle slip by doppler and phase difference */ //调用 detslp_dop 通过多普勒和相位差检查接收机和基站观测数据是否有周跳
        detslp_dop(rtk,obs,iu[i],1,nav);
        detslp_dop(rtk,obs,ir[i],2,nav);
        
        /* update half-cycle valid flag */
        for (k=0;k<nf;k++) {
            rtk->ssat[sat[i]-1].half[k]=
                !((obs[iu[i]].LLI[k]&2)||(obs[ir[i]].LLI[k]&2));
        }
    }

    //遍历每一个频率
    for (k=0;k<nf;k++) {
        //若为instantaneous模式，
        //或者卫星载波相位的中断次数rtk->ssat[i-1].outc大于配置中所设置的最大次数rtk->opt.maxout，
        //重置载波偏移值
        /* reset phase-bias if instantaneous AR or expire obs outage counter */
        for (i=1;i<=MAXSAT;i++) {
            
            reset=++rtk->ssat[i-1].outc[k]>(uint32_t)rtk->opt.maxout;
            
            if (rtk->opt.modear==ARMODE_INST&&rtk->x[IB(i,k,&rtk->opt)]!=0.0) {
                initx(rtk,0.0,0.0,IB(i,k,&rtk->opt));
            }
            else if (reset&&rtk->x[IB(i,k,&rtk->opt)]!=0.0) {
                initx(rtk,0.0,0.0,IB(i,k,&rtk->opt));
                trace(3,"udbias : obs outage counter overflow (sat=%3d L%d n=%d)\n",
                      i,k+1,rtk->ssat[i-1].outc[k]);
                rtk->ssat[i-1].outc[k]=0;
            }
            if (rtk->opt.modear!=ARMODE_INST&&reset) {
                rtk->ssat[i-1].lock[k]=-rtk->opt.minlock;
            }
        }
        //对共视星进行循环，对单差相位偏移状态量的协方差阵加入过程噪声rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt)
        //如果发现有周跳或者异常值，则单差相位偏移状态量重置为0。
        /* reset phase-bias if detecting cycle slip */
        for (i=0;i<ns;i++) {
            j=IB(sat[i],k,&rtk->opt);
            rtk->P[j+j*rtk->nx]+=rtk->opt.prn[0]*rtk->opt.prn[0]*fabs(tt);
            slip=rtk->ssat[sat[i]-1].slip[k];
            if (rtk->opt.ionoopt==IONOOPT_IFLC) slip|=rtk->ssat[sat[i]-1].slip[1];
            if (rtk->opt.modear==ARMODE_INST||!(slip&1)) continue;
            rtk->x[j]=0.0;
            rtk->ssat[sat[i]-1].lock[k]=-rtk->opt.minlock;
        }
        bias=zeros(ns,1);


        //利用单差伪距、单差载波相位计算一个 单差相位偏移 估计值，来对单差相位偏移状态量进行更新
        
        /* estimate approximate phase-bias by phase - code 
        对共视星进行循环，利用“单差伪距”和“单差载波相位”计算一个“单差相位偏移”估计值，
        来对单差相位偏移状态量进行更新。由于如果忽略伪距误差，那么伪距减去载波相位，
        则应该是载波相位（m）的相位偏移(m)，所以这里计算的是一个大致的相位偏移bias[i]。
        如果配置为无电离层组合，计算则按照无电离层组合的方式来计算。
        最后，仅计算所有有效星（单差相位偏移状态不为0）的offset = sum of (bias - phase-bias)。
        其实就是计算每颗有效星bias[i]与单单差相位偏移状态量的偏差，然后把所有有效星的这个偏差值加起来，
        之后会除以有效星的个数，最终就是求一个偏差平均值rtk->com_bias。
        */
       //对共视星进行循环
        for (i=j=0,offset=0.0;i<ns;i++) {
            //不是消电离层模式
            if (rtk->opt.ionoopt!=IONOOPT_IFLC) {
                cp=sdobs(obs,iu[i],ir[i],k); /* cycle */        //载波相位单差观测值
                pr=sdobs(obs,iu[i],ir[i],k+NFREQ);              //伪距单差观测值
                freqi=sat2freq(sat[i],obs[iu[i]].code[k],nav);  //载波相位频率
                if (cp==0.0||pr==0.0||freqi==0.0) continue;
                
                bias[i]=cp-pr*freqi/CLIGHT;     //大致的相位偏移
            }
            //消电离层模式
            else {
                cp1=sdobs(obs,iu[i],ir[i],0);
                cp2=sdobs(obs,iu[i],ir[i],1);
                pr1=sdobs(obs,iu[i],ir[i],NFREQ);
                pr2=sdobs(obs,iu[i],ir[i],NFREQ+1);
                freq1=sat2freq(sat[i],obs[iu[i]].code[0],nav);
                freq2=sat2freq(sat[i],obs[iu[i]].code[1],nav);
                if (cp1==0.0||cp2==0.0||pr1==0.0||pr2==0.0||freq1==0.0||freq2<=0.0) continue;
                
                C1= SQR(freq1)/(SQR(freq1)-SQR(freq2));
                C2=-SQR(freq2)/(SQR(freq1)-SQR(freq2));
                
                bias[i]=(C1*cp1*CLIGHT/freq1+C2*cp2*CLIGHT/freq2)-(C1*pr1+C2*pr2);
            }
            if (rtk->x[IB(sat[i],k,&rtk->opt)]!=0.0) {
                offset += bias[i]-rtk->x[IB(sat[i],k,&rtk->opt)]; //与原来状态量中单差模糊度的偏差的总和
                j++;
            }
        }
     
        /* correct phase-bias offset to enssure phase-code coherency */
        if (j>0) {
            for (i=1;i<=MAXSAT;i++) {
                if (rtk->x[IB(i,k,&rtk->opt)]!=0.0) rtk->x[IB(i,k,&rtk->opt)]+=offset/j;
            }
        }
        //利用求得的bias[i]来对没有进行初始化的卫星，进行单差相位偏移状态量的初始化。
        /* set initial states of phase-bias */
        for (i=0;i<ns;i++) {
            if (bias[i]==0.0||rtk->x[IB(sat[i],k,&rtk->opt)]!=0.0) continue;
            initx(rtk,bias[i],SQR(rtk->opt.std[0]),IB(sat[i],k,&rtk->opt));
        }
        free(bias);
    }
}
```

#### 4.sdobs()：求单差观测值

```c
static double sdobs(const obsd_t *obs, int i, int j, int k)
{
    double pi=(k<NFREQ)?obs[i].L[k]:obs[i].P[k-NFREQ];
    double pj=(k<NFREQ)?obs[j].L[k]:obs[j].P[k-NFREQ];
    return pi==0.0||pj==0.0?0.0:pi-pj;
}
```

## 八、周跳检测

### 1、周跳探测与修复概念

周跳的探测与修复就是运用一定的方法探测出在何时发生了整周跳变，并求出丢失的整周数，然后将中断后的整周计数恢复为正确的计数，使这部分观测值正常使用。

#### 1.周跳产生原因

- 在GNSS测量中，接收机开机时，观测拍频的小数部分，并初始化整周计数。连续跟踪的情况下，小数部分的相位从$2\pi$变成０时计数器加１。
- 因此，在某一给定的历元下，观测的累积相位$\Delta_\phi$等于相位整周计数N加上小数部分。**整周数Ｎ被认为是一个未知数**。在接收载波信号过程中不发生失锁的倩况下，整周数Ｎ就一直保持某一固定值。
- 当**累积相位出现整周数的跳变时，整周计数就要重新进行初始化**。由于卫星信号受到阻挡等原因而导致**卫星信号失锁**，当某个历元载波信号重新被接收到后，计数器整周计数丢失，整周数发生错误，但小数部分仍然是正确的，这就是周跳现象。
- 引起整周计数中断原因主要有：
  - **信号遮挡**（特别是基于载波相位的动态定位）
  - **低信噪比**，如恶劣的电离层条件、多路径效应、接收机的高速运动或者卫星高度角过低
  - 接收机软件问题、卫星振荡器故障

#### 2.周跳对定位精度的影响

- 观测数据中大于10周的周跳，在数据预处理时不难发现，可Ｗ消除。而对于小于10周的周跳，特别是1-5周的周跳，以及半周跳和1/4周跳不易发现，而对含有周跳的观测值，周跳的影响视为观测的偶然误差，因而严重影响测量的精度。
- 即使**只有一颗卫星存在一个周跳，也会对测点产生几厘米的误差**。一个点位坐标是由４颗上卫星所确定的，故周跳对点位坐标的影响取决于卫星数、几何结构、周跳影响各分量的大小和周跳次数。
- 凡精度要求达到厘米级或分米级的GNSS测量，都必须将观测数据中的周跳全部清除。

#### 3.周跳检验量

- 载波相位观测量的时间序列，表现出来的是一条随时间变化的**光滑曲线**，因为整周模糊度不随时间变化，一旦出现周跳，这种光滑性就被破坏，而且自该历元起，后继历元相位观测值序列均发生**等量阶跃**。
- **判读周跳实质上就是探测观测序列是否发生阶跃突变**，在探测之前最好能尽量**消除**观测序列中的各项**误差项**，使得观测序列能准确反映出周跳的变化。
- 由载波相位的观测方程可知，在方程中随时间变化的项主要有接收机至卫星间的几何距离、接收机和卫星的钟差以及及大气层延迟等三项。**对GNSS观测值进行适当的組合，将上面三项中的大部分误差消除，即可构成周跳检测量**。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/da426007994d49c2b09ab1c918aeb42d.png)

#### 4.LLI：失锁标识符

> 参考博客：[周跳探测——LLI](http://t.csdn.cn/TXAze)

- 范围为0-7，对应二进制数000-111；有三个位，其中bit0、bit1仅用于相位
- 0或空格：正常或未知
- **Bit0**：先前与当前观测值之间失锁，可能发生了周跳（只针对相位观测值）
- **Bit1**：接收机进行半周模糊度解算，或程序不能处理半周数据而跳过该观测值的记录（只对当前历元有效）
- **Bit2**：bit 2置1表示为反欺骗(AS)下的观测值(可能会受到噪声增加的影响) 
- 理论上只要板卡足够好，它自身的LLI标志就能把载波数据的周跳、半周跳告知你，就不需要其他探测方法了。LLI的是从信道方面探测周跳的，效果应该比我们从数据方法探测周跳更灵敏。
- 有的板卡或许LLI探测太灵敏了，把没有问题的数据也当作周跳进行标志，可能会导致观测数据不足，故有时会把LLI周跳探测方法关闭。   

### 2、detslp_ll()：根据LLI进行周跳检测

```c
static void detslp_ll(rtk_t *rtk, const obsd_t *obs, int i, int rcv)
{
    uint32_t slip,LLI;
    int f,sat=obs[i].sat;
    
    trace(3,"detslp_ll: i=%d rcv=%d\n",i,rcv);
    
    //nf是频点个数(1:L1,2:L1+L2,3:L1+L2+L5)
    for (f=0;f<rtk->opt.nf;f++) {
        
        if (obs[i].L[f]==0.0||
            fabs(timediff(obs[i].time,rtk->ssat[sat-1].pt[rcv-1][f]))<DTTOL) {
            continue;
        }
        /* restore previous LLI */
        if (rcv==1) LLI=getbitu(&rtk->ssat[sat-1].slip[f],0,2); /* rover */
        else        LLI=getbitu(&rtk->ssat[sat-1].slip[f],2,2); /* base  */
        
        /* detect slip by cycle slip flag in LLI */
        if (rtk->tt>=0.0) { /* forward */
            //bit0为1，表明当前历元可能发生了周跳
            if (obs[i].LLI[f]&1) {  
                errmsg(rtk,"slip detected forward  (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,obs[i].LLI[f]);
            }
            slip=obs[i].LLI[f];
        }
        else { /* backward */
            //前一历元bit0位为1，表明前一历元发生可能发生了周跳
            if (LLI&1) {
                errmsg(rtk,"slip detected backward (sat=%2d rcv=%d F=%d LLI=%x)\n",
                       sat,rcv,f+1,LLI);
            }
            slip=LLI;
        }
        /* detect slip by parity unknown flag transition in LLI */
        //(LLI&2)&&!(obs[i].LLI[f]&2)--前一历元bit1 位为1并且当前历元bit1位不为1
        //(!(LLI&2)&&(obs[i].LLI[f]&2) ----前一历元bit1 位为0且当前历元bit1位为1
        //即前后历元的bit1位不相同，表明可能发生了半周跳。
        if (((LLI&2)&&!(obs[i].LLI[f]&2))||(!(LLI&2)&&(obs[i].LLI[f]&2))) {
            errmsg(rtk,"slip detected half-cyc (sat=%2d rcv=%d F=%d LLI=%x->%x)\n",
                   sat,rcv,f+1,LLI,obs[i].LLI[f]);
            slip|=1;
        }
        /* save current LLI */
        if (rcv==1) setbitu(&rtk->ssat[sat-1].slip[f],0,2,obs[i].LLI[f]);
        else        setbitu(&rtk->ssat[sat-1].slip[f],2,2,obs[i].LLI[f]);
        
        /* save slip and half-cycle valid flag */
        rtk->ssat[sat-1].slip[f]|=(uint8_t)slip;
        rtk->ssat[sat-1].half[f]=(obs[i].LLI[f]&2)?0:1;
    }
}
```

### 3、detslp_gf()：利用几何无关组合进行周跳检测
> 需要双频

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/48ba3d892a474c9fb8765baf5a2b9a3d.png)


上式理论等于0，若不为0（大于阈值0.05m），则认为检测到了周跳

```c
static void detslp_gf(rtk_t *rtk, const obsd_t *obs, int i, int j,
                      const nav_t *nav)
{
    int k,sat=obs[i].sat;
    double g0,g1;
    
    trace(3,"detslp_gf: i=%d j=%d\n",i,j);
    
    for (k=1;k<rtk->opt.nf;k++) {
        if ((g1=gfobs(obs,i,j,k,nav))==0.0) return;
         
        g0=rtk->ssat[sat-1].gf[k-1];
        rtk->ssat[sat-1].gf[k-1]=g1;
         
        if (g0!=0.0&&fabs(g1-g0)>rtk->opt.thresslip) {
            rtk->ssat[sat-1].slip[0]|=1;
            rtk->ssat[sat-1].slip[k]|=1;
            errmsg(rtk,"slip detected GF jump (sat=%2d L1-L%d GF=%.3f %.3f)\n",
                   sat,k+1,g0,g1);
        }
    }
}
```

- **gfobs()**：求几何无关组合观测值

  ```c
  static double gfobs(const obsd_t *obs, int i, int j, int k, const nav_t *nav)
  {
      double freq1,freq2,L1,L2;
      
      freq1=sat2freq(obs[i].sat,obs[i].code[0],nav);
      freq2=sat2freq(obs[i].sat,obs[i].code[k],nav);
      L1=sdobs(obs,i,j,0);
      L2=sdobs(obs,i,j,k);
      if (freq1==0.0||freq2==0.0||L1==0.0||L2==0.0) return 0.0;
      return L1*CLIGHT/freq1-L2*CLIGHT/freq2;
  }
  ```

## 九、ddres()：计算双差残差、设计矩阵、新息向量

![image-20241006144647874](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20241006144647874.png)

站星双差、矩阵v、双差设计矩阵`H`，双差协方差矩阵

`R`

### 1、单差、双差的概念

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1f3ade17ced34b49bf6b125f3a56cf06.png)


### 2、双差残差、设计矩阵

> 摘自：[RTKLIB相对定位部分算法梳理](http://t.csdn.cn/vpqQi)

**非差残差** = 观测 - 卫地距(此处卫地距包含误差，即：`卫地距 = 真实卫地距 + 单位视线向量e * delta_r `) ，然后**站间单差星间双差**，完成H阵前三列与状态x前三项的乘法计算。后续继续**减去双差模糊度**，就完成了`v = Hx - Z`的计算，即**新息向量**。
![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5e632e7d6a3342719f4b2e1752b25286.png)


### 3、传入参数：

```c
rtk_t    *rtk      IO  rtk控制结构体
nav_t    *nav      I   导航数据
double   dt        I   接收机和基站的时间差
double   *x        IO  状态变量
double   *P        IO  状态变量的误差协方差阵
int      sat       I   接收机和基站共同观测的卫星号列表
double   *y        IO  相位/码残差
double   *e        IO  观测矢量 (ecef)
double   *azel     O   方位角和俯仰角 (rad)
int      *iu       I   接收机和基站共同观测的卫星在接收机观测值中的index值列表
int      *ir       I   接收机和基站共同观测的卫星在基站观测值中的index值列表
int      ns        I   接收机和基站共同观测的卫星个数
double   *v        O   实际观测量与预测观测量的残差（双差观测值新息向量）
double   *R        O   测量误差的协方差
double   *H        O   观测矩阵
int      *vflg     O   数据有效标志
```
### 4、执行流程：

* 计算**基线**长度`bl`，基线向量`dr`
* 调用`ecef2pos()`，计算基准站、流动站**LLH**坐标`posu`、`posr`
* for循环，遍历每一个卫星、每一个频率，所有伪距残差`rtk->ssat[i].resp[j]`、载波相位残差`rtk->ssat[i].resc[j]`重置为0 
* 遍历所有共视卫星

  * 电离层模式>=`IONOOPT_EST`，调用`ionmapf()`计算电离层延迟因子，`im[i]`取基准站、流动站的平均
  * 对流层模式>=`TROPOPT_EST`，调用`prectrop()`计算对流层延迟因子，流动站`tropu[i]`、基准站`tropr[i]`
* 遍历所有系统`for (m=0;m<6;m++)`
* 遍历所有频率`for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++)`；若为`PMODE_DGPS`伪距双差，需要限制遍历范围， 载波相位在`0-nf`，`nf-2nf`为伪距，因此伪距差分定位从`nf`开始

  * 寻找仰角最高的卫星作为**参考卫星**，选取失败则continue
  * 将`H+nv*rtk->nx`的地址赋给`H`，此时更改`Hi`中的值即更改H矩阵中的值 
  * **双差残差**`v[nv]`计算：【参考星(移动站) - 参考星(基准站)】- 【非参考星(移动站) - 非参考星(基准站)】 
  * 构建**设计矩阵**`H` ，移动站非参考星视线向量 - 移动站参考星视线向量
  * 若要估计**电离层**参数，模式`IONOOPT_EST`，用电离层延迟因子修正v和H
  * 若要估计**对流层**参数，模式`TROPOPT_EST`，用对流层延迟因子修正v和H 
  * 用**单差模糊度**参数修正`v`和`H`
  * 将双差载波残差存到`rtk->ssat[sat[j]-1].resc[f   ]`、双差伪距残差存到`rtk->ssat[sat[j]-1].resp[f-nf]`
  * **新息判断**，残差与设置新息值进行判断，根据选项maxinno的值检测是否要排除此观测数据 
  * 计算**单差协方差**`Ri`、`Rj `
  * 设置**数据有效标志**`vlfg`
* 如果是动基线模式，`constbl()`增加**基线长度约束** 
* 调用`ddcov()`，计算载波相位/伪距**双差量测噪声协方差阵**`R`
* 释放内存，返回有效数据数`nv`

```c
static int ddres(rtk_t *rtk, const nav_t *nav, double dt, const double *x,
                 const double *P, const int *sat, double *y, double *e,
                 double *azel, double *freq, const int *iu, const int *ir,
                 int ns, double *v, double *H, double *R, int *vflg)
{
    prcopt_t *opt=&rtk->opt;
    double bl,dr[3],posu[3],posr[3],didxi=0.0,didxj=0.0,*im;
    double *tropr,*tropu,*dtdxr,*dtdxu,*Ri,*Rj,freqi,freqj,*Hi=NULL;
    int i,j,k,m,f,nv=0,nb[NFREQ*4*2+2]={0},b=0,sysi,sysj,nf=NF(opt);
    
    trace(3,"ddres   : dt=%.1f nx=%d ns=%d\n",dt,rtk->nx,ns);
    
    bl=baseline(x,rtk->rb,dr);  //计算基线长度bl，基线向量dr
    ecef2pos(x,posu); ecef2pos(rtk->rb,posr);   //基准站、流动站 XYZ-LLH
    
    Ri=mat(ns*nf*2+2,1); Rj=mat(ns*nf*2+2,1); im=mat(ns,1);
    tropu=mat(ns,1); tropr=mat(ns,1); dtdxu=mat(ns,3); dtdxr=mat(ns,3);
    
    for (i=0;i<MAXSAT;i++) for (j=0;j<NFREQ;j++) {
        rtk->ssat[i].resp[j]=rtk->ssat[i].resc[j]=0.0;  //所有伪距残差、载波相位残差重置为0
    }
    /* compute factors of ionospheric and tropospheric delay */     
    for (i=0;i<ns;i++) {
        //若 电离层模式>=IONOOPT_EST，调用 ionmapf() 计算电离层延迟因子
        //基准站、流动站，im[i]取基准站、流动站的平均
        if (opt->ionoopt>=IONOOPT_EST) {    
            im[i]=(ionmapf(posu,azel+iu[i]*2)+ionmapf(posr,azel+ir[i]*2))/2.0;
        }
        //若 对流层模式>=TROPOPT_EST，调用 prectrop() 计算对流层延迟因子
        //流动站tropu[i]、基准站tropr[i]
        if (opt->tropopt>=TROPOPT_EST) {    
            tropu[i]=prectrop(rtk->sol.time,posu,0,azel+iu[i]*2,opt,x,dtdxu+i*3);
            tropr[i]=prectrop(rtk->sol.time,posr,1,azel+ir[i]*2,opt,x,dtdxr+i*3);
        }
    }
    
    // 遍历不同系统
    for (m=0;m<6;m++) /* m=0:GPS/SBS,1:GLO,2:GAL,3:BDS,4:QZS,5:IRN */    
    //遍历每一个频率，计算双差残差
    //若为PMODE_DGPS伪距双差，需要限制遍历范围， 载波相位在0-nf，nf至2nf为伪距，因此伪距差分定位从nf开始
    for (f=opt->mode>PMODE_DGPS?0:nf;f<nf*2;f++) {
        //寻找仰角最高的卫星作为参考卫星
        /* search reference satellite with highest elevation */ 
        for (i=-1,j=0;j<ns;j++) {
            sysi=rtk->ssat[sat[j]-1].sys;   
            if (!test_sys(sysi,m)) continue;
            if (!validobs(iu[j],ir[j],f,nf,y)) continue;
            if (i<0||azel[1+iu[j]*2]>=azel[1+iu[i]*2]) i=j;
        }
        if (i<0) continue;  //选取失败则continue
        
        //遍历所有卫星，计算双差
        /* make DD (double difference) */
        for (j=0;j<ns;j++) {
            if (i==j) continue;
            sysi=rtk->ssat[sat[i]-1].sys;
            sysj=rtk->ssat[sat[j]-1].sys;
            freqi=freq[f%nf+iu[i]*nf];
            freqj=freq[f%nf+iu[j]*nf];
            if (!test_sys(sysj,m)) continue;
            if (!validobs(iu[j],ir[j],f,nf,y)) continue;

            //用传入的非差相位/码残差y计算双差残差v，并计算对应的H
            if (H) {    
                //将H+nv*rtk->nx的地址赋给H，此时更改Hi中的值即更改H矩阵中的值
                Hi=H+nv*rtk->nx;
                for (k=0;k<rtk->nx;k++) Hi[k]=0.0;
            }
            //双差残差计算
            //【参考星(移动站) - 参考星(基准站)】- 【非参考星(移动站) - 非参考星(基准站)】
            /* DD residual */
            v[nv]=(y[f+iu[i]*nf*2]-y[f+ir[i]*nf*2])-    
                  (y[f+iu[j]*nf*2]-y[f+ir[j]*nf*2]);
            
            //移动站位置偏导
            //移动站非参考星视线向量 - 移动站参考星视线向量
            /* partial derivatives by rover position */     
            if (H) {
                for (k=0;k<3;k++) {
                    Hi[k]=-e[k+iu[i]*3]+e[k+iu[j]*3];   //每一行前3列为观测向量差
                }
            }

            //若要估计电离层参数，模式IONOOPT_EST，用电离层延迟因子修正v和H
            //注意伪距和载波相位的电离层延迟大小相同，但是符号相反
            /* DD ionospheric delay term */
            if (opt->ionoopt==IONOOPT_EST) {    
                didxi=(f<nf?-1.0:1.0)*im[i]*SQR(FREQ1/freqi);   //载波相位为负
                didxj=(f<nf?-1.0:1.0)*im[j]*SQR(FREQ1/freqj);   //伪距为正
                v[nv]-=didxi*x[II(sat[i],opt)]-didxj*x[II(sat[j],opt)];
                if (H) {
                    Hi[II(sat[i],opt)]= didxi;
                    Hi[II(sat[j],opt)]=-didxj;
                }
            }

            //若要估计对流层参数，模式TROPOPT_EST，用对流层延迟因子修正v和H
            /* DD tropospheric delay term */
            if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {
                v[nv]-=(tropu[i]-tropu[j])-(tropr[i]-tropr[j]);
                for (k=0;k<(opt->tropopt<TROPOPT_ESTG?1:3);k++) {
                    if (!H) continue;
                    Hi[IT(0,opt)+k]= (dtdxu[k+i*3]-dtdxu[k+j*3]);
                    Hi[IT(1,opt)+k]=-(dtdxr[k+i*3]-dtdxr[k+j*3]);
                }
            }

            //用相位偏移修正v和H
            //如果不是无电离层组合，
            //从载波相位双差残差中扣除双差模糊度部分（即phase-bias），
            //并对H矩阵中和模糊度相关的部分进行赋值
            /* DD phase-bias term */
            if (f<nf) {
                if (opt->ionoopt!=IONOOPT_IFLC) {
                    v[nv]-=CLIGHT/freqi*x[IB(sat[i],f,opt)]-
                           CLIGHT/freqj*x[IB(sat[j],f,opt)];
                    if (H) {
                        Hi[IB(sat[i],f,opt)]= CLIGHT/freqi;
                        Hi[IB(sat[j],f,opt)]=-CLIGHT/freqj;
                    }
                }
                else {
                    v[nv]-=x[IB(sat[i],f,opt)]-x[IB(sat[j],f,opt)];
                    if (H) {
                        Hi[IB(sat[i],f,opt)]= 1.0;
                        Hi[IB(sat[j],f,opt)]=-1.0;
                    }
                }
            }
            //将双差载波，伪距残差，输出到相应的结构体中
            if (f<nf) rtk->ssat[sat[j]-1].resc[f   ]=v[nv];
            else      rtk->ssat[sat[j]-1].resp[f-nf]=v[nv];
            
            //根据选项maxinno的值检测是否要排除此观测数据
            /* test innovation */
            if (opt->maxinno>0.0&&fabs(v[nv])>opt->maxinno) {
                if (f<nf) {
                    rtk->ssat[sat[i]-1].rejc[f]++;
                    rtk->ssat[sat[j]-1].rejc[f]++;
                }
                errmsg(rtk,"outlier rejected (sat=%3d-%3d %s%d v=%.3f)\n",
                       sat[i],sat[j],f<nf?"L":"P",f%nf+1,v[nv]);
                continue;
            }

            //计算单差的测量误差协方差Ri、i为参考星，j为非参考星
            /* SD (single-differenced) measurement error variances */
            Ri[nv]=varerr(sat[i],sysi,azel[1+iu[i]*2],bl,dt,f,opt);
            Rj[nv]=varerr(sat[j],sysj,azel[1+iu[j]*2],bl,dt,f,opt);
            
            //设置数据有效标志，首先是载波，然后是伪距；根据f<nf判断
            /* set valid data flags */
            if (opt->mode>PMODE_DGPS) {
                if (f<nf) rtk->ssat[sat[i]-1].vsat[f]=rtk->ssat[sat[j]-1].vsat[f]=1;
            }
            else {
                rtk->ssat[sat[i]-1].vsat[f-nf]=rtk->ssat[sat[j]-1].vsat[f-nf]=1;
            }
            trace(4,"sat=%3d-%3d %s%d v=%13.3f R=%8.6f %8.6f\n",sat[i],
                  sat[j],f<nf?"L":"P",f%nf+1,v[nv],Ri[nv],Rj[nv]);
            
            vflg[nv++]=(sat[i]<<16)|(sat[j]<<8)|((f<nf?0:1)<<4)|(f%nf);
            nb[b]++;
        }
        b++;
    }
    /* end of system loop */
    
    //如果是动基线的模式，增加基线长度约束
    /* baseline length constraint for moving baseline */
    if (opt->mode==PMODE_MOVEB&&constbl(rtk,x,P,v,H,Ri,Rj,nv)) {
        vflg[nv++]=3<<4;
        nb[b++]++;
    }
    if (H) {trace(5,"H=\n"); tracemat(5,H,rtk->nx,nv,7,4);}
    
    //调用ddcov()，计算载波相位/伪距双差量测噪声协方差阵R
    /* DD measurement error covariance */
    ddcov(nb,b,Ri,Rj,nv,R);
    
    free(Ri); free(Rj); free(im);
    free(tropu); free(tropr); free(dtdxu); free(dtdxr);
    
    return nv;
}
```

### 5、ionmapf()：SLM(Single-Layer Model)投影映射模型

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/cba6d457002942348502efe8e261f4af.png)


```c
extern double ionmapf(const double *pos, const double *azel)
{
    if (pos[2]>=HION) return 1.0;   //如果接收机比电离层还高，return 1
    return 1.0/cos(asin((RE_WGS84+pos[2])/(RE_WGS84+HION)*sin(PI/2.0-azel[1])));
}
```

#### 6、prectrop()：精密对流层模型
传入参数：
```c
I      gtime_t time     当前历元时间
I      double *pos      基站/移动站位置：纬、经、高
I      int r            0表示移动站，1表示基站，用来计算对流层状态量在卡尔曼滤波状态矢量的索引
I      double *azel     高度角，方位角
I      prcopt_t *opt    处理选项
I      double *x        卡尔曼滤波状态矢量
IO     double *dtdx     对流层湿分量、北向和东向梯度因子
```

```c
static double prectrop(gtime_t time, const double *pos, int r,
                       const double *azel, const prcopt_t *opt, const double *x,
                       double *dtdx)
{
    double m_w=0.0,cotz,grad_n,grad_e;
    int i=IT(r,opt);
    
    /* wet mapping function */
    tropmapf(time,pos,azel,&m_w);   //计算干湿延迟投影系数
    
    if (opt->tropopt>=TROPOPT_ESTG&&azel[1]>0.0) {
        
        /* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6] */
        cotz=1.0/tan(azel[1]);
        grad_n=m_w*cotz*cos(azel[0]);
        grad_e=m_w*cotz*sin(azel[0]);
        m_w+=grad_n*x[i+1]+grad_e*x[i+2];
        dtdx[1]=grad_n*x[i];
        dtdx[2]=grad_e*x[i];
    }
    else dtdx[1]=dtdx[2]=0.0;
    dtdx[0]=m_w;
    return m_w*x[i];
}
```

#### 7、constbl()：动基线约束

动基线模式：流动站、基准站都在移动，**相对距离保持不变**，基准站位置由SPP得来，流动站位置由短基线相对定位得到，**只有相对位置有意义**，绝对位置就是单点定位的精度。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/3b0f29bb0e9a4558bdb6fbe343a83b78.png)


```c
static int constbl(rtk_t *rtk, const double *x, const double *P, double *v,
                   double *H, double *Ri, double *Rj, int index)
{
    const double thres=0.1; /* threshold for nonliearity (v.2.3.0) */
    double xb[3],b[3],bb,var=0.0;
    int i;
     
    trace(3,"constbl : \n");
    
    /* no constraint */
    if (rtk->opt.baseline[0]<=0.0) return 0;
    
    //
    /* time-adjusted baseline vector and length */
    for (i=0;i<3;i++) {
        xb[i]=rtk->rb[i];
        b[i]=x[i]-xb[i];
    }
    bb=norm(b,3);   
    
    //var为单点定位方差阵p的平均
    /* approximate variance of solution */
    if (P) {
        for (i=0;i<3;i++) var+=P[i+i*rtk->nx];
        var/=3.0;
    }
    //检查方差非线性化的程度，var>SQR(thres*bb)，就return
    /* check nonlinearity */
    if (var>SQR(thres*bb)) {
        trace(3,"constbl : equation nonlinear (bb=%.3f var=%.3f)\n",bb,var);
        return 0;
    }

    /* constraint to baseline length */
    v[index]=rtk->opt.baseline[0]-bb;
    if (H) {
        for (i=0;i<3;i++) H[i+index*rtk->nx]=b[i]/bb;
    }
    Ri[index]=0.0;
    Rj[index]=SQR(rtk->opt.baseline[1]);
    
    trace(4,"baseline len   v=%13.3f R=%8.6f %8.6f\n",v[index],Ri[index],Rj[index]);
    
    return 1;
}
```

#### 8、ddcov()：计算双差量测噪声协方差阵 

根据之前计算的**单差协方差**`Ri`、`Rj `来计算，`i==j`时为`Ri[k+i]+Rj[k+i]`，`i!=j`时为`Ri[k+i]`

```c
static void ddcov(const int *nb, int n, const double *Ri, const double *Rj,
                  int nv, double *R)
{
    int i,j,k=0,b;
    
    trace(3,"ddcov   : n=%d\n",n);
    
    for (i=0;i<nv*nv;i++) R[i]=0.0;
    for (b=0;b<n;k+=nb[b++]) {
        
        for (i=0;i<nb[b];i++) for (j=0;j<nb[b];j++) {
            R[k+i+(k+j)*nv]=Ri[k+i]+(i==j?Rj[k+i]:0.0);     //i==j时为Ri[k+i]+Rj[k+i]，i!=j时为Ri[k+i]
        }
    }
    trace(5,"R=\n"); tracemat(5,R,nv,nv,8,6);
}
```

## 十、filter()：卡尔曼滤波

- **X**：n阶参数向量，就是参数，不再是最小二乘中的参数的增量 
- **P**：nn阶协方差阵。X、P在filter()函数中既是传入参数，也是传出参数，迭代。
- **H**：nm阶设计矩阵的转置
- **V**：m阶新息向量
- **R**：mm阶量测噪声协方差阵
- **Xp**、**Pp**：预测参数和预测协方差

```c
extern  int filter_(const double *x, const double *P, const double *H,
                   const double *v, const double *R, int n, int m,
                   double *xp, double *Pp)
{
    double *F=mat(n,m),*Q=mat(m,m),*K=mat(n,m),*I=eye(n);
    int info;
    
    matcpy(Q,R,m,m);
    matcpy(xp,x,n,1);
    matmul("NN",n,m,n,1.0,P,H,0.0,F);       /* Q=H'*P*H+R */
    matmul("TN",m,m,n,1.0,H,F,1.0,Q);
    if (!(info=matinv(Q,m))) {
        matmul("NN",n,m,m,1.0,F,Q,0.0,K);   /* K=P*H*Q^-1 */
        matmul("NN",n,1,m,1.0,K,v,1.0,xp);  /* xp=x+K*v */
        matmul("NT",n,n,m,-1.0,K,H,1.0,I);  /* Pp=(I-K*H')*P */
        matmul("NN",n,n,n,1.0,I,P,0.0,Pp);
    }
    free(F); free(Q); free(K); free(I);
    return info;
}
```

```c
extern int filter(double *x, double *P, const double *H, const double *v,
                  const double *R, int n, int m)
{
    double *x_,*xp_,*P_,*Pp_,*H_;
    int i,j,k,info,*ix;
    
    ix=imat(n,1); for (i=k=0;i<n;i++) if (x[i]!=0.0&&P[i+i*n]>0.0) ix[k++]=i;
    x_=mat(k,1); xp_=mat(k,1); P_=mat(k,k); Pp_=mat(k,k); H_=mat(k,m);
    for (i=0;i<k;i++) {
        x_[i]=x[ix[i]];
        for (j=0;j<k;j++) P_[i+j*k]=P[ix[i]+ix[j]*n];
        for (j=0;j<m;j++) H_[i+j*k]=H[ix[i]+j*n];
    }
    info=filter_(x_,P_,H_,v,R,k,m,xp_,Pp_);
    for (i=0;i<k;i++) {
        x[ix[i]]=xp_[i];
        for (j=0;j<k;j++) P[ix[i]+ix[j]*n]=Pp_[i+j*k];
    }
    free(ix); free(x_); free(xp_); free(P_); free(Pp_); free(H_);
    return info;
}
```

## 十一、整周模糊度固定

### 1、模糊度固定概念

#### 1.概述

在以载波相位观测量为根据进行精密相对定位时，整周模糊度的确定是关键。整周模糊度的主要作用是用来计算站星几何距离，高精度的站星几何距离才能获取高精度的解算结果。

#### 2.部分模糊度固定

理论上，正确固定的模糊度数越多，定位精度和可靠性越高，然而需要指出的是，整体模糊度固定(Full Ambiguity Resolution, FAR)并不总是最优的选择。随着多频多系统的发展，有效观测值数量显著增加，致使模糊度参数的维度陡增，极大增加了整体模糊度固定的难度，使得模糊度固定的成功率和可靠性都显著下降。目前的部分模糊度固定算法归纳起来可大致分为三类：

- 根据观测值信噪比、卫星高度角等信息对参与固定的模糊度进行排序，选取噪声较小的模糊度参与固定而剔除噪声较大的模糊度参数
- 将各频率的原始模糊度组合成宽窄巷模糊度，固定宽项模糊度后，通过提高窄巷模糊度方差-协方差矩阵的精度来改善窄巷模糊度的固定效果
- 原始模糊度进行降相关处理后，根据一定策略和准则直接选取模糊度的子集进行固定

#### 3.Ratio factor

ratio-test用来验证整周模糊度；该值表示具有第二最佳整数向量的残差与最佳整数向量的平方和之比。 

#### 4.LAMBDA算法固定模糊度

##### ①LAMBDA算法目的

提高整周模糊度估计和捜索的效率，**减少计算量**。

##### ②LAMBDA算法流程

1. 建立双差方差，最小二乘估计整周模糊度浮点解$N_1$及其协方差矩阵$Q_1$
2. 由$Q_1$构造转换矩阵$Z$
3. 采用$Z$整数转换矩阵将$N_1$和$Q_1$转换成$N_2$和$Q_2$
4. 进行整周模糊度搜索求得$N_2$的整数解$N_3$及其协方差阵$Q_3$
5. 对$N_3$和$Q_3$进行$Z$逆变换得到$N_1$的整数解及其协方差矩阵

##### ③LAMBDA算法核心

- 基于整数变换的模糊度**降相关**——为了保证LAMBDA捜索的快捷性和准确性，需要首先对原始模糊度的协因数矩阵加以调整，使得原始模糊度间的相关性降低。
- 基于**序贯条件最小二乘**估计的整周捜索LAMBDA搜索是整个过程的关键和主体，其主要目的是通过连续的迭代找出最接近真的的模糊度整数值。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/52c78060560c477689d8c4aaff88cfb6.png" style="zoom: 67%;" />


### 2、三种模糊度固定选项

#### 1.continuous

认为模糊度是连续解，通过前面历元的解算结果滤波提高后续历元模糊度固定精度，相邻历元的无周跳模糊度给与一个极小的过程噪声。固定率不太稳定，有时可能比较高，但大部分情况可能固定率相对较低，可能原因是rtklib数据预处理不够完善，浮点解精度很差。 

#### 2.instantaneous

该模式中相邻历元模糊度没有相关性，即每个历元做一个最小二乘，然后直接就去尝试固定模糊度。单历元模式的优势，就是本历元结果仅与当前历元有关，不需要考虑周跳处理问题，不需要考虑滤波发散问题；缺点是未充分利用历元间的相关性。

#### 3.fix and hold

该模式是在Continues模式的基础上，如果模糊度固定正确，即会将固定的模糊度约束浮点滤波器。当然，如果模糊度错误固定，会导致浮点解出现较大偏差，导致较长时间的无法固定。

> 一种改进方法是：做工程可做两套，Instantaneous和Fix and Hold，发现Fix and Hold错了，就用Instantaneous的解把它替换掉，相当于把模糊度和方差初始化了一次，避免漂移和模糊度重新收敛的过程。

### 3、resamb_LAMBDA()：LAMBDA算法入口函数

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20210628192054418.png) 

#### 1.传入参数

```c
rtk_t *rtk		I	RTK控制结构体
double *bias	O	
double *xa		O 	固定解
```

#### 2.执行流程

- 整周模糊度ratio值`rtk->sol.ratio`赋初值0.0
- 检查配置中所设置的ratio值`rtk->opt.thresar[0]`，如果该阈值小于1，则return 0 
- 调用`ddidx()`函数，创建将卡尔曼状态量从单差转到双差的转换矩阵`D`，主要是将单差相位偏移状态量转换为双差相位偏移 
- 根据转移矩阵，求解双差整周模糊度以及协方差阵
- 调用`lambda()`函数计算双差整周模糊度最优解以及残差。估计结果存在`b`，`s`中 
- 计算Ratio值，如果Ratio值大于阈值，求解固定解rtk->xa以及固定解的协方差rtk->Pa 
- 调用`restamb()`，重新存储单差的模糊度，便于固定解有效性验证

```c
static int resamb_LAMBDA(rtk_t *rtk, double *bias, double *xa)
{
    prcopt_t *opt=&rtk->opt;
    int i,j,nb,info,nx=rtk->nx,na=rtk->na;
    double *DP,*y,*b,*db,*Qb,*Qab,*QQ,s[2];
    int *ix;

    trace(3,"resamb_LAMBDA : nx=%d\n",nx);
    
    //整周模糊度ratio赋初值0.0
    rtk->sol.ratio=0.0;
    
    //检查配置中所设置的ratio值，如果该阈值小于1，则return 0
    if (rtk->opt.mode<=PMODE_DGPS||rtk->opt.modear==ARMODE_OFF||
        rtk->opt.thresar[0]<1.0) {
        return 0;
    }

    //调用ddidx()函数，创建将卡尔曼状态量从单差转到双差的转换矩阵D
    //主要是将单差相位偏移状态量转换为双差相位偏移
    /* index of SD to DD transformation matrix D */
    ix=imat(nx,2);
    if ((nb=ddidx(rtk,ix))<=0) {
        errmsg(rtk,"no valid double-difference\n");
        free(ix);
        return 0;
    }

    //na实际就是之前卡尔曼滤波中除了单差相位偏移之外的所有状态量个数（例如：位置+速度+加速度+电离层+对流层……）
    //nb则是双差相位偏移的个数（即需要解算的整周模糊度个数）
    y=mat(nb,1); DP=mat(nb,nx-na); b=mat(nb,2); db=mat(nb,1); Qb=mat(nb,nb);
    Qab=mat(na,nb); QQ=mat(na,nb);
    
    //根据转移矩阵，求解双差整周模糊度以及协方差阵
    /* y=D*xc, Qb=D*Qc*D', Qab=Qac*D' */    //(E.7.15) (E.7.16) 
    for (i=0;i<nb;i++) {
        y[i]=rtk->x[ix[i*2]]-rtk->x[ix[i*2+1]]; 
    }
    //
    for (j=0;j<nx-na;j++) for (i=0;i<nb;i++) {
        DP[i+j*nb]=rtk->P[ix[i*2]+(na+j)*nx]-rtk->P[ix[i*2+1]+(na+j)*nx];
    }
    //整周的协方差阵
    for (j=0;j<nb;j++) for (i=0;i<nb;i++) {
        Qb[i+j*nb]=DP[i+(ix[j*2]-na)*nb]-DP[i+(ix[j*2+1]-na)*nb];
    }
    //整周和状态
    for (j=0;j<nb;j++) for (i=0;i<na;i++) {
        Qab[i+j*na]=rtk->P[i+ix[j*2]*nx]-rtk->P[i+ix[j*2+1]*nx];
    }

    //调用lambda()函数计算双差整周模糊度最优解以及残差。估计结果存在b，s中
    /* LAMBDA/MLAMBDA ILS (integer least-square) estimation */
    if (!(info=lambda(nb,2,y,Qb,b,s))) {
        trace(4,"N(1)="); tracemat(4,b   ,1,nb,10,3);
        trace(4,"N(2)="); tracemat(4,b+nb,1,nb,10,3);
        
        //计算Ratio值
        rtk->sol.ratio=s[0]>0?(float)(s[1]/s[0]):0.0f;
        if (rtk->sol.ratio>999.9) rtk->sol.ratio=999.9f;
        
        //如果Ratio值大于阈值，求解固定解rtk->xa以及固定解的协方差rtk->Pa
        /* validation by popular ratio-test */
        if (s[0]<=0.0||s[1]/s[0]>=opt->thresar[0]) {    
            
            /* transform float to fixed solution (xa=xa-Qab*Qb\(b0-b)) */
            for (i=0;i<na;i++) {
                rtk->xa[i]=rtk->x[i];
                for (j=0;j<na;j++) rtk->Pa[i+j*na]=rtk->P[i+j*nx];
            }
            for (i=0;i<nb;i++) {
                bias[i]=b[i];
                y[i]-=b[i];
            }
            if (!matinv(Qb,nb)) {
                matmul("NN",nb,1,nb, 1.0,Qb ,y,0.0,db);
                matmul("NN",na,1,nb,-1.0,Qab,db,1.0,rtk->xa);
                
                /* covariance of fixed solution (Qa=Qa-Qab*Qb^-1*Qab') */
                matmul("NN",na,nb,nb, 1.0,Qab,Qb ,0.0,QQ);
                matmul("NT",na,na,nb,-1.0,QQ ,Qab,1.0,rtk->Pa);
                
                trace(3,"resamb : validation ok (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                      nb,s[0]==0.0?0.0:s[1]/s[0],s[0],s[1]);
                
                //调用restamb()，重新存储单差的模糊度
                /* restore SD ambiguity */
                restamb(rtk,bias,nb,xa);
            }
            else nb=0;
        }
        else { /* validation failed */
            errmsg(rtk,"ambiguity validation failed (nb=%d ratio=%.2f s=%.2f/%.2f)\n",
                   nb,s[1]/s[0],s[0],s[1]);
            nb=0;
        }
    }
    else {
        errmsg(rtk,"lambda error (info=%d)\n",info);
        nb=0;
    }
    free(ix);
    free(y); free(DP); free(b); free(db); free(Qb); free(Qab); free(QQ);
    
    return nb; /* number of ambiguities */
}
```

#### 3.lambda()：lambda整数最小二乘估计

**传入参数**：

```c
int    n      I  number of float parameters      //浮点解数量
int    m      I  number of fixed solutions       //固定解数量
double *a     I  float parameters (n x 1)        //浮点参数向量
double *Q     I  covariance matrix of float parameters (n x n)   //浮点参数协方差阵
double *F     O  fixed solutions (n x m)     	//固定解
double *s     O  sum of squared residulas of fixed solutions (1 x m) //总固定残差向量
```

**执行流程**：

- 调用`LD()`，首先对浮点协方差阵进行LD分解
- 调用`reduction()`，lambda降相关性
- z变换，将双差模糊度进行变换
- 调用`search()`,mlambda search，结果存储在`E`和`s`中（整数解） 
- 调用`solve()`，逆Z变换，将在新空间中固定的模糊度逆变换回双差模糊度空间中，存储在`F`中 

```c
extern int lambda(int n, int m, const double *a, const double *Q, double *F,
                  double *s)
{
    int info;
    double *L,*D,*Z,*z,*E;
    
    if (n<=0||m<=0) return -1;
    L=zeros(n,n); D=mat(n,1); Z=eye(n); z=mat(n,1); E=mat(n,m);
    
    //调用LD()，首先对浮点协方差阵进行LD分解
    /* LD factorization */
    if (!(info=LD(n,Q,L,D))) {

        //调用reduction()，lambda降相关性
        /* lambda reduction */
        reduction(n,L,D,Z);

        // z变换，将双差模糊度进行变换
        matmul("TN",n,1,n,1.0,Z,a,0.0,z); /* z=Z'*a */
        
        //调用search(),mlambda search，结果存储在E和s中（整数解）
        /* mlambda search */
        if (!(info=search(n,m,L,D,z,E,s))) {
            //逆Z变换，将在新空间中固定的模糊度逆变换回双差模糊度空间中，存储在F中
            info=solve("T",Z,E,n,m,F); /* F=Z'\E */
        }
    }
    free(L); free(D); free(Z); free(z); free(E);
    return info;
}
```

#### 4.reduction()：LAMBDA降相关

```c
static void reduction(int n, double *L, double *D, double *Z)
{
    int i,j,k;
    double del;
    
    j=n-2; k=n-2;   //调序变换
    
    //对第0,1，...，k-1，k列进行降相关
    while (j>=0) {
        if (j<=k) for (i=j+1;i<n;i++) gauss(n,L,Z,i,j); //从最后一列开始，各列非对角线元素从上往下依次降相关
        del=D[j]+L[j+1+j*n]*L[j+1+j*n]*D[j+1];
        //检验条件，若不满足检验条件则开始进行调序变换
        if (del+1E-6<D[j+1]) { /* compared considering numerical error */
            perm(n,L,D,j,del,Z);
            k=j; j=n-2; //完成调序变换后重新从最后一列开始进行降相关及排序，k记录最后一次进行过调序变换的列序号
        }
        else j--;
    }
}
```

#### 5.gauss()：整数高斯变换

```c
static void gauss(int n, double *L, double *Z, int i, int j)
{
    int k,mu;
    
    if ((mu=(int)ROUND(L[i+j*n]))!=0) {
        for (k=i;k<n;k++) L[k+n*j]-=(double)mu*L[k+i*n];
        for (k=0;k<n;k++) Z[k+n*j]-=(double)mu*Z[k+i*n];
    }
}
```

#### 6.perm()：条件方差重新排列

```c
static void perm(int n, double *L, double *D, int j, double del, double *Z)
{
    int k;
    double eta,lam,a0,a1;
    
    eta=D[j]/del;
    lam=D[j+1]*L[j+1+j*n]/del;
    D[j]=eta*D[j+1]; D[j+1]=del;
    for (k=0;k<=j-1;k++) {
        a0=L[j+k*n]; a1=L[j+1+k*n];
        L[j+k*n]=-L[j+1+j*n]*a0+a1;
        L[j+1+k*n]=eta*a0+lam*a1;
    }
    L[j+1+j*n]=lam;
    for (k=j+2;k<n;k++) SWAP(L[k+j*n],L[k+(j+1)*n]);
    for (k=0;k<n;k++) SWAP(Z[k+j*n],Z[k+(j+1)*n]);
}
```

#### 7.search()：mlambda搜索

```c
static int search(int n, int m, const double *L, const double *D,
                  const double *zs, double *zn, double *s)
{
    int i,j,k,c,nn=0,imax=0;
    double newdist,maxdist=1E99,y;
    double *S=zeros(n,n),*dist=mat(n,1),*zb=mat(n,1),*z=mat(n,1),*step=mat(n,1);
    
    k=n-1; dist[k]=0.0; //k表示当前层，从最后一层（n-1）开始计算
    zb[k]=zs[k];//即zn
    z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);    //四舍五入取整；取整后的数与未取整的数作差；step记录z[k]是四舍还是五入
    for (c=0;c<LOOPMAX;c++) {
        newdist=dist[k]+y*y/D[k];
        if (newdist<maxdist) {      //如果当前累积目标函数计算值小于当前超椭圆半径
            //情况1：若还未计算至第一层，继续计算累积目标函数值
            if (k!=0) { 
                dist[--k]=newdist;  //记录下当前层的累积目标函数值，dist[k]表示了第k,k+1,...,n-1层的目标函数计算和
                for (i=0;i<=k;i++)
                    S[k+i*n]=S[k+1+i*n]+(z[k+1]-zb[k+1])*L[k+1+i*n];
                zb[k]=zs[k]+S[k+k*n];   //计算Zk，即第k个整数模糊度参数的备选组的中心
                z[k]=ROUND(zb[k]); y=zb[k]-z[k]; step[k]=SGN(y);    //四舍五入取整；取整后的数与未取整的数作差；记录是四舍还是五入
            }
            //情况2：若已经计算至第一层，意味着所有层的累积目标函数值计算完毕
            else {
                //nn为当前候选解数，m为我们需要的固定解数，这里为2，表示需要一个最优解及一个次优解
                //s记录候选解的目标函数值，imax记录之前候选解中的最大目标函数值的坐标
                if (nn<m) { //若候选解数还没满
                    if (nn==0||newdist>s[imax]) imax=nn;    //若当前解的目标函数值比之前最大的目标函数值都大，那么更新imax使s[imax]指向当前解中具有的最大目标函数值
                    for (i=0;i<n;i++) zn[i+nn*n]=z[i];  //zn存放所有候选解
                    s[nn++]=newdist;    //s记录当前目标函数值newdist，并加加当前候选解数nn
                }
                else {  //若候选解数已满（即当前zn中已经存了2个候选解）
                    if (newdist<s[imax]) {  //若当前解的目标函数值比s中的最大目标函数值 
                        for (i=0;i<n;i++) zn[i+imax*n]=z[i];    //用当前解替换zn中具有较大目标函数值的解
                        s[imax]=newdist;    //用当前解的目标函数值替换s中的最大目标函数值
                        for (i=imax=0;i<m;i++) if (s[imax]<s[i]) imax=i;    //更新imax保证imax始终指向s中的最大目标函数值
                    }
                    maxdist=s[imax];    //用当前最大的目标函数值更新超椭圆半径
                }
                //在第一层，取下一个有效的整数模糊度参数进行计算（若zb为5.3，则z取值顺序为5,6,4,7，...）
                z[0]+=step[0]; y=zb[0]-z[0]; step[0]=-step[0]-SGN(step[0]);
            }
        }
        //情况3：如果当前累积目标函数计算值大于当前超椭圆半径
        else {
            if (k==n-1) break;  //如果当前层为第n-1层，意味着后续目标函数各项的计算都会超出超椭圆半径，因此终止搜索
            else {  //若当前层不是第n-1层
                k++;    //退后一层，即从第k层退到第k+1层
                z[k]+=step[k]; y=zb[k]-z[k]; step[k]=-step[k]-SGN(step[k]); //计算退后一层后，当前层的下一个有效备选解
            }
        }
    }
    // 对s中的目标函数值及zn中的候选解进行排序（以s中目标函数值为排序标准，进行升序排序）
    // RTKLIB中最终可以得到一个最优解一个次优解，存在zn中，两解对应的目标函数值，存在s中
    for (i=0;i<m-1;i++) { /* sort by s */
        for (j=i+1;j<m;j++) {
            if (s[i]<s[j]) continue;
            SWAP(s[i],s[j]);
            for (k=0;k<n;k++) SWAP(zn[k+i*n],zn[k+j*n]);
        }
    }
    free(S); free(dist); free(zb); free(z); free(step);
    
    if (c>=LOOPMAX) {
        fprintf(stderr,"%s : search loop count overflow\n",__FILE__);
        return -1;
    }
    return 0;
}
```

### 4、holdamb()：Fix and hold模式下模糊度保持

**执行流程**：

- 传入`resamb_LAMBDA()`计算的当前时刻固定解`xa`
- 循环遍历各个卫星，查找满足条件的卫星，并设置相应标志位`rtk->ssat[i].fix=3`
- 计算固定解双差和浮点解双差 的差值，形成量测信息，并更新`H`阵 
- 若观测量数量有效，构建`R`矩阵阵，调用`filter()`量测更新 

```c
static void holdamb(rtk_t *rtk, const double *xa)
{
    double *v,*H,*R;
    int i,n,m,f,info,index[MAXSAT],nb=rtk->nx-rtk->na,nv=0,nf=NF(&rtk->opt);
    
    trace(3,"holdamb :\n");
    
    v=mat(nb,1); H=zeros(nb,rtk->nx);
    
    //循环遍历各个卫星，查找满足条件的卫星，并设置相应标志位rtk->ssat[i].fix=3
    for (m=0;m<5;m++) for (f=0;f<nf;f++) {
        
        for (n=i=0;i<MAXSAT;i++) {
            if (!test_sys(rtk->ssat[i].sys,m)||rtk->ssat[i].fix[f]!=2||
                rtk->ssat[i].azel[1]<rtk->opt.elmaskhold) {
                continue;
            }
            index[n++]=IB(i+1,f,&rtk->opt);
            rtk->ssat[i].fix[f]=3; /* hold */
        }
        //计算固定解双差和浮点解双差 的差值，形成量测信息，并更新H阵
        /* constraint to fixed ambiguity */
        for (i=1;i<n;i++) {
            //xa为固定解、rtk->x为浮点解
            v[nv]=(xa[index[0]]-xa[index[i]])-(rtk->x为浮点解[index[0]]-rtk->x[index[i]]);
            
            H[index[0]+nv*rtk->nx]= 1.0;
            H[index[i]+nv*rtk->nx]=-1.0;
            nv++;
        }
    }
    //若观测量数量有效，设置R阵，并调用filter()量测更新
    if (nv>0) {
        R=zeros(nv,nv);
        for (i=0;i<nv;i++) R[i+i*nv]=VAR_HOLDAMB;
        
        /* update states with constraints */
        if ((info=filter(rtk->x,rtk->P,H,v,R,rtk->nx,nv))) {
            errmsg(rtk,"filter error (info=%d)\n",info);
        }
        free(R);
    }
    free(v); free(H);
}
```



## 十三、valpos()：解的有效性验证

计算流动站非差、双差残差，残差平方和`v[i]*v[i] `是否小于`fact*R[i+i*nv]`

```c
static int valpos(rtk_t *rtk, const double *v, const double *R, const int *vflg,
                  int nv, double thres)
{
    double fact=thres*thres;
    int i,stat=1,sat1,sat2,type,freq;
    char *stype;
    
    trace(3,"valpos  : nv=%d thres=%.1f\n",nv,thres);
    
    /* post-fit residual test */    
    for (i=0;i<nv;i++) {
        if (v[i]*v[i]<=fact*R[i+i*nv]) continue;    
        sat1=(vflg[i]>>16)&0xFF;    //参考卫星号
        sat2=(vflg[i]>> 8)&0xFF;    //非参考卫星号
        type=(vflg[i]>> 4)&0xF;     //种类：0载波 1伪距 3动基线
        freq=vflg[i]&0xF;           //第几个频率(从0开始)
        stype=type==0?"L":(type==1?"L":"C");
        errmsg(rtk,"large residual (sat=%2d-%2d %s%d v=%6.3f sig=%.3f)\n",
              sat1,sat2,stype,freq+1,v[i],SQRT(R[i+i*nv]));
    }
    return stat;
}
```