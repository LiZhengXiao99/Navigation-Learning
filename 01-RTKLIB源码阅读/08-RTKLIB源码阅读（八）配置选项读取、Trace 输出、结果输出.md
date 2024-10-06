[TOC]

## 一、配置文件读取

![image-20231024152747192](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024152747192.png)

### 1、Option文件格式介绍

* 配置文件包含了文件选项（filopt_t ）、结果输出选项（solopt_t ）、解算方式选项（prcopt_t ）三大块，用于实时和后处理定位解算程序 RTKNAVI、RTKPOST、RTKRCV、RNX2RTKP。

* 文件中都以键值对 **Keyword = Value** 形式记录不同的配置项。

* 对于枚举选项，可选值是选项序号 (0,1,2,...) 或选项字符串 (off, on, ...)。 

* 以 # 开头的行和行中#之后的文本被视为注释。

### 2、存Option的类型

#### 1. prcopt_t 结构体：存算法处理选项

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

  

#### 2. solopt_t 结构体：存输出结果设置

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

  

#### 3. filopt_t 结构体：存文件设置

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

#### 4.opt_t 结构体：存一条有选项信息的结构体

  **opt_t 数组 sysopts**：存所有的选项 

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

## 三、Trace 输出

> 在 rtklib.h 中加入 #define TRACE，启用 trace ，不定义则将 trace 函数全赋空值：

![image-20231024152507571](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024152507571.png)

### 1、rtkcmn.c关于trace的静态全局变量

  ```c
static FILE *fp_trace=NULL;     //trace的文件指针
static char file_trace[1024];   //trace文件名
static int level_trace=0;       //trace等级（1-5)，等级越高输出的信息越多
static uint32_t tick_trace=0;   //以毫米计的系统时间，在tracet()中用到：fprintf(fp_trace,"%d %9.3f: ",level,(tickget()-tick_trace)/1000.0);
static gtime_t time_trace={0};  //打开trace的时间，获取的系统时间，并转为GPST
static lock_t lock_trace;       //trace的进程锁
  ```

### 2、Trace 相关函数

#### 1.trace()：将传入的 trace 格式化字符串写入 trace 文件

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

#### 2. tracet()：写入带秒数的 trace 格式字符串

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

#### 3. traceclose()：关闭trace文件描述符，将文件指针置空

  ```c
extern void traceclose(void)
{
    if (fp_trace&&fp_trace!=stderr) fclose(fp_trace);   //关闭trace文件描述符
    fp_trace=NULL;      //将文件指针置空
    file_trace[0]='\0';
}
  ```

#### 4. traceopen()：创建或打开trace文件

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

#### 5. tracelevel()：将传入的trace等级赋值给level_trace

  ```c
extern void tracelevel(int level)
{
    level_trace=level;
}
  ```

#### 6. traceswap()：根据时间分trace文件

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

  

#### 7. tracemat()：写入矩阵

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

#### 8. traceobs()：写入obsd_t

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

#### 9. tracenav()：写入导航电文

写入`nav->eph`、`nav->ion_gps`/`ion_gal`/`ion_bds`电离层信息、星历数据的的信息。

*   **tracegnav()**:写入nav->geph星历信息。

*   **tracehnav()**:写入nav->seph信息。

 *   **tracepeph()** 写入nav->peph 精密星历信息

*   **tracepclk()**:写入nav->pclk 精密钟差信息

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

#### 10. traceb()：写入buff缓冲区数据

在 skytraq.c 和 ublox.c 中被调用。

  ```c
extern void traceb(int level, const uint8_t *p, int n)
{
    int i;
    if (!fp_trace||level>level_trace) return;
    for (i=0;i<n;i++) fprintf(fp_trace,"%02X%s",*p++,i%8==7?" ":"");
    fprintf(fp_trace,"\n");
}
  ```



## 四、结果输出

![image-20231024152344219](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024152344219.png)

### 1、结果相关

#### 1. solopt_t 结构体：存结果输出选项

![image-20231025204536781](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025204536781.png)

#### 2. sol_t、solbuf_t：存结果



```c
typedef struct {        /* solution type */
    gtime_t time;       /* time (GPST) */
    double rr[6];       /* position/velocity (m|m/s) */
                        /* {x,y,z,vx,vy,vz} or {e,n,u,ve,vn,vu} */
    float  qr[6];       /* position variance/covariance (m^2) */
                        /* {c_xx,c_yy,c_zz,c_xy,c_yz,c_zx} or */
                        /* {c_ee,c_nn,c_uu,c_en,c_nu,c_ue} */
    float  qv[6];       /* velocity variance/covariance (m^2/s^2) */
    double dtr[6];      /* receiver clock bias to time systems (s) */
    uint8_t type;       /* type (0:xyz-ecef,1:enu-baseline) */
    uint8_t stat;       /* solution status (SOLQ_???) */
    uint8_t ns;         /* number of valid satellites */
    float age;          /* age of differential (s) */
    float ratio;        /* AR ratio factor for valiation */
    float thres;        /* AR ratio threshold for valiation */
} sol_t;
```

* `time`：结果时间。
* `rr`：结果位置速度，可以存 ECEF 下的 XYZ，也可以存 ENU。
* `qr`：结果位置的协方差，协方差阵右上和左下是对称的，所以存 6 个元素就行。
* `qv`：结果速度协方差。
* `dtr`：接收机钟差。
* `type`：标识结果是 ECEF 还是 ENU。
* `stat`：结果种类（单点解、浮点解、固定解）。
* `ns`：有效卫星数。
* `age`：差分龄期，相对定位基准站流动站时间差。
* `ratio`：模糊度固定阈值。
* `thres`：模糊度固定阈值。



#### 3. solstat_t、solstatbuf_t 存结算的状态



```c
typedef struct {        /* solution status type */
    gtime_t time;       /* time (GPST) */
    uint8_t sat;        /* satellite number */
    uint8_t frq;        /* frequency (1:L1,2:L2,...) */
    float az,el;        /* azimuth/elevation angle (rad) */
    float resp;         /* pseudorange residual (m) */
    float resc;         /* carrier-phase residual (m) */
    uint8_t flag;       /* flags: (vsat<<5)+(slip<<3)+fix */
    uint16_t snr;       /* signal strength (*SNR_UNIT dBHz) */
    uint16_t lock;      /* lock counter */
    uint16_t outc;      /* outage counter */
    uint16_t slipc;     /* slip counter */
    uint16_t rejc;      /* reject counter */
} solstat_t;
```

* time：结果时间
* sat：连续卫星编号 satellite number
* frq：频率号
* az,el：高度角、方位角
* resp：伪距残差
* resc：载波相位残差
* flag：
* snr：
* lock：
* outc：
* slipc：
* rejc：



### 2、结果文件头输出

#### 1. outhead

* 如果指定了文件输出路径 outfile，递归创建结构文件，没指定则输出到终端，然后以写的方式打开结果文件。
* 调用 `outheader` 写文件头内容，写完之后关闭文件。

```c
static int outhead(const char *outfile, char **infile, int n,
                   const prcopt_t *popt, const solopt_t *sopt)
{
    FILE *fp=stdout;    // fp 默认初始为stdout
    
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

#### 4. outheader

* 如果是 NMEA、STAT 格式的结果，不需要输出文件头，直接返回
* 

* 调用 `outprcopt()` 输出处理选项

* 相对定位模式调用 `outrpos()` 输出基准站坐标

* 调用 `outsolhead()` ，其通过调用 `outsolheads()` 输出结果字段头，如：

  ```c
  p+=sprintf(p,"%14s%s%14s%s%14s%s%3s%s%3s%s%8s%s%8s%s%8s%s%8s%s%8s%s%8s"
                "%s%6s%s%6s",
                "x-ecef(m)",sep,"y-ecef(m)",sep,"z-ecef(m)",sep,"Q",sep,"ns",
                sep,"sdx(m)",sep,"sdy(m)",sep,"sdz(m)",sep,"sdxy(m)",sep,
                "sdyz(m)",sep,"sdzx(m)",sep,"age(s)",sep,"ratio");
  ```

### 3、结果文件体输出

#### 1. outsol



![image-20231018181248650](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231018181248650.png)

```c
extern void outsol(FILE *fp, const sol_t *sol, const double *rb,
                   const solopt_t *opt)
{
    uint8_t buff[MAXSOLMSG+1];
    int n;
    
    trace(3,"outsol  :\n");
    
    if ((n=outsols(buff,sol,rb,opt))>0) {
        fwrite(buff,n,1,fp);
    }
}
```

`outsol()` 调用 `outsols()`，然后通过调用 `outpos()`、`outecef()`、`outenu()`、`outnmea_rmc()`、`outnmea_gga()` 来输出对应形式的结果
