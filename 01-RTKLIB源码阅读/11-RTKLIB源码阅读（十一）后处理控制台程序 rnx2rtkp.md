

[TOC]





## 一、从 main 函数到定位解算

### main()

main 函数主要就是处理传入的 main 函数参数。先找配置文件，找到之后，调用 resetsysopts 重置所有配置、调用 loadopts 读取配置文件内容到缓冲区，调用 getsysopts 将 缓冲区 中存的配置加载到配置结构体 porcopt_t/solopt_t/filopt_t 中。再是一个 for 循环，处理其它的参数，最后调用 postpos 进行下一步处理。

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

### postpos()

后处理定位主入口函数，如果主函数自己写，那就是先设置好起止时间、文件路径、选项，再调用此函数。函数先调用 openses 





### execses_b()





### execses_r()





### execses()



### procpos()



### rtkpos()









## 二、配置选项读取

### resetsysopts()

函数很简单，就是把选项都赋空值或 0、截止高度角设为 15

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

### loadopts()

打开选项文件，while 循环读取文件内容，判断有内容后调用 searchopt 在 opt_t 数组中根据配置选项名找对应选项，调用 str2opt 字符串转为对应的 opt 值，存到缓冲区中

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

### getsysopts()

调用 buff2sysopts 将 把选项缓冲区`antpostype_ `、`elmask_`、`elmaskar_`、`elmaskhold_` 、`antpos_ `、`exsats_ `、`snrmask_ `  中存的配置加载到配置结构体 porcopt_t/solopt_t/filopt_t 中，然后判断，如果没有配置则使用默认配置。

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





















## 三、卫星位置钟差计算

> 我之前的博客有详细解析：[RTKLIB学习总结（六）导航电文、卫星位置计算](https://lizhengxiao.blog.csdn.net/article/details/129986385)

### satposs()

遍历观测数据，找伪距观测值，除以光速得到信号传播时间，用数据接收时刻减去伪距信号传播时间得到信号发射时刻。

调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，不作为最终计算的钟差。信号发射时刻减去钟差 `dt`，得到 GPS 时间下的卫星信号发射时刻。

**调用 satpos() 对此观测值进行下一步卫星位置钟差的计算**；`satpos()` 函数对星历计算选项进行判断，**广播星历模式调用 ephpos()**，**精密星历模式调用 peph2pos()**。最后检测钟差值，如果没有精密星历，则调用 `ephclk()` 用广播星历计算钟差。

```c
/* satellite positions and clocks ----------------------------------------------
* compute satellite positions, velocities and clocks
* args   : gtime_t teph     I   time to select ephemeris        (gpst) 用于选择星历的时刻 (gpst)
*          obsd_t *obs      I   observation data                OBS观测数据
*          int    n         I   number of observation data      OBS数
*          nav_t  *nav      I   navigation data                 NAV导航电文
*          int    ephopt    I   ephemeris option (EPHOPT_???)   星历选项 (EPHOPT_???)
*          double *rs       O   satellite positions and velocities (ecef)   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
*          double *dts      O   satellite clocks       卫星钟差，长度为2*n， {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variances (m^2)    卫星位置和钟差的协方差 (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)   卫星健康标志 (-1:correction not available)
* return : none
* notes  : rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)
*          rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)
*          dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)
*          var[i]        = obs[i] sat position and clock error variance (m^2)
*          svh[i]        = obs[i] sat health flag
*          if no navigation data, set 0 to rs[], dts[], var[] and svh[]
*          satellite position and clock are values at signal transmission time
*          satellite position is referenced to antenna phase center
*          satellite clock does not include code bias correction (tgd or bgd)
*          any pseudorange and broadcast ephemeris are always needed to get
*          signal transmission time
*-----------------------------------------------------------------------------*/
extern void satposs(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                    int ephopt, double *rs, double *dts, double *var, int *svh)
{
    gtime_t time[2*MAXOBS]={{0}};
    double dt,pr;
    int i,j;
    
    trace(3,"satposs : teph=%s n=%d ephopt=%d\n",time_str(teph,3),n,ephopt);
    //遍历每一条观测数据
    for (i=0;i<n&&i<2*MAXOBS;i++) {     
        for (j=0;j<6;j++) rs [j+i*6]=0.0;   //首先初始化，将对当前观测数据的 rs、dts、var和svh数组的元素置 0
        for (j=0;j<2;j++) dts[j+i*2]=0.0;
        var[i]=0.0; svh[i]=0;
        //通过判断某一频率下信号的伪距是否为 0，来得到此时所用的频率个数。
        /* search any pseudorange */
        for (j=0,pr=0.0;j<NFREQ;j++) if ((pr=obs[i].P[j])!=0.0) break;
        
        if (j>=NFREQ) {
            trace(3,"no pseudorange %s sat=%2d\n",time_str(obs[i].time,3),obs[i].sat);
            continue;
        }
        //用数据接收时刻减去伪距信号传播时间，得到卫星信号的发射时刻
        /* transmission time by satellite clock */
        time[i]=timeadd(obs[i].time,-pr/CLIGHT);    
        
        //调用 ephclk 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 dt。
        //注意，此时的钟差是没有考虑相对论效应和 TGD 的            
        /* satellite clock bias by broadcast ephemeris */
        if (!ephclk(time[i],teph,obs[i].sat,nav,&dt)) {
            trace(3,"no broadcast clock %s sat=%2d\n",time_str(time[i],3),obs[i].sat);
            continue;
        }
        time[i]=timeadd(time[i],-dt);   //信号发射时刻减去钟差 dt，得到 GPS 时间下的卫星信号发射时刻
        
        //调用 satpos 函数，计算信号发射时刻卫星的位置(ecef,m)、速度(ecef,m/s)、钟差((s|s/s))。
        //注意，这里计算出的钟差是考虑了相对论效应的了，只是还没有考虑 TGD
        /* satellite position and clock at transmission time */
        if (!satpos(time[i],teph,obs[i].sat,ephopt,nav,rs+i*6,dts+i*2,var+i,
                    svh+i)) {
            trace(3,"no ephemeris %s sat=%2d\n",time_str(time[i],3),obs[i].sat);
            continue;
        }
        //如果没有精密钟差，则用广播星历的钟差替代
        /* if no precise clock available, use broadcast clock instead */
        if (dts[i*2]==0.0) {    //如果没有钟差值
            if (!ephclk(time[i],teph,obs[i].sat,nav,dts+i*2)) continue;
            dts[1+i*2]=0.0;     //钟漂设为0
            *var=SQR(STD_BRDCCLK);  //钟差的方差设为30的平方
        }
    }
    for (i=0;i<n&&i<2*MAXOBS;i++) {
        trace(4,"%s sat=%2d rs=%13.3f %13.3f %13.3f dts=%12.3f var=%7.3f svh=%02X\n",
              time_str(time[i],6),obs[i].sat,rs[i*6],rs[1+i*6],rs[2+i*6],
              dts[i*2]*1E9,var[i],svh[i]);
    }
}
```

### satpos()

根据星历选项，计算单颗卫星的 P(ecef,m)、V(ecef,m/s)、C((s|s/s)) 

```c
/* satellite position and clock ------------------------------------------------
* compute satellite position, velocity and clock
* args   : gtime_t time     I   time (gpst)                     time (gpst)
*          gtime_t teph     I   time to select ephemeris (gpst) 用于选择星历的时刻 (gpst)
*          int    sat       I   satellite number                卫星号
*          nav_t  *nav      I   navigation data                 NAV导航数据
*          int    ephopt    I   ephemeris option (EPHOPT_???)   星历选项 (EPHOPT_???)
*          double *rs       O   sat position and velocity (ecef)    卫星位置和速度，长度为6*n
*                               {x,y,z,vx,vy,vz} (m|m/s)        
*          double *dts      O   sat clock {bias,drift} (s|s/s)      卫星钟差，长度为2*n， {bias,drift} (s|s/s)
*          double *var      O   sat position and clock error variance (m^2)  卫星位置和钟差的协方差 (m^2)
*          int    *svh      O   sat health flag (-1:correction not available)   卫星健康标志 (-1:correction not available)
* return : status (1:ok,0:error)
* notes  : satellite position is referenced to antenna phase center     卫星位置相对于天线相位中心来说
*          satellite clock does not include code bias correction (tgd or bgd)
*-----------------------------------------------------------------------------*/
extern int satpos(gtime_t time, gtime_t teph, int sat, int ephopt,
                  const nav_t *nav, double *rs, double *dts, double *var,
                  int *svh)
{
    trace(4,"satpos  : time=%s sat=%2d ephopt=%d\n",time_str(time,3),sat,ephopt);
    
    *svh=0;
    
    switch (ephopt) {   //根据星历选项调用对应的解算函数
        case EPHOPT_BRDC  : return ephpos     (time,teph,sat,nav,-1,rs,dts,var,svh);    //广播星历
        case EPHOPT_SBAS  : return satpos_sbas(time,teph,sat,nav,   rs,dts,var,svh);    //sbas
        case EPHOPT_SSRAPC: return satpos_ssr (time,teph,sat,nav, 0,rs,dts,var,svh);    //参考天线相位中心
        case EPHOPT_SSRCOM: return satpos_ssr (time,teph,sat,nav, 1,rs,dts,var,svh);    //参考质心，还需要天线相位中心改正
        case EPHOPT_PREC  :                                                             //精密星历
            if (!peph2pos(time,sat,nav,1,rs,dts,var)) break; else return 1;
    }
    *svh=-1;
    return 0;
}
```

### ephclk()

单观测值卫星钟差计算。由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2clk()` 根据广播星历参数 、、 计算卫星钟差（迭代 3 次）；

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2clk()` 根据广播星历参数 、  计算卫星钟差（迭代 3 次）。

```c
/* satellite clock with broadcast ephemeris ----------------------------------
 * arge:gtime_t  time      I   信号发射时刻
 *      gtime_t  teph      I   用于选择星历的时刻 (gpst)
 *      int      sat       I   卫星号 (1-MAXSAT)
 *      nav_t    *nav      I   导航数据
 *      double   *dts      O   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
 * return:(1:ok,0:error)
----------------------------------------------------------------------------*/
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  double *dts)
{
    eph_t  *eph;
    geph_t *geph;
    seph_t *seph;
    int sys;
    
    trace(4,"ephclk  : time=%s sat=%2d\n",time_str(time,3),sat);
    //调用 satsys 函数，根据卫星编号确定该卫星所属的导航系统和该卫星在该系统中的 PRN编号
    sys=satsys(sat,NULL);
    
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP||sys==SYS_IRN) {
        if (!(eph=seleph(teph,sat,-1,nav))) return 0;   //调用 seleph 函数来选择最接近 teph 的那个星
        *dts=eph2clk(time,eph); //调用 eph2clk 函数，通过广播星历和信号发射时间计算出卫星钟差
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,-1,nav))) return 0;
        *dts=geph2clk(time,geph);
    }
    else if (sys==SYS_SBS) {
        if (!(seph=selseph(teph,sat,nav))) return 0;
        *dts=seph2clk(time,seph);
    }
    else return 0;
    
    return 1;
}
```

### ephpos()

与 `ephclk()` 同理，由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2pos()` 根据广播星历中的开普勒轨道参数和摄动改正计算卫星位置（对北斗 MEO、IGSO 卫星会进行特殊处理）、校正卫星钟差的相对论效应、调用 `var_uraeph()` 用 URA 值来标定方差。

如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2pos()` 根据广播星历中 PZ-90 坐标系下卫星状态向量四阶龙格库塔迭代计算卫星位置。

计算完一次位置之后，加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟漂。

```C
/* satellite position and clock by broadcast ephemeris -----------------------
 * arge:gtime_t  time      I   信号发射时刻
 *      gtime_t  teph      I   用于选择星历的时刻 (gpst)
 *      int      sat       I   卫星号 (1-MAXSAT)
 *      nav_t    *nav      I   导航数据
 *      int      iode      I   星历数据期号
 *      double   *rs       O   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
 *      double   *dts      O   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
 *      double   *var      O   卫星位置和钟差的协方差 (m^2)
 *      int      *svh      O   卫星健康标志 (-1:correction not available)
 * return:(1:ok,0:error)
----------------------------------------------------------------------------*/
static int ephpos(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  int iode, double *rs, double *dts, double *var, int *svh)
{
    eph_t  *eph;
    geph_t *geph;
    seph_t *seph;
    double rst[3],dtst[1],tt=1E-3;
    int i,sys;
    
    trace(4,"ephpos  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);
    
    sys=satsys(sat,NULL);   //调用 satsys 函数，确定该卫星所属的导航系统。
    
    *svh=-1;
    
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP||sys==SYS_IRN) {
        if (!(eph=seleph(teph,sat,iode,nav))) return 0; //调用 seleph 函数来选择广播星历。
        eph2pos(time,eph,rs,dts,var);   //根据选中的广播星历，调用 eph2pos 函数来计算信号发射时刻卫星的 位置、钟差和相应结果的误差。
        time=timeadd(time,tt);
        eph2pos(time,eph,rst,dtst,var);
        *svh=eph->svh;
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,iode,nav))) return 0;
        geph2pos(time,geph,rs,dts,var);
        time=timeadd(time,tt);
        geph2pos(time,geph,rst,dtst,var);
        *svh=geph->svh;
    }
    else if (sys==SYS_SBS) {
        if (!(seph=selseph(teph,sat,nav))) return 0;
        seph2pos(time,seph,rs,dts,var);
        time=timeadd(time,tt);
        seph2pos(time,seph,rst,dtst,var);
        *svh=seph->svh;
    }
    else return 0;
    //在信号发射时刻的基础上给定一个微小的时间间隔，再次计算新时刻的 P、V、C。与3结合，通过扰动法计算出卫星的速度和频漂。
    //并没有使用那些位置和钟差公式对时间求导的结果
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;    //卫星速度rs[i+3]
    dts[1]=(dtst[0]-dts[0])/tt;                     //钟漂dts[1]

    return 1;
}
```

### peph2pos()

调用 `pephpos()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密星历，然后地球自转改正，调用 `interppol()` 内维尔插值获取卫星位置、线性插值获取钟差，最后计算标准差。

调用 `pephclk()` 根据精密星历计算卫星位置钟差，其中先二分查找时间最接近的精密钟差，再线性插值获取钟差、计算标准差。

计算相对论效应改正量，调用 `satantoff()` 计算卫星天线相位偏差改正。加上改正量得到卫星位置钟差。

加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟飘。

```c
/* satellite position/clock by precise ephemeris/clock -------------------------
* compute satellite position/clock with precise ephemeris/clock
* args   : gtime_t time       I   time (gpst)
*          int    sat         I   satellite number
*          nav_t  *nav        I   navigation data
*          int    opt         I   sat postion option
*                                 (0: center of mass, 1: antenna phase center)
*          double *rs         O   sat position and velocity (ecef)
*                                 {x,y,z,vx,vy,vz} (m|m/s)
*          double *dts        O   sat clock {bias,drift} (s|s/s)
*          double *var        IO  sat position and clock error variance (m)
*                                 (NULL: no output)
* return : status (1:ok,0:error or data outage)
* notes  : clock includes relativistic correction but does not contain code bias
*          before calling the function, nav->peph, nav->ne, nav->pclk and
*          nav->nc must be set by calling readsp3(), readrnx() or readrnxt()
*          if precise clocks are not set, clocks in sp3 are used instead
*-----------------------------------------------------------------------------*/
extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    gtime_t time_tt;
    double rss[3],rst[3],dtss[1],dtst[1],dant[3]={0},vare=0.0,varc=0.0,tt=1E-3;
    int i;
    
    trace(4,"peph2pos: time=%s sat=%2d opt=%d\n",time_str(time,3),sat,opt);
    
    if (sat<=0||MAXSAT<sat) return 0;
    
    /* satellite position and clock bias */ //计算卫星位置，钟差
    if (!pephpos(time,sat,nav,rss,dtss,&vare,&varc)||
        !pephclk(time,sat,nav,dtss,&varc)) return 0;
    
    time_tt=timeadd(time,tt);       //计算增加tt后的位置位置，钟差
    if (!pephpos(time_tt,sat,nav,rst,dtst,NULL,NULL)||
        !pephclk(time_tt,sat,nav,dtst,NULL)) return 0;
    
    /* satellite antenna offset correction */       
    if (opt) {
        satantoff(time,rss,sat,nav,dant);   //卫星天线相位中心改正
    }
    for (i=0;i<3;i++) {
        rs[i  ]=rss[i]+dant[i];         //rs[i]卫星位置，pephpos()计算值+satantoff()天线相位中心改正
        rs[i+3]=(rst[i]-rss[i])/tt;     //rs[i+3]卫星速度
    }
    /* relativistic effect correction */
    if (dtss[0]!=0.0) {
        dts[0]=dtss[0]-2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;    //dts[0]钟差,相对论效应改正
        dts[1]=(dtst[0]-dtss[0])/tt;                        //dts[1]钟漂，两次的钟差相减再除以tt
    }
    else { /* no precise clock */
        dts[0]=dts[1]=0.0;          //没有精密钟差，dts赋值0，satposs中会ephclk()广播星历的钟差替代
    }
    if (var) *var=vare+varc;
    
    return 1;
}
```



