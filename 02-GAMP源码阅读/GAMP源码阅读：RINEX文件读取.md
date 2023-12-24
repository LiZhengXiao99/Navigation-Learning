>原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

GAMP 的文件读取与 RTKLIB 大致相同，只做了一点点增强：

![image-20231031083026816](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231031083026816.png)

### 1、readobsnav()：Rinex 文件读取主入口函数

```c
gtime_t ts                  解算开始时间 
gtime_t te                  解算结束时间 
double ti                   解算时间间隔 
char **infile               传入文件路径数组 
const int *index            对应文件下标 
int n                       infile[]元素个数 
const prcopt_t *prcopt      处理选项
obs_t *obs                  存观测数据OBS
nav_t *nav                  存导航电文数据NAV
sta_t *sta                  测站结构体，存obs文件头读取到的一部分的信息
```

> **index[]的作用**：会传给`execses_b()`，再传给`execses_r()`，再传给`execses()`，再传给`readobsnav() `。如果不需要根据 `tu` 分时间段解算，`index` 存的就是 0~n，如果需要分时间段解算，`index` 存的是对应时间段内文件的下标。

* 先初始化 `obs`、`nav->eph`、`nav->geph`；遍历 `infile[]`，如果下标和上一次循环的不同，记录当前`index[i]`值到`ind `。调用`readrnxt()`读取文件，其先调用 `readrnxfile()` 读取文件，如果测站名字为空，就给依据头文件自动赋 4 个字符的名字。

* 然后判断是否有观测数据和星历数据，成功读取到数据，就调用`sortobs()`，根据 time、rcv、sat ，对`obs->data`的元素进行排序、去重，得到历元数`nepoch`。

* 最后调用`uniqnav()`，其通过调用 `uniqeph()`、`uniqgeph()` 进行星历数据的排序去重，通过调用 `satwavelen()` 获取所有载波相位的波长到 `nav->lam。`

```c
static int readobsnav(gtime_t ts, gtime_t te, double ti, char *infile[MAXINFILE],
	                  const int *index, int n, const prcopt_t *prcopt,
	                  obs_t *obs, nav_t *nav, sta_t *sta)
{
	int i,j,ind=0,nobs=0,rcv=1,nep;

	// 初始化 obs、nav->eph、nav->geph
	obs->data=NULL; obs->n =obs->nmax =0;
	nav->eph =NULL; nav->n =nav->nmax =0;
	nav->geph=NULL; nav->ng=nav->ngmax=0;
	PPP_Glo.nEpoch=0;

	// 遍历 infile[]，调用readrnxt()读取文件
	for (i=0;i<n;i++) {
		// 如果下标和上一次循环的不同，记录当前index[i]值到ind
		if (index[i]!=ind) {
			if (obs->n>nobs) rcv++;
			ind=index[i]; nobs=obs->n; 
		}
		/* read rinex obs and nav file */
		nep=readrnxt(infile[i],rcv,ts,te,ti,prcopt->rnxopt,obs,nav,rcv<=2?sta+rcv-1:NULL);
	}

	// 判断是否有观测数据和星历数据
	if (obs->n<=0) {
		printf("*** ERROR: no obs data!\n");
		return 0;
	}
	if (nav->n<=0&&nav->ng<=0) {
		printf("*** ERROR: no nav data!\n");
		return 0;
	}

	// 调用sortobs()，根据 time、rcv、sat ，对 obs->data 的元素进行排序、去重，得到历元数nepoch
	/* sort observation data */
	PPP_Glo.nEpoch=sortobs(obs);

	// 最后调用uniqnav()，其通过调用 uniqeph()、uniqgeph() 进行星历数据的排序去重，
	// 通过调用 satwavelen() 获取所有载波相位的波长到 nav->lam。
	/* delete duplicated ephemeris */
	uniqnav(nav);

	/* set time span for progress display */
	if (ts.time==0||te.time==0) {
		for (i=0;   i<obs->n;i++) if (obs->data[i].rcv==1) break;
		for (j=obs->n-1;j>=0;j--) if (obs->data[j].rcv==1) break;
		if (i<j) {
			if (ts.time==0) ts=obs->data[i].time;
			if (te.time==0) te=obs->data[j].time;
			settspan(ts,te);
		}
	}

	// 判断有无 GLONASS 星历，为啥不放在前面？？？
	if (prcopt->navsys&SYS_GLO) {
		if (nav->ng<=0) {
			printf("*** ERROR: nav->ng<=0!\n");
		}
	}

	return 1;
}
```

```c
extern int readrnxt(const char *file, int rcv, gtime_t ts, gtime_t te,
                    double tint, const char *opt, obs_t *obs, nav_t *nav,
                    sta_t *sta)
{
    int i,stat=0;
    const char *p;
    char type=' ',*files[MAXEXFILE]={0};

    /*if (!*file) {
        return readrnxfp(stdin,ts,te,tint,opt,0,1,&type,obs,nav,sta);
    }
    for (i=0;i<MAXEXFILE;i++) {
        if (!(files[i]=(char *)malloc(1024))) {
            for (i--;i>=0;i--) free(files[i]);
            return -1;
        }
    }*/
    /* expand wild-card */
    /*if ((n=expath(file,files,MAXEXFILE))<=0) {
        for (i=0;i<MAXEXFILE;i++) free(files[i]);
        return 0;
    }*/
    /* read rinex files */
    //for (i=0;i<n&&stat>=0;i++) {
        // 调用 readrnxfile() 读取文件
        stat=readrnxfile(file,ts,te,tint,opt,0,rcv,&type,obs,nav,sta);
    //}

    // 如果测站名字为空，就给依据头文件自动赋 4 个字符的名字
    /* if station name empty, set 4-char name from file head */
    if (type=='O'&&sta) {
        if (!(p=strrchr(file,FILEPATHSEP))) p=file-1;
        if (!*sta->name) setstr(sta->name,p+1,4);
    }
    for (i=0;i<MAXEXFILE;i++) free(files[i]);
    
    return stat;
}
```

### 2、readrnxfile()：传入文件路径，读取起止时间内数据

* 如果传入了测站信息结构体 sta，调用 `init_sta()` 初始化，值赋 0，指针赋空。

* 根据文件名判断文件来源（COD、IGS、IGR、GFZ、ESA、IAC、其它），以此设置 index。

* 以读的方式打开解压后的文件，调用 `readrnxfp()` ，从文件描述符 fp 中读取文件，读完之后，关闭打开的文件描述符 `fp`。

```c
static int readrnxfile(const char *file, gtime_t ts, gtime_t te, double tint,
                       const char *opt, int flag, int index, char *type,
                       obs_t *obs, nav_t *nav, sta_t *sta)
{
    FILE *fp;
    int stat;
    //char tmpfile[1024];
    
    // 如果传入了测站信息结构体 sta，调用 init_sta() 初始化
    if (sta) init_sta(sta);
    
    // 判断文件名长度是否合理
    if ( strlen(file)<2 ) 
        return ' ';

    // 以读的方式打开解压后的文件
    if (!(fp=fopen(file,"r"))) {
        return ' ';
    }

    // 根据文件名判断文件来源（COD、IGS、IGR、GFZ、ESA、IAC、其它），以此设置 index。
    if (strstr(file,"cod")||strstr(file,"COD")) index=10;
    else if (strstr(file,"igs")||strstr(file,"IGS")) index=9;
    else if (strstr(file,"igr")||strstr(file,"IGR")) index=8;
    else if (strstr(file,"gfz")||strstr(file,"GFZ")) index=7;
    else if (strstr(file,"esa")||strstr(file,"ESA")) index=6;
    else if (strstr(file,"iac")||strstr(file,"IAC")) index=-1;
    else index=0;

    // 调用 readrnxfp() ，从文件描述符 fp 中读取文件
    /* read rinex file */
    stat=readrnxfp(fp,ts,te,tint,opt,flag,index,type,obs,nav,sta);
    
    // 读完之后，关闭打开的文件描述符 fp
    fclose(fp);
    
    /* delete temporary file */
    //if (cstat) remove(tmpfile);
    
    return stat;
}
```

### 4、readrnxfp()：传入文件描述符，调用对应的读取函数

* 调用 `readrnxh()` 读取头文件。并获取文件类型 `type`。
* 根据 `type` 调用对应的函数进行分类读取，`readrnxobs()` 读观测文件，`readrnxnav()`读星历文件，调用 `readrnxnav()` 读钟差文件。

```c
static int readrnxfp(FILE *fp, gtime_t ts, gtime_t te, double tint,
                     const char *opt, int flag, int index, char *type,
                     obs_t *obs, nav_t *nav, sta_t *sta)
{
    double ver;
    int sys,tsys;
    char tobs[NUMSYS][MAXOBSTYPE][4]={{""}};
    
    // 调用 readrnxh() 读取头文件。并获取文件类型 type
    /* read rinex header */
    if (!readrnxh(fp,&ver,type,&sys,&tsys,tobs,nav,sta)) return 0;
    
    // flag 置 0 就不读钟差
    /* flag=0:except for clock,1:clock */
    if ((!flag&&*type=='C')||(flag&&*type!='C')) return 0;
    
    // 根据 type 调用对应的函数进行分类读取
    /* read rinex body */
    switch (*type) {
        case 'O': return readrnxobs(fp,ts,te,tint,opt,index,ver,tsys,tobs,obs);
        case 'N': return readrnxnav(fp,opt,ver,sys    ,nav);
        case 'G': return readrnxnav(fp,opt,ver,SYS_GLO,nav);
        case 'J': return readrnxnav(fp,opt,ver,SYS_QZS,nav); /* extension */
        case 'L': return readrnxnav(fp,opt,ver,SYS_GAL,nav); /* extension */
        case 'C': return readrnxclk(fp,opt,index,nav);
    }

    return 0;
}
```

### 5、readrnxh()：读取文件头

* 函数的主体在一个 while 大循环中，循环读取每一行，直到出现 "END OF HEADER" 
* 首先进行第一行版本号读取，记录版本号、卫星系统以及观测文件类型。
* PGM / RUN BY / DATE 跳过不读。
* 比 RTKLIB 多了 WIDELANE SATELLITE FRACTIONAL BIASES 读取。
* 其它类型的行，根据文件类型，调用 `decode_obsh()`、`decode_navh()`、`decode_gnavh()`、`decode_hnavh()`、`decode_navh()` 读取。

![image-20231028161435181](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161435181.png)

```c++
static int readrnxh(FILE *fp, double *ver, char *type, int *sys, int *tsys,
                    char tobs[][MAXOBSTYPE][4], nav_t *nav, sta_t *sta)
{
    double bias;
    char buff[MAXRNXLEN],*label=buff+60;
    int i=0,block=0,sat;
    
    *ver=2.10; *type=' '; *sys=SYS_GPS; *tsys=TSYS_GPS;
    
    // while 循环，每次读取一行，直到读到 END OF HEADER
    while (fgets(buff,MAXRNXLEN,fp)) {
        // 判定观测文件头部分所有字符总长度是否正常
        if (strlen(buff)<=60) continue;
        // 首先进行第一行版本号读取，记录版本号以及观测文件类型
        else if (strstr(label,"RINEX VERSION / TYPE")) {
            *ver=str2num(buff,0,9);
            *type=*(buff+20);
            // 通过定位字符位置读取字符，判断系统类型并记录
            /* satellite system */
            switch (*(buff+40)) {
                case ' ':
                case 'G': *sys=SYS_GPS;  *tsys=TSYS_GPS; break;
                case 'R': *sys=SYS_GLO;  *tsys=TSYS_UTC; break;
                case 'E': *sys=SYS_GAL;  *tsys=TSYS_GAL; break; /* v.2.12 */
                case 'J': *sys=SYS_QZS;  *tsys=TSYS_QZS; break; /* v.3.02 */
                case 'C': *sys=SYS_CMP;  *tsys=TSYS_CMP; break; /* v.2.12 */
                case 'M': *sys=SYS_NONE; *tsys=TSYS_GPS; break; /* mixed */
                default :
                    printf("not supported satellite system: %c\n",*(buff+40));
                    break;
            }
            continue;
        }
        else if (strstr(label,"PGM / RUN BY / DATE")) continue;
        else if (strstr(label,"COMMENT")) { /* opt */
            // 比 RTKLIB 多了 WIDELANE SATELLITE FRACTIONAL BIASES 读取
            /* read cnes wl satellite fractional bias */
            if (strstr(buff,"WIDELANE SATELLITE FRACTIONAL BIASES")||
                strstr(buff,"WIDELANE SATELLITE FRACTIONNAL BIASES")) {
                block=1;
            }
            else if (block) {
                /* cnes/cls grg clock */
                if (!strncmp(buff,"WL",2)&&(sat=satid2no(buff+3))&&
                    sscanf(buff+40,"%lf",&bias)==1) {
                    nav->wlbias[sat-1]=bias;
                }
                /* cnes ppp-wizard clock */
                else if ((sat=satid2no(buff+1))&&sscanf(buff+6,"%lf",&bias)==1) {
                    nav->wlbias[sat-1]=bias;
                }
            }
            continue; 
        }
        // 通过判断文件类型分配不同函数读取文件头
        /* file type */
        switch (*type) {
            case 'O': decode_obsh(fp,buff,*ver,tsys,tobs,nav,sta); break;
            case 'N': decode_navh (buff,nav); break;
            case 'G': decode_gnavh(buff,nav); break;
            case 'J': decode_navh (buff,nav); break; /* extension */
            case 'L': decode_navh (buff,nav); break; /* extension */
        }
        if (strstr(label,"END OF HEADER")) return 1;
        
        if (++i>=MAXPOSHEAD&&*type==' ') break; /* no rinex file */
    }
    return 0;
}
```

### 6、观测文件读取

#### 1. decode_obsh()：解析观测数据文件头

最关键的是解析观测值类型如下图，存到 `tobs` 三维数组中，【星座类型】【观测类型】【字符串数】，后面读文件体的时候要按文件头的观测值类型来读。

![image-20231028161101023](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161101023.png)

```c++
static void decode_obsh(FILE *fp, char *buff, double ver, int *tsys,
                        char tobs[][MAXOBSTYPE][4], nav_t *nav, sta_t *sta)
{
    /* default codes for unknown code */
    const char *defcodes[]={
        "CWX   ",   /* GPS: L125___ */
        "CC    ",   /* GLO: L12____ */
        "X XXXX",   /* GAL: L1_5678 */
        "CXXX  ",   /* QZS: L1256__ */
        "C X   ",   /* SBS: L1_5___ */
        "X  XX "    /* BDS: L1__67_ */
    };
    double del[3];
    int i,j,k,n,nt,prn,fcn;
    const char *p;
    char *label=buff+60,str[4];
    
    if      (strstr(label,"MARKER NAME"         )) {
        if (sta) setstr(sta->name,buff,60);
    }
    else if (strstr(label,"MARKER NUMBER"       )) { /* opt */
        if (sta) setstr(sta->marker,buff,20);
    }
    else if (strstr(label,"MARKER TYPE"         )) ; /* ver.3 */
    else if (strstr(label,"OBSERVER / AGENCY"   )) ;
    else if (strstr(label,"REC # / TYPE / VERS" )) {
        if (sta) {
            setstr(sta->recsno, buff,   20);
            setstr(sta->rectype,buff+20,20);
            setstr(sta->recver, buff+40,20);
        }
    }
    else if (strstr(label,"ANT # / TYPE"        )) {
        if (sta) {
            setstr(sta->antsno,buff   ,20);
            setstr(sta->antdes,buff+20,20);
        }
    }
    else if (strstr(label,"APPROX POSITION XYZ" )) {
        if (sta) {
            for (i=0,j=0;i<3;i++,j+=14) sta->pos[i]=str2num(buff,j,14);
        }
    }
    else if (strstr(label,"ANTENNA: DELTA H/E/N")) {
        if (sta) {
            for (i=0,j=0;i<3;i++,j+=14) del[i]=str2num(buff,j,14);
            sta->del[2]=del[0]; /* h */
            sta->del[0]=del[1]; /* e */
            sta->del[1]=del[2]; /* n */
        }
    }
    else if (strstr(label,"ANTENNA: DELTA X/Y/Z")) ; /* opt ver.3 */
    else if (strstr(label,"ANTENNA: PHASECENTER")) ; /* opt ver.3 */
    else if (strstr(label,"ANTENNA: B.SIGHT XYZ")) ; /* opt ver.3 */
    else if (strstr(label,"ANTENNA: ZERODIR AZI")) ; /* opt ver.3 */
    else if (strstr(label,"ANTENNA: ZERODIR XYZ")) ; /* opt ver.3 */
    else if (strstr(label,"CENTER OF MASS: XYZ" )) ; /* opt ver.3 */
    else if (strstr(label,"SYS / # / OBS TYPES" )) { /* ver.3 */
        if (!(p=strchr(syscodes,buff[0]))) {
            printf("invalid system code: sys=%c\n",buff[0]);
            return;
        }
        i=(int)(p-syscodes);
        n=(int)str2num(buff,3,3);
        for (j=nt=0,k=7;j<n;j++,k+=4) {
            if (k>58) {
                if (!fgets(buff,MAXRNXLEN,fp)) break;
                k=7;
            }
            if (nt<MAXOBSTYPE-1) setstr(tobs[i][nt++],buff+k,3);
        }
        *tobs[i][nt]='\0';
        
        /* change beidou B1 code: 3.02 draft -> 3.02 */
        if (i==5) {
            for (j=0;j<nt;j++) if (tobs[i][j][1]=='2') tobs[i][j][1]='1';
        }
        /* if unknown code in ver.3, set default code */
        for (j=0;j<nt;j++) {
            if (tobs[i][j][2]) continue;
            if (!(p=strchr(frqcodes,tobs[i][j][1]))) continue;
            tobs[i][j][2]=defcodes[i][(int)(p-frqcodes)];
        }
    }
    else if (strstr(label,"WAVELENGTH FACT L1/2")) ; /* opt ver.2 */
    else if (strstr(label,"# / TYPES OF OBSERV" )) { /* ver.2 */
        n=(int)str2num(buff,0,6);
        for (i=nt=0,j=10;i<n;i++,j+=6) {
            if (j>58) {
                if (!fgets(buff,MAXRNXLEN,fp)) break;
                j=10;
            }
            if (nt>=MAXOBSTYPE-1) continue;
            if (ver<=2.99) {
                setstr(str,buff+j,2);
                convcode(ver,SYS_GPS,str,tobs[0][nt]);
                convcode(ver,SYS_GLO,str,tobs[1][nt]);
                convcode(ver,SYS_GAL,str,tobs[2][nt]);
                convcode(ver,SYS_QZS,str,tobs[3][nt]);
                convcode(ver,SYS_SBS,str,tobs[4][nt]);
                convcode(ver,SYS_CMP,str,tobs[5][nt]);
            }
            nt++;
        }
        *tobs[0][nt]='\0';
    }
    else if (strstr(label,"SIGNAL STRENGTH UNIT")) ; /* opt ver.3 */
    else if (strstr(label,"INTERVAL"            )) ; /* opt */
    else if (strstr(label,"TIME OF FIRST OBS"   )) {
        if      (!strncmp(buff+48,"GPS",3)) *tsys=TSYS_GPS;
        else if (!strncmp(buff+48,"GLO",3)) *tsys=TSYS_UTC;
        else if (!strncmp(buff+48,"GAL",3)) *tsys=TSYS_GAL;
        else if (!strncmp(buff+48,"QZS",3)) *tsys=TSYS_QZS; /* ver.3.02 */
        else if (!strncmp(buff+48,"BDT",3)) *tsys=TSYS_CMP; /* ver.3.02 */
    }
    else if (strstr(label,"TIME OF LAST OBS"    )) ; /* opt */
    else if (strstr(label,"RCV CLOCK OFFS APPL" )) ; /* opt */
    else if (strstr(label,"SYS / DCBS APPLIED"  )) ; /* opt ver.3 */
    else if (strstr(label,"SYS / PCVS APPLIED"  )) ; /* opt ver.3 */
    else if (strstr(label,"SYS / SCALE FACTOR"  )) ; /* opt ver.3 */
    else if (strstr(label,"SYS / PHASE SHIFTS"  )) ; /* ver.3.01 */
    else if (strstr(label,"GLONASS SLOT / FRQ #")) { /* ver.3.02 */
        if (nav) {
            for (i=0,p=buff+4;i<8;i++,p+=8) {
                if (sscanf(p,"R%2d %2d",&prn,&fcn)<2) continue;
                if (1<=prn&&prn<=MAXPRNGLO) nav->glo_fcn[prn-1]=fcn+8;
            }
        }
    }
    else if (strstr(label,"GLONASS COD/PHS/BIS" )) { /* ver.3.02 */
        if (nav) {
            for (i=0,p=buff;i<4;i++,p+=13) {
                if      (strncmp(p+1,"C1C",3)) nav->glo_cpbias[0]=str2num(p,5,8);
                else if (strncmp(p+1,"C1P",3)) nav->glo_cpbias[1]=str2num(p,5,8);
                else if (strncmp(p+1,"C2C",3)) nav->glo_cpbias[2]=str2num(p,5,8);
                else if (strncmp(p+1,"C2P",3)) nav->glo_cpbias[3]=str2num(p,5,8);
            }
        }
    }
    else if (strstr(label,"LEAP SECONDS"        )) { /* opt */
        if (nav) nav->leaps=(int)str2num(buff,0,6);
    }
    else if (strstr(label,"# OF SALTELLITES"    )) ; /* opt */
    else if (strstr(label,"PRN / # OF OBS"      )) ; /* opt */
}
```

#### 2. readrnxobs()：读取o文件中全部观测值数据

重复调用`readrnxobsb()`函数，直到所有的观测值全被读完，或者是出现了某个历元没有卫星的情况为止 

* 为`data[]` 开辟空间
* while大循环调用`readrnxobsb()`每次读取一个历元的观测数据，获取观测值数n
 * 遍历`data[]`，如果时间系统为UTC，转为GPST，调用`saveslips()`
* 调用`screent()`，判断传入的时间是否符合起始时间ts，结束时间te，时间间隔tint
* 遍历`data[]` ，调用`restslips()`，`addobsdata()`将`data[]`信息存到`obs`中 

```c++
/* read RINEX observation data -----------------------------------------------
 * args:FILE *fp                    I   传入的Rinex文件指针
 *      gtime_t ts                  I   开始时间
 *      gtime_t te                  I   结束时间
 *      double tint                 I   时间间隔
 *      const char *opt             I   选项
 *      int rcv                     I   接收机号
 *      double ver                  I   Rinex文件版本
 *      int *tsys                   I   时间系统
 *      char tobs[][MAXOBSTYPE][4]  I   观测值类型数组
 *      obs_t *obs                  O   obsd_t类型的观测值数组
 *      sta_t *sta                  O   卫星数组
 ----------------------------------------------------------------------------*/
 static int readrnxobs(FILE *fp, gtime_t ts, gtime_t te, double tint,
                      const char *opt, int rcv, double ver, int tsys,
                      char tobs[][MAXOBSTYPE][4], obs_t *obs)
{
    obsd_t *data;
    unsigned char slips[MAXSAT][NFREQ]={{0}};
    int i,n,flag=0,stat=0;
    
    rcv=1;
    
    if (!obs||rcv>MAXRCV) return 0;
    
    if (!(data=(obsd_t *)malloc(sizeof(obsd_t)*MAXOBS))) return 0;
    
    // 循环调用 readrnxobsb() 每次读一个历元的观测值
    /* read rinex obs data body */
    while ((n=readrnxobsb(fp,opt,ver,tobs,&flag,data))>=0&&stat>=0) {
        for (i=0;i<n;i++) {

            // 如果是 UTC 时间，转为 GPST
            /* utc -> gpst */
            if (tsys==TSYS_UTC) data[i].time=utc2gpst(data[i].time);
            
            // 调用 saveslips() 保存周跳标记 LLI 到 slips
            /* save cycle-slip */
            saveslips(slips,data+i);
        }
        // 调用 screent() 按判断观测值是否在解算时间内
        /* screen data by time */
        if (n>0&&!screent(data[0].time,ts,te,tint)) continue;
        
        // 遍历 data[]，将信息存到 obs 中
        for (i=0;i<n;i++) {
            /* restore cycle-slip */
            restslips(slips,data+i);
            
            data[i].rcv=(unsigned char)rcv;
            
            // 调用 addobsdata()，在 obs_t 类型的 obs 添加新的观测值 obsd_t 类型的 data，
            // 检验内存够不够，不够就 realloc()
            /* save obs data */
            if ((stat=addobsdata(obs,data+i))<0) break;
        }
    }
    
    free(data);
    
    return stat;
}
```

#### 3. readrnxobsb()：读取一个观测历元的观测数据

* 调用`set_sysmask()`获取卫星系统掩码mask，mask在之后`decode_obsdata()`中会用到，mask中没有的卫星系统不用。

* 调用set_index()，将将tobs数组中存的观测值类型信息存到sigind_t类型的index[]结构体数组中，此时传入的tobs数组是二维数组，每个传入的tobs都存了一个卫星系统的观测值类型，同理index[]的一个元素就存一个卫星系统的所有观测值类型。

* while大循环，fgets()存一行的数据

  * 如果是第一行，则调用`decode_obsepoch()`函数解码首行数据（包括历元时刻、卫星数、卫星编号、历元状态等信息），并将信息保存 ，获取的卫星数量nsat是判断循环次数的关键。
  * 如果不是第一行则调用`decode_obsdata()`函数对该行观测数据进行数据解码，读取一个历元内一颗卫星的观测值 ，到data[n]
  * 知道读取数量 i 等于`decode_obsepoch()`获取的卫星数量`nsat`，结束循环，返回读取的观测值数（卫星数）


```c++
/* read RINEX observation data body ------------------------------------------
 * args:FILE *fp    I               I   传入的Rinex文件指针
 *      const char *opt             I   选项
 *      double ver                  I   Rinex文件版本
 *      int *tsys                   I   时间系统
 *      char tobs[][MAXOBSTYPE][4]  I   观测值类型数组
 *      int *flag                   I   历元信息状态
 *      obsd_t *data                O   obsd_t类型的观测值数组
 *      sta_t *sta                  O   卫星数组
 ------------------------------------------------------------------------------*/
static int readrnxobsb(FILE *fp, const char *opt, double ver,
                       char tobs[][MAXOBSTYPE][4], int *flag, obsd_t *data)
{
    gtime_t time={0};
    sigind_t index[6]={{0}};
    char buff[MAXRNXLEN];
    int i=0,n=0,nsat=0,sats[MAXOBS]={0},mask;
    
    /* set system mask */
    mask=set_sysmask(opt);

    // 调用 set_index()，每个系统建立一个索引
    // 建立索引。将三维观测值类型数组退化成二维数组，建立一个索引数组
    // 通过判断 nsys 值对 set_index 进行传参，然后记录在 sigind_t 结构体中
    /* set signal index */
    set_index(ver,SYS_GPS,opt,tobs[0],index  );
    set_index(ver,SYS_GLO,opt,tobs[1],index+1);
    set_index(ver,SYS_GAL,opt,tobs[2],index+2);
    set_index(ver,SYS_QZS,opt,tobs[3],index+3);
    set_index(ver,SYS_SBS,opt,tobs[4],index+4);
    set_index(ver,SYS_CMP,opt,tobs[5],index+5);
    
    // 利用 fgets() 函数缓存一行数据
    /* read record */
    while (fgets(buff,MAXRNXLEN,fp)) {
        // 记录一个观测历元的有效性、时间和卫星数
        /* decode obs epoch */
        // 如果是第一行，则调用 decode_obsepoch() 函数解码首行数据（包括历元时刻、卫星数、卫星编号、历元状态等信息），并将信息保存
        if (i==0) {
            if ((nsat=decode_obsepoch(fp,buff,ver,&time,flag,sats))<=0) {
                continue;
            }
        }
        else if (*flag<=2||*flag==6) {
            
            data[n].time=time;
            data[n].sat=(unsigned char)sats[i-1];
            
            // 如果不是第一行则调用 decode_obsdata() 函数对该行观测数据进行数据解码
            /* decode obs data */
            if (decode_obsdata(fp,buff,ver,mask,index,data+n)&&n<MAXOBS) n++;
        }
        if (++i>nsat) return n;
    }
    return -1;
}
```

#### 4. decode_obsepoch()：解码历元首行数据

**2、3版本观测值文件有区别**：

- 2版本：

  ![image-20231028161530991](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161530991.png)

  - 每历元首行数据前26位为历元时间（yy mm dd hh mm ss)，年是 2 位表示，str2time() 函数中可以把年的前两位也补上。
  - 29位epoch flag ，记录该历元状况，0表示正常，3:new site,4:header info,5:external event 
  - 30~32位为卫星数量
  - 33~68：各个卫星的PRN号，观测到的卫星数>12时，一行的信息存储不下会自动换行，并且卫星的PRN号与前一行对其 
  - 历元信息往下一行就是记录观测值的数据块，以每颗卫星为单位，依照头文件中的观测值类型及顺序，从左到右依次排列，每行记录5个观测值，一行不够时转下行。当所有卫星数据记录完后，转到下一个历元。 观测值的顺序与文件头中**“SYS / # / OBS TYPES”**记录的观测类型顺序一致。


   - 3版本：

     ![image-20231028161613724](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028161613724.png)

     - 每历元数据用用**>**开头 
     - 2~29位为历元时间（yyyy mm dd hh mm ss)
     - 32位为 epoch flag
     - 后面是接收机钟差（s）
     - 每组数据中的每一行表示一颗卫星的观测值，观测值的顺序与文件头中**“SYS / # / OBS TYPES”**记录的观测类型顺序一致 

**程序执行流程**：

* 2版本：
  * 读取卫星数到 `n` 
  * 读取`epoh flag` 到 `flag`
  * 读取历元时间 `time`
  * 循环读取卫星ID(G10、G32、G26) ，读到68列，还没把卫星读完，就fgets()读取新的一行 
  * 将卫星ID转为`satellite number`，存到`sats[]`数组中 
* 3版本：
  * 读取卫星数量 `n`
  * 读取`epoh flag` 到 `flag`
  * 读取历元时间 `time`

```c++
/* decode observation epoch --------------------------------------------------
 * args:FILE *fp        I   传入的Rinex文件指针
 *      char *buff      I   fgets()读取到一行数据的首地址
 *      double ver      I   Rinex文件版本
 *      gtime_t *time   O   历元时间
 *      int *flag       O   epoh flag (o:ok,3:new site,4:header info,5:external event)
 *      int *sats       O   历元卫星信息，2版本才有
 * return：卫星数量
 ----------------------------------------------------------------------------*/
static int decode_obsepoch(FILE *fp, char *buff, double ver, gtime_t *time,
                           int *flag, int *sats)
{
    int i,j,n;
    char satid[8]="";
    
    if (ver<=2.99) { /* ver.2 */
        if ((n=(int)str2num(buff,29,3))<=0) return 0;   // 读取卫星数到 n
        
        /* epoch flag: 3:new site,4:header info,5:external event */
        *flag=(int)str2num(buff,28,1);      // 读取 epoh flag
        
        if (3<=*flag&&*flag<=5) return n;

        if (str2time(buff,0,26,time)) {     // 读取历元时间
            printf("rinex obs invalid epoch: epoch=%26.26s\n",buff);
            return 0;
        }
        for (i=0,j=32;i<n;i++,j+=3) {       // 循环读取卫星ID(G10、G32、G26)
            if (j>=68) {                    // 读到 68 列，还没把卫星读完，就 fgets() 读取新的一行
                if (!fgets(buff,MAXRNXLEN,fp)) break;
                j=32;
            }
            if (i<MAXOBS) {
                strncpy(satid,buff+j,3);
                sats[i]=satid2no(satid);    // 将卫星ID转为 satellite number，存到sats[]数组中
            }
        }
    }
    else { /* ver.3 */
        if ((n=(int)str2num(buff,32,3))<=0) return 0;   // 读取卫星数量
        
        *flag=(int)str2num(buff,31,1);      // 读取 epoh flag
        
        if (3<=*flag&&*flag<=5) return n;
        
        // 识别历元第一个字符是否匹配以及历元时间是否可以正常转换
        if (buff[0]!='>'||str2time(buff,1,28,time)) {
            printf("rinex obs invalid epoch: epoch=%29.29s\n",buff);
            return 0;
        }
    }

    return n;
}
```

#### 5. decode_obsdata()：读取一个历元内一颗卫星的观测值

* 3 版本，读取卫星 `satellite number` 存到 `obs->sat `。
* 星系统和`mask`做与运算，判断卫星系统是否启用。
* 根据卫星系统分配索引 `ind`。
* 根据索引`ind`中的观测值类型，循环读取观测值，读取一个历元内，一颗卫星的观测值，记录有效的观测值到 `val[i]` ，记录记录信号失锁到 `lli[i]`。
* 初始化`obs`各观测值数组，赋空。
* 遍历观测值类型，同频率的观测码，下标分别存到 `k[]`，`l[]`中 ，`p[]` 存频率索引，后面 `obs->P[0]` 就是利用 L1 载波观测到的伪距，`obs->P[1]`就是利用L2载波观测到的伪距 
* 同一个频率有不同的观测码，取优先级高的。
* 根据索引`ind`中的观测值类型，遍历观测值，`val[i]`、`lli[i]`存入`obs`中。

```c++
/* decode observation data ---------------------------------------------------
 * args:FILE *fp        I   传入的Rinex文件指针
 *      char *buff      I   fgets()读取到一行数据的首地址
 *      double ver      I   Rinex文件版本
 *      int mask        I   卫星系统掩码
 *      sigind_t *index I   观测数据类型索引
 *      obsd_t *obs     O   观测数据OBS   
 ----------------------------------------------------------------------------*/
static int decode_obsdata(FILE *fp, char *buff, double ver, int mask,
                          sigind_t *index, obsd_t *obs)
{
    sigind_t *ind;
    double val[MAXOBSTYPE]={0};
    unsigned char lli[MAXOBSTYPE]={0};
    char satid[8]="";
    int i,j,n,m,stat=1,p[MAXOBSTYPE],k[16],l[16];
    
    // 3版本，读取卫星 satellite number 存到 obs->sat
    if (ver>2.99) { /* ver.3 */
        strncpy(satid,buff,3);
		//strncpy(obs->csat,buff,3);
        obs->sat=(unsigned char)satid2no(satid);     
    }
    if (!obs->sat) {
        //printf("decode_obsdata: unsupported sat sat=%s\n",satid);
        stat=0;
    }
     // 卫星系统和 mask 做与运算，判断卫星系统是否启用
    else if (!(satsys(obs->sat,NULL)&mask)) {      
        stat=0;
    }

    // 根据卫星系统分配索引
    /* read obs data fields */ 
    switch (satsys(obs->sat,NULL)) {
        case SYS_GLO: ind=index+1; break;
        case SYS_GAL: ind=index+2; break;
        case SYS_QZS: ind=index+3; break;
        case SYS_SBS: ind=index+4; break;
        case SYS_CMP: ind=index+5; break;
        default:      ind=index  ; break;
    }

    // 根据索引 ind 中的观测值类型，循环读取观测值，读取一个历元内，一颗卫星的观测值
    // 2 版本从 0 开始，3 版本从 3 开始，一次读取 16 个字符(每一个卫星的观测数据）
    for (i=0,j=ver<=2.99?0:3;i<ind->n;i++,j+=16) {
        
        // 2版本，一行读不完就 fgets 读下一行
        if (ver<=2.99&&j>=80) { /* ver.2 */
            if (!fgets(buff,MAXRNXLEN,fp)) break;
            j=0;
        }
        if (stat) {
            val[i]=str2num(buff,j,14)+ind->shift[i];        // 记录有效的观测值
            lli[i]=(unsigned char)str2num(buff,j+14,1)&3;   // 记录信号失锁，判定周跳
        }
    }
    if (!stat) return 0;
    
    // 初始化 obs 各观测值数组，赋空
    for (i=0;i<NFREQ+NEXOBS;i++) {
        obs->P[i]=obs->L[i]=0.0; obs->D[i]=0.0f;
        obs->SNR[i]=obs->LLI[i]=obs->code[i]=0;
    }

    // 遍历观测值类型，同频率的观测码，下标分别存到 k[]，l[]
    /* assign position in obs data */
    for (i=n=m=0;i<ind->n;i++) {
        
        p[i]=ver<=2.11?ind->frq[i]-1:ind->pos[i];
        
        if (ind->type[i]==0&&p[i]==0) k[n++]=i; /* C1? index */
        if (ind->type[i]==0&&p[i]==1) l[m++]=i; /* C2? index */
    }
    if (ver<=2.11) {
        // 同一个频率有不同的观测码，取优先级高的
        /* if multiple codes (C1/P1,C2/P2), select higher priority */
        if (n>=2) {
            if (val[k[0]]==0.0&&val[k[1]]==0.0) {
                p[k[0]]=-1; p[k[1]]=-1;
            }
            else if (val[k[0]]!=0.0&&val[k[1]]==0.0) {
                p[k[0]]=0; p[k[1]]=-1;
            }
            else if (val[k[0]]==0.0&&val[k[1]]!=0.0) {
                p[k[0]]=-1; p[k[1]]=0;
            }
            else if (ind->pri[k[1]]>ind->pri[k[0]]) {
                p[k[1]]=0; p[k[0]]=NEXOBS<1?-1:NFREQ;
            }
            else {
                p[k[0]]=0; p[k[1]]=NEXOBS<1?-1:NFREQ;
            }
        }
        if (m>=2) {
            if (val[l[0]]==0.0&&val[l[1]]==0.0) {
                p[l[0]]=-1; p[l[1]]=-1;
            }
            else if (val[l[0]]!=0.0&&val[l[1]]==0.0) {
                p[l[0]]=1; p[l[1]]=-1;
            }
            else if (val[l[0]]==0.0&&val[l[1]]!=0.0) {
                p[l[0]]=-1; p[l[1]]=1; 
            }
            else if (ind->pri[l[1]]>ind->pri[l[0]]) {
                p[l[1]]=1; p[l[0]]=NEXOBS<2?-1:NFREQ+1;
            }
            else {
                p[l[0]]=1; p[l[1]]=NEXOBS<2?-1:NFREQ+1;
            }
        }
    }

    // obs->P 代表着这个观测值结构体中的伪距观测值。不管是伪距观测值还是载波相位观测值和多普勒观测值，都是利用各种载波得到的
    // obs->P[0] 就是利用 L1 载波观测到的伪距，obs->P[1] 就是利用 L2 载波观测到的伪距…
    // 保存数据部分，每一个观测类型的组成包括：观测值（保留三位小数） + LLI + 信号强度，所以 obs 指向的三个可能代表的就是这三个
    // 遍历观测值，存入 obs 中
    /* save obs data */ 
	j=0;
    for (i=0;i<ind->n;i++) {
        if (p[i]<0||val[i]==0.0) continue;
        switch (ind->type[i]) {
            case 0: obs->P[p[i]]=val[i]; obs->code[p[i]]=ind->code[i]; 
				obs->type[j++]=code2obs(obs->code[p[i]],&p[i]); break;
            case 1: obs->L[p[i]]=val[i]; obs->LLI [p[i]]=lli[i];      break;
            case 2: obs->D[p[i]]=(float)val[i];                        break;
            case 3: obs->SNR[p[i]]=(unsigned char)(val[i]*4.0+0.5);    break;
        }
    }

    return 1;
}
```

### 7、星历文件读取

#### 1. decode_navh()、decode_gnavh()、decode_hnavh()

以 decode_navh() 为例，对应着格式一点点读：

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/819ea1b1474442a09edd951a7d9409dd.png)

```c++
static void decode_navh(char *buff, nav_t *nav)
{
    int i,j;
    char *label=buff+60;
    
    if      (strstr(label,"ION ALPHA"           )) { /* opt ver.2 */
        if (nav) {
            for (i=0,j=2;i<4;i++,j+=12) nav->ion_gps[i]=str2num(buff,j,12);
        }
    }
    else if (strstr(label,"ION BETA"            )) { /* opt ver.2 */
        if (nav) {
            for (i=0,j=2;i<4;i++,j+=12) nav->ion_gps[i+4]=str2num(buff,j,12);
        }
    }
    else if (strstr(label,"DELTA-UTC: A0,A1,T,W")) { /* opt ver.2 */
        if (nav) {
            for (i=0,j=3;i<2;i++,j+=19) nav->utc_gps[i]=str2num(buff,j,19);
            for (;i<4;i++,j+=9) nav->utc_gps[i]=str2num(buff,j,9);
        }
    }
    else if (strstr(label,"IONOSPHERIC CORR"    )) { /* opt ver.3 */
        if (nav) {
            if (!strncmp(buff,"GPSA",4)) {
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_gps[i]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"GPSB",4)) {
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_gps[i+4]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"GAL",3)) {
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_gal[i]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"QZSA",4)) { /* v.3.02 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_qzs[i]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"QZSB",4)) { /* v.3.02 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_qzs[i+4]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"BDSA",4)) { /* v.3.02 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_cmp[i]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"BDSB",4)) { /* v.3.02 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_cmp[i+4]=str2num(buff,j,12);
            }
        }
    }
    else if (strstr(label,"TIME SYSTEM CORR"    )) { /* opt ver.3 */
        if (nav) {
            if (!strncmp(buff,"GPUT",4)) {
                nav->utc_gps[0]=str2num(buff, 5,17);
                nav->utc_gps[1]=str2num(buff,22,16);
                nav->utc_gps[2]=str2num(buff,38, 7);
                nav->utc_gps[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"GLUT",4)) {
                nav->utc_glo[0]=str2num(buff, 5,17);
                nav->utc_glo[1]=str2num(buff,22,16);
            }
            else if (!strncmp(buff,"GAUT",4)) { /* v.3.02 */
                nav->utc_gal[0]=str2num(buff, 5,17);
                nav->utc_gal[1]=str2num(buff,22,16);
                nav->utc_gal[2]=str2num(buff,38, 7);
                nav->utc_gal[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"QZUT",4)) { /* v.3.02 */
                nav->utc_qzs[0]=str2num(buff, 5,17);
                nav->utc_qzs[1]=str2num(buff,22,16);
                nav->utc_qzs[2]=str2num(buff,38, 7);
                nav->utc_qzs[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"BDUT",4)) { /* v.3.02 */
                nav->utc_cmp[0]=str2num(buff, 5,17);
                nav->utc_cmp[1]=str2num(buff,22,16);
                nav->utc_cmp[2]=str2num(buff,38, 7);
                nav->utc_cmp[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"SBUT",4)) { /* v.3.02 */
                nav->utc_cmp[0]=str2num(buff, 5,17);
                nav->utc_cmp[1]=str2num(buff,22,16);
                nav->utc_cmp[2]=str2num(buff,38, 7);
                nav->utc_cmp[3]=str2num(buff,45, 5);
            }
        }
    }
    else if (strstr(label,"LEAP SECONDS"        )) { /* opt */
        if (nav) nav->leaps=(int)str2num(buff,0,6);
    }
}
```

#### 2. readrnxnav()：读取星历文件，添加到nav结构体中

* **add_eph**()：nav->eph[] 中添加 eph 星历数据，nav->n 表示 eph 数量。
* **add_geph**()：nav->geph[] 中添加 GLONASS 星历数据，nav->ng 表示 geph 数量。
* **add_seph**()：nav->seph[] 中添加 SBAS 星历数据，nav->ns 表示 seph 数量。

```c++
static int readrnxnav(FILE *fp, const char *opt, double ver, int sys,
                      nav_t *nav)
{
	eph_t eph={0};
	geph_t geph={0};
    int stat,type;
    
    if (!nav) return 0;
    
    /* read rinex navigation data body */
    while ((stat=readrnxnavb(fp,opt,ver,sys,&type,&eph,&geph))>=0) {
        
        /* add ephemeris to navigation data */
        if (stat) {
            switch (type) {
                case 1 : stat=add_geph(nav,&geph); break;
                default: stat=add_eph (nav,&eph ); break;
            }
            if (!stat) return 0;
        }
    }
    return nav->n>0||nav->ng>0;
}
```

#### 3. readrnxnavb()：读取一个历元的星历数据，添加到 eph 结构体中

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7c62093881b847b8943b0910e78b23b8.png)

* 调用`set_sysmask()`获取卫星系统掩码

* 循环读取一行行，记录TOC，读取到`data[]`，i记录读取的数据数量，读够数量调用`decode_eph()`等函数赋值给`eph_t`结构体 

```c++
/* read rinex navigation data body -------------------------------------------*/
static int readrnxnavb(FILE *fp, const char *opt, double ver, int sys,
                       int *type, eph_t *eph, geph_t *geph)
{
    gtime_t toc;
    double data[64];
    int i=0,j,prn,sat=0,sp=3,mask;
    char buff[MAXRNXLEN],id[8]="",*p;
    
    /* set system mask */
    mask=set_sysmask(opt);
    
    // 循环读取一行行，读取到 data[]，i 记录读取的数据数量，读够数量进入 decode_eph() 赋值给 eph_t 结构体
    while (fgets(buff,MAXRNXLEN,fp)) {
        if (i==0) {
            /* decode satellite field */
            if (ver>=3.0||sys==SYS_GAL||sys==SYS_QZS) { /* ver.3 or GAL/QZS */
                strncpy(id,buff,3);
                sat=satid2no(id);
                sp=4;       // 3以上版本，GALileo，QZSS sp 都为 4
                if (ver>=3.0) sys=satsys(sat,NULL);
            }
            else {
                prn=(int)str2num(buff,0,2);
                
                if (sys==SYS_SBS) {
                    sat=satno(SYS_SBS,prn+100);
                }
                else if (sys==SYS_GLO) {
                    sat=satno(SYS_GLO,prn);
                }
                else if (93<=prn&&prn<=97) { /* extension */
                    sat=satno(SYS_QZS,prn+100);
                }
                else sat=satno(SYS_GPS,prn);
            }
            /* decode toc field */
            if (str2time(buff+sp,0,19,&toc)) {  // 读取卫星钟时间 TOC
                printf("rinex nav toc error: %23.23s\n",buff);
                return 0;
            }
            /* decode data fields */
            for (j=0,p=buff+sp+19;j<3;j++,p+=19) {  // 首行数据读3列，除了TOC还有3列
                data[i++]=str2num(p,0,19);
            }
        }
        else {
            /* decode data fields */
            for (j=0,p=buff+sp;j<4;j++,p+=19) {     // 其它行数据都读 4 列
                data[i++]=str2num(p,0,19);
            }
            /* decode ephemeris */
            if (sys==SYS_GLO&&i>=15) {
                if (!(mask&sys)) return 0;
                *type=1;
                return decode_geph(ver,sat,toc,data,geph);
            }
            else if (i>=31) {
                if (!(mask&sys)) return 0;
                *type=0;
                return decode_eph(ver,sat,toc,data,eph);
            }
        }
    }
    return -1;
}
```