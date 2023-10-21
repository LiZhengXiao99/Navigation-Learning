> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、后处理执行流程

### 1、



### 2、postpos()



#### 1. 功能

后处理定位的主入口函数，根据 tu 拆分计算时间段，调用调用`execses_b()`进行下一步解算。输入文件包括观测文件、导航文件、精密星历文件等，postpos在处理输入文件时有两种方法，一种是输入文件可以只包含替换，然后通过函数`reppath()`处理，将关键词用时间、基准站编号、流动站编号等代替，另一种是直接调用输入文件的文件名，postpos主要是来判断是哪一种输入方式，然后调用相应函数。

#### 2. 输入参数

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

* 处理一切正常会接收 execxes_b() 的返回值，失败返回 0，内存失败返回-1
* execses_b() 正常会接收 execses_b() 的返回值，失败返回 0
* execses_r() 正常会接收 execses() 的返回值，失败返回 0，aborts 返回 1

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



### 3、execses_b()、execses_r()

#### 1. 功能

execses_b() 和 execses_r() 函数非常类似，execsec_b() 会调用调用 `readpreceph()` 读取精密星历和 SBAS 数据，把传入`infile[]`文件中基准站替换符进行替换，之后调用`execses_r()`。`execses_r()`把传入`infile[]`文件中流动站站替换符进行替换，再调用`execses()`

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

  * 传入execses_r()，再传入execses()，用于控制输出，如果值为 0，很多不输出
  * 在 postpos 函数中赋值传入，替换输出文件替换符出错的时候设为 0，其它情况为 1

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

  

#### 3. 执行流程

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

* **readpreceph()**：遍历 infile[]，判断，调用 readsp3() 读取精密星历、调用 readrnxc() 读取精密钟差，调用 sbsreadmsg() 读取sbas文件，将 RCTM 的路径赋值给 rtcm_file，调用 init_rtcm() 初始化 rtcm 控制结构体。

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



### 4、execses()

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

  

### 5、procpos()

#### 1. 功能

从这个函数开始正式整个流动站和基准站逐历元处理。每次循环都通过inputobs函数读取一个历元的数据，并调用 rtkpos 函数对该历元的数据进行解算。 

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

  

### 6、rtkpos()

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





## 二、Option/Configuration文件读取

### 1、Option文件格式介绍

* 配置文件包含了processing options、solution options、file options三大块，用于RTKNAVI、RTKPOST、RTKRCV、RNX2RTKP。

* 文件中都以**Keyword = Value**形式记录不同的配置项。

* 对于枚举选项，可选值是选项序号(0,1,2,...) 或选项字符串(off, on, ...)。 

* 以#开头的行和行中#之后的文本被视为注释。

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

> 在 rtklib.h 中加入 #define TRACE，启用 trace ，不定义则将 trace 函数全赋空值，如下：

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

#### 10.traceb()：写入buff缓冲区数据

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



### 1、

#### 1. solopt_t 结构体：存结果输出选项

![image-20231018181736061](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231018181736061.png)

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



#### 3. solstat_t、solstatbuf_t 存结果的状态



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

* 结果时间





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











