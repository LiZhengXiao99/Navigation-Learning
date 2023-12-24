> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

### 1、satposs_rtklib()：卫星位置钟差计算入口函数

```c
gtime_t teph     I     (gpst) 用于选择星历的时刻 (gpst)
obsd_t *obs      I      OBS观测数据
int    n         I      OBS数
nav_t  *nav      I      NAV导航电文
int    ephopt    I      星历选项 (EPHOPT_???)
double *rs       O      卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double *dts      O      卫星钟差，长度为2*n， {bias,drift} (s|s/s)
double *var      O      卫星位置和钟差的协方差 (m^2)
int    *svh      O      卫星健康标志 (-1:correction not available)
```

![image-20230929100826545](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100826545.png)

* 遍历观测数据，找伪距观测值，除以光速得到信号传播时间，用数据接收时刻减去伪距信号传播时间得到信号发射时刻。

* 调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，不作为最终计算的钟差。信号发射时刻减去钟差 `dt`，得到 GPS 时间下的卫星信号发射时刻。

* **调用 `satpos()` 对此观测值进行下一步卫星位置钟差的计算**；`satpos()` 函数对星历计算选项进行判断，**广播星历模式调用 `ephpos()`**，**精密星历模式调用 `peph2pos()`**。最后检测钟差值，如果没有精密星历，则调用 `ephclk()` 用广播星历计算钟差。

```c
extern void satposs_rtklib(gtime_t teph, const obsd_t *obs, int n, const nav_t *nav,
                    int ephopt, double *rs, double *dts, double *var, int *svh)
{
    gtime_t time[MAXOBS]={{0}};
    double dt,pr;
    int i,j;
    
    for (i=0;i<n&&i<2*MAXOBS;i++) {
        for (j=0;j<6;j++) rs [j+i*6]=0.0;
        for (j=0;j<2;j++) dts[j+i*2]=0.0;
        var[i]=0.0; svh[i]=0;
        
        /* search any psuedorange */
        for (j=0,pr=0.0;j<NFREQ;j++) if ((pr=obs[i].P[j])!=0.0) break;
        
        if (j>=NFREQ) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no pseudorange %s sat=%2d\n",
				time_str(obs[i].time,3),obs[i].sat);
			outDebug(OUTWIN,OUTFIL,0);
            continue;
        }
        /* transmission time by satellite clock */
        time[i]=timeadd(obs[i].time,-pr/CLIGHT);
        
        /* satellite clock bias by broadcast ephemeris */
        if (!ephclk(time[i],teph,obs[i].sat,nav,&dt)) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no broadcast clock %s sat=%2d\n",
				time_str(time[i],3),obs[i].sat);
			outDebug(0,OUTFIL,0);
            continue;
        }
        time[i]=timeadd(time[i],-dt);
        
        /* satellite position and clock at transmission time */
        if (!satpos(time[i],teph,obs[i].sat,ephopt,nav,rs+i*6,dts+i*2,var+i,
                    svh+i)) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: no ephemeris %s sat=%2d\n",
				time_str(time[i],3),obs[i].sat);
			outDebug(0,0,0);
            continue;
        }
        /* if no precise clock available, use broadcast clock instead */
        if (dts[i*2]==0.0) {
            if (!ephclk(time[i],teph,obs[i].sat,nav,dts+i*2)) continue;
            dts[1+i*2]=0.0;
            *var=SQR(STD_BRDCCLK);
        }
    }
}
```

### 2、ephclk()：广播星历计算卫星钟差

> 不作为最终卫星钟差，计算出的钟差只是对原本卫星信号发射时间做改进，让 ephpos() 计算出更准确的卫星位置。

![image-20230929100921394](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100921394.png)

* 单观测值卫星钟差计算。由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

* 如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2clk()` 根据广播星历参数 $a_0$、$a_1$、$a_2$ 计算卫星钟差（迭代 3 次）；

* 如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2clk()` 根据广播星历参数 $t_{aun}$、$g_{aun}$  计算卫星钟差（迭代 3 次）。

```c
static int ephclk(gtime_t time, gtime_t teph, int sat, const nav_t *nav,
                  double *dts)
{
    eph_t  *eph;
    geph_t *geph;
    int sys;
    
    sys=satsys(sat,NULL);
    
    if (sys==SYS_GPS||sys==SYS_GAL||sys==SYS_QZS||sys==SYS_CMP) {
        if (!(eph=seleph(teph,sat,-1,nav))) return 0;
        *dts=eph2clk(time,eph);
    }
    else if (sys==SYS_GLO) {
        if (!(geph=selgeph(teph,sat,-1,nav))) return 0;
        *dts=geph2clk(time,geph);
    }
    else return 0;
    
    return 1;
}
```

#### 1. eph2clk()：时钟校正参数（$a_{f0}、a_{f1}、a_{f2}$）计算卫星钟差

相对于 GPS 时间，卫星上作为时间和频率信号来源的原子钟也存在时间偏差和频率漂移。为确保各颗卫星的时钟与GPS时间同步，GPS地面监控部分通过对卫星信号进行检测，将卫星时钟在GPS时间t的卫星钟差 $\Delta t^{(s)}$ 描述为如下二项式：
$$
\Delta t^{(s)}=a_{f0}+a_{f1}(t-t_{oc})+a_{f2}(t-t_{oc})^2
$$

```c
extern double eph2clk(gtime_t time, const eph_t *eph)
{
    double t;
    int i;
    
    t=timediff(time,eph->toc);  // 计算与星历参考时间的偏差 dt = t-toc
    // 利用二项式校正计算出卫星钟差，从 dt中减去这部分，然后再进行一次上述操作，得到最终的 dt
    for (i=0;i<2;i++) {
        t-=eph->f0+eph->f1*t+eph->f2*t*t;
    }
    // 使用二项式校正得到最终的钟差
    return eph->f0+eph->f1*t+eph->f2*t*t;
}
```

除此以外，卫星钟差一般还需考虑相对论效应校正、群波延迟校正、钟漂校正：

##### 相对论效应校正 $\Delta t_r$

综合狭义相对论和广义相对论，在高空中高速运行的卫星原子钟比地面上一模一样的原子钟每天要快 38000ns ，每秒快 0.44ns 。如果不考虑相对论效应，GPS 发上天两分钟内，卫星原子钟就会失去定位作用。在地面上设计原子钟时可以减小一点点它的频率，上天以后其时钟频率在地面上看来正好等于设计值。同时因为GPS运行轨道是椭圆而不是圆，地面上计算机还有根据卫星当前位置做相对论效应的校如下：
$$
\Delta t_r=Fe_s\sqrt{a_s} \sin E_k
$$

##### 群波延迟校正 $T_{GD}$

由第一数据块给出，只适用于单频。这样对于 L1 单频接收机，卫星时钟总钟差值如下：
$$
\delta t^{(s)}=\Delta t^{(s)}+\Delta t_{r}-T_{G D}
$$

##### 钟漂校正

对上面卫星时钟总钟差值求导得：
$$
\delta f^{(s)}=a_{f 1}+2 a_{f 2}\left(t-t_{o c}\right)+\Delta \dot{t}_r
$$
群波延迟校正 $T_{GD}$ 的导数为 0，相对论效应校正 $\Delta t_r$ 如下：
$$
\Delta \dot{t}_r=F e_s \sqrt{a_s} \dot{E}_k \cos E_k
$$

#### 2. geph2clk()：时钟校正参数（$\tau_{n}、\gamma_{n}$）计算 GLONASS 卫星钟差

$$
d T^{s}(t)=-\tau_{n}+\gamma_{n}\left(t-t_{b}\right)
$$

```c
extern double geph2clk(gtime_t time, const geph_t *geph)
{
    double t;
    int i;
    
    t=timediff(time,geph->toe);
    
    for (i=0;i<2;i++) {
        t-=-geph->taun+geph->gamn*t;
    }
    return -geph->taun+geph->gamn*t;
}
```

用（$\tau_{n}、\gamma_{n}$）计算 GLONASS 卫星钟差的时候已经考虑了相对论效应了，无需再改正。

### 3、ephpos()：广播星历计算卫星位置钟差

![image-20230929101151404](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101151404.png)

* 与 `ephclk()` 同理，由于 GLONASS 系统的计算和其它的区别较大，先进行判断。

* 如果不是 GLONASS 则调用 `seleph()` 选择与观测值对应的星历，调用 `eph2pos()` 根据广播星历中的开普勒轨道参数和摄动改正计算卫星位置（对北斗 MEO、IGSO 卫星会进行特殊处理）、校正卫星钟差的相对论效应、调用 `var_uraeph()` 用 URA 值来标定方差。

* 如果是 GLONASS 则调用 `selgeph()` 选择与观测值对应的星历，调用 `geph2pos()` 根据广播星历中 PZ-90 坐标系下卫星状态向量四阶龙格库塔迭代计算卫星位置。

* 计算完一次位置之后，加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟漂。

```c
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
    // 在信号发射时刻的基础上给定一个微小的时间间隔，再次计算新时刻的 P、V、C。与3结合，通过扰动法计算出卫星的速度和频漂。
    // 并没有使用那些位置和钟差公式对时间求导的结果
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;    // 卫星速度rs[i+3]
    dts[1]=(dtst[0]-dts[0])/tt;                     // 钟漂dts[1]

    return 1;
}
```

#### 1. eph2pos()：由广播星历参数计算卫星位置钟差

就是套公式，北斗需要坐标系旋转

```c
extern void eph2pos(gtime_t time, const eph_t *eph, double *rs, double *dts,
                    double *var)
{
    double tk,M,E,Ek,sinE,cosE,u,r,i,O,sin2u,cos2u,x,y,sinO,cosO,cosi,mu,omge;
    double xg,yg,zg,sino,coso;
    int n,sys,prn;
    
    trace(4,"eph2pos : time=%s sat=%2d\n",time_str(time,3),eph->sat);
    
    if (eph->A<=0.0) {  //通过卫星轨道半长轴 A 判断星历是否有效，无效则返回
        rs[0]=rs[1]=rs[2]=*dts=*var=0.0;
        return;
    }
    tk=timediff(time,eph->toe); //计算规化时间 tk (E.4.2)
    
    switch ((sys=satsys(eph->sat,&prn))) {  //根据不同卫星系统设置相应的地球引力常数 mu 和 地球自转角速度 omge
        case SYS_GAL: mu=MU_GAL; omge=OMGE_GAL; break;
        case SYS_CMP: mu=MU_CMP; omge=OMGE_CMP; break;
        default:      mu=MU_GPS; omge=OMGE;     break;
    }
    M=eph->M0+(sqrt(mu/(eph->A*eph->A*eph->A))+eph->deln)*tk;   //计算平近点角 M (E.4.3)
    
    //用牛顿迭代法来计算偏近点角 E。参考 RTKLIB manual P145 (E.4.19) (E.4.4)
    for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++) {
        Ek=E; E-=(E-eph->e*sin(E)-M)/(1.0-eph->e*cos(E));
    }
    if (n>=MAX_ITER_KEPLER) {
        trace(2,"eph2pos: kepler iteration overflow sat=%2d\n",eph->sat);
        return;
    }
    sinE=sin(E); cosE=cos(E);
    
    trace(4,"kepler: sat=%2d e=%8.5f n=%2d del=%10.3e\n",eph->sat,eph->e,n,E-Ek);
    
    //计算摄动改正后的 升交点角距u 卫星矢径长度r 轨道倾角i
    u=atan2(sqrt(1.0-eph->e*eph->e)*sinE,cosE-eph->e)+eph->omg;     //(E.4.5) (E.4.6) (E.4.10)
    r=eph->A*(1.0-eph->e*cosE);         //(E.4.11)
    i=eph->i0+eph->idot*tk;             //(E.4.12)
    sin2u=sin(2.0*u); cos2u=cos(2.0*u); 
    u+=eph->cus*sin2u+eph->cuc*cos2u;   //(E.4.7)
    r+=eph->crs*sin2u+eph->crc*cos2u;   //(E.4.8)
    i+=eph->cis*sin2u+eph->cic*cos2u;   //(E.4.9)
    
    x=r*cos(u); y=r*sin(u);     
    cosi=cos(i);
    
    // 北斗的MEO、IGSO卫星计算方法与GPS, Galileo and QZSS相同，只是一些参数不同
    // GEO卫星的 O 和最后位置的计算稍有不同 
    /* beidou geo satellite */
    if (sys==SYS_CMP&&(prn<=5||prn>=59)) { /* ref [9] table 4-1 */
        O=eph->OMG0+eph->OMGd*tk-omge*eph->toes;        //(E.4.29)
        sinO=sin(O); cosO=cos(O);
        xg=x*cosO-y*cosi*sinO;
        yg=x*sinO+y*cosi*cosO;
        zg=y*sin(i);
        sino=sin(omge*tk); coso=cos(omge*tk);
        rs[0]= xg*coso + yg*sino*COS_5 + zg*sino*SIN_5;     //ECEF位置(E.4.30)
        rs[1]=-xg*sino + yg*coso*COS_5 + zg*coso*SIN_5;
        rs[2]=-yg*SIN_5 + zg*COS_5;
    }
    else {
        O=eph->OMG0+(eph->OMGd-omge)*tk-omge*eph->toes; //计算升交点赤经O (E.4.13)
        sinO=sin(O); cosO=cos(O);
        rs[0]=x*cosO-y*cosi*sinO;   //计算卫星ECEF位置存入 rs 中 (E.4.14)
        rs[1]=x*sinO+y*cosi*cosO;
        rs[2]=y*sin(i);
    }
    tk=timediff(time,eph->toc);     //(E.4.15)
    
    *dts=eph->f0+eph->f1*tk+eph->f2*tk*tk;  //利用三个二项式模型系数 af0、af1、af2计算卫星钟差
    
    /* relativity correction */ 
    *dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);  //相对论效应改正卫星钟差
    
    /* position and clock error variance */
    *var=var_uraeph(sys,eph->sva);  //用 URA 值来标定方差
}
```

#### 2. var_uraeph()：用URA用户测距精度标定卫星位置方差

![image-20231029192908369](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231029192908369.png)

> GLONASS 不计算，直接设为 5*5

```c
static double var_uraeph(int sys, int ura)
{
    const double ura_value[]={   
        2.4,3.4,4.85,6.85,9.65,13.65,24.0,48.0,96.0,192.0,384.0,768.0,1536.0,
        3072.0,6144.0
    };
    if (sys==SYS_GAL) { /* galileo sisa (ref [7] 5.1.11) */
        if (ura<= 49) return SQR(ura*0.01);
        if (ura<= 74) return SQR(0.5+(ura- 50)*0.02);
        if (ura<= 99) return SQR(1.0+(ura- 75)*0.04);
        if (ura<=125) return SQR(2.0+(ura-100)*0.16);
        return SQR(STD_GAL_NAPA);
    }
    else { /* gps ura (ref [1] 20.3.3.3.1.1) */
        return ura<0||14<ura?SQR(6144.0):SQR(ura_value[ura]);
    }
}
```

#### 3. geph2pos()：由 GLONASS 星历计算卫星位置钟差

GLONASS 卫星播发的是 PZ-90 坐标系下参考时刻的卫星状态向量，每半个小时广播一次。如果需要得到某个时间的卫星位置必须通过运动模型积分得到。

```c
extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
                     double *var)
{
    double t,tt,x[6];
    int i;
    
    trace(4,"geph2pos: time=%s sat=%2d\n",time_str(time,3),geph->sat);
    
    t=timediff(time,geph->toe);
    
    *dts=-geph->taun+geph->gamn*t;  // 计算钟差dts(E.4.26)
    
    for (i=0;i<3;i++) {
        x[i  ]=geph->pos[i];
        x[i+3]=geph->vel[i];
    }

    //步长 TSTEP:60s
    for (tt=t<0.0?-TSTEP:TSTEP;fabs(t)>1E-9;t-=tt) {
        if (fabs(t)<TSTEP) tt=t;
        glorbit(tt,x,geph->acc);	// 调用 glorbit() 龙格库塔迭代积分
    }
    for (i=0;i<3;i++) rs[i]=x[i];
    
    *var=SQR(ERREPH_GLO);   // glonass卫星的方差直接定为 5*5
}
```

#### 4. glorbit()：龙格库塔迭代

$$
\begin{aligned} \mathrm{y}_{\mathrm{n}+1} & =\mathrm{y}_{\mathrm{n}}+\frac{\mathrm{h}}{6}\left(\mathrm{k}_{1}+2 \mathrm{k}_{2}+2 \mathrm{k}_{3}+\mathrm{k}_{4}\right) \\ \mathrm{k}_{1} & =\mathrm{f}\left(\mathrm{y}_{\mathrm{n}}\right) \\ \mathrm{k}_{2} & =\mathrm{f}\left(\mathrm{y}_{\mathrm{n}}+\mathrm{k}_{1} \frac{\mathrm{h}}{2}\right) \\ \mathrm{k}_{3} & =\mathrm{f}\left(\mathrm{y}_{\mathrm{n}}+\mathrm{k}_{2} \frac{\mathrm{h}}{2}\right) \\ \mathrm{k}_{4} & =\mathrm{f}\left(\mathrm{y}_{\mathrm{n}}+\mathrm{k}_{3} \mathrm{~h}\right)\end{aligned}
$$

```c
static void glorbit(double t, double *x, const double *acc)
{
    double k1[6],k2[6],k3[6],k4[6],w[6];
    int i;
    
    deq(x,k1,acc); for (i=0;i<6;i++) w[i]=x[i]+k1[i]*t/2.0;
    deq(w,k2,acc); for (i=0;i<6;i++) w[i]=x[i]+k2[i]*t/2.0;
    deq(w,k3,acc); for (i=0;i<6;i++) w[i]=x[i]+k3[i]*t;
    deq(w,k4,acc);
    for (i=0;i<6;i++) x[i]+=(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i])*t/6.0;
}
```

#### 5. deq()：微分方程计算

$$
\begin{array}{l}\frac{d x}{d t}=v_{x}, \frac{d y}{d t}=v_{y}, \frac{d z}{d t}=v_{z} \\ \frac{d v_{x}}{d t}=-\frac{\mu}{r^{3}} x-\frac{3}{2} J_{2} \frac{\mu a_{e}^{2}}{r^{5}} x\left(1-\frac{5 z^{2}}{r^{2}}\right)+\omega_{e}^{2} x+2 \omega_{e} v_{y}+a_{x} \\ \frac{d v_{y}}{d t}=-\frac{\mu}{r^{3}} y-\frac{3}{2} J_{2} \frac{\mu a_{e}^{2}}{r^{5}} y\left(1-\frac{5 z^{2}}{r^{2}}\right)+\omega_{e}^{2} y-2 \omega_{e} v_{x}+a_{y} \\ \frac{d v_{z}}{d t}=-\frac{\mu}{r^{3}} z-\frac{3}{2} J_{2} \frac{\mu a_{e}^{2}}{r^{5}} z\left(3-\frac{5 z^{2}}{r^{2}}\right)+a_{z}\end{array}
$$

其中：

* $a_{e}$ : 地球长半轴 $(6378136.0 \mathrm{~m})$
* $\mu$ : 地球引力常数 $\left(398600.44 \times 10^{9} \mathrm{~m}^{3} / \mathrm{s}^{2}\right)$
* $\omega_{e}$ : 地球自转角速度 $\left(7.292115 \times 10^{-5} \mathrm{rad} / \mathrm{s}\right)$
* $J_{2}$ : 地电位的纬向二次谐波$\left(1082625.7 \times 10^{-9}\right)$
* $r=\sqrt{x^{2}+y^{2}+z^{2}}$

```c
static void deq(const double *x, double *xdot, const double *acc)
{
    double a,b,c,r2=dot(x,x,3),r3=r2*sqrt(r2),omg2=SQR(OMGE_GLO);
    
    if (r2<=0.0) {
        xdot[0]=xdot[1]=xdot[2]=xdot[3]=xdot[4]=xdot[5]=0.0;
        return;
    }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a=1.5*J2_GLO*MU_GLO*SQR(RE_GLO)/r2/r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b=5.0*x[2]*x[2]/r2;                    /* 5*z^2/r^2 */
    c=-MU_GLO/r3-a*(1.0-b);                /* -mu/r^3-a(1-b) */
    xdot[0]=x[3]; xdot[1]=x[4]; xdot[2]=x[5];
    xdot[3]=(c+omg2)*x[0]+2.0*OMGE_GLO*x[4]+acc[0];
    xdot[4]=(c+omg2)*x[1]-2.0*OMGE_GLO*x[3]+acc[1];
    xdot[5]=(c-2.0*a)*x[2]+acc[2];
}
```

### 4、peph2pos()：精密星历计算卫星位置、钟差、速度、钟漂

![image-20230929101206005](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929101206005.png)

* 调用 `pephpos()` 根据精密星历计算卫星位置，其中先二分查找时间最接近的精密星历，然后地球自转改正，调用 `interppol()` 内维尔插值获取卫星位置、线性插值获取钟差，最后计算标准差。

* 调用 `pephclk()` 根据精密星历计算卫星钟差，其中先二分查找时间最接近的精密钟差，再线性插值获取钟差、计算标准差。

* 计算相对论效应改正量，调用 `satantoff()` 计算卫星天线相位偏差改正。加上改正量得到卫星位置钟差。

* 加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟飘。

* 调用 `satantoff()` 天线相位中心改正。

* 钟差做相对论效应改正：
  $$
  d T^{s}(t)=\frac{\left(t_{i+1}-t\right) d T^{s}\left(t_{i}\right)+\left(t-t_{i}\right) d T^{s}\left(t_{i+1}\right)}{t_{i+1}-t_{i}}-2 \frac{\boldsymbol{r}^{s}(t)^{T} \boldsymbol{v}^{s}(t)}{c^{2}}
  $$

```c
extern int peph2pos(gtime_t time, int sat, const nav_t *nav, int opt,
                    double *rs, double *dts, double *var)
{
    double rss[3],rst[3],dtss[1],dtst[1],dant[3]={0},vare=0.0,varc=0.0,tt=1E-3;
    int i;
    
    if (sat<=0||MAXSAT<sat) return 0;
    
    // 调用 pephpos() 根据精密星历计算卫星位置
    // 调用 pephclk() 根据精密星历计算卫星钟差
    /* satellite position and clock bias */
    if (!pephpos(time,sat,nav,rss,dtss,&vare,&varc)||
        !pephclk(time,sat,nav,dtss,&varc)) return 0;
    
    // 加上一个极小的时间，再计算一次位置，两次计算出的时间作差求得卫星速度钟飘
    time=timeadd(time,tt);
    if (!pephpos(time,sat,nav,rst,dtst,NULL,NULL)||
        !pephclk(time,sat,nav,dtst,NULL)) return 0;

    // 调用 satantoff() 天线相位中心改正
    /* satellite antenna offset correction */
    if (opt) {
        satantoff(time,rss,sat,nav,dant);
    }

    for (i=0;i<3;i++) {
        rs[i  ]=rss[i]+dant[i];
        rs[i+3]=(rst[i]-rss[i])/tt;
    }

    // 钟差做相对论效应改正
    /* relativistic effect correction */
    if (dtss[0]!=0.0) {
        dts[0]=dtss[0]-2.0*dot(rs,rs+3,3)/CLIGHT/CLIGHT;
        dts[1]=(dtst[0]-dtss[0])/tt;
    }
    else    /* no precise clock */
        dts[0]=dts[1]=0.0;
    
    *var=vare+varc;

    return 1;
}
```

#### 1.  精密星历读取流程

* `nav->peph[]` 存精密星历数据，`nav->ne` 精密钟差数量。
* `nav->pclk[]` 存精密钟差数据，`nav->nc` 精密钟差数量。
  * **execses_b**() 中调用`readpreceph()`。
  * **readpreceph**() 中：`readsp3()`读取精密星历，`readrnxc()` 读取精密钟差。
  * **readsp3**() 中：`readsp3h() `读文件头，`readsp3b()` 读文件体，`combpeph() `对精密星历按时间、index 排序，再将相同星历合并。
  * **readrnxc**() 中：`readrnxfile()` 读取精密星历文件，`combpclk()  `排序合并精密钟差。

#### 2. pephpos()：精密星历计算卫星位置，钟差

执行流程如下：

  * 如果时间早于第一个精密星历时间，或迟于最后一个超过 15 分钟，直接 return 0。
  * 二分查找 `nav->peph[]` 中时间差最接近的精密星历的下标 index。
  * 调用 `posWithEarhRotation()` 计算地球自转改正后精密星历的位置。
  * 调用 `interppol()` 内维尔多项式插值获取卫星位置。
  * 钟差就直接是线性插值。
  * 最后计算方差 `varc`，用原来存的标准差 `nav->peph[index+i].std[sat-1][3]` 加上 `EXTERR_CLK`(0.003) 乘以时间 `t `得到标准差 `std`，平方得到方差。

```c
static int pephpos(gtime_t time, int sat, const nav_t *nav, double *rs,
                   double *dts, double *vare, double *varc)
{
    double t[NMAX+1],p[3][NMAX+1],c[2],*pos,std=0.0,s[3],sinl;
    int i,j,k,index,sys;
	int id[NMAX+1],kInd,bBadClk;
    
    rs[0]=rs[1]=rs[2]=dts[0]=0.0;
    
    // 如果时间早于第一个精密星历时间，或迟于最后一个超过 15 分钟，return 0
    if (nav->ne<NMAX+1||timediff(time,nav->peph[0].time)<-MAXDTE||
        timediff(time,nav->peph[nav->ne-1].time)>MAXDTE)
        return 0;

    // 二分查找 nav->peph[] 中时间差最接近的精密星历的下标 index
    /* binary search */
    for (i=0,j=nav->ne-1;i<j;) {
        k=(i+j)/2;
        if (timediff(nav->peph[k].time,time)<0.0) i=k+1; else j=k;
    }
    index=i<=0?0:i-1;

    // 轨道多项式插值
    /* polynomial interpolation for orbit */
    i=index-(NMAX+1)/2;
    if (i<0) i=0; 
    else if (i+NMAX>=nav->ne) i=nav->ne-NMAX-1;

    for (j=k=0;j<NMAX*50;j++) {
        if (index+j>=0&&index+j<nav->ne&&k<=NMAX) {
            id[k]=index+j;
            t[k]=timediff(nav->peph[id[k]].time,time);
            pos=nav->peph[id[k]].pos[sat-1];
            if (norm(pos,3)>0.0) {
                posWithEarhRotation(k,pos,p,t[k]);
                k++;
            }
        }
        if (k==NMAX+1) break;

        if (index-j>=0&&index-j<nav->ne&&k<=NMAX&&j!=0) {
            id[k]=index-j;
            t[k]=timediff(nav->peph[id[k]].time,time);
            pos=nav->peph[id[k]].pos[sat-1];
            if (norm(pos,3)>0.0) {
                posWithEarhRotation(k,pos,p,t[k]);
                k++;
            }
        }
        if (k==NMAX+1) break;
    }
    if (k<=NMAX) return 0;

    for (i=0;i<=NMAX;i++) {
        for (j=i+1;j<=NMAX;j++) {
            if (t[i]<=t[j]) continue;
            sinl=t[j]; t[j]=t[i];   t[i]=sinl;
            k=id[j];   id[j]=id[i]; id[i]=k;
            for (k=0;k<3;k++) {
                sinl=p[k][j];
                p[k][j]=p[k][i];
                p[k][i]=sinl;
            }
        }
    }

    kInd=0;
    for (i=0;i<=NMAX;i++) {
        if (fabs(t[kInd])<=fabs(t[i])) kInd=i;
    }
    index=id[kInd];

    if (t[0]>900.0||t[NMAX]<-900.0) {
        sprintf(PPP_Glo.chMsg,"%s t[0]=%-5.1f t[%d]=%-5.1f\n",PPP_Glo.sFlag[sat-1].id,t[0],NMAX,t[NMAX]);
        outDebug(0,0,0);
        return 0;
    }
    
    // 内维尔多项式插值获取卫星位置
    for (i=0;i<3;i++) {
        rs[i]=interppol(t,p[i],NMAX+1);
    }

    if (vare) {
        for (i=0;i<3;i++) s[i]=nav->peph[index].std[sat-1][i];
        std=norm(s,3);
        
        /* extrapolation error for orbit */
        if      (t[0   ]>0.0) std+=EXTERR_EPH*SQR(t[0   ])/2.0;
        else if (t[NMAX]<0.0) std+=EXTERR_EPH*SQR(t[NMAX])/2.0;
        *vare=SQR(std);
    }

    // 线性插值获取钟差
    /* linear interpolation for clock */
    t[0]=timediff(time,nav->peph[index  ].time);
    t[1]=timediff(time,nav->peph[index+1].time);
    c[0]=nav->peph[index  ].pos[sat-1][3];
    c[1]=nav->peph[index+1].pos[sat-1][3];
    
    // 计算标准差
    bBadClk=0;
    if (t[0]<=0.0) {
        if ((dts[0]=c[0])==0.0) bBadClk=1;
        std=nav->peph[index].std[sat-1][3]*CLIGHT-EXTERR_CLK*t[0];
    }
    else if (t[1]>=0.0) {
        if ((dts[0]=c[1])==0.0) bBadClk=1;
        std=nav->peph[index+1].std[sat-1][3]*CLIGHT+EXTERR_CLK*t[1];    
    }
    else if (c[0]!=0.0&&c[1]!=0.0) {
        dts[0]=(c[1]*t[0]-c[0]*t[1])/(t[0]-t[1]);
        i=t[0]<-t[1]?0:1;
        std=nav->peph[index+i].std[sat-1][3]+EXTERR_CLK*fabs(t[i]);
    }
    else {
        bBadClk=1;
    }
    if (varc) *varc=SQR(std);

    sys=PPP_Glo.sFlag[sat-1].sys;
    if (sys==SYS_CMP&&bBadClk) return 1;

    if (bBadClk) {
        //return 0;
    }
    return 1;
}
```

#### 3. interppol()：Neville 插值

Neville 算法是一种计算插值多项式方法，由给定的 n+1个节点，存在一个唯一的幂次 ≤n 的多项式存在，并且通过给定点；所以可以由两个 n-1 次插值多项式构造一个 n 次多项式的线性逐次插值。给定 $\mathrm{n}+1$ 个节点及其对应函数值 $\left(x_{i}, y_{i}\right)$ ，假设 $P_{i, j}$ 表示 $j-i$ 阶多项式，并且满足通过节点 $\left(x_{k}, y_{k}\right) \quad k=i, i+1, \cdots, j$ 。 $P_{i, j}$ 满足以下迭代关系：
$$
\begin{array}{l}
p_{i, i}(x)=y_{i} \\
P_{i, j}(x)=\frac{\left(x_{j}-x\right) p_{i, j-1}(x)+\left(x-x_{i}\right) p_{i+1, j}(x)}{x_{j}-x_{i}}, \quad 0 \leq i \leq j \leq n
\end{array}
$$
以 $n=4$ 的节点举例，其迭代过程为：
$$
\begin{array}{l}
p_{1,1}(x)=y_{1} \\
p_{2,2}(x)=y_{2}, p_{1,2}(x) \\
p_{3,3}(x)=y_{3}, p_{2,3}(x), p_{1,3}(x) \\
p_{4,4}(x)=y_{4}, p_{3,4}(x), p_{2,4}(x), p_{1,4}(x)
\end{array}
$$

代码中的主要部分是一个双重循环。外层循环变量`j`从1开始到`n-1`，内层循环变量`i`从0开始到`n-j-1`。每次内层循环结束后，`y[i]`就会被更新为新的插值结果。

```c
static double interppol(const double *x, double *y, int n)
{
    int i,j;
    
    for (j=1;j<n;j++) {
        for (i=0;i<n-j;i++) {
            y[i]=(x[i+j]*y[i]-x[i]*y[i+1])/(x[i+j]-x[i]);
        }
    }
    return y[0];
}
```

#### 4. posWithEarhRotation()：地球旋转改正后的位置

* 首先，定义了两个变量 `sinl` 和 `cosl`，这两个变量分别表示在给定的时间间隔内，地球自转的角度的正弦值和余弦值。这里 `OMGE` 是地球的角速度，乘以 `dt` 就得到了这段时间内地球自转的角度。

* 然后，根据地球自转的公式，利用正弦和余弦函数，对物体的初始位置 `pos[0]` 和 `pos[1]` 进行修正，得到物体在考虑地球自转后的新位置 `p[0][k]` 和 `p[1][k]`。

* 最后，由于地球的自转并不影响物体在垂直于地球表面的方向的位置，所以 `p[2][k]` 直接等于 `pos[2]`。

```c
static void posWithEarhRotation(const int k, double pos[3], double p[3][NMAX+1], double dt)
{
	double sinl,cosl;
#if 0
	p[0][k]=pos[0];
	p[1][k]=pos[1];
#else
	/* correciton for earh rotation ver.2.4.0 */
	sinl=sin(OMGE*dt);
	cosl=cos(OMGE*dt);
	p[0][k]=cosl*pos[0]-sinl*pos[1];
	p[1][k]=sinl*pos[0]+cosl*pos[1];
#endif
	p[2][k]=pos[2];
}
```

#### 5. pephclk()：精密钟差计算卫星钟差

简单的线性插值：
$$
d T^{s}(t)=\frac{\left(t_{i+1}-t\right) d T^{s}\left(t_{i}\right)+\left(t-t_{i}\right) d T^{s}\left(t_{i+1}\right)}{t_{i+1}-t_{i}}
$$
IGS 的精密钟差计算完之后，需要考虑相对论效应的影响，减去后面一项：
$$
d T^{s}(t)=\frac{\left(t_{i+1}-t\right) d T^{s}\left(t_{i}\right)+\left(t-t_{i}\right) d T^{s}\left(t_{i+1}\right)}{t_{i+1}-t_{i}}-2 \frac{\boldsymbol{r}^{s}(t)^{T} \boldsymbol{v}^{s}(t)}{c^{2}}
$$

代码与 `pephpos()` 类似，先找到时间最近的精密星历下标，线性插值得钟差，再计算方差。

```c
static int pephclk(gtime_t time, int sat, const nav_t *nav, double *dts,
                   double *varc)
{
    double t[2],c[2],std;
    int i,j,k,index;
    
    if (nav->nc<2||timediff(time,nav->pclk[0].time)<-MAXDTE||
        timediff(time,nav->pclk[nav->nc-1].time)>MAXDTE)
        return 1;

    // 二分查找 nav->peph[] 中时间差最接近的精密星历的下标 index
    /* binary search */
    for (i=0,j=nav->nc-1;i<j;) {
        k=(i+j)/2;
        if (timediff(nav->pclk[k].time,time)<0.0) i=k+1; else j=k;
    }
    index=i<=0?0:i-1;
    
    // 钟差线性插值
    /* linear interpolation for clock */
    t[0]=timediff(time,nav->pclk[index  ].time);
    t[1]=timediff(time,nav->pclk[index+1].time);
    c[0]=nav->pclk[index  ].clk[sat-1][0];
    c[1]=nav->pclk[index+1].clk[sat-1][0];

    for (i=index;i>=0;i--) {
        if (nav->pclk[i].clk[sat-1][0]!=0.0) {
            t[0]=timediff(time,nav->pclk[i].time);
            c[0]=nav->pclk[i].clk[sat-1][0];
            break;
        }
    }

    for (i=index+1;i<nav->nc;i++) {
        if (nav->pclk[i].clk[sat-1][0]!=0.0) {
            t[1]=timediff(time,nav->pclk[i].time);
            c[1]=nav->pclk[i].clk[sat-1][0];
            index=i-1;
            break;
        }
    }

    if (t[0]<=0.0) {
        if ((dts[0]=c[0])==0.0) return 0;
        std=nav->pclk[index].std[sat-1][0]*CLIGHT-EXTERR_CLK*t[0];
    }
    else if (t[1]>=0.0) {
        if ((dts[0]=c[1])==0.0) return 0;
        std=nav->pclk[index+1].std[sat-1][0]*CLIGHT+EXTERR_CLK*t[1];
    }
    else if (c[0]!=0.0&&c[1]!=0.0) {
        dts[0]=(c[1]*t[0]-c[0]*t[1])/(t[0]-t[1]);
        i=t[0]<-t[1]?0:1;
        std=nav->pclk[index+i].std[sat-1][0];

        if (std*CLIGHT>0.05) std=std+EXTERR_CLK*fabs(t[i]);
        else                 std=std*CLIGHT+EXTERR_CLK*fabs(t[i]);
    }
    else  {
        return 0;
    }

    if (varc) *varc=SQR(std);
    return 1;
}
```

