> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1698494049(1).png)

[TOC]

## 一、SPP 解算

* 状态向量 x：8 维



### 1、spp()：单点定位主入口函数

![image-20230929100318941](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100318941.png)

默认使用广播星历计算卫星位置、钟差，使用克罗布歇模型通过广播星历中的参数计算电离层延迟，使用 Saastamoinen 模型计算对流层延迟。

* 调用 `satposs_rtklib()` 计算卫星位置、卫星钟差：
  * `rs[(0:2)+i*6]`：obs[i] sat position {x,y,z} (m) 
  * `rs [(3:5)+i*6]`：obs[i] sat velocity {vx,vy,vz} (m/s) 
  * `dts[(0:1)+i*2]`：obs[i] sat clock {bias,drift} (s|s/s) 
  * `var[i] `：卫星位置和钟差的协方差 (m^2) 
  * `svh[i]` :卫星健康标志 (-1:correction not available) 

* 调用 `estpos()` 计算接收机位置：加权最小二乘，其中会调用 valsol 进行卡方检验和GDOP检验。
* 存入方位角和俯仰角 ，赋值解算状态结构体 ssat。





### 2、estpos() 

![image-20231028085159260](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028085159260.png)





### 3、estpose_()

![image-20231028085015054](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028085015054.png)

先给矩阵开辟内存空间，待估参数 dx 赋值为 0，然后进入最小二乘迭代求解：

* 先调用 `rescode()` 计算当前迭代的伪距残差 v、设计矩阵 H、伪距残差的方差 var、所有观测卫星的方位角和仰角 azel，定位时有效性 vsat、定位后伪距残差 resp、参与定位的卫星个数` ns`和方程个数` nv`。

* 然后对 H、V 协方差加权，以 varerr() 计算出的伪距残差的标准差的倒数作为权重，对`H`和`v`分别左乘权重对角阵，得到加权之后的 `H` 和 `v `。
  $$
  \begin{array}{l}\boldsymbol{W}=\operatorname{diag}\left(\sigma_{1}^{-2}, \sigma_{2}^{-2}, \ldots, \sigma_{m}^{-2}\right) \\ \sigma^{2}=F^{s} R_{r}\left(a_{\sigma}{ }^{2}+b_{\sigma}{ }^{2} / \sin E l_{r}^{s}\right)+{\sigma_{\text {eph }}}^{2}+{\sigma_{\text {ion }}}^{2}+{\sigma_{\text {trop }}}^{2}+{\sigma_{\text {bias }}}^{2}\end{array}
  $$

* 调用 `lsqPlus()` 最小二乘解算，计算状态 `x` 的增量 `dx`

* 最后状态反馈，把 `dx` 置加到 `x` 上

结束迭代有两种情况：

* 最小二乘求解后，dx 的模 d0 小于 1E-4 ，存结果到 sol，调用 `valsol()` 卡方和 GDOP 检验结果有效，结果状态设为单点解。

* 超出迭代次数上限（10次），不存结果，输出迭代发散的消息。

```c
static int estpos_(int *bDeleted, double *x,const obsd_t *obs, int n, const double *rs, const double *dts, const double *vare, 
                   const int *svh, const nav_t *nav, const prcopt_t *opt, sol_t *sol, double *azel, int *vsat, double *resp, char *msg)
{
    int i,j,k,info,stat=0,nx,nv,bElevCVG;
    double dx[NX_SPP],Q[NX_SPP*NX_SPP],dop[4],*v,*H,*var,sig,d0;

    bElevCVG=0;

    v=mat(n+5,1); H=mat(NX_SPP,n+5); var=mat(n+5,1);

    for (i=0;i<NX_SPP;i++) dx[i]=0.0;

    for (i=0;i<MAXITR;i++) {

        // 调用 rescode() 计算设计矩阵 H、残差 V
        /* pseudorange residuals */
        nv=rescode(i,bElevCVG,obs,n,rs,dts,vare,svh,nav,x,opt,v,H,var,azel,vsat,resp,&nx,bDeleted);

        if (nv<nx) {
            sprintf(msg,"lack of valid sats ns=%d",nv);
            break;
        }

        // H、V 协方差加权
        /* weight by variance */
        for (j=0;j<nv;j++) {
            sig=sqrt(var[j]);
            v[j]/=sig;
            for (k=0;k<NX_SPP;k++) H[k+j*NX_SPP]/=sig;
        }

        // 调用 lsqPlus() 最小二乘解算
        /* least square estimation */
        if ((info=lsqPlus(H,v,NX_SPP,nv,dx,Q))) {
            sprintf(msg,"lsq error info=%d",info);
            sprintf(PPP_Glo.chMsg, "%s\n",msg);
            outDebug(OUTWIN,OUTFIL,OUTTIM);

            break;
        }

        // 状态反馈
        for (j=0;j<NX_SPP;j++) x[j]+=dx[j];

        d0=norm(dx,NX_SPP);

        if (d0<1E4) bElevCVG=1;
        else        bElevCVG=0;

        if (d0<1E-4) {
            sol->type=0;
            sol->time=timeadd(obs[0].time,-x[3]/CLIGHT);    // 加上钟差
            sol->dtr[0]=x[3]/CLIGHT; /* receiver clock bias (s) */
            sol->dtr[1]=x[4]/CLIGHT; /* glo-gps time offset (s) */
            sol->dtr[2]=x[5]/CLIGHT; /* bds-gps time offset (s) */
            sol->dtr[3]=x[6]/CLIGHT; /* gal-gps time offset (s) */
			sol->dtr[4]=x[7]/CLIGHT; /* qzs-gps time offset (s) */
            for (j=0;j<6;j++) sol->rr[j]=j<3?x[j]:0.0;
            for (j=0;j<3;j++) sol->qr[j]=(float)Q[j+j*NX_SPP];
            sol->qr[3]=(float)Q[1];    /* cov xy */
            sol->qr[4]=(float)Q[2+NX_SPP]; /* cov yz */
            sol->qr[5]=(float)Q[2];    /* cov zx */
            sol->ns[0]=(unsigned char)nv;
            sol->rms=sol->dop[0]=sol->dop[1]=sol->dop[2]=sol->dop[3]=0.0;

            // 调用 valsol() 卡方和 GDOP 检验结果有效性
            /* validate solution */
            if ((stat=valsol(azel,vsat,n,opt,v,nv,nx,msg,dop)))
                sol->stat=SOLQ_SINGLE;

            sol->dop[0]=dop[0];
            sol->dop[1]=dop[1];
            sol->dop[2]=dop[2];
            sol->dop[3]=dop[3];

            break;
        }
    }
    if (i>=MAXITR) sprintf(msg,"iteration divergent i=%d",i);

    free(v); free(H); free(var);
    return stat;
}
```









### 4、valsol()

> 低成本接收机可能通不过检验，可禁用此函数

```c
const double *azel  方位角、高度角
const int *vsat     观测卫星在当前定位时是否有效 (1*n)
int n               观测值个数
const prcopt_t *opt 处理选项
const double *v     定位方程的右端部分，伪距残差
int nv              观测值数
int nx              待估计参数数
char *msg           错误消息
```


$$
\begin{array}{l}v_{s}=\frac{\left(P_{r}^{s}-\left(\hat{\rho}_{r}^{s}+c \hat{d} t_{r}-c d T^{s}+I_{r}^{s}+T_{r}^{s}\right)\right)}{\sigma_{s}} \\ \boldsymbol{v}=\left(v_{1}, v_{2}, v_{3}, \ldots, v_{m}\right)^{T} \\ \frac{\boldsymbol{v}^{T} \boldsymbol{v}}{m-n-1}<\chi_{\alpha}^{2}(m-n-1) \\ G D O P<G D O P_{\text {thres }}\end{array}
$$




## 二、resode()：残差计算、设计矩阵构建

计算当前迭代的伪距残差 v、设计矩阵 H、伪距残差的方差 var、所有观测卫星的方位角和仰角 azel，定位时有效性 vsat、定位后伪距残差 resp、参与定位的卫星个数 ns 和方程个数 nv 

```c
int      iter      I   迭代次数，在estpos()里迭代调用，第i次迭代就传i
int      bElevCVG  I	
obsd_t   *obs      I   观测量数据
int      n         I   观测量数据的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *dts      I   卫星钟差，长度为2*n， {bias,drift} (s|s/s)
double   *vare     I   卫星位置和钟差的协方差 (m^2)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   导航数据
double   *x        I   本次迭代开始之前的定位值,7*1,前3个是本次迭代开始之前的定位值，第4个是钟差，后三个分别是gps系统与glonass、galileo、bds系统的钟差。
prcopt_t *opt      I   处理过程选项
double   *v        O   定位方程的右端部分，伪距残差
double   *H        O   定位方程中的几何矩阵
double   *var      O   参与定位的伪距残差的方差
double   *azel     O   对于当前定位值，所有观测卫星的 {方位角、高度角} (2*n)
int      *vsat     O   所有观测卫星在当前定位时是否有效 (1*n)
double   *resp     O   所有观测卫星的伪距残差，(P-(r+c*dtr-c*dts+I+T)) (1*n)
int      *ns       O   参与定位的卫星的个数
int      *bDeleted O   
```



![image-20231028084841720](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028084841720.png)

* 将之前得到的定位解信息赋值给`rr`和`dtr`数组。

* 调用`ecef2pos()`将将接收机位置`rr`由 ECEF-XYZ 转换为大地坐标系LLH`pos`

* 遍历当前历元所有`OBS[]`，即遍历每颗卫星：

  * 将`vsat[]`、`azel[]`和`resp[]`数组置 0，因为在前后两次定位结果中，每颗卫星的上述信息都会发生变化。`time`赋值OBS的时间，`sat`赋值OBS的卫星。

  * 检测当前观测卫星是否和下一个相邻数据重复；重复则不处理这一条，continue去处理下一条。

  * 调用`satexclude()`函数判断卫星是否需要排除，如果排除则continue去处理下一个卫星。

  * 调用`geodist()`函数，计算卫星和当前接收机位置之间的几何距离`r`和接收机到卫星的方向向量`e`。 

  * 调用`satazel()`函数，计算在接收机位置处的站心坐标系中卫星的方位角和仰角；若仰角低于截断值`opt->elmin`，continue不处理此数据。

  * 调用`snrmask()`，根据接收机高度角和信号频率来检测该信号是否可用。

  * 调用` ionocorr()` 函数，计算电离层延时`I`,所得的电离层延时是建立在 L1 信号上的，当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的比率，对上一步得到的电离层延时进行修正。 

  * 调用`tropcorr()`函数,计算对流层延时`T`。

  * 调用`prange()`函数，计算经过DCB校正后的伪距值`p`。

  * 计算伪距残差`v[nv]`，即经过钟差，对流层，电离层改正后的伪距。

  * 组装设计矩阵`H`
    $$
    \boldsymbol{h}(\boldsymbol{x})=\left(\begin{array}{c}\rho_{r}^{1}+c d t_{r}-c d T^{1}+I_{r}^{1}+T_{r}^{1} \\ \rho_{r}^{2}+c d t_{r}-c d T^{2}+I_{r}^{2}+T_{r}^{2} \\ \rho_{r}^{3}+c d t_{r}-c d T^{3}+I_{r}^{3}+T_{r}^{s 3} \\ \vdots \\ \rho_{r}^{m}+c d t_{r}-c d T^{m}+I_{r}^{m}+T_{r}^{m}\end{array}\right) \boldsymbol{H}=\left(\begin{array}{cc}-e_{r}^{1 T} & 1 \\ -e_{r}^{2 T} & 1 \\ -e_{r}^{3 T} & 1 \\ \vdots & \vdots \\ -e_{r}^{m T} & 1\end{array}\right)
    $$

  * 处理不同系统（GPS、GLO、GAL、CMP）之间的时间偏差，修改矩阵`H `。

  * 调用`varerr()`函数，计算此时的导航系统误差

  * 为了防止不满秩的情况，把矩阵`H`补满秩了，`H[j+nv*NX]=j==i+3?1.0:0.0; `









### 1、geodist()：计算近似几何距离、接收机到卫星方向的观测矢量，sagnac 效应改正

地球自转引起的误差是信号传输过程中 GNSS 卫星信号发射时刻和接收机接收到信号的时刻之间地球自转对 GNSS 观测值产生的影响。相当于地球自转使得卫星空间位置在信号播发、收到的过程中接收机在地固系坐标轴上相对于 $\mathrm{Z}$ 轴发生了一定角度的旋转，使得 GNSS 卫星的在信号发射时刻的位置发生了变化，也称为 Sagnac 效应。

![image-20231028162356346](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028162356346.png)

地球自转引起的距离改正公式如下：
$$
\begin{array}{c}{\left[\begin{array}{l}x^{s^{\prime}} \\ y^{s^{s^{\prime}}} \\ z^{s^{\prime}}\end{array}\right]=\left[\begin{array}{ccc}\cos \omega \tau & \sin \omega \tau & 0 \\ -\sin \omega \tau & \cos \omega \tau & 0 \\ 0 & 0 & 1\end{array}\right]\left[\begin{array}{l}x^{s} \\ y^{s} \\ z^{s}\end{array}\right]} \\ \delta_{\text {sagnac, } r, j}=\frac{\omega_{e}}{c}\left(x^{s} y_{r}-y^{s} x_{r}\right)\end{array}
$$
上式中 $\left(x^{s^\prime}, y^{s^{\prime}}, z^{s^{\prime}}\right)$ 为地球自旋转后卫星的坐标值, $\left(x^{s}, y^{s}, z^{s}\right)$ 为地球自旋转前卫星的坐标值, $\omega_{e}$ 代表地球自传角速度值, $\tau$ 为卫星发射信号时刻到接收机接收卫星时刻的历元数。

改正地球自转后的近似几何距离近似几何距离如下：
$$
\begin{array}{l} \rho_{r}^{s} \approx\left\|\boldsymbol{r}_{r}\left(t_{r}\right)-\boldsymbol{r}^{s}\left(t^{s}\right)\right\|+\frac{\omega_{e}}{c}\left(x^{s} y_{r}-y^{s} x_{r}\right) \end{array}
$$
接收机到卫星方向的观测矢量计算公式如下：
$$
\boldsymbol{e}_{r}^{s}=\frac{\boldsymbol{r}^{s}\left(t^{s}\right)-\boldsymbol{r}_{r}\left(t_{r}\right)}{\left\|\boldsymbol{r}^{s}\left(t^{s}\right)-\boldsymbol{r}_{r}\left(t_{r}\right)\right\|}
$$
对应的代码如下，传入 ECEF 卫星坐标 `rs`、接收机近似 ECEF 坐标 `rr`，计算之后近似几何距离做返回值返回，观测矢量做参数三 `e` 返回：

```c
 extern double geodist(const double *rs, const double *rr, double *e)
 {
     double r;
     int i;
     
     if (norm(rs,3)<RE_WGS84) return -1.0;   // 检查卫星到 WGS84坐标系原点的距离是否大于基准椭球体的长半径。
     for (i=0;i<3;i++) e[i]=rs[i]-rr[i];     // 求卫星和接收机坐标差e[]
     r=norm(e,3);                            // 求未经萨格纳克效应改正的距离
     for (i=0;i<3;i++) e[i]/=r;  // 接收机到卫星的单位向量e[]	(E.3.9)
     return r+OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT; 	//(E.3.8b)
 }
```

### 2、satazel()：计算方位角高度角

![image-20231028163827373](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028163827373.png)

方位角范围在 $[0,2 \pi]$，高度角范围在 $[-\frac{\pi}{2},\frac{\pi}{2}]$；以接收机为原点，建立站心坐标系 ENU，根据卫星 ENU 下方向矢量可以得到高度角、方位角，公式如下：
$$
\begin{array}{l}\boldsymbol{e}_{r, \text { enu }}^{s}=\boldsymbol{E}_{r} \boldsymbol{e}_{r}^{s}=\left(e_{e}, e_{n}, e_{u}\right)^{T} \\ A z_{r}^{s}=\operatorname{ATAN} 2\left(e_{e}, e_{n}\right) \\ E l_{r}^{s}=\arcsin \left(e_{u}\right)\end{array}
$$
对应的代码如下，传入接收机 LLH 坐标 `pos`、接收机到卫星方向的观测矢量 `e`，计算之后返回弧度制的高度角，如果传了参三 `azel`，那么 `azel[0]` 是方位角、`azel[1]` 是高度角。

```c
extern double satazel(const double *pos, const double *e, double *azel)
{
    double az=0.0,el=PI/2.0,enu[3];
    
    if (pos[2]>-RE_WGS84) {
        ecef2enu(pos,e,enu);
        az=dot(enu,enu,2)<1E-12?0.0:atan2(enu[0],enu[1]);
        if (az<0.0) az+=2*PI;
        el=asin(enu[2]);
    }
    if (azel) {azel[0]=az; azel[1]=el;}
    return el;
}
```

### 3、prange()：差分码偏差改正

DCB 差分码偏差，针对伪距，是由不同类型的 GNSS 信号在卫星和接收机不同通道产生的时间延迟（硬件延迟／码偏差）差异 。由于卫星播发的测距码类型很多， C1、 P1、 P2 等 ，不同的测距信号虽然在同一台卫星钟的驱动下生成的，因而花费的时间也不同。我们把卫星钟脉冲驱动下开始生成测距信号至信号生成并最终离开卫星发射天线相位中心之间所花费的时间称为信号在卫星内部的时延。DCB体现的就是不同码信号时延的差。分为：

* **频内偏差**：相同频率不同码之间存在的偏差（如P1-C1、P2-C2等）
 * **频间偏差**：不同频率之间存在的偏差（如P1-P2）



### 4、satexclude()：排除不可用卫星的观测值

* 卫星健康标志 `svh` < 0，排除。
* 根据 `opt->exsats[sat-1]` 判断是否使用了该卫星。
* 根据 `opt->navsys` 判断是否使用了该卫星系统。

```c
extern int satexclude(int sat, int svh, const prcopt_t *opt)
{
	int sys=PPP_Glo.sFlag[sat-1].sys;
    
    // 卫星健康标志 svh < 0：排除
    if (svh<0) return 1; /* ephemeris unavailable */
    
    if (opt) {
        // 根据 opt->exsats[sat-1] 判断是否使用了该卫星
        if (opt->exsats[sat-1]==1) return 1; /* excluded satellite */
        if (opt->exsats[sat-1]==2) return 0; /* included satellite */

        // 根据 opt->navsys 判断是否使用了该卫星系统
        if (!(sys&opt->navsys)) return 1; /* unselected sat sys */
    }
    if (sys==SYS_QZS) svh&=0xFE; /* mask QZSS LEX health */
    if (svh) {
		sprintf(PPP_Glo.chMsg,"*** WARNING: unhealthy satellite: sat=%3d svh=%02X\n",sat,svh);
		outDebug(0,0,0);
        return 1;
    }
    return 0;
}
```

### 5、ionocorr()：电离层改正

GNSS 电磁波信号在传播过程中会受到电离层的干扰，产生电离层延迟（也称延时），因为电离层中存在相当多的自由电子和离子，能使无线电波改变传播速度，发生折射、反射和散射，产生极化面的旋转并受到不同程度的吸收。 电离层延迟与单位面积的横截面在信号传播路径上拦截的电子总量 $N_e$ 成正比，且与载波频率 $f$ 的平方成反比，弥散性的电离层降低了测距码的传播速度，而加快了载波相位的传播速度，如下所示：
$$
I=I_ \rho=-I_\phi=40.28\frac{N_e}{f^2}
$$
可以总结出电离层几个对于解算有用的特点：

* **对伪距和载波影响相反**：UOFC 组合，但伪距载波之间延迟
* **与频率有关**：消电离层组合，但放大噪声
* **延迟与电子总量 TEC 成正比**：TEC 文件电离层改正，作为参数估计（参数太多）



卫星在其播发的导航电文中提供了 8 个电离层延迟参数：
$$
p_{\text {ion }}=\left(\alpha_{0}, \alpha_{1}, \alpha_{2}, \alpha_{3}, \beta_{0}, \beta_{1}, \beta_{2}, \beta_{3}\right)^{T}
$$
根据根据参数 $\alpha_0，\alpha_1，\alpha_2，\alpha_3$ 确定振幅 $A$，根据根据参数 $\beta_0，\beta_1，\beta_2，\beta_3$ 确定周期 $T$，再给定一个以秒为单位的当地时间 $t$，就能算出**天顶电离层延迟**。再由天顶对流层延迟根据倾斜率转为卫星方向对流层延迟。
$$
\begin{array}{l}\psi=0.0137 /(E l+0.11)-0.022 \\ \varphi_{i}=\varphi+\psi \cos A z \\ \lambda_{i}=\lambda+\psi \sin A z / \cos \varphi_{i} \\ \varphi_{m}=\varphi_{i}+0.064 \cos \left(\lambda_{i}-1.617\right) \\ t=4.32 \times 10^{4} \lambda_{i}+t \\ F=1.0+16.0 \times(0.53-E l)^{3} \\ x=2 \pi(t-50400) / \sum_{n=0}^{3} \beta_{n} \varphi_{m}{ }^{n} \\ I_{r}^{s}=\left\{\begin{array}{cc}F \times 5 \times 10^{-9} \\ F \times\left(5 \times 10^{-9}+\sum_{n=1}^{4} \alpha_{n} \varphi_{m}{ }^{n} \times\left(1-\frac{x^{2}}{2}+\frac{x^{4}}{24}\right)\right) & (|x|>1.57)\end{array}\right.\end{array}
$$






计算的是 L1 信号的电离层延时 I ，当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的比例关系，对上一步得到的电离层延时进行修正。





### 6、tropcorr()：对流层改正

与电离层延迟对应，对流层延迟一般指非电离大气对电磁波的折射，折射效应大部分发生在对流层，因此称为对流层延迟误差。对流层延迟影响与信号高度角有关，天顶方向影响能够达到2.3m，而高度角较小时，其影响量可达 20m。对流层可视为非弥散介质，其折射率 $n$ 与电磁波频率 $f$ 无关，于是 GPS 信号的群速率和相速率在对流层相等，无法使用类似消电离层的方法消除。对流层延迟可以分为干延迟和湿延迟两部分，总的对流层延迟可以根据天顶方向干湿延
迟分量及其投影系数确定：
$$
d_{trop}=d_{zpd}M_d(\theta)+d_{zpw}M_w(\theta)
$$
可以总结出电离层几个对于解算有用的特点：

* **非弥散介质**：对所有卫星的信号影响相同。
* **分干湿延迟**：
* 


$$
\begin{array}{l}p=1013.25 \times\left(1-2.2557 \times 10^{-5} h\right)^{5.2568} \\ T=15.0-6.5 \times 10^{-3} h+273.15 \\ e=6.108 \times \exp \left\{\frac{17.15 T-4684.0}{T-38.45}\right\} \times \frac{h_{r e l}}{100}\end{array}
$$

$$
T_{r}^{s}=\frac{0.002277}{\cos z}\left\{p+\left(\frac{1255}{T}+0.05\right) e-\tan ^{2} z\right\}
$$


其中 $z$ 是天顶角，与高度角互余：$z=\pi / 2-E l_{r}^{s}$





### 7、varerr()：计算伪距量测协方差

$$
\begin{array}{l} \sigma^{2}=F^{s} R_{r}\left(a_{\sigma}{ }^{2}+b_{\sigma}{ }^{2} / \sin E l_{r}^{s}\right)+{\sigma_{\text {eph }}}^{2}+{\sigma_{\text {ion }}}^{2}+{\sigma_{\text {trop }}}^{2}+{\sigma_{\text {bias }}}^{2}\end{array}
$$

其中：

* $F^{s}$：卫星系统误差因子，GLONASS 1.5，SBAS fact 3，其它 1。
* $R_{r}$：
* $a_{\sigma}, b_{\sigma}$：
* $\sigma_{e p h}$：
* $\sigma_{\text {ion }}$：
* $\sigma_{\text {trop }}$：
* $\sigma_{\text {bias }}$：

`varerr()` 函数只计算了第一项：

```c
static double varerr(const prcopt_t *opt, double el, int sys)
{
    double fact,varr;

    fact=sys==SYS_GLO?PPP_Glo.prcOpt_Ex.errRatioGLO:(sys==SYS_CMP?PPP_Glo.prcOpt_Ex.errRatioBDS:
		(sys==SYS_GAL?PPP_Glo.prcOpt_Ex.errRatioGAL:(sys==SYS_QZS?PPP_Glo.prcOpt_Ex.errRatioQZS:opt->err[0])));
    varr=(SQR(opt->err[1])+SQR(opt->err[2])/sin(el));   // sin(el) 多了方，疑似bug

    if (opt->ionoopt==IONOOPT_IF12) varr*=SQR(3.0); /* iono-free */ //消电离层组合，方差*3*3
    return SQR(fact)*varr;
}
```

后面的对流层、电离层、星历误差在 `rescode()` 中加入：

```c
var[nv]=varerr(opt,azel[1+i*2],sys)+vare[i]+vion+vtrp;
```

### 8、getHVR_spp





## 三、lsqplus()：最小二乘估计











