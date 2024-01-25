> 原始 Markdown 文档、Visio 流程图、XMind 思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、SPP 解算

### 1、spp()：单点定位主入口函数

![image-20230929100318941](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230929100318941.png)

```c
obsd_t *obs      I   observation data            OBS观测数据
int    n         I   number of observation data  OBS数
nav_t  *nav      I   navigation data             NAV导航电文数据
prcopt_t *opt    I   processing options          处理过程选项
sol_t  *sol      IO  solution                    结果
double *azel     IO  azimuth/elevation angle (rad) (NULL: no output)     方位角和俯仰角
ssat_t *ssat     IO  satellite status              (NULL: no output)     卫星状态
char   *msg      O   error message for error exit
```

默认使用广播星历计算卫星位置、钟差，使用克罗布歇模型通过广播星历中的参数计算电离层延迟，使用 Saastamoinen 模型计算对流层延迟。

* 调用 `satposs_rtklib()` 计算卫星位置、卫星钟差：
  * `rs[(0:2)+i*6]`：卫星位置 {x,y,z} (m) 
  * `rs [(3:5)+i*6]`：卫星速度 {vx,vy,vz} (m/s) 
  * `dts[(0:1)+i*2]`：卫星钟差 {bias,drift} (s|s/s) 
  * `var[i] `：卫星位置和钟差的协方差 (m^2) 
  * `svh[i]`：卫星健康标志 (-1:correction not available) 

* 调用 `estpos()` 计算接收机位置：加权最小二乘，其中会调用 valsol 进行卡方检验和GDOP检验。
* 存入方位角和俯仰角 ，赋值解算状态结构体 ssat。

```c
extern int spp(const obsd_t *obs, int n, const nav_t *nav,const prcopt_t *opt, 
	sol_t *sol, double *azel, ssat_t *ssat, char *msg)
{
    prcopt_t opt_=*opt;
    double *rs,*dts,*var,*azel_,*resp;
    int i,sat,stat,vsat[MAXOBS]={0},svh[MAXOBS];
    
    sol->stat=SOLQ_NONE;
    
    if (n<=0) {strcpy(msg,"no observation data"); return 0;}
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel_=zeros(2,n); resp=mat(1,n);

    // 处理选项赋值
    // 默认使用广播星历计算卫星位置、钟差
    // 使用克罗布歇模型通过广播星历中的参数计算电离层延迟
    // 使用 Saastamoinen 模型计算对流层延迟
    opt_.sateph =EPHOPT_BRDC;
    opt_.ionoopt=IONOOPT_BRDC;
    opt_.tropopt=TROPOPT_SAAS;

    // 调用 satposs_rtklib() 计算卫星位置、卫星钟差
    /* satellite positons, velocities and clocks */
    satposs_rtklib(obs[0].time,obs,n,nav,opt_.sateph,rs,dts,var,svh);
    
    // 调用 estpos() 计算接收机位置
    /* estimate receiver position with pseudorange */
    stat=estpos(obs,n,rs,dts,var,svh,nav,&opt_,sol,azel_,vsat,resp,msg);
    
    opt_.sateph =EPHOPT_BRDC;       // 使用广播星历计算卫星位置、钟差
    opt_.ionoopt=IONOOPT_BRDC;      // 使用克罗布歇模型通过广播星历中的参数计算电离层延迟
    opt_.tropopt=TROPOPT_SAAS;      // 使用 Saastamoinen 模型计算对流层延迟

    if (azel) {
        for (i=0;i<n*2;i++) azel[i]=azel_[i];
    }
    if (ssat) {
        for (i=0;i<MAXSAT;i++) {
            ssat[i].vs=0;
            ssat[i].azel[0]=ssat[i].azel[1]=0.0;
            ssat[i].resp_pos[0]=ssat[i].resc_pos[0]=0.0;
            ssat[i].snr[0]=0;
        }

        for (i=0;i<NSYS;i++) sol->ns[i]=0;

        for (i=0;i<n;i++) {
            if (!vsat[i]) continue;
            sat=obs[i].sat;
            ssat[sat-1].vs=1;
            ssat[sat-1].azel[0]=azel_[  i*2];
            ssat[sat-1].azel[1]=azel_[1+i*2];
            ssat[sat-1].resp_pos[0]=resp[i];
            ssat[sat-1].snr[0]=obs[i].SNR[0];

            if (PPP_Glo.sFlag[sat-1].sys==SYS_GPS) sol->ns[0]++;
            else if (PPP_Glo.sFlag[sat-1].sys==SYS_GLO) sol->ns[1]++;
            else if (PPP_Glo.sFlag[sat-1].sys==SYS_CMP) sol->ns[2]++;
            else if (PPP_Glo.sFlag[sat-1].sys==SYS_GAL) sol->ns[3]++;
			else if (PPP_Glo.sFlag[sat-1].sys==SYS_QZS) sol->ns[4]++;
            else { 
                sprintf(PPP_Glo.chMsg,"*** WARNING: unsupported satellite system %d %d!\n", PPP_Glo.sFlag[sat-1].sys,sat); 
                outDebug(OUTWIN,OUTFIL,OUTTIM);
            }
        }
    }

    free(rs); free(dts); free(var); free(azel_); free(resp);
    return stat;
}
```

### 2、estpos()

![image-20231028085159260](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028085159260.png)

```c
obsd_t   *obs      I   观测量数据
int      n         I   观测量数据的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *vare     I   卫星位置和钟差的协方差 (m^2)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   导航数据
prcopt_t *opt      I   处理过程选项
prcopt_t *opt      I   处理过程选项
sol_t    *sol      IO  solution
double   *azel     IO  方位角和俯仰角 (rad)
int      *vsat     IO  卫星在定位时是否有效
double   *resp     IO  定位后伪距残差 (P-(r+c*dtr-c*dts+I+T))
char     *msg      O   错误消息
```

这个函数是 GAMP 新加的，原本 `RTKLIB estpos()` 的内容都移到  `estpose_()` 中了；这个函数看着着实费劲，有段看半天没看明白，后来把代码复制给 AI，才知道那啥计算阶乘、组合数。

先初始化待估参数 `x`、`x`_：

* 把待估参数 `x`、`x_` 都赋值为 1。
* 如果给出了静态 PPP 模式的真实坐标 `PPP_Glo.crdTrue`，`x` 前三位直接赋值。
* 如果没有给出静态 PPP 精确坐标，计算当前时间（`PPP_Glo.tNow`）和解决方案时间（`sol->time`）之间的差值，取绝对值，并赋值给 dt；将 dt 除以采样间隔 PPP_Glo.sample 并四舍五入，将结果赋值给 `PPP_Glo.delEp`；用于判断 GNSS 观测值确时时间。
* 如果 `dt` 大于 1800，且 `delEp` 大于 100，或者 `x` 的模长小于 10，将 `x` 的前三位赋值 100；否则 `x` 的前三位赋值上一时刻的结果。

```c
int bDeleted[MAXSAT];
int i,j,stat=0,n,nb=0,*it,nMin=4;
double x[NX_SPP]={0},x_[NX_SPP],dt=0.0;

// 把待估参数都赋值为 0 
for (i=0;i<NX_SPP;i++) x[i]=x_[i]=1.0;
//for (i=0;i<MAXSAT;i++) bDeleted[i]=true;

// 如果没有给出静态 PPP 精确坐标
if (PPP_Glo.crdTrue[0]==0.0) {
    // 计算当前时间（PPP_Glo.tNow）和解决方案时间（sol->time）之间的差值，取绝对值，并赋值给 dt，
    dt=fabs(timediff(PPP_Glo.tNow,sol->time));  
    // 这行代码将 dt 除以 PPP_Glo.sample（可能是采样间隔）并四舍五入，然后将结果赋值给 PPP_Glo.delEp
    PPP_Glo.delEp=myRound(dt/PPP_Glo.sample);   

    // 如果 dt 大于 1800，且 delEp 大于 100，或者 x 的模长小于 10，将 x 的 XYZ 赋值 100；
    // 否则 x 的 XYZ 赋值上一时刻的结果
    if (dt>1800&&PPP_Glo.delEp>100) {
        for (i=0;i<3;i++) x[i]=100.0;
    }
    else {
		for (i=0;i<3;i++) x[i]=sol->rr[i];  
		if (norm(x,3)<=10) 
			for (i=0;i<3;i++) x[i]=100.0;
    }
}
else {
    for (i=0;i<3;i++) x[i]=PPP_Glo.crdTrue[i];
}

sol->time=obs[0].time;

for (i=0;i<NX_SPP;i++) x_[i]=x[i];
```

接下来一个 `for` 循环，是为了解算失败的时候排除卫星重新解算，类似 RAIM。其中 `nb` 表示随机排除的卫星数，`select_combination()` 生成排除卫星的组合，需要排除卫星的 `bDeleted[]` 对应项置 0，`rescode()` 中计算 `H`、`V` 的时候就不考虑了：

* 先不排除卫星计算，如果 `valsol()` 判断解有效(即stat=1)，直接结束循环。
* 然后 nb = 1，就是排除一颗卫星，解算成功，结束循环。
* 排除一颗卫星也不能解算成功，那就排除两颗、三颗，解算成功，结束循环。
* 排除三颗也解算不成功，那这个历元就是解算失败了。

不理解为啥写成这样随机排除卫星的，RTKLIB 的 RAIM-FDE 是排除残差最大的卫星重新计算。

```c
// 先不排除卫星计算，如果 valsol 判断解有效(即stat=1)，直接结束循环
// nb 表示随机排除的卫星数，select_combination() 生成排除卫星的组合，需要排除卫星的 bDeleted[] 对应项置 0，rescode() 中计算 H、V 的时候就不考虑了
for (nb=0;nb<=3;nb++) {
    
    // 卫星数少于 nmin(默认 4 颗) 无法计算，直接退出
    if (nobs-nb<nMin) {
        sprintf(msg,"lack of valid sats ns=%d/%d",nobs,nb);
        break;
    }

    // 计算从 nobs 中取 nb 的组合数：C(nobs, nb) = n! / [(n-nb)! * nb!]
    for (i=0,n=1;i<nb;i++)
        n=n*(nobs-i)/(i+1);     // 计算 n 的阶乘

	if (nb<=0) it=imat(n,1);
	else       it=imat(n*nb,1);

    comb_j=0;

    // 调用 select_combination() 
    select_combination(0,0,nobs,nb,it);
    //////////////////////////////////////////////////////////////////////////

    for (i=stat=0;i<n;i++) {
        for (j=0;j<nobs;j++) bDeleted[obs[j].sat-1]=1;
        for (j=i*nb;j<i*nb+nb;j++) bDeleted[obs[it[j]-1].sat-1]=0;
        for (j=0;j<NX_SPP;j++) x_[j]=x[j];

        // SPP 解算
        stat=estpos_(bDeleted,x_,obs,nobs,rs,dts,vare,svh,nav,opt,sol,azel,vsat,resp,msg);

        if (stat==1) break;
    }

    free(it);

    if (stat==1) break;
}
```

### 3、estpose_()

![image-20231028085015054](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028085015054.png)

```c
int *bDeleted      I   需要排除卫星的 bDeleted[] 对应项置 0，rescode() 中计算 H、V 的时候就不考虑了
obsd_t   *obs      I   观测量数据
int      n         I   观测量数据的数量
double   *rs       I   卫星位置和速度，长度为6*n，{x,y,z,vx,vy,vz}(ecef)(m,m/s)
double   *vare     I   卫星位置和钟差的协方差 (m^2)
int      *svh      I   卫星健康标志 (-1:correction not available)
nav_t    *nav      I   导航数据
prcopt_t *opt      I   处理过程选项
prcopt_t *opt      I   处理过程选项
sol_t    *sol      IO  solution
double   *azel     IO  方位角和俯仰角 (rad)
int      *vsat     IO  卫星在定位时是否有效
double   *resp     IO  定位后伪距残差 (P-(r+c*dtr-c*dts+I+T))
char     *msg      O   错误消息
```

先给矩阵开辟内存空间，待估参数 `dx` 赋值为 0，然后进入最小二乘迭代求解：

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

### 4、valsol()：GDOP和卡方检验结果有效性

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

```c
static int valsol(const double *azel, const int *vsat, int n,
                  const prcopt_t *opt, const double *v, int nv, int nx,
                  char *msg)
{
    double azels[MAXOBS*2],dop[4],vv;
    int i,ns;
    
    trace(3,"valsol  : n=%d nv=%d\n",n,nv);
    
    /* Chi-square validation of residuals */    //对残差卡方检验
    vv=dot(v,v,nv);		//chisqr:卡方值表
    if (nv>nx&&vv>chisqr[nv-nx-1]) {        //(E.6.33) 且观测值数大于待估计参数数  nv-nx-1:多余观测数
        sprintf(msg,"chi-square error nv=%d vv=%.1f cs=%.1f",nv,vv,chisqr[nv-nx-1]);
        return 0;
    }

    /* large GDOP check */  //GDOP检验
    for (i=ns=0;i<n;i++) {
        if (!vsat[i]) continue;
        azels[  ns*2]=azel[  i*2];
        azels[1+ns*2]=azel[1+i*2];
        ns++;
    }
    dops(ns,azels,opt->elmin,dop);              
    if (dop[0]<=0.0||dop[0]>opt->maxgdop) {     //(E.6.34)
        sprintf(msg,"gdop error nv=%d gdop=%.1f",nv,dop[0]);
        return 0;
    }
    return 1;
}
```

## 二、rescode()：残差计算、设计矩阵构建

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
int      *bDeleted IO   需要排除卫星的 bDeleted[] 对应项置 0，rescode() 中计算 H、V 的时候就不考虑了
```

![image-20231028084841720](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231028084841720.png)

* 将之前得到的定位解信息赋值给`rr`和`dtr`数组。

* 调用`ecef2pos()`将将接收机位置`rr`由 ECEF-XYZ 转换为大地坐标系LLH`pos`

* 遍历当前历元所有`OBS[]`，即遍历每颗卫星：

  * 将`vsat[]`、`azel[]`和`resp[]`数组置 0，因为在前后两次定位结果中，每颗卫星的上述信息都会发生变化。`time`赋值OBS的时间，`sat`赋值OBS的卫星。

  * 如果这颗卫星 bDeleted 被排除了，或者卫星系统没使用，跳过当前观测值不处理。

  * 去除重复观测值，检测当前观测卫星是否和下一个相邻数据重复；重复则不处理这一条，continue 去处理下一条。

  * 调用 `satexclude()` 函数判断卫星是否需要排除，如果排除则 continue 去处理下一个卫星。

  * 调用 `geodist()` 函数，计算卫星和当前接收机位置之间的几何距离 `r` 和接收机到卫星的方向向量 `e`。 

  * 调用 `satazel()` 函数，计算在接收机位置处的站心坐标系中卫星的方位角和仰角；若仰角低于截断值 `opt->elmin`，continue 不处理此数据。

  * 调用 `prange()` 函数，计算经过 DCB 校正后的伪距值 `p`，如果设置了消电离层组合直接得到组合后的伪距 `P`。

  * 调用 ` ionocorr()` 函数，计算电离层延时`I`,所得的电离层延时是建立在 L1 信号上的，当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的比率，对上一步得到的电离层延时进行修正。 

  * 调用 `tropcorr()` 函数，计算对流层延时`T`。

  * 计算伪距残差 `v[nv]`，即经过钟差，对流层，电离层改正后的伪距与近似距离直接的差值。

  * 组装设计矩阵 `H` 
    $$
    \boldsymbol{h}(\boldsymbol{x})=\left(\begin{array}{c}\rho_{r}^{1}+c d t_{r}-c d T^{1}+I_{r}^{1}+T_{r}^{1} \\ \rho_{r}^{2}+c d t_{r}-c d T^{2}+I_{r}^{2}+T_{r}^{2} \\ \rho_{r}^{3}+c d t_{r}-c d T^{3}+I_{r}^{3}+T_{r}^{s 3} \\ \vdots \\ \rho_{r}^{m}+c d t_{r}-c d T^{m}+I_{r}^{m}+T_{r}^{m}\end{array}\right) \boldsymbol{H}=\left(\begin{array}{cc}-e_{r}^{1 T} & 1 \\ -e_{r}^{2 T} & 1 \\ -e_{r}^{3 T} & 1 \\ \vdots & \vdots \\ -e_{r}^{m T} & 1\end{array}\right)
    $$

  * 处理不同系统（GPS、GLO、GAL、CMP）之间的时间偏差，作为参数估计，增广矩阵 `v`、`H `。

  * 调用`varerr()`函数，计算此时的导航系统量测噪声协方差阵：
    $$
    \begin{array}{l} \sigma^{2}=F^{s} R_{r}\left(a_{\sigma}{ }^{2}+b_{\sigma}{ }^{2} / \sin E l_{r}^{s}\right)+{\sigma_{\text {eph }}}^{2}+{\sigma_{\text {ion }}}^{2}+{\sigma_{\text {trop }}}^{2}+{\sigma_{\text {bias }}}^{2}\end{array}
    $$
  
  * 设置 `bMulGNSS` 标志，0 为单系统、1 为多系统。
  * 调用 `getHVR_spp()`，根据残差 v 进行错差探测，构建剔除粗差后的 H、V、R
  * 返回定位方程数 `nv`。

```c
static int rescode(const int iter, int bElevCVG, const obsd_t *obs, int n, const double *rs, const double *dts, 
	               const double *vare, const int *svh, const nav_t *nav, const double *x, const prcopt_t *opt, 
				   double *v, double *H, double *var, double *azel, int *vsat, double *resp, int *nx, int *bDeleted)
{
    int bObserved[5];
    int i,j,nv=0,ns[5]={0},sys,satsn[MAXOBS],sat;
    double r,dion,dtrp,vion,vtrp,rr[3],pos[3],dtr,e[3],P,elev_t[MAXSAT],vmeas,lam_L1;
	double elmin;
	int bMulGNSS;
    
    *nx=NX_SPP;

    for (i=0;i<5;i++)            {bObserved[i]=0; ns[i]=0;}
    for (i=0;i<n;i++)            v[i]=var[i]=0.0;
    for (i=0;i<NX_SPP*(n+5);i++) H[i]=0.0;
    for (i=0;i<MAXSAT;i++)       elev_t[i]=0.0;

    //将之前得到的定位解信息赋值给 rr 和 dtr 数组，以进行关于当前解的伪距残差的相关计算
    for (i=0;i<3;i++)            rr[i]=x[i]; dtr=x[3];
    
    // 调用ecef2pos()将将接收机位置rr由 ECEF-XYZ 转换为大地坐标系LLHpos
    // rr{x,y,z}->pos{lat,lon,h} 
    ecef2pos(rr,pos);
    
    // 遍历当前历元观测值，即遍历每颗卫星
    for (i=0;i<n&&i<MAXOBS;i++) {
        sat=obs[i].sat;         // sat 赋值 OBS 的卫星

        // 将 vsat[]、azel[]和resp[] 数组置 0，因为在前后两次定位结果中，每颗卫星的上述信息都会发生变化。
        // time 赋值 OBS 的时间，sat 赋值 OBS 的卫星
        vsat[i]=0; azel[i*2]=azel[1+i*2]=resp[i]=0.0;

        sys=PPP_Glo.sFlag[sat-1].sys;

        // 如果这颗卫星 bDeleted 被排除了，或者卫星系统没使用，跳过当前观测值不处理
        if (bDeleted[sat-1]==0) continue;
        if (!(sys&opt->navsys)) continue;
        
        // 去除重复观测值
        // 检测当前观测卫星是否和下一个相邻数据重复；重复则不处理这一条，continue去处理下一条。
        /* reject duplicated observation data */
        if (i<n-1&&i<MAXOBS-1&&obs[i].sat==obs[i+1].sat) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: duplicated observation data %s sat=%2d\n",
				time_str(obs[i].time,3),obs[i].sat);
			outDebug(OUTWIN,OUTFIL,0);
            i++;
            continue;
        }

        // 调用 geodist() 计算近似几何距离
        // 调用 satazel() 计算方位角高度角，剔除高度角过低的卫星观测值
        /* geometric distance/azimuth/elevation angle */
        if ((r=geodist(rs+i*6,rr,e))<=0.0) continue;    // 近似几何距离小于 0，排除
        satazel(pos,e,azel+i*2);

        if (bElevCVG) {
            if (PPP_Glo.prcOpt_Ex.bElevCheckEx) {   // 如果启用了高度角检查，截止高度角不能小于两度
                elmin=0.0;
                elmin=MIN(opt->elmin,2.0*D2R);
                elmin=MAX(opt->elmin/3.0,elmin);
                if (azel[1+i*2]<elmin) continue;    // 高度角小于设截止高度角，排除
            }
            else {
                if (azel[1+i*2]<opt->elmin) continue;
            }
        }
        else {
            if (azel[1+i*2]<=0.0)
                azel[1+i*2]=MIN(3.0*D2R,opt->elmin*0.75);

            if (iter>=1) {
                if (azel[1+i*2]<opt->elmin) continue;
            }
        }
        
        // 调用 prange() 码偏差改正，得到改正后的伪距 P，如果设置了消电离层组合直接得到组合后的伪距 P
        /* psudorange with code bias correction */
        if ((P=prange(obs+i,nav,azel+i*2,opt,&vmeas))==0.0) continue;

        // 调用 satexclude() 排除不可用卫星的观测值
        /* excluded satellite */
        if (satexclude(obs[i].sat,svh[i],opt)) continue;
        
        // 调用 ionocorr() 克罗布歇电离层改正，计算 L1 信号上计算电离层延时 I
        // 当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的关系，对上一步得到的电离层延时进行修正
        /* ionospheric corrections */
        if (!ionocorr(obs[i].time,sys,nav,pos,azel+i*2,opt,&dion,&vion)) continue;
		/* GPS-L1 -> L1/B1 */
		if ((lam_L1=nav->lam[obs[i].sat-1][0])>0.0) {
			dion*=SQR(lam_L1/lam_carr[0]);
		}

        // 调用 tropcorr() Saastamoinen 对流层改正
        /* tropospheric corrections */
        if (!tropcorr(obs[i].time,nav,pos,azel+i*2,opt,&dtrp,&vtrp)) continue;

        // 构建残差向量 V = P-(r+c*dtr-c*dts+I+T)   (E.6.31)
        /* pseudorange residual */
        v[nv]=resp[i]=P-(r+dtr-CLIGHT*dts[i*2]+dion+dtrp);
        
        // 构建设计矩阵 H，前 3 行为中计算得到的视线向，第 4 行为 1，其它行为 0
        /* design matrix */
        for (j=0;j<4;j++) H[j+nv*NX_SPP]=j<3?-e[j]:1.0;
        
        // 修改矩阵 H，处理不同系统（GPS、GLO、GAL、CMP）之间的时间偏差
        /* time system and receiver bias offset */
        if (sys==SYS_GLO)      {v[nv]-=x[4];   H[4+nv*NX_SPP]=1.0; ns[1]++; bObserved[1]=1;}
        else if (sys==SYS_CMP) {v[nv]-=x[5];   H[5+nv*NX_SPP]=1.0; ns[2]++; bObserved[2]=1;}
        else if (sys==SYS_GAL) {v[nv]-=x[6];   H[6+nv*NX_SPP]=1.0; ns[3]++; bObserved[3]=1;}
		else if (sys==SYS_QZS) {v[nv]-=x[7];   H[7+nv*NX_SPP]=1.0; ns[4]++; bObserved[4]=1;}
        else                   {H[4+nv*NX_SPP]=H[5+nv*NX_SPP]=0.0; ns[0]++; bObserved[0]=1;}
        
        vsat[i]=1; resp[i]=v[nv];

        satsn[nv]=obs[i].sat;
        elev_t[sat-1]=azel[1+i*2]*R2D;
        
        // 调用 varerr() 函数，计算此时的导航系统误差，然后累加计算用户测距误差(URE)
        /* error variance */
        var[nv]=varerr(opt,azel[1+i*2],sys)+vare[i]+vion+vtrp;

        if (azel[1+i*2]<opt->elmin) var[nv]*=100.0;

        nv++;
    }


    for (i=j=0;i<5;i++) {
        if (bObserved[i]) j++;
    }

    bMulGNSS=0;
    if (j>=2) bMulGNSS=1;

    // 调用 getHVR_spp()，根据残差 v 进行错差探测，构建剔除粗差后的 H、V、R
    i=nv;
    nv=getHVR_spp(bMulGNSS,iter,opt->navsys,bElevCVG,bDeleted,satsn,H,v,var,elev_t,nv,*nx);

    for (i=0;i<5;i++) {
        if (ns[i]>0) continue;
        for (j=0;j<nv;j++) 
            H[(3+i)+j*(*nx)]=0.0;
    }

    for (i=0;i<5;i++) {
        if (ns[i]==0) *nx=*nx-1;
    }

    //*nx=*nx-1;

    return nv;

    // v 和 resp 的主要区别在于长度不一致
    // v 是需要参与定位方程组的解算的，维度为 nv*1；
    // resp 仅表示所有观测卫星的伪距残余，维度为 n*1，对于没有参与定位的卫星，该值为 0
}
```

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

### 3、prange()：差分码偏差改正、消电离层组合

DCB 差分码偏差，针对伪距，是由不同类型的 GNSS 信号在卫星和接收机不同通道产生的时间延迟（硬件延迟／码偏差）差异 。由于卫星播发的测距码类型很多， C1、 P1、 P2 等 ，不同的测距信号虽然在同一台卫星钟的驱动下生成的，因而花费的时间也不同。我们把卫星钟脉冲驱动下开始生成测距信号至信号生成并最终离开卫星发射天线相位中心之间所花费的时间称为信号在卫星内部的时延。DCB 体现的就是不同码信号时延的差。分为：

* **频内偏差**：相同频率不同码之间存在的偏差（如 P1-C1、P2-C2 等）
 * **频间偏差**：不同频率之间存在的偏差（如 P1-P2）

一般来说接收机端的DCB被接收机钟差所吸收，可以跟接收机钟差一起解算。若需提高定位精度，卫星端的码偏差需进行校正。目前，码偏差产品主要分为 2 类：① 广播星历播发的时间群延迟(time group delay，TGD)产品；② IGS 分析中心提供的高精度后处理差分码偏差(differential code bias，DCB)产品。因此，TGD 产品相比于 DCB 产品，精度低于 DCB 产品且适用于实时场景。DCB 与 TGD 直接计算的关系如下：
$$
T G D=\frac{1}{1-\gamma} D C B_{12}
$$

$$
D C B_{12}=(1-\gamma) \times T G D
$$

对于 GPS 而言，其广播星历及精密星历是采用 P1、P2 无电离层组合进行卫星钟差估计。因此，广播星历钟差及精密星历钟差均包含 P1、P2 无电离层组合的硬件延迟。当用户基于 P1、P2 无电离层组合定位解算时，无需考虑硬件延迟；反之，若用户使用 P1、P2 单频或其他组合时，均需要考虑硬件延迟的影响，否则会影响定位解算的精度。若用户使用 C/A 码，用户需借助外部文件获取 P-C 将 C/A 码归化到 P 码，然后再进行 TGD/DCB 改正。

与 GPS 不同，BDS 广播星历的钟差基准参考 B3 频点，多数机构的精密钟差基准是 B1/B3 无电离层组合，需要特别处理。

代码中：

* 先取 DCB 数据 P1_P2、P1_C1、P2_C2，没有 DCB 就用广播星历中的 TGD 乘以光速为 P1_P2。
*  如果是消电离层组合，将 C1、C2 伪距做 DCB 改正，加上 P1_C1、P2_C2 归化到 P1、P2，计算得到消电离层观测值 PC。
* 如果单频，将 C1 伪距做 DCB 改正，归化到 P1得到 PC。
* DCB 方差设为 ERR_CBIAS(0.3) 的平方。
* 返回改正或者消电离层组合后的伪距观测值 PC。

```c
static double prange(const obsd_t *obs, const nav_t *nav, const double *azel,
                     const prcopt_t *opt, double *var)
{
    const double *lam=nav->lam[obs->sat-1];
    double PC,P1,P2,P1_P2,P1_C1,P2_C2,gamma,tgd1=0.0,tgd2=0.0;
    int i=0,j=1,sys;
    
    *var=0.0;
    
    if (!(sys=satsys(obs->sat,NULL))) return 0.0;
    
    /* L1-L2 for GPS/GLO/QZS, L1-L5 for GAL/SBS */
    //if (NFREQ>=3&&(sys&(SYS_GAL|SYS_SBS))) j=2;
    
    if (NFREQ<2||lam[i]==0.0||lam[j]==0.0) return 0.0;
    
    gamma=SQR(lam[j])/SQR(lam[i]); /* f1^2/f2^2 */
    P1=obs->P[i];
    P2=obs->P[j];

    // 从 nav->cbias 取 DCB 数据
    P1_P2=nav->cbias[obs->sat-1][0];
    P1_C1=nav->cbias[obs->sat-1][1];
    P2_C2=nav->cbias[obs->sat-1][2];
    
    // 如果没有 P1_P2，调用 gettgd() 使用广播星历 TGD 乘以光速代替
    /* if no P1-P2 DCB, use TGD instead */
	P1_P2=0.0;
    if (P1_P2==0.0&&(sys&(SYS_GPS|SYS_GAL|SYS_QZS|SYS_CMP))) {
        if (sys==SYS_CMP) 
            P1_P2=(1.0-gamma)*gettgd(obs->sat,nav,&tgd1,&tgd2);
        else
            P1_P2=(1.0-gamma)*gettgd(obs->sat,nav,NULL,NULL);
    }

    // 如果是消电离层组合，将 C1、C2 伪距做 DCB 改正，加上 P1_C1、P2_C2 归化到 P1、P2，计算得到消电离层观测值 PC
    if (opt->ionoopt==IONOOPT_IF12) { /* dual-frequency */
        if (P1==0.0||P2==0.0) return 0.0;

        // C1、C2 归化到 P1、P2
        if (obs->code[i]==CODE_L1C) P1+=P1_C1; /* C1->P1 */
        if (obs->code[j]==CODE_L2C) P2+=P2_C2; /* C2->P2 */
        
        // P1、P2 做消电离层组合得到 PC
        /* iono-free combination */
        PC=(gamma*P1-P2)/(gamma-1.0);

        if (sys==SYS_CMP) PC=PC+(tgd2-gamma*tgd1)/(gamma-1.0);
    }

    // 如果单频，将 C1 伪距做 DCB 改正，归化到 P1 
    else { /* single-frequency */
        if (P1==0.0) return 0.0;
        if (obs->code[i]==CODE_L1C) P1+=P1_C1;   /* C1->P1 */
        PC=P1-P1_P2/(1.0-gamma);
    }
    
    *var=SQR(ERR_CBIAS);
    
    return PC;
}
```

### 4、gettgd()：获取群波延迟

广播星历中的 TGD 参数乘以光速；北斗有两个做参数返回。

```c
static double gettgd(int sat, const nav_t *nav, double *tgd1, double *tgd2)
{
    int i;
    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].sat!=sat) continue;

        // 如果是北斗
        if (PPP_Glo.sFlag[sat-1].sys==SYS_CMP) {
            if (tgd1) *tgd1=CLIGHT*nav->eph[i].tgd[0];
            if (tgd2) *tgd2=CLIGHT*nav->eph[i].tgd[1];
            return CLIGHT*(nav->eph[i].tgd[0]);
        }

        // 群波延迟参数 * 光速
        return CLIGHT*nav->eph[i].tgd[0];
    }
    return 0.0;
}
```

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

### 5、ionocorr()：根据选项模型计算 L1 电离层延迟 I

受太阳辐射的影响，距地面 60km 以上的大气层处于部分电离或完全电离状态，该区域被称为电离层。当电磁波信号通过电离层时，传播速度和传播路径会发生改变，给 GNSS 观测值带来误差，即电离层延迟。电离层延迟大小由电子密度和信号频率决定，影响可达数十米。电离层延迟与单位面积的横截面在信号传播路径上拦截的电子总量 $N_e$ 成正比，且与载波频率 $f$ 的平方成反比，弥散性的电离层降低了测距码的传播速度，而加快了载波相位的传播速度，如下所示：
$$
I=I_ \rho=-I_\phi=40.28\frac{N_e}{f^2}
$$
在一些条件下，可使用经验模型改正或约束观测值中的电离层延迟，常用的电离层延迟经验模型有 Klobuchar 模型、Bent 模型和电离层格网模型等。

可以总结出电离层几个对于解算有用的特点：

* **对伪距和载波影响相反**：UOFC 组合，但伪距载波之间延迟。
* **与频率有关**：消电离层组合，但放大噪声。
* **延迟与电子总量 TEC 成正比**：建立电离层格网模型 TEC 文件电离层改正，或者将 STEC 作为参数估计。

`ionocorr()` 是电离层改正的入口函数，根据电离层选项，调用对应的函数，计算出 L1 频率的电离层改正量：

* **克罗布歇模型**：调用 `ionmodel()` 计算改正量，方差设为 0.5 乘以电离层改正量再平方。 
* **电离层格网模型**：调用 `iontec()` 计算改正量，方差也在 `iontec()` 中计算。
* **IF 消电离层组合**：无需计算改正量，IF 组合在 `prange()` DCB 改正的时候实现了，方差设为 0.02 * 0.02。

```c
static int ionocorr(gtime_t time, const int sys, const nav_t *nav, const double *pos,
                    const double *azel, const prcopt_t *opt, double *ion,
                    double *var)
{
    // 克罗布歇模型（广播星历模型）
    /* broadcast model */
    if (opt->ionoopt==IONOOPT_BRDC) {
        //*ion=ionmodel(time,nav->ion_gps,pos,azel);

        if (sys==SYS_GPS) *ion=ionmodel(time,nav->ion_gps,pos,azel);
        //else if ( SYS_CMP==sys )  *ion=ionmodel(time,nav->ion_cmp,pos,azel);
        else *ion=ionmodel(time,nav->ion_gps,pos,azel);

        *var=SQR(*ion*ERR_BRDCI);
        return 1;
    }
    // 电离层格网模型
	/* ionex tec model */   
	else if (opt->ionoopt==IONOOPT_TEC) {
		return iontec(time,nav,pos,azel,3,ion,var);
	} 
    // IF 消电离层组合
    else if (opt->ionoopt==IONOOPT_IF12) {
        *ion=0.0;
        *var=SQR(0.02);
        return 1;
    }

    *ion=0.0;
    *var=opt->ionoopt==IONOOPT_OFF?SQR(ERR_ION):0.0;
    return 1;
}
```

### 6、ionmodel()：克罗布歇模型电离层延迟计算

SPP 中使用克罗布歇模型计算 L1 的电离层改正量，将晚间的电离层时延视为常数，取值为 5ns，把白天的时延看成是余弦函数中正的部分。于是天顶方向调制在 L1 载波上的测距码的电离层时延可表示为：
$$
T_{g}=5 \times 10^{-9}+A \cos \frac{2 \pi}{P}\left(t-14^{h}\right)
$$
振幅 $A$ 和周期 $P$ 是模型中需要计算的部分，分别为：
$$
\begin{array}{l}
A=\sum_{i=0}^{3} \alpha_{i}\left(\varphi_{m}\right)^{i} \\
P=\sum_{i=0}^{3} \beta_{i}\left(\varphi_{m}\right)^{i}
\end{array}
$$
全球定位系统向单频接收机用户提供的电离层延迟改正时就采用上述模型。其中 $\alpha_i$ 和 $\beta_i$ 是地面控制系统根据该天为一年中的第几天（将一年分为 37 个区间）以及前 5 天太阳的平均辐射流量（共分10档）从 370 组常数中选取的，然后编入星导航电文播发给用户。卫星在其播发的导航电文中提供了这 8 个电离层延迟参数：
$$
p_{\text {ion }}=\left(\alpha_{0}, \alpha_{1}, \alpha_{2}, \alpha_{3}, \beta_{0}, \beta_{1}, \beta_{2}, \beta_{3}\right)^{T}
$$
根据根据参数 $\alpha_0，\alpha_1，\alpha_2，\alpha_3$ 确定振幅 $A$，根据根据参数 $\beta_0，\beta_1，\beta_2，\beta_3$ 确定周期 $T$，再给定一个以秒为单位的当地时间 $t$，就能算出**天顶电离层延迟**。再由天顶对流层延迟根据倾斜率转为卫星方向对流层延迟。

电离层分布在离地面 60-1000km 的区域内。当卫星不在测站的天顶时，信号传播路径上每点的地方时和纬度均不相同，为了简化计算，我们将整个电离层压缩为一个单层，将整个电离层中的自由电子都集中在该单层上，用它来代替整个电离层。这个电离层就称为中心电离层。中心电离层离地面的高度通常取 350km。需要计算卫星言号传播路径与中心电离层的交点 $P^{\prime}$ 的时角 $t$ 和地磁纬度 $\varphi_{m}$ ，因为只有 $P^{\prime}$ 才能反映卫星信号所受到的电离层延迟的总的情况。

综上，已知大地经度、 大地纬度、卫星的高度角和卫星测站的方位角，电离层延迟计算方法如下：

计算测站 $P$ 和 $P^{\prime}$ 在地心的夹角：
$$
\psi=0.0137 /(E l+0.11)-0.022
$$
计算交点 $P^{\prime}$ 的地心纬度：
$$
\varphi_{i}=\varphi+\psi \cos A z
$$

$$
\left\{\begin{array}{ll}\varphi_{l}>+0.416 & \varphi_{l}=+0.416 \\ \varphi{l}>-0.416 & \varphi_{l}=-0.416\end{array}\right.
$$

计算交点 $P^{\prime}$ 的地心经度：
$$
\lambda_{i}=\lambda+\psi \sin A z / \cos \varphi_{i}
$$
计算地磁纬度：
$$
\varphi_{m}=\varphi_{i}+0.064 \cos \left(\lambda_{i}-1.617\right)
$$
计算观测瞬间交点 $P^{\prime}$ 处的地方时：
$$
t=4.32 \times 10^{4} \lambda_{i}+t
$$

$$
\left\{\begin{array}{ll}t>86400 & t=t-86400 \\ t<0 & t=t+86400\end{array}\right.
$$

计算倾斜因子：
$$
 F=1.0+16.0 \times(0.53-E l)^{3}
$$
最后计算电离层时间延迟：
$$
\begin{array}{l}x=2 \pi(t-50400) / \sum_{n=0}^{3} \beta_{n} \varphi_{m}{ }^{n} \\ I_{r}^{s}=\left\{\begin{array}{cc}F \times 5 \times 10^{-9} \\ F \times\left(5 \times 10^{-9}+\sum_{n=1}^{4} \alpha_{n} \varphi_{m}{ }^{n} \times\left(1-\frac{x^{2}}{2}+\frac{x^{4}}{24}\right)\right) & (|x|>1.57)\end{array}\right.\end{array}
$$

计算的是 L1 信号的电离层延时 I ，当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的比例关系，对上一步得到的电离层延时进行修正，不考虑模糊度情况下改正公式为：
$$
I=\frac{\Phi_{2}-\Phi_{1}}{1-\left(f_{1} / f_{2}\right)^{2}}
$$

```c
extern double ionmodel(gtime_t t, const double *ion, const double *pos,
                       const double *azel)
{
    const double ion_default[]={ /* 2004/1/1 */
        0.1118E-07,-0.7451E-08,-0.5961E-07, 0.1192E-06,
        0.1167E+06,-0.2294E+06,-0.1311E+06, 0.1049E+07
    };
    double tt,f,psi,phi,lam,amp,per,x;
    int week;
    
    if (pos[2]<-1E3||azel[1]<=0) return 0.0;
    if (norm(ion,8)<=0.0) ion=ion_default;  // 若没有电离层参数，用默认参数


    /* earth centered angle (semi-circle) */    // 地球中心角
    psi=0.0137/(azel[1]/PI+0.11)-0.022;         // 计算地心角(E.5.6)
    
    /* subionospheric latitude/longitude (semi-circle) */   
    phi=pos[0]/PI+psi*cos(azel[0]);                 // 计算穿刺点地理纬度(E.5.7)
    if      (phi> 0.416) phi= 0.416;        // phi不超出(-0.416,0.416)范围
    else if (phi<-0.416) phi=-0.416;
    lam=pos[1]/PI+psi*sin(azel[0])/cos(phi*PI);     // 计算穿刺点地理经度(E.5.8)
    
    /* geomagnetic latitude (semi-circle) */
    phi+=0.064*cos((lam-1.617)*PI);                 // 计算穿刺点地磁纬度(E.5.9)
    
    /* local time (s) */
    tt=43200.0*lam+time2gpst(t,&week);              // 计算穿刺点地方时(E.5.10)
    tt-=floor(tt/86400.0)*86400.0; /* 0<=tt<86400 */
    
    /* slant factor */
    f=1.0+16.0*pow(0.53-azel[1]/PI,3.0);            // 计算投影系数(E.5.11)
    
    /* ionospheric delay */
    amp=ion[0]+phi*(ion[1]+phi*(ion[2]+phi*ion[3]));
    per=ion[4]+phi*(ion[5]+phi*(ion[6]+phi*ion[7]));
    amp=amp<    0.0?    0.0:amp;
    per=per<72000.0?72000.0:per;
    x=2.0*PI*(tt-50400.0)/per;                      // (E.5.12)
    
    return CLIGHT*f*(fabs(x)<1.57?5E-9+amp*(1.0+x*x*(-0.5+x*x/24.0)):5E-9);     //(E.5.13)
}
```

### 7、tropcorr()：对流层改正

> 推荐论文：多模型融合的对流层天顶延迟估计方法—雷雨

对流层一般指距离地面 50km 内的大气层，是大气层质量的主要部分。当导航信号穿过对流层时，由于传播介质密度的增加，信号传播路径和传播速度会发生改变，由此引起的 GNSS 观测值误差称为对流层延迟。对流层延迟一般可分为干延迟和湿延迟，对于载波相位和伪距完全相同，一般在米级大小，可通过模型改正和参数估计的方法来削弱其影响。修正模型如下：
$$
T=M_{d r y} T_{d r y}+M_{w e t} T_{w e t}
$$
式中，$T_{d r y}, T_{w e t}$ 分别表示接收机天顶对流层的干延迟和湿延迟；$M_{d r y}, M_{w e t}$ 分别表示干延迟和湿延迟的投影函数。对流层干延迟比较稳定，主要与测站高度、大气温度和大气压相关，可通过模型改正，常用模型有 Saastamoninen 模型、Hopfield 模型等。湿延迟不同于干延迟，变化较大，主要与水汽含量相关，一般估计天顶对流层湿延迟，通过投影函数计算各卫星的电离层湿延迟，常用的投影函数有全球投影函数（Global Mapping Function，GMF）、Niell 投影函数（NMF）和 Vienna投影函数（Vienna Mapping Function，VMF）等。

`tropcorr()` 是模型改正的入口函数，通过调用 tropmodel() 计算干延迟和湿延迟改正量。

```c
static int tropcorr(gtime_t time, const nav_t *nav, const double *pos,
                    const double *azel, const prcopt_t *opt, double *trp,
                    double *var)
{
    double trpw=0.0;
    *trp=0.0;

    /* saastamoinen model */
    if (opt->tropopt==TROPOPT_SAAS||opt->tropopt==TROPOPT_EST) {
        *trp=tropmodel(time,pos,azel,REL_HUMI,&trpw,0); // 返回干延迟
        *trp+=trpw;             // 加上湿延迟
        *var=SQR(ERR_SAAS);

        if (*trp>100.0) *trp=100.0;
        else if (*trp<0.05) *trp=0.05;

        return 1;
    }

    /* no correction */
    *trp=0.0;
    *var=opt->tropopt==TROPOPT_OFF?0.0:SQR(ERR_TROP);
    
	return 1;
}
```

### 8、tropmodel()：Saastamoinen 模型计算对流层延迟

#### 1. Saastamoinen 模型

Saastamoinen 模型将对流层分为两层进行积分，第 1 层为从地表到 $10 \mathrm{~km}$ 高度的对流层顶，该层的温度变化率为；第 2 层为从 $10 \mathrm{~km}$ 到 $50 \mathrm{~km}$ 高度的平流层顶，该层的温度视为常数。 Saastamoinen 模型首次将被积函数按照天顶距三角函数展开逐项进行积分, 并把对流层天顶延迟分为对流层干延迟和湿延迟两个分量之和，两个分量的表达式为：
$$
\left\{\begin{array}{l}Z H D=\frac{0.002277 P}{f(\varphi, h)} \\ Z W D=\frac{0.002277 e}{f(\varphi, h)}\left(0.05+\frac{1255}{T}\right) \\ f(\varphi, h)=1-0.00266 \cos (2 \varphi)-0.00028 h\end{array}\right.
$$
或者：
$$
T_{r}^{s}=\frac{0.002277}{\cos z}\left\{p+\left(\frac{1255}{T}+0.05\right) e-\tan ^{2} z\right\}
$$
其中，$P, T, e, \varphi$ 和 $h$ 分别为地表气压 $(\mathrm{hPa})$ 、地表温度 $(\mathrm{K})$ 、水汽压 $(\mathrm{hPa})$ 、测站纬度 $(\mathrm{rad})$ 和高程 $(\mathrm{km})$ ，可以由标准大气模型基于经验计算，也可由 GPT 模型提供。

#### 2. 标准大气模型

根据经验模型计算求大气压 P、温度 T、大气水汽压力 e：
$$
\begin{array}{l}p=1013.25 \times\left(1-2.2557 \times 10^{-5} h\right)^{5.2568} \\ T=15.0-6.5 \times 10^{-3} h+273.15 \\ e=6.108 \times \exp \left\{\frac{17.15 T-4684.0}{T-38.45}\right\} \times \frac{h_{r e l}}{100}\end{array}
$$


其中 $z$ 是天顶角，与高度角互余：$z=\pi / 2-E l_{r}^{s}$

#### 3. GPT 模型

GPT 系列模型是 Boehm 等利用欧洲中尺度天气预报中心 (European Centre for Medium-Range Weather Forecasts, ECMWF) 长期的再分析气象资料建立的全球气象参数经验模型, 仅需知道测站地理位置信息与年积日便可以获得地表温度、大气压力和水汽压等气象参数，在全球范围内得到广泛应用。

作者用的 ftp://tai.bipm.org/iers/convupdt/chapter9/GPT.F 文件对应的模型，把 IGS 中心提供的函数改成 c 语言版本的 `getGPT()` 函数，这个 GPT.F 可以通过资源管理器下载，用 notepade++、或者 VScode 查看：

![5f5cec497ffb8ae62cb16a7ae2519cff](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/5f5cec497ffb8ae62cb16a7ae2519cff.png)

#### 4. tropmodel() 代码

用标准大气模型或 GPT 模型计算一些标准大气值，包括总压 p、大气温度 T、水汽压力 e；然后计算了静力学延迟(干延迟) trph计算了湿延迟 trpw。

> humi 传 0，就只计算干延迟，湿延迟作为参数估计

```c
extern double tropmodel(gtime_t time, const double *pos, const double *azel,
                        double humi, double *zwd, int atmodel)
{
    const double temp0=15.0;   /* temparature at sea level */
    double hgt,pres,temp,e,z,trph,trpw;
	int b1,b2;
	mjd_t mjd;
	double dmjd,undo,d1,d2;
    
    if (pos[2]<-100.0||pos[2]>1E6||azel[1]<=0) {
        //���㿪ʼ�ĵ�һ����Ԫ�߳�������-100.0����������������Ϣ
        b1=(PPP_Glo.iEpoch==1)&&(PPP_Glo.revs==0);
        b2=(PPP_Glo.nEpoch-1==PPP_Glo.iEpoch)&&(PPP_Glo.revs==1);

        //����deltaEp����
        if (!b1&&!b2&&PPP_Glo.delEp<20) {
            sprintf(PPP_Glo.chMsg,"*** WARNING: tropmodel: height=%7.3f elev=%4.1f\n",pos[2],azel[1]*R2D);
            outDebug(OUTWIN,OUTFIL,OUTTIM);
        }

        return 0.0;
    }
    if (pos[2]>=1.0/2.2557E-5) return 0.0;

    hgt=pos[2]<0.0?0.0:pos[2];

    //�̹߳���ᵼ�¼��������ֵ���������ӳ���Ϊ�����
    if (hgt>15000.0) {
        //���㿪ʼ�ĵ�һ����Ԫ�߳�������-100.0����������������Ϣ
        b1=(PPP_Glo.iEpoch)==1&&(PPP_Glo.revs==0)&&(PPP_Glo.prcOpt_Ex.solType==0||
			PPP_Glo.prcOpt_Ex.solType==3||PPP_Glo.prcOpt_Ex.solType==4);
        b2=(PPP_Glo.nEpoch-1==PPP_Glo.iEpoch)&&(PPP_Glo.revs==1)&&(PPP_Glo.prcOpt_Ex.solType==1
			||PPP_Glo.prcOpt_Ex.solType==2);

        if (!b1&&!b2&&PPP_Glo.delEp<20) {
			sprintf(PPP_Glo.chMsg,"*** WARNING: tropmodel: height=%7.3f\n",hgt);
            outDebug(OUTWIN,OUTFIL,OUTTIM);
        }
        hgt=15000.0;
    }

    // 计算大气参数，标准大气模型 or GPT 模型
    /* standard atmosphere */
    if (atmodel!=1) {
        pres=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);         // 求大气压P (E.5.1)
        temp=temp0-6.5E-3*hgt+273.16;                       // 求温度temp (E.5.2)
        e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45)); // 求大气水汽压力e (E.5.3)
    }
    else {
        time2mjd(time,&mjd);
        dmjd=mjd.day+(mjd.ds.sn+mjd.ds.tos)/86400.0;
        
        undo=0.0;
        getGPT(pos,dmjd,&pres,&temp,&undo);

        d1=1013.25*pow(1.0-2.2557E-5*hgt,5.2568);
        d2=15.0-6.5E-3*hgt;

        temp+=273.16;
        e=6.108*humi*exp((17.15*temp-4684.0)/(temp-38.45));
    }
    
    // 计算对流层延迟
    /* saastamoninen model */
    z=PI/2.0-azel[1];                   // 求天顶角z 卫星高度角azel[1]的余角
    trph=0.0022768*pres/(1.0-0.00266*cos(2.0*pos[0])-0.00028*hgt/1E3)/cos(z);   // 干延迟
    trpw=0.002277*(1255.0/temp+0.05)*e/cos(z);  // 湿延迟

    *zwd=trpw;

    return trph;
}
```

### 7、varerr()：计算量噪声测协方差阵

不同的卫星直接的量测噪声没有相关性，所以量噪声测协方差阵是对角阵，对角线上每个元素由以下几部分组成：
$$
\begin{array}{l} \sigma^{2}=F^{s} R_{r}\left(a_{\sigma}{ }^{2}+b_{\sigma}{ }^{2} / \sin E l_{r}^{s}\right)+{\sigma_{\text {eph }}}^{2}+{\sigma_{\text {ion }}}^{2}+{\sigma_{\text {trop }}}^{2}+{\sigma_{\text {bias }}}^{2}\end{array}
$$

其中：

* $F^{s}$：卫星系统误差因子，GLONASS 1.5，SBAS fact 3，其它 1。
* $R_{r}$、$a_{\sigma}, b_{\sigma}$：测距误差因子，建模成高度角模型。
* $\sigma_{e p h}$：卫星位置误差因子。
* $\sigma_{\text {ion }}$：电离层延迟误差因子。
* $\sigma_{\text {trop }}$：对流层延迟误差因子。
* $\sigma_{\text {bias }}$：DCB 误差因子。

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

### 8、getHVR_spp()：剔除粗差数据，组建新的 H、V、R

* 原方程个数 `nv` 小于等于 4，不进行粗差探测，直接返回 `nv`。

* for 循环遍历卫星系统，各卫星系统分别进行粗差探测，传入的 `v`、`var` 是多系统在一起的，再一个 for 循环取出当前卫星系统 `i` 下的 `v`、`var`、和在  `v`、`var`中的下标序列到 `dv`、`dvar`、`ix`；调用 `getHVR_s()` 进行粗差探测。
* 统计粗差情况，不严重直接退出程序，不进行粗差剔除。

* 根据 `ibadsn` 组建剔除后粗差后的 `H`、`v`、`var`

* 返回剔除粗差后的方程数 `newnv`

```c
static int getHVR_spp(int bMulGnss, const int iter, const int sys, int bElevCVG, int bDeleted[MAXSAT], 
	                  int *sat, double *H, double *v, double *var, double *elev, int nv, int nx)
{
	int bbad;
	double *dv,*dvar,std_ex[NSYS_USED],ave_ex[NSYS_USED];
	int ibadsum[NSYS_USED],ibadsn[NSYS_USED][MAXOBS],navsys[8]={SYS_GPS,SYS_GLO,SYS_CMP,SYS_GAL,SYS_QZS,0};
	int i,j,k,newnv=0,ibadsum_,*ix;
	int nv_nx;

    // 量测个数 nv 小于等于 4，直接返回 nv
    if (nv<=4) return nv;

    bbad=0;
    dv=mat(nv,1);
    dvar=mat(nv,1);
    ix=imat(nv,1);

    //////////////////////////////////////////////////////////////////////////
    // 遍历使用的各卫星系统，i 是卫星系统，j 是卫星在自己系统内下标，k 是卫星在所有系统(v、var)的下标
    for (i=0;i<NSYS_USED;i++) {

        ibadsum[i]=0;
        std_ex[i]=ave_ex[i]=0.0;

        for (j=0;j<MAXOBS;j++) ibadsn[i][j]=-1;

        if (!(sys&navsys[i])) continue;
        
        // 传入的 v、var 是多系统在一起的
        // 这个 for 循环提取出当前卫星系统 i 下的 v、var、和它的下标 到 dv、dvar、ix
        // 调用 getHVR_s() 
        for (k=j=0;k<nv;k++) {
            if (navsys[i]==PPP_Glo.sFlag[sat[k]-1].sys) {
                ix[j]=k;
                dv[j]=v[k];
                dvar[j]=var[k];
                j++;
            }
        }
		ibadsum[i]=getHVR_s(bMulGnss,iter,sat,ix,dv,dvar,j,ibadsn[i],std_ex[i],ave_ex[i],bElevCVG);
    }
    //////////////////////////////////////////////////////////////////////////

    free(dv); free(dvar); free(ix);

    //////////////////////////////////////////////////////////////////////////
    ibadsum_=0;
    k=ibadsum[0];
    for (i=0;i<NSYS_USED;i++) {
        ibadsum_+=ibadsum[i];

        if (i==0) continue;

        for (j=0;j<ibadsum[i];j++)
            ibadsn[0][k++]=ibadsn[i][j];
    }
    //////////////////////////////////////////////////////////////////////////
    if (ibadsum_>0) {
        if (ibadsum_>=3&&nv-ibadsum_>=6) bbad=1;
        if (ibadsum_<=2) {
            if (nv<=6) {
                if (nv-ibadsum_>4) {
                    bbad=1;
                }
            }
            else if (nv-ibadsum_>=5) {
                bbad=1;
            }
        }
    }

    if (bbad==0) return nv;

    for (i=j=0;i<nv;i++) {
        v[newnv]=v[i];
        var[newnv]=var[i];

        nv_nx=newnv*nx;
        for (k=0;k<nx;k++) H[nv_nx+k]=H[i*nx+k];

        // ibadsn 位对应为粗差，就直接 continue，不执行 newnv++
        // 下一次 v、var、H 会将此次的数据覆盖，达到剔除粗差的效果
        // 这个这个判断写在前面更好，代码好理解，而且少几次赋值
        if (j<ibadsum_) {
            if (ibadsn[0][j]==i) {
                j++;

                continue; 
            }
        }
        newnv++;
    }

    // 返回剔除粗差后的方程数
    nv=newnv;
    return nv;  
}
```

#### 1. getHVR_s()

```c
static int getHVR_s(int bMulGnss, const int iter, int *sat, int *ix, double *v, double *var, int nv, 
	                int *ibadsn, double std_ex, double ave_ex, int bElevCVG)
{
    double dValue,factor=1.0,dt;
    int i,j,ind,nbad=2,nBadRes,ibadsum=0,sn[MAXOBS];
	int b,b1,b2,b3,id[100];
	double dtmp,dFactor[100];

    if (!bElevCVG) {
        factor=3.0;
        factor=2.25;
    }

    if (nv>=14)      nbad=5;
    else if (nv>=11) nbad=4;
    else if (nv>=8)  nbad=3;
    else             nbad=2;

    // 调用 findGross() 
    nBadRes=findGross(1,bMulGnss,v,nv,nbad,&std_ex,&ave_ex,sn,5.0,1.0,2.5);

    dValue=factor*std_ex;
    if (std_ex<5000.0) {
        for (i=ibadsum=0;i<nBadRes;i++) {
            b=0;
            j=sn[i];
            ind=j;
            dt=fabs(v[ind]-ave_ex);
            if (fabs(dt)<1.0) continue;

            if (!bElevCVG) {
                if (fabs(dt)<10.0) continue;
            }

            if (dt>800.0)     {if (dt> 10*dValue) b=1;}
            else if (dt>20.0) {if (dt>5.0*dValue) b=1;}
            else if (dt>10.0) {if (dt>6.0*dValue) b=1;}
            else if (dt>3.0)  {if (dt>7.0*dValue) b=1;}

            if (b==0) continue;

            ibadsn[ibadsum]=ix[ind];
            ibadsum++;
        }
    }
    else {
        if (iter>=0) {
            for (i=0;i<100;i++) {
                id[i]=-1;
                dFactor[i]=0.0;
            }

            dtmp=1.0/std_ex;
            for (i=0;i<nBadRes;i++) {
                j=sn[i];
                ind=j;
                id[i]=ix[ind];
                dFactor[i]=(v[ind]-ave_ex)*dtmp;
            }

            for (i=0;i<nBadRes;i++) {
                for (j=i+1;j<nBadRes;j++) {
                    if (fabs(dFactor[i])>=fabs(dFactor[j])) continue;

                    dtmp=dFactor[i];    dFactor[i]=dFactor[j];  dFactor[j]=dtmp;
                    ind=id[i];          id[i]=id[j];            id[j]=ind;
                }
            }

            for (j=0;j<nBadRes;j++) {
                if (fabs(dFactor[j])<=3.0) break;
            }

            if (nBadRes>1) {
                if (j==0) {
                    return 0;
                }

                b1=fabs(dFactor[j-1])>15.0;
                b2=fabs(dFactor[j-1])>fabs(2.0*dFactor[j]);
                b3=fabs(dFactor[j-1])-fabs(dFactor[j])>2.0;

                if (b1&&b2&&b3) {
                    for (i=ibadsum=0;i<=j-1;i++) {
                        ibadsn[ibadsum]=id[i];
                        ibadsum++;
                    }
                }
            }
            else if (nBadRes==1) {
                b1=fabs(dFactor[0])>25.0;
                if (b1) {
                    ibadsn[0]=id[0];
                    ibadsum=1;
                }
            }
        }
    }

    return ibadsum;
}
```

#### 2. findGross()

```c
extern int findGross(int ppp, int bMulGnss, double *v, const int nv, const int nbad, 
	                 double *std_ex, double *ave_ex, int *ibadsn, const double ratio, 
					 const double minv, const double stdmin)
{
	int i,j,badn=0,*ibadsn_t,badn_min=0;
	double dstd_min=1.0e9,dstd=0.0,dave=0.0,dave_min=0.0;
	int kk=4;

	if (nv<=1) return 0;

	ibadsn_t=imat(nv,1);	// 用于标记粗差

	if (bMulGnss) {
		kk--;
	}

	if (kk<=1) kk=1;

	for (i=0;i<=nbad;i++) {
		if (ppp&&(nv<i+kk||nv<2*i+1)&&i) continue;

		badn=findGross_(i,v,nv,&dstd,&dave,ibadsn_t,ratio,minv,stdmin);

		if (dstd>0.0&&dstd<dstd_min) {
			dstd_min=dstd;
			dave_min=dave;
			badn_min=badn;

			if (ibadsn)
				for (j=0;j<badn;j++) ibadsn[j]=ibadsn_t[j];
		}

		if (dstd>0.0&&dstd<=stdmin) break;
	}

	if (std_ex) *std_ex=dstd_min;
	if (ave_ex) *ave_ex=dave_min;

	free(ibadsn_t);

	return badn_min;
}
```

#### 3. findGross_()：粗差探测

找出在一组数据中，哪些数据对整体数据的标准差 `Std_ex` 和平均值 `Ave_ex` 产生了较大的影响，在 `ibadsn` 标记。

1. 首先检查一些条件，例如数据块的数量和大小、比率等。如果这些条件不满足，那么就返回0，并可能设置一些期望值。
2. 定义并初始化了一些变量和数组。
3. 计算组合的数量，然后调用`select_combination`函数来选择一个组合。
4. 在选择的组合中，通过迭代计算每个数据块的标准差，并找出最小标准差和对应的平均值。
5. 如果找到的标准差小于设定的最小标准差，并且大于 0，那么就更新最小标准差和对应的平均值。
6. 在计算完所有组合后，释放分配的内存。
7. 通过检查每个数据块的值和标准差，找出对标准差影响较大的数据块，并标记它们。
8. 如果提供了`ibadsn`指针，那么就将标记的数据块的索引存储在其中。
9. 如果设置了期望值，那么就设置它们为找到的最小标准差和对应的平均值。
10. 释放最后一块分配的内存，返回标记为不良的数据块的数量。

```c
static int findGross_(const int nb, double *dv, const int nv, double *std_ex, double *ave_ex, int *ibadsn,
	                  const double ratio, const double minv, const double minstd)
{
	int bbad=0,*bused;
	int i,j,n,*it,*ibadsn_t;
	int j0,j9;
	double dstd_min=1.0e9,dt0=0.0,dt1=0.0,dave_min=0.0;

	// 代码首先检查一些条件，例如数据块的数量和大小、比率等。如果这些条件不满足，那么就返回 0，并可能设置一些期望值
	if ((nv-nb<=nb)||nv<=0||ratio<=1.0) {
		if (std_ex)	*std_ex=-1.0;
		if (ave_ex)	*ave_ex=0.0;
		return 0;
	}

	bused=imat(nv,1);
	ibadsn_t=imat(nv,1);

	for (i=0,n=1;i<nb;i++)
		n=n*(nv-i)/(i+1);

	if (nb<=0) it=imat(n,1);
	else       it=imat(n*nb,1);

	comb_j=0;
	select_combination(0,0,nv,nb,it);

	// 在选择的组合中，通过迭代计算每个数据块的标准差，并找出最小标准差和对应的平均值
	for (i=0;i<n;i++) {
		j0=i*nb;
		j9=j0+nb;
		for (j=0;j<nv;j++)	bused[j]=1;
		for (j=j0;j<j9;j++)	bused[it[j]-1]=0;

		dt0=calStd_ex(dv,nv,bused,&dt1);

		// 如果找到的标准差小于设定的最小标准差，并且大于 0，那么就更新最小标准差和对应的平均值。
		if (dt0<dstd_min&&dt0>0.0) {
			dstd_min=dt0;
			dave_min=dt1;
		}

		if (dt0<minstd && dt0>0.0) break;
	}
	free(bused); free(it);

	// 通过检查每个数据块的值和标准差，找出对标准差影响较大的数据块，并标记它们
	for (i=j=0;i<nv;i++) {
		bbad=0;
		dt0=fabs(dv[i]-dave_min);
		if ( dt0>ratio*dstd_min&&dstd_min>1.0e-8) {
			if (minv>0.0) {
				if (fabs(dt0)>minv) bbad=1;
			}
			else
				bbad=1;
		}
		if (bbad) {
			if (j<nv)
				ibadsn_t[j]=i;
			else {
				//
			}
			j++;
		}
	}
	if (ibadsn) {
		for (i=0;i<j;i++) ibadsn[i]=ibadsn_t[i];
	}

	if (std_ex) *std_ex=dstd_min;
	if (ave_ex) *ave_ex=dave_min;

	free(ibadsn_t);

	return j;
}
```

#### 4. calStd_ex()：计算均值、标准差

1. 首先，函数通过循环计算所有被标记为"bused"的数据的平均值。
2. 如果"bused"的数据数量为0（也就是没有数据被使用），那么函数返回-1.0，表示计算失败。
3. 否则，计算所有被标记为"bused"的数据的平均值，并将结果存储在变量`dave`中。
4. 然后，函数通过循环计算所有被标记为"bused"的数据的方差（即每个数据点与平均值的差的平方）。
5. 最后，计算方差的平方根得到标准差，并将结果作为函数的返回值。同时，如果提供了`ave`指针，将平均值也存储在该指针指向的位置。

```c
static double calStd_ex(const double *v, const int n, int *bused, double *ave)
{
	int i,j;
	double dave=0.0,std=0.0;

	// 首先，函数通过循环计算所有被标记为"used"的数据的平均值
	for (i=j=0;i<n;i++) {
		if (bused) {
			if (bused[i]==0) continue;
		}

		dave+=v[i];
		j++;
	}

	// 如果"bused"的数据数量为0（也就是没有数据被使用），那么函数返回-1.0，表示计算失败。
	if (j<=0) {
		return -1.0;
	}

	// 否则，计算所有被标记为"bused"的数据的平均值，并将结果存储在变量dave中。
	dave/=j;

	// 然后，函数通过循环计算所有被标记为"bused"的数据的方差（即每个数据点与平均值的差的平方）。
	for (i=0;i<n;i++) {
		if (bused) {
			if (bused[i]==0) continue;
		}

		std+=(v[i]-dave)*(v[i]-dave);
	}

	// 最后，计算方差的平方根（这就是标准差），并将结果作为函数的返回值。同时，如果提供了ave指针，将平均值也存储在该指针指向的位置。
	std=sqrt(std/j);

	if (ave) *ave=dave;

	return std;
}
```

#### 5. select_combination()：生成组合

这个函数的主要逻辑是生成所有可能的组合，每个组合的元素数量从`l`开始，逐步增加到`m`。每次找到一个新的组合时，它都会复制到`sn`数组中。当找到一个完整的组合时（即长度等于`m`），它会立即返回，不再继续寻找其他组合。

- `l`：当前组合的长度
- `p`：一个固定点，可能是一个界限或起始点，影响递归的方向或方式
- `n`：集合的大小或元素的数量
- `m`：组合的长度
- `sn`：一个整数数组，用于存储生成的组合

```c
extern void select_combination(const int l, const int p, const int n, const int m, int *sn)
{
	int i;
	
    // 如果l等于m，说明已经找到了一个完整的组合，该组合以rcd[0]到rcd[m-1]的形式存储在数组rcd中。
    // 此时，将这个组合复制到sn数组中，并增加comb_j的值。然后返回，不再继续寻找其他组合。
	if (l==m) {
		for (i=0;i<m;i++)
			sn[comb_j*m+i]=rcd[i];
		comb_j++;
		return;
	}
	// 如果l不等于m，则函数将循环遍历从p到n-(m-l)的所有整数。
    // 对于每个整数i，它都会将i+1存储在rcd[l]中，然后递归调用自身，将参数l增加1，i+1增加1，以便在下一次迭代中生成下一个元素。
	for (i=p;i<=n-(m-l);i++) {
		rcd[l]=i+1;
		select_combination(l+1,i+1,n,m,sn);
	}
}
```

## 三、lsqplus()：最小二乘估计

### 1、原理

最小二乘准则为：
$$
V^{T} P V=\min
$$
最小二乘的一般观测模型和误差方程如下所示：
$$
\left\{\begin{array}{l}
L_{n \times 1}=B_{n \times t} X_{t \times 1}+d_{n \times 1} \\
x_{t \times 1}=X_{t \times 1}-X_{0, t \times 1} \\
l_{n \times 1}=L_{n \times 1}-\left(B_{n \times t} X_{0, t \times 1}+d_{n \times 1}\right) \\
V_{n \times 1}=B_{n \times 1} x_{t \times 1}-l_{n \times 1}, P_{n \times n}
\end{array}\right.
$$
式中，$n, t$ 分别表示观测值个数和待估参数个数；$L, X, X_{0}$ 分别表示观测值向量、待估参数向量和待估参数初值向量；$B, d$ 分别表示系数矩阵和常数项；$V, l$ 分别表示改正数向量和误差方程常数项；$P$ 表示观测值权值矩阵。

根据最小二乘准则和误差方程，可得改正数及其中误差的求解过程为：
$$
\left\{\begin{array}{l}
\frac{\partial V^{T} P V}{\partial x}=2 V^{T} P \frac{\partial V}{\partial x}=V^{T} P B=0 \\
B^{T} P V=0 \\
x=\left(B^{T} P B\right)^{-1} B^{T} P l \\
\sigma_{0}=\sqrt{V^{T} P V /(n-t)}, Q_{x x}=\left(B^{T} P B\right)^{-1}
\end{array}\right.
$$
式中 $\sigma_{0}, Q_{x x}$ 分别表示中误差和协因数矩阵。

### 2、lsqPlus()

我的理解这个函数就是对 RTKLIB 原版的最小二乘函数 `lsq()` 做了一层封装，先找出对最小二乘求解有贡献的未知数，记录它们的原来的下标到 `ix`，按照 `ix` 重新构建参数向量、设计矩阵、新息向量，计算最小二乘之后再还原回原来的维度。

```c
extern int lsqPlus(const double *A, const double *y, const int nx, const int nv, double *x, double *Q)
{
	int i,j,k,info=0;
	int *ix;
	double *A_,*x_,*Q_;

	// 创建一个 nx1 的整型矩阵 ix，用于存参数的下标
	ix=imat(nx,1);

	// 两个嵌套循环用于选择对线性方程有贡献的未知数。
	// 对于每一个未知数 i，如果在系数矩阵 A 中对应的元素绝对值大于 10e-10，
	// 则将该未知数的索引存储在 ix 中
	for (i=k=0;i<nx;i++) {
		for (j=0;j<nv;j++) {
			if (fabs(A[j*nx+i])>1.0e-10) {
				ix[k++]=i;
				j=1000;
			}
		}
	}

	A_=mat(k*nv,1); x_=mat(k,1); Q_=mat(k*k,1);

	// 接下来的两个嵌套循环用于从系数矩阵 A 和未知数 x 中抽取对应于选定的未知数的行和列
	for (j=0;j<k;j++) {
		for (i=0;i<nv;i++) {
			A_[i*k+j]=A[i*nx+ix[j]];
		}

		x_[j]=x[ix[j]];
	}

	// 调用和 RTKLIB 相同的 lsq 函数最小二乘求解
	/* least square estimation */
	info=lsq(A_,y,k,nv,x_,Q_);

	for (i=0;i<nx*nx;i++) Q[i]=0.0;

	for (i=0;i<k;i++) {
		x[ix[i]]=x_[i];

		for (j=0;j<k;j++)
			Q[ix[i]+ix[j]*nx]=Q_[i+j*k];
	}

	free(ix); free(A_); free(x_); free(Q_);

	return info;
}
```

### 3、lsq()

- **A**：nm 阶设计矩阵的转置，m<n 则无法计算。
- **y**：m 阶观测残差，**y=v=l-HX** 。
- **X**：传出参数、待估计的 n 阶参数向量的增量。
- **Q**：传出参数、nn协方差阵。

```c
extern int lsq(const double *A, const double *y, int n, int m, double *x,
               double *Q)
{
    double *Ay;
    int info;
    
    if (m<n) return -1;
    Ay=mat(n,1);
    matmul("NN",n,1,m,1.0,A,y,0.0,Ay); /* Ay=A*y */
    matmul("NT",n,n,m,1.0,A,A,0.0,Q);  /* Q=A*A' */
    if (!(info=matinv(Q,n))) matmul("NN",n,1,n,1.0,Q,Ay,0.0,x); /* x=Q^-1*Ay */
    free(Ay);
    return info;
}
```

