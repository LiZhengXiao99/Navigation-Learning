[TOC]

## 一、pppos()：精密单点定位主入口函数

### 1、函数调用流程图

![1684128200085](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1684128200085.png)

### 2、传入参数

```c
rtk_t    *rtk      IO  rtk控制结构体
obsd_t   *obs      I   OBS观测数据
int      n         I   OBS观测数据的数量
nav_t    *nav      I   导航数据
```

### 3、执行流程

* 遍历所有卫星、所有频率，结果`rtk->ssat[i].fix[j]`赋值0 
* **时间更新**：调用`udstate_ppp()`，更新状态值`rtk->x`及其误差协方差`rtk->P`
* 调用`satposs()`，计算**卫星位置**`rs`、卫星钟差`dts`
* 调用`testeclipse()`，排除需要排除的卫星
* 调用`tidedisp()`，计算海洋潮汐以及极潮对接收机位置产生的影响
* 滤波迭代、默认次数(8次)  
  * 调用`matcpy()`，复制 `rtk->x`到`xp`，`rtk->P`到`Pp`
  * 调用`ppp_res()`，计算残差`V`，设计矩阵`H`
  * 调用`filter()`，kalman滤波**量测更新**解算出`xp`、`Pp`
  * 解算后，进行后验残差估计，符合限制则输出为精密单点定位状态，将解算结果`xp`、`Pp`赋值到`rtk->x`、`rtk->P`，break迭代
* 如果迭代次数超过设置次数，输出“迭代溢出”
* 如果解的状态是`SOLQ_PPP`
  * 调用`ppp_ar()`进行**模糊度固定**，`ppp_res()`进行**后验残差**估计
  * 检验通过，将解算结果`xp`、`Pp`赋值到`rtk->xa`、`rtk->Pa，`计算三个未知参数的方差，方差向量的模如果`<MAX_STD_FIX`，解的状态设为固定解`SOLQ_FIX`
  * 检验不通过，`rtk->nfix`值设置为0
  * 调用`update_stat()`，更新solution状态

```c
extern void pppos(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    const prcopt_t *opt=&rtk->opt;
    double *rs,*dts,*var,*v,*H,*R,*azel,*xp,*Pp,dr[3]={0},std[3];
    char str[32];
    int i,j,nv,info,svh[MAXOBS],exc[MAXOBS]={0},stat=SOLQ_SINGLE;
    
    time2str(obs[0].time,str,2);
    trace(3,"pppos   : time=%s nx=%d n=%d\n",str,rtk->nx,n);
    
    rs=mat(6,n); dts=mat(2,n); var=mat(1,n); azel=zeros(2,n);
    
    //遍历所有卫星、所有频率，rtk->ssat[i].fix[j]赋值0
    for (i=0;i<MAXSAT;i++) for (j=0;j<opt->nf;j++) rtk->ssat[i].fix[j]=0;
    //卡尔曼滤波的时间更新，分为位置参数更新，钟差参数更新，对流层、电离层。相位偏差的时间参数更新
    //更新状态值 rtk->x 及其误差协方差 rtk->P
    /* temporal update of ekf states */
    udstate_ppp(rtk,obs,n,nav);     
    
    /* satellite positions and clocks */    //计算卫星位置rs、卫星钟差dts
    satposs(obs[0].time,obs,n,nav,rtk->opt.sateph,rs,dts,var,svh);
    
    /* exclude measurements of eclipsing satellite (block IIA) */
    if (rtk->opt.posopt[3]) {   //调用 testeclipse 进行排除需要排除的卫星
        testeclipse(obs,n,nav,rs);
    }
    /* earth tides correction */    //计算海洋潮汐以及极潮对接收机位置产生的影响
    if (opt->tidecorr) {
        tidedisp(gpst2utc(obs[0].time),rtk->x,opt->tidecorr==1?1:7,&nav->erp,
                 opt->odisp[0],dr);
    }
    nv=n*rtk->opt.nf*2+MAXSAT+3;    
    xp=mat(rtk->nx,1); 
    Pp=zeros(rtk->nx,rtk->nx);
    v=mat(nv,1);        //残差 v 
    H=mat(rtk->nx,nv);  //设计矩阵 H
    R=mat(nv,nv);       //测量误差的协方差 R
    
    //滤波迭代、默认次数(8次) 
    for (i=0;i<MAX_ITER;i++) {  
        
        matcpy(xp,rtk->x,rtk->nx,1);
        matcpy(Pp,rtk->P,rtk->nx,rtk->nx);  
        
        /* prefit residuals */  
        if (!(nv=ppp_res(0,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel))) {
            trace(2,"%s ppp (%d) no valid obs data\n",str,i+1);
            break;
        }   

        /* measurement update of ekf states */
        if ((info=filter(xp,Pp,H,v,R,rtk->nx,nv))) {    //扩展卡尔曼滤波量测更新，
                // filter用于EKF状态的量测更新，先验状态和后验状态信息的更新
                // filter_进行EKF，量测更新部分的解算                                                            
            trace(2,"%s ppp (%d) filter error info=%d\n",str,i+1,info);
            break;
        }
        /* postfit residuals */ //解算后，进行后验残差估计，符合限制则输出为精密单点定位状态
        if (ppp_res(i+1,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel)) {
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
            stat=SOLQ_PPP;
            break;      //结束迭代
        }
    }
    if (i>=MAX_ITER) {
        trace(2,"%s ppp (%d) iteration overflows\n",str,i);
    }
    if (stat==SOLQ_PPP) {
        
        /* ambiguity resolution in ppp */   //模糊度固定，RTKLIB中的ppp_ar没实现直接return 0
        if (ppp_ar(rtk,obs,n,exc,nav,azel,xp,Pp)&&
            ppp_res(9,obs,n,rs,dts,var,svh,dr,exc,nav,xp,rtk,v,H,R,azel)) {     //ppp_res计算残差
            
            matcpy(rtk->xa,xp,rtk->nx,1);
            matcpy(rtk->Pa,Pp,rtk->nx,rtk->nx);
            
            for (i=0;i<3;i++) std[i]=sqrt(Pp[i+i*rtk->nx]);
            if (norm(std,3)<MAX_STD_FIX) stat=SOLQ_FIX;
        }
        else {
            rtk->nfix=0;
        }
        /* update solution status */
        update_stat(rtk,obs,n,stat);    //更新solution状态
        
        /* hold fixed ambiguities */
        if (stat==SOLQ_FIX&&test_hold_amb(rtk)) {
            matcpy(rtk->x,xp,rtk->nx,1);
            matcpy(rtk->P,Pp,rtk->nx,rtk->nx);
            trace(2,"%s hold ambiguity\n",str);
            rtk->nfix=0;
        }
    }
    free(rs); free(dts); free(var); free(azel);
    free(xp); free(Pp); free(v); free(H); free(R);
}
```



## 二、udstate_ppp()：Kalman滤波时间更新

### 1、传入参数

```c
rtk_t    *rtk      IO  rtk控制结构体
obsd_t   *obs      I   OBS观测数据
int      n         I   OBS观测数据的数量
nav_t    *nav      I   导航数据
```

### 2、执行流程

* 调用`udpos_ppp()`，位置参数更新
* 调用`udclk_ppp()`，钟差参数更新
* 如果在配置中选择的对流层模型为ZTD estimation或者ZTD+grad estimation，调用`udtrop_ppp()`，对流层参数更新 
* 如果在配置中选择电离层模型为estimation，调用`udiono_ppp()`，电离层参数更新
* 如果选择的频率数>=3，调用`uddcb_ppp()`，L5接收机硬件延迟参数更新
* 调用`udbias_ppp()`，整周模糊度参数更新

```c
static void udstate_ppp(rtk_t *rtk, const obsd_t *obs, int n, const nav_t *nav)
{
    trace(3,"udstate_ppp: n=%d\n",n);
    
    /* temporal update of position */   //调用 udpos_ppp 根据不同模式初始化状态 rtk->x 中的位置值
    udpos_ppp(rtk);     //位置参数更新
    
    /* temporal update of clock */  //调用 udclk_ppp 初始化状态 rtk->x 中的钟差值（6个，因有6个系统）
    udclk_ppp(rtk);     //钟差参数更新
    
    /* temporal update of tropospheric parameters */
    if (rtk->opt.tropopt==TROPOPT_EST||rtk->opt.tropopt==TROPOPT_ESTG) {
        udtrop_ppp(rtk);    //对流层参数更新
    }
    /* temporal update of ionospheric parameters */
    if (rtk->opt.ionoopt==IONOOPT_EST) {
        udiono_ppp(rtk,obs,n,nav);  //电离层参数更新
    }
    /* temporal update of L5-receiver-dcb parameters */
    if (rtk->opt.nf>=3) {
        uddcb_ppp(rtk);     //更新L5接收机硬件延迟参数
    }
    /* temporal update of phase-bias */ //调用 udbias_ppp 更新载波相位偏移状态值以及其误差协方差。
    udbias_ppp(rtk,obs,n,nav);      //整周模糊度更新
}
```

### 3、udpos_ppp()：位置参数更新

**执行流程**：

* 如果是PPP固定解模式`PMODE_PPP_FIXED`，直接用已知点的固定坐标初始化 
* 如果是首历元，赋值单点定位的解`rtk->sol.rr`、`VAR_POS`，动力学模式，还要赋值速度、加速度
* 如果是PPP静态`PMODE_PPP_STATIC`模式，状态量不变，只给`P`矩阵加过程噪声
* 如果`PMODE_PPP_KINEMA`但非dynamics模式，用单点定位解`rtk->sol.rr[i]`和`VAR_POS`赋值
* 动力学模式动态PPP，构建状态转移矩阵`F`，进行状态转移
* 为`Q`矩阵加速度部分加过程噪声 

```c
static void udpos_ppp(rtk_t *rtk)
{
    double *F,*P,*FP,*x,*xp,pos[3],Q[9]={0},Qv[9];
    int i,j,*ix,nx;
    
    trace(3,"udpos_ppp:\n");
    
    //如果是PPP固定解模式，直接用已知点的固定坐标初始化
    /* fixed mode */
    if (rtk->opt.mode==PMODE_PPP_FIXED) {       
        for (i=0;i<3;i++) initx(rtk,rtk->opt.ru[i],1E-8,i);
        return;
    }
    //如果是首历元，赋值单点定位的解
    /* initialize position for first epoch */
    if (norm(rtk->x,3)<=0.0) {
        for (i=0;i<3;i++) initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        if (rtk->opt.dynamics) {    //动力学模式，赋值速度、加速度
            for (i=3;i<6;i++) initx(rtk,rtk->sol.rr[i],VAR_VEL,i);
            for (i=6;i<9;i++) initx(rtk,1E-6,VAR_ACC,i);
        }
    }
    //如果是PMODE_PPP_STATIC 模式，状态量不变，只给P矩阵加过程噪声
    /* static ppp mode */
    if (rtk->opt.mode==PMODE_PPP_STATIC) {      
        for (i=0;i<3;i++) {
            rtk->P[i*(1+rtk->nx)]+=SQR(rtk->opt.prn[5])*fabs(rtk->tt);
        }
        return;
    }

    //如果PMODE_PPP_KINEMA但非dynamics模式，用单点定位解rtk->sol.rr[i]和VAR_POS赋值
    /* kinmatic mode without dynamics */    
    if (!rtk->opt.dynamics) {
        for (i=0;i<3;i++) {
            initx(rtk,rtk->sol.rr[i],VAR_POS,i);
        }
        return;
    }
    //动力学模式动态PPP，构建状态转移矩阵F
    /* generate valid state index */        
    ix=imat(rtk->nx,1);
    for (i=nx=0;i<rtk->nx;i++) {
        if (rtk->x[i]!=0.0&&rtk->P[i+i*rtk->nx]>0.0) ix[nx++]=i;
    }
    if (nx<9) {     //参数需要大于9个
        free(ix);
        return;
    }
    
    //状态转移矩阵构建
    /* state transition of position/velocity/acceleration */    
    F=eye(nx); P=mat(nx,nx); FP=mat(nx,nx); x=mat(nx,1); xp=mat(nx,1);   
    for (i=0;i<6;i++) {
        F[i+(i+3)*nx]=rtk->tt;
    }
    for (i=0;i<3;i++) {
        F[i+(i+6)*nx]=SQR(rtk->tt)/2.0;
    }
    for (i=0;i<nx;i++) {
        x[i]=rtk->x[ix[i]];
        for (j=0;j<nx;j++) {
            P[i+j*nx]=rtk->P[ix[i]+ix[j]*rtk->nx];
        }
    }
    // 状态转移
    /* x=F*x, P=F*P*F+Q */
    matmul("NN",nx,1,nx,1.0,F,x,0.0,xp);
    matmul("NN",nx,nx,nx,1.0,F,P,0.0,FP);
    matmul("NT",nx,nx,nx,1.0,FP,F,0.0,P);
    
    for (i=0;i<nx;i++) {
        rtk->x[ix[i]]=xp[i];
        for (j=0;j<nx;j++) {
            rtk->P[ix[i]+ix[j]*rtk->nx]=P[i+j*nx];
        }
    }

    //为Q矩阵加速度部分加过程噪声
    /* process noise added to only acceleration */  
    Q[0]=Q[4]=SQR(rtk->opt.prn[3])*fabs(rtk->tt);
    Q[8]=SQR(rtk->opt.prn[4])*fabs(rtk->tt);
    ecef2pos(rtk->x,pos);   //XYZ->BLH
    covecef(pos,Q,Qv);
    for (i=0;i<3;i++) for (j=0;j<3;j++) {
        rtk->P[i+6+(j+6)*rtk->nx]+=Qv[i+j*3];
    }
    free(ix); free(F); free(P); free(FP); free(x); free(xp);
}
```

### 4、udclk_ppp()：钟差参数更新 

**执行流程：**

* 检测是否为精密星历`EPHOPT_PREC`，认为精密钟差是白噪声,每个历元初始化，精密星历的时间是基于gps时间，忽略系统间的偏差
* 不是精密星历则利用前一秒的结果，并考虑系统间时差信息

```c
static void udclk_ppp(rtk_t *rtk)
{
    double dtr;
    int i;
    
    trace(3,"udclk_ppp:\n");

    //认为钟差是白噪声,每个历元初始化
    /* initialize every epoch for clock (white noise) */    
    for (i=0;i<NSYS;i++) {
        //检测是否为精密星历EPHOPT_PREC，精密星历的时间是基于gps时间，忽略系统间的偏差
        if (rtk->opt.sateph==EPHOPT_PREC) {     //精密星历
            /* time of prec ephemeris is based gpst */
            /* negelect receiver inter-system bias  */
            dtr=rtk->sol.dtr[0];
        }
        //不是精密星历则利用前一秒的结果，并考虑系统间时差信息
        else {
            dtr=i==0?rtk->sol.dtr[0]:rtk->sol.dtr[0]+rtk->sol.dtr[i];
        }
        initx(rtk,CLIGHT*dtr,VAR_CLK,IC(i,&rtk->opt));
    }
}
```





### 5、udtrop_ppp()：对流层参数更新 



### 6、udiono_ppp()：电离层参数更新



### 7、uddcb_ppp()：L5接收机硬件延迟参数更新



### 8、udbias_ppp()：整周模糊度参数更新 

#### 1.传入参数

```c
IO    rtk_t *rtk			rtk solution structure
I     const obsd_t *obs	当前历元观测值
I     int n 				当前移动站观测值数目
I     const nav_t *nav     星历 
```

#### 2.执行流程

* 日界线检测
* **周跳检测**：清除卫星周跳标志位`rtk->ssat[i].slip`，调用`detslp_ll()`，通过LLI检测周跳，调用`detslp_gf()`，通过几何无关组合检测周跳，调用`detslp_mw()`，MW组合检测周跳
* 遍历各个频点：
* 遍历各个卫星，判断是否需要**重置单差相位偏移**状态量，如果检测到相位中断大于门限，或`ARMODE_INST`单历元模糊度固定模式，或检测到时钟跳变，则重新赋值模糊度参数。
* 遍历各个卫星，对每一组观测数据，调用`corrmeas()`，进行天线相位中心修正，相位缠绕修正，卫星硬件延迟修正
* `bias`赋赋值0；消电离层模式，不用再考虑电离层延迟，直接计算`bias[i]=Lc-Pc`，否则先通过双频伪距计算电离层延迟`ion=(obs[i].P[0]-obs[i].P[f])/(1.0-SQR(freq1/freq2));`，利用修正后的电离层延迟计算`bias[i]=L[f]-P[f]+2.0*ion*SQR(freq1/freq2); `
* 检测值是否有效，是否周跳，如果正常，统计初始模糊度和状态量之间的整体偏差`offset`
* 判断如果满足补偿门限，若载波和伪距跳变太大，为了保持一致性，需要进行校正，在原有的载波相位偏差状态量上加上`offset`的平均值，以此来作为载波相位偏差的时间更新
* 遍历各个卫星，给`P`矩阵加过程噪声
* 如果状态量无效，或有周跳发生，则用偏移值`bias`重置`rtk->x`，重置模糊度固定标志位`rtk->ambc[sat-1].flags[k]=0`

## 三、

