>原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

![1698494049(1)](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1698494049(1).png)


[TOC]

![image-20231103180051412](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103180051412.png)

## 一、对流层延迟改正

### 1、原理

对流层一般指距离地面 50km 内的大气层，是大气层质量的主要部分。当导航信号穿过对流层时，由于传播介质密度的增加，信号传播路径和传播速度会发生改变，由此引起的 GNSS 观测值误差称为对流层延迟。对流层延迟一般可分为干延迟和湿延迟，对于载波相位和伪距完全相同，一般在米级大小，可通过模型改正和参数估计的方法来削弱其影响。修正模型如下：
$$
T=M_{d r y} T_{d r y}+M_{w e t} T_{w e t}
$$
式中，$T_{d r y}, T_{w e t}$ 分别表示接收机天顶对流层的干延迟和湿延迟；$M_{d r y}, M_{w e t}$ 分别表示干延迟和湿延迟的投影函数。对流层干延迟比较稳定，主要与测站高度、大气温度和大气压相关，可通过模型改正，常用模型有 Saastamoninen 模型、Hopfield 模型等。湿延迟不同于干延迟，变化较大，主要与水汽含量相关，一般估计天顶对流层湿延迟，通过投影函数计算各卫星的电离层湿延迟，常用的投影函数有全球投影函数（Global Mapping Function，GMF）、Niell 投影函数（NMF）和 Vienna投影函数（Vienna Mapping Function，VMF）等。

### 2、model_trop()：对流层改正入口函数

* Saastamoinen 模型：调用 `tropmodel()` 计算对流层延迟，方差设为 0.3*0.3。
* 估计对流层：从 `x` 中取估计的 ZTD，调用 `trop_model_prec()` 改正，方差在其中计算(0.01*0.01)。

计算得到的对流层改正量，作为参数 `dtrp` 传出

```c
static int model_trop(gtime_t time, const double *pos, const double *azel,
	const prcopt_t *opt, const double *x, double *dtdx,
	const nav_t *nav, double *dtrp, double *shd, double *var)
{
	double trp[3]={0};
	double zwd;

	// Saastamoinen 模型改正计算延迟，方差设为 0.3*0.3
	if (opt->tropopt==TROPOPT_SAAS) {
		*dtrp=tropmodel(time,pos,azel,REL_HUMI,&zwd,0);
		*dtrp+=zwd;
		*var=SQR(ERR_SAAS);
		return 1;
	}
	// 估计对流层模式，从 x 中取估计的 ZTD，调用 trop_model_prec() 改正，方差在其中计算
	if (opt->tropopt==TROPOPT_EST||opt->tropopt==TROPOPT_ESTG) {
		matcpy(trp,x+IT(opt),opt->tropopt==TROPOPT_EST?1:3,1);
		*dtrp=trop_model_prec(time,pos,azel,opt,trp,dtdx,shd,var);
		return 1;
	}
	return 0;
}
```

### 3、tropmodel()：Saastamoinen 模型改正计算延迟

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

### 4、trop_model_prec()：精密对流层模型

* 先调用 `tropmodel()` 相对湿度传 0 计算干延迟 `zhd`。

* 调用 `tropmapf()` NMF 投影函数计算干延迟投影系数 `m_h`、湿延迟投影系数 `m_w`。

* 格网梯度模型计算梯度系数
  $$
  \begin{array}{l}m\left(E l_{r}^{s}\right)=m_{W}\left(E l_{r}^{s}\right)\left\{1+\cot E l_{r}^{s}\left(G_{N, r} \cos A z_{r}^{s}+G_{E, r} \sin A z_{r}^{s}\right)\right\} \\ T_{r}^{s}=m_{H}\left(E l_{r}^{s}\right) Z_{H, r}+m\left(E l_{r}^{s}\right)\left(Z_{T, r}-Z_{H, r}\right)\end{array}
  $$

* 方差设为 0.01*0.01。

* 根据干湿延迟和投影系数计算对流层延迟改正量，返回。
  $$
  T=M_{d r y} T_{d r y}+M_{w e t} T_{w e t}
  $$

```c
static double trop_model_prec(gtime_t time, const double *pos, const double *azel, 
	const prcopt_t *opt, const double *x, double *dtdx, double *shd, double *var)
{
	const double zazel[]={0.0,PI/2.0};
	double zhd,zwd,m_h,m_w,cotz,grad_n,grad_e;

	// 利用对流层误差模型进行改正，然后将残余误差当作一个未知参数进行估计
	/* zenith hydrostatic delay */
	zhd=tropmodel(time,pos,zazel,0.0,&zwd,1);
    PPP_Glo.zhd=zhd;

	// 调用 tropmapf() NMF 投影函数计算干延迟投影系数 m_h、湿延迟投影系数 m_w
	/* mapping function */
	m_h=tropmapf(time,pos,azel,&m_w);

	*shd=m_h*zhd;

	// 格网梯度模型计算梯度系数
	if (opt->tropopt>=TROPOPT_ESTG&&azel[1]>0.0) {
		/* m_w=m_0+m_0*cot(el)*(Gn*cos(az)+Ge*sin(az)): ref [6]  (E.5.5) */
		cotz=1.0/tan(azel[1]);
		grad_n=m_w*cotz*cos(azel[0]);
		grad_e=m_w*cotz*sin(azel[0]);
		m_w+=grad_n*x[1]+grad_e*x[2];
		dtdx[1]=grad_n*(x[0]);			// (E.5.6)
		dtdx[2]=grad_e*(x[0]);
	}
	dtdx[0]=m_w;
	*var=SQR(0.01);	// 方差设为 0.01*0.01

	//return m_h*zhd+m_w*(x[0]-zhd);
	return m_h*zhd+m_w*(x[0]);
}
```

### 5、tropmapf()：计算干湿延迟投影系数

干投影函数是通过返回值获得的，而湿投影是通过输入/输出参数`mapfw`获得的，有两种投影函数的计算方法，分别是 GMF 和 NMF ，默认使用的是 NMF 方法，RTKLIB 可以通过定义`IERS_MODEL`宏来使用 GMF 方法；GAMP 作者好像实现了一个 `tropmapf_gmf()` 函数，但是没有调用。

```c
extern double tropmapf(gtime_t time, const double pos[], const double azel[],
                       double *mapfw)
{
#ifdef IERS_MODEL
    const double ep[]={2000,1,1,12,0,0};
    double mjd,lat,lon,hgt,zd,gmfh,gmfw;
#endif
    
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
    return tropmapf_nmf(time,pos,azel,mapfw); /* NMF */
#endif
}
```

### 6、tropmapf_nmf()：Neill 投影函数

$$
\begin{array}{c}M_{d y}=\frac{1+\frac{a_{d r v}}{1+\frac{b_{d r y}}{1+c_{d r y}}}}{\sin E+\frac{a_{d r y}}{\sin E+\frac{b_{d r y}}{\sin E+c_{d r y}}}}+\left(\frac{1}{\sin E}-\frac{1+\frac{a_{h t}}{1+\frac{b_{h t}}{1+c_{h t}}}}{\sin E+\frac{a_{h t}}{\sin E+\frac{b_{h t}}{\sin E+c_{h t}}}}\right) \\ M_{\text {wet }}=\frac{1+\frac{a_{\text {wet }}}{1+\frac{b_{w e t}}{1+c_{\text {wet }}}}}{\sin E+\frac{a_{w e t}}{\sin E+\frac{b_{\text {wet }}}{\sin E+c_{w e t}}}} \\ a_{d r y}=a_{c v g}(\varphi)+a_{u m p}(\varphi) \cos \left(2 \pi \frac{D O Y-28}{365.25}\right)\end{array}
$$

其中，$E$ 为卫星高度角，$h$ 为测站高程，系数 $a_{h t}=2.53 e-5, b_{h t}=5.49 e-3$，$c_{h t}=1.14 e-3, a_{a v g}(\varphi), a_{a m p}(\varphi)$ 查表内插的方法获得，$D O Y$ 为年积日，如果测站在南半球时 $D O Y-180$

```c
extern double tropmapf_nmf(gtime_t time, const double pos[], const double azel[],
                  double *mapfw)
{
    /* ref [5] table 3 */
    /* hydro-ave-a,b,c, hydro-amp-a,b,c, wet-a,b,c at latitude 15,30,45,60,75 */
    const double coef[][5]={
        { 1.2769934E-3, 1.2683230E-3, 1.2465397E-3, 1.2196049E-3, 1.2045996E-3},
        { 2.9153695E-3, 2.9152299E-3, 2.9288445E-3, 2.9022565E-3, 2.9024912E-3},
        { 62.610505E-3, 62.837393E-3, 63.721774E-3, 63.824265E-3, 64.258455E-3},
        
        { 0.0000000E-0, 1.2709626E-5, 2.6523662E-5, 3.4000452E-5, 4.1202191E-5},
        { 0.0000000E-0, 2.1414979E-5, 3.0160779E-5, 7.2562722E-5, 11.723375E-5},
        { 0.0000000E-0, 9.0128400E-5, 4.3497037E-5, 84.795348E-5, 170.37206E-5},
        
        { 5.8021897E-4, 5.6794847E-4, 5.8118019E-4, 5.9727542E-4, 6.1641693E-4},
        { 1.4275268E-3, 1.5138625E-3, 1.4572752E-3, 1.5007428E-3, 1.7599082E-3},
        { 4.3472961E-2, 4.6729510E-2, 4.3908931E-2, 4.4626982E-2, 5.4736038E-2}
    };
    const double aht[]={ 2.53E-5, 5.49E-3, 1.14E-3}; /* height correction */
    
    double y,cosy,ah[3],aw[3],dm,el=azel[1],lat=pos[0]*R2D,hgt=pos[2];
    int i;
    
    if (el<=0.0) {
        if (mapfw) *mapfw=0.0;
        return 0.0;
    }
    /* year from doy 28, added half a year for southern latitudes */
    y=(time2doy(time)-28.0)/365.25+(lat<0.0?0.5:0.0);
    
    cosy=cos(2.0*PI*y);
    lat=fabs(lat);
    
    for (i=0;i<3;i++) {
        ah[i]=interpc(coef[i  ],lat)-interpc(coef[i+3],lat)*cosy;
        aw[i]=interpc(coef[i+6],lat);
    }
    /* ellipsoidal height is used instead of height above sea level */
    dm=(1.0/sin(el)-mapf(el,aht[0],aht[1],aht[2]))*hgt/1E3;
    
    if (mapfw) *mapfw=mapf(el,aw[0],aw[1],aw[2]);   // mapfw 湿延迟函数
    
    return mapf(el,ah[0],ah[1],ah[2])+dm;   // 返回干延迟投影函数
}
```

## 二、电离层延迟改正

### 1、原理

受太阳辐射的影响，距地面 60km 以上的大气层处于部分电离或完全电离状态，该区域被称为电离层。当电磁波信号通过电离层时，传播速度和传播路径会发生改变，给 GNSS 观测值带来误差，即电离层延迟。电离层延迟大小由电子密度和信号频率决定，影响可达数十米。由于 GNSS 信号的特性，同一频率载波相位和伪距观测值电离层延迟大小相等，符号相反。在一些条件下，可使用经验模型改正或约束观测值中的电离层延迟，常用的电离层延迟经验模型有 Klobuchar 模型、Bent 模型和电离层格网模型等。本文对电离层延迟的处理使用最常用的两种方法：使用非差非组合模型估计各卫星第一频率倾斜电离层延迟；使用无电离层组合消除电离层延迟一阶项，忽略二阶及以上项。

### 2、model_iono()：电离层改正入口函数

* **TEC 格网模型改正**：调用 `iontec()` 计算改正量，方差也在 `iontec()` 中计算。
* **广播星历改正**：调用 `ionmodel()` 克罗布歇模型计算改正量，方差设为 0.5 乘以电离层改正量再平方。
* **电离层估计**：从参数向量 `x` 中取改正量，方差设为 0。
* **L1/L2消电离层组合**：电离层改正量和方差设为 0。

计算得到的 L1 频率的电离层改正量，作为参数 `dion` 传出，当使用其它频率信号时，依据所用信号频组中第一个频率的波长与 L1 波长的比例关系，对上一步得到的电离层延时进行修正，不考虑模糊度情况下改正公式为：
$$
I=\frac{\Phi_{2}-\Phi_{1}}{1-\left(f_{1} / f_{2}\right)^{2}}
$$

### 3、ionmodel()：克罗布歇模型计算电离层改正量

使用克罗布歇模型计算 L1 的电离层改正量，将晚间的电离层时延视为常数，取值为 5ns，把白天的时延看成是余弦函数中正的部分。于是天顶方向调制在 L1 载波上的测距码的电离层时延可表示为：
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

### 4、iontec()：TEC 模型计算电离层改正量

![image-20231104110835242](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104110835242.png)

格网改正模型电离层格网模型文件 IONEX，通过内插获得穿刺点位置，并结合当天电离层格网数据求出穿刺点的垂直电子含量，获得电离层延迟误差 。

#### 1. readtec()：电离层 IONEX 文件读取

IONEX 文件分四大部分：**文件头**结束于`END OF HEADER`。**多组总电子含量**，每组以`START OF TEC MAP` ，结束于`END OF TEC MAP` 。**多组电子含量均方根误差** ，与总电子含量对应，开始于`START OF RMS MAP`，结束于`END OF RMS MAP`，**DCB数据块**开始于`START OF AUS DATA`，结束于`END OF AUS DATA`，也称**辅助数据块**（Auxiliary Data Blocks)。

![image-20231104111521575](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104111521575.png)

**readtec() 执行流程**：

![image-20231104111854787](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104111854787.png)

* 开辟内存空间
* 扩展`file`的*到`efile[]` ，遍历`efile[] `：
* `fopen()`以读的方式打开
* 调用`readionexh()`读 ionex 文件头 
* 调用`readionexb()`读 ionex 文件体 ，其中会调用 `addtec() `将 tec 格网数据存到`nav->tec[]`
* 读取完之后，调用`combtec()`合并 tec 格网数据
* 存 DCB 参数到`nav->cbias`

```c
extern void readtec(const char *file, nav_t *nav, int opt)
{
    FILE *fp;
    double lats[3]={0},lons[3]={0},hgts[3]={0},rb=0.0,nexp=-1.0;
    double dcb[MAXSAT]={0},rms[MAXSAT]={0};
    int i,n;
    char *efiles[MAXEXFILE];
    
    trace(3,"readtec : file=%s\n",file);
    
    /* clear of tec grid data option */
    if (!opt) { //如果没有opt，释放nav->tec，nav->ntmax置0
        free(nav->tec); nav->tec=NULL; nav->nt=nav->ntmax=0;
    }
    for (i=0;i<MAXEXFILE;i++) {     //为efile[]开辟空间
        if (!(efiles[i]=(char *)malloc(1024))) {
            for (i--;i>=0;i--) free(efiles[i]);
            return;
        }
    }
    /* expand wild card in file path */
    n=expath(file,efiles,MAXEXFILE);    //扩展file的*到efile[]
    
    //遍历efile[]
    for (i=0;i<n;i++) {
        if (!(fp=fopen(efiles[i],"r"))) {       //以读的方式打开
            trace(2,"ionex file open error %s\n",efiles[i]);
            continue;
        }
        /* read ionex header */     //调用readionexh()读ionex文件头
        if (readionexh(fp,lats,lons,hgts,&rb,&nexp,dcb,rms)<=0.0) {     
            trace(2,"ionex file format error %s\n",efiles[i]);
            continue;
        }
        /* read ionex body */       //调用readionexb()读ionex文件体
        readionexb(fp,lats,lons,hgts,rb,nexp,nav);
        
        fclose(fp);
    }
    for (i=0;i<MAXEXFILE;i++) free(efiles[i]);
    
    /* combine tec grid data */
    if (nav->nt>0) combtec(nav);    //调用combtec()合并tec格网数据
    
    /* P1-P2 dcb */
    for (i=0;i<MAXSAT;i++) {
        nav->cbias[i][0]=CLIGHT*dcb[i]*1E-9; /* ns->m */	//存DCB到nav->cbias
    }
}
```

调用函数：

* **readionexh**()：读取文件头，循环读取每一行，根据注释读取前面内容，如果遇到`START OF AUX DATA` ，调用 readionexdcb() 读取 DCB 参数。
* **readionexdcb**()：循环读取 DCB 参数和对应的均方根误差，直到`END OF AUS DATA`
* **readionexb**()：循环读取 TEC 格网数据和均方根误差，`type`为 1 是 TEC 格网，`type`为2是均分根误差，调用`addtec()`将tec格网数据存到`nav->tec[]`
* **combtec**()：合并`nav->tec[]`中时间相同的项。

#### 2. iontec()：TEC 格网改正入口函数

![image-20231104112203634](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104112203634.png)

由所属时间段两端端点的 TEC 网格数据**时间插值**计算出电离层延迟 (L1) (m) 


   * 检测高度角和接收机高度是否大于阈值 
   * 从 `nav_t.tec`中找出第一个`tec[i].time`>`time`信号接收时间 
   * 确保 time是在所给出的`nav_t.tec`包含的时间段之中 ，通过确认所找到的时间段的右端点减去左端点，来确保时间间隔不为0 
   * 调用`iondelay()`来计算所属时间段两端端点的电离层延时
   * 由两端的延时，插值计算出观测时间点处的值 

```c
extern int iontec(gtime_t time, const nav_t *nav, const double *pos,
                  const double *azel, int opt, double *delay, double *var)
{
    double dels[2],vars[2],a,tt;
    int i,stat[2];

    // 检测高度角和接收机高度是否大于阈值
    if (azel[1]<MIN_EL||pos[2]<MIN_HGT) {
        *delay=0.0;
        *var=VAR_NOTEC;
        return 1;
    }
    // 从 nav_t.tec 中找出第一个 tec[i].time>time 信号接收时间
    for (i=0;i<nav->nt;i++) {
        if (timediff(nav->tec[i].time,time)>0.0) break;
    }
    // 确保 time 是在所给出的 nav_t.tec 包含的时间段之中
    if (i==0||i>=nav->nt) {
        printf("%s: tec grid out of period\n",time_str(time,0));
        return 0;
    }
    // 通过确认所找到的时间段的右端点减去左端点，来确保时间间隔 != 0
    if ((tt=timediff(nav->tec[i].time,nav->tec[i-1].time))==0.0) {
        printf("tec grid time interval error\n");
        return 0;
    }
    // 调用 iondelay 来计算所属时间段两端端点的电离层延时
    /* ionospheric delay by tec grid data */
    stat[0]=iondelay(time,nav->tec+i-1,pos,azel,opt,dels  ,vars  );
    stat[1]=iondelay(time,nav->tec+i  ,pos,azel,opt,dels+1,vars+1);
    
    // 由两端的延时，插值计算出观测时间点处的值
    if (!stat[0]&&!stat[1]) {   // 两个端点都计算出错，输出错误信息，返回 0
        printf("%s: tec grid out of area pos=%6.2f %7.2f azel=%6.1f %5.1f\n",
              time_str(time,0),pos[0]*R2D,pos[1]*R2D,azel[0]*R2D,azel[1]*R2D);
        return 0;
    }
    // 两个端点都有值，线性插值出观测时间点的值，返回 1
    if (stat[0]&&stat[1]) { /* linear interpolation by time */
        a=timediff(time,nav->tec[i-1].time)/tt;
        *delay=dels[0]*(1.0-a)+dels[1]*a;
        *var  =vars[0]*(1.0-a)+vars[1]*a;
    }
    // 只有一个端点有值，将其结果作为观测时间处的值，返回 1
    else if (stat[0]) { /* nearest-neighbour extrapolation by time */
        *delay=dels[0];
        *var  =vars[0];
    }
    else {
        *delay=dels[1];
        *var  =vars[1];
    }

    return 1;
}
```

#### 3. iondelay()：计算指定时间电离层延迟

   * while大循环`tec->ndata[2]`次：
   * 调用`ionppp()`函数，计算当前电离层高度，穿刺点的位置 {lat,lon,h} (rad,m)和倾斜率
   * 按照`M-SLM`映射函数重新计算倾斜率 
   * 在日固系中考虑地球自转，重新计算穿刺点经度 
   * 调用`interptec()`格网插值获取`vtec `
   * `*delay+=fact*fs*vtec `,`*var+=fact*fact*fs*fs*rms*rms `

```c
static int iondelay(gtime_t time, const tec_t *tec, const double *pos,
                    const double *azel, int opt, double *delay, double *var)
{
    const double fact=40.30E16/FREQ1/FREQ1; /* tecu->L1 iono (m) */
    double fs,posp[3]={0},vtec,rms,hion,rp;
    int i;
    
    *delay=*var=0.0;
    
    for (i=0;i<tec->ndata[2];i++) { /* for a layer */
        hion=tec->hgts[0]+tec->hgts[2]*i;

        // 调用ionppp()函数，计算当前电离层高度，穿刺点的位置 {lat,lon,h} (rad,m)和倾斜率( )
        /* ionospheric pierce point position */
        fs=ionppp(pos,azel,tec->rb,hion,posp);
        
        // 按照 M-SLM 映射函数重新计算倾斜率
        /*if (opt&2) {*/
        /* modified single layer mapping function (M-SLM) ref [2] */
        rp=tec->rb/(tec->rb+hion)*sin(0.9782*(PI/2.0-azel[1]));
        fs=1.0/sqrt(1.0-rp*rp);

        // 在日固系中考虑地球自转，重新计算穿刺点经度
        //} 
        //if (opt&1) {
        //    /* earth rotation correction (sun-fixed coordinate) */
        //    posp[1]+=2.0*PI*timediff(time,tec->time)/86400.0;
        //}

        // 调用 interptec() 格网插值获取 vtec
        /* interpolate tec grid data */
        if (!interptec(tec,i,posp,&vtec,&rms)) return 0;
        
        *delay+=fact*fs*vtec;
        *var+=fact*fact*fs*fs*rms*rms;
    }
    
    return 1;
}
```

#### 4. ionppp()：计算电离层穿刺点

位置 {lat,lon,h} (rad,m)和倾斜率做返回值 
$$
\begin{array}{l}z=\pi / 2-E l_{r}^{s} \\ z^{\prime}=\arcsin \left(\frac{R_{E}}{R_{E}+H} \sin z\right) \\ \alpha=z-z^{\prime} \\ \phi_{I P P}=\arcsin \left(\cos \alpha \sin \phi_{r}+\sin \alpha \cos \phi_{r} \cos A z_{r}^{s}\right)\end{array}
$$

```c
extern double ionppp(const double *pos, const double *azel, double re,
                     double hion, double *posp)
{
    double cosaz,rp,ap,sinap,tanap;
    // z 并不是仰角 azel[1]，而是仰角关于的补角，所以程序中在计算 rp 是采用的是 cos(azel[1]) 的写法
    rp=re/(re+hion)*cos(azel[1]);
    ap=PI/2.0-azel[1]-asin(rp);     // (E.5.14) (E.5.16)
    sinap=sin(ap);
    tanap=tan(ap);
    cosaz=cos(azel[0]);
    posp[0]=asin(sin(pos[0])*cos(ap)+cos(pos[0])*sinap*cosaz);      // (E.5.17)
    
    if ((pos[0]> 70.0*D2R&& tanap*cosaz>tan(PI/2.0-pos[0]))||
        (pos[0]<-70.0*D2R&&-tanap*cosaz>tan(PI/2.0+pos[0]))) {
        posp[1]=pos[1]+PI-asin(sinap*sin(azel[0])/cos(posp[0]));    // (E.5.18a)
    }
    else {
        posp[1]=pos[1]+asin(sinap*sin(azel[0])/cos(posp[0]));       // (E.5.18b)
    }
    return 1.0/sqrt(1.0-rp*rp); // 返回倾斜率

    // 可能因为后面再从 TEC 网格数据中插值时，并不需要高度信息，所以这里穿刺点位置 posp[2] 没有赋值
}
```

#### 5. dataindex()：获取 TEC 格网数据下标

先判断点位是否在格网中，之后获取网格点的 tec 数据在 `tec.data` 中的下标

```c
static int dataindex(int i, int j, int k, const int *ndata) //(i:lat,j:lon,k:hgt)
{
    if (i<0||ndata[0]<=i||j<0||ndata[1]<=j||k<0||ndata[2]<=k) return -1;
    return i+ndata[0]*(j+ndata[1]*k);
}
```

#### 6. interptec()：插值计算穿刺点处 TEC

通过在经纬度网格点上进行双线性插值，计算第 k 个高度层时穿刺点处的电子数总量 TEC
$$
\operatorname{TEC}\left(t, \phi_{I P P}, \lambda_{I P P}\right)=\frac{\left(t-t_{i}\right) T E C\left(t_{i}, \phi_{I P P}, \lambda_{I P P}+\omega\left(t-t_{i}\right)\right)+\left(t_{i+1}-t\right) T E C\left(t_{i+1}, \phi_{I P P}, \lambda_{I P P}+\omega\left(t-t_{i+1}\right)\right)}{t_{i+1}-t_{i}}
$$

```c
static int interptec(const tec_t *tec, int k, const double *posp, double *value,
                     double *rms)
{
    double dlat,dlon,a,b,d[4]={0},r[4]={0};
    int i,j,n,index;
    
    *value=*rms=0.0;    // 将 value和 rms所指向的值置为 0
    
    if (tec->lats[2]==0.0||tec->lons[2]==0.0) return 0;
    
    // 将穿刺点的经纬度分别减去网格点的起始经纬度，再除以网格点间距，对结果进行取整，
    // 得到穿刺点所在网格的序号和穿刺点所在网格的位置(比例) i,j 
    dlat=posp[0]*R2D-tec->lats[0];
    dlon=posp[1]*R2D-tec->lons[0];
    if (tec->lons[2]>0.0) dlon-=floor( dlon/360)*360.0; /*  0<=dlon<360 */
    else                  dlon+=floor(-dlon/360)*360.0; /* -360<dlon<=0 */
    
    a=dlat/tec->lats[2];
    b=dlon/tec->lons[2];
    i=(int)floor(a); a-=i;
    j=(int)floor(b); b-=j;
    
    // 调用 dataindex() 函数分别计算这些网格点的 tec 数据在 tec.data中的下标，
    // 按从左下到右上的顺序
    // 从而得到这些网格点处的 TEC 值和相应误差的标准差
    /* get gridded tec data */
    for (n=0;n<4;n++) {
        if ((index=dataindex(i+(n%2),j+(n<2?0:1),k,tec->ndata))<0) continue;
        d[n]=tec->data[index];
        r[n]=tec->rms [index];
    }
    if (d[0]>0.0&&d[1]>0.0&&d[2]>0.0&&d[3]>0.0) {
        // 穿刺点位于网格内，使用双线性插值计算出穿刺点的 TEC 值
        /* bilinear interpolation (inside of grid) */
        *value=(1.0-a)*(1.0-b)*d[0]+a*(1.0-b)*d[1]+(1.0-a)*b*d[2]+a*b*d[3];
        *rms  =(1.0-a)*(1.0-b)*r[0]+a*(1.0-b)*r[1]+(1.0-a)*b*r[2]+a*b*r[3];
    }
    // 穿刺点不位于网格内，使用最邻近的网格点值作为穿刺点的 TEC 值，不过前提是网格点的 TEC>0
    /* nearest-neighbour extrapolation (outside of grid) */
    else if (a<=0.5&&b<=0.5&&d[0]>0.0) {*value=d[0]; *rms=r[0];}
    else if (a> 0.5&&b<=0.5&&d[1]>0.0) {*value=d[1]; *rms=r[1];}
    else if (a<=0.5&&b> 0.5&&d[2]>0.0) {*value=d[2]; *rms=r[2];}
    else if (a> 0.5&&b> 0.5&&d[3]>0.0) {*value=d[3]; *rms=r[3];}
    // 否则，选用四个网格点中 >0 的值的平均值作为穿刺点的 TEC 值
    else {
        i=0;
        for (n=0;n<4;n++) if (d[n]>0.0) {i++; *value+=d[n]; *rms+=r[n];}
        if(i==0) return 0;
        *value/=i; *rms/=i;
    }
    return 1;
}
```

## 三、卫星天线相位中心改正

### 1、原理

GNSS 的距离测量值为接收机天线至卫星天线的几何距离，而一般精密产品给出的卫星的坐标以卫星质量中心为参考，我们把相位中心和质量中心之间的差异称为卫星天线相位误差。实际测量中，天线相位引起的误差随时间变化，在对其处理时，我们一般将其分为常量和变量两个部分。常量部分称为卫星天线相位中心偏差（Phase Center Offset，PCO），表示卫星质量中心和卫星平均相位中心的差异。变量部分称为相位中心变化（Phase Center Variation，PCV），表示天线瞬时相位中心和平均相位中心之间的差异。从 2006 年 11 月 5 日，IGS 开始使用绝对相位中心改正模型 IGS_05，该模型给出了与天底角相关的卫星 PCV 和不同型号接收机的 PCO。tp://sopac-ftp.ucsd.edu/archive/garner/ gamit/tables/ 可以下载最新的 IGS 天线改正文件，其包括最新 GNSS 卫星和接收机的天线 PCO 和 PCV 改正信息。一般通过 igs14.atx 文件链接到最新的天线文件。

![image-20231103103908693](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103103908693.png)

### 2、文件读取

![image-20231104184233459](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104184233459.png)

> 听说 RTKLIB 的 readantex() 函数有bug，接收机端同时出现 GPS、GLO 的PCO、PCV时，会用 GLO 系统的值覆盖 GPS，不知道 GAMP改了没有。

### 3、setpcv()：设置天线参数

查找各颗卫星的天线参数，并放在 `nav->pcvs` 中；查找接收机的天线参数，并放在 `popt->pcvr` 中，需要注意的是，只有当 `popt->anttype[i]` 是 `*(auto)` 的时候，通过 RINEX 头识别到的接收机天线类型，以及天线的 H/E/N 偏差才会起作用。

### 4、satantoff()：卫星 PCO 改正

分别计算 L1 和 L2 的卫星端 PCO。如果 PPP 中使用的是无电离层组合，因此会计算无电离层组合后的 PCO 修正值。之后在 peph2pos() 函数中，会在由精密星历计算的卫星位置上进行 PCO 修正。
$$
\begin{array}{l}\boldsymbol{e}_{z}^{s}=-\frac{\boldsymbol{r}^{s}}{\left|\boldsymbol{r}^{s}\right|} \\ \boldsymbol{e}_{s}=\frac{\boldsymbol{r}_{\text {sun }}-\boldsymbol{r}^{s}}{\left|\boldsymbol{r}_{\text {sun }}-\boldsymbol{r}^{s}\right|} \\ \boldsymbol{e}_{y}^{s}=\frac{\boldsymbol{e}_{z}^{s} \times \boldsymbol{e}_{s}}{\left|\boldsymbol{e}_{z}^{s} \times \boldsymbol{e}_{s}\right|} \\ \boldsymbol{e}_{x}^{s}=\boldsymbol{e}_{y}^{s} \times \boldsymbol{e}_{z}^{s} \\ \boldsymbol{E}_{s}=\left(\boldsymbol{e}_{x}^{s}, \boldsymbol{e}_{y}^{s}, \boldsymbol{e}_{z}^{s}\right)\end{array}
$$

```c
extern void satantoff(gtime_t time, const double *rs, int sat, const nav_t *nav,
                      double *dant)
{
    const double *lam=nav->lam[sat-1];
    const pcv_t *pcv=nav->pcvs+sat-1;
    double ex[3],ey[3],ez[3],es[3],r[3],rsun[3],gmst,erpv[5]={0};
    double gamma,C1,C2,dant1,dant2;
    int i,j=0,k=1,sys;
    
    /* sun position in ecef */
    sunmoonpos(gpst2utc(time),erpv,rsun,NULL,&gmst);
    
    /* unit vectors of satellite fixed coordinates */
    for (i=0;i<3;i++) r[i]=-rs[i];
    if (!normv3(r,ez)) return;              // (E.8.5)
    for (i=0;i<3;i++) r[i]=rsun[i]-rs[i];
    if (!normv3(r,es)) return;              // (E.8.6)
    cross3(ez,es,r);
    if (!normv3(r,ey)) return;              // (E.8.7)
    cross3(ey,ez,ex);                       // (E.8.8)
    
	sys=satsys(sat,NULL);
    //if (NFREQ>=3&&(sys&(SYS_GAL|SYS_SBS))) k=2;
    if (NFREQ<2||lam[j]==0.0||lam[k]==0.0) return;
    
    // 把 L1 频率转到 L2
    gamma=SQR(lam[k])/SQR(lam[j]);
    C1=gamma/(gamma-1.0);
    C2=-1.0 /(gamma-1.0);
    
	if (sys==SYS_GPS) {
		j=0;
		k=1;
	}
	else if (sys==SYS_GLO) {
		j=0+NFREQ;
		k=1+NFREQ;
	}
	else if (sys==SYS_CMP) {
		j=0+2*NFREQ;
		k=1+2*NFREQ;
	}
	else if (sys==SYS_GAL) {
		j=0+3*NFREQ;
		k=1+3*NFREQ;
	}
	else if (sys==SYS_QZS) {
		j=0+4*NFREQ;
		k=1+4*NFREQ;
	}
    /* iono-free LC */
    for (i=0;i<3;i++) {
        dant1=pcv->off[j][0]*ex[i]+pcv->off[j][1]*ey[i]+pcv->off[j][2]*ez[i];
        dant2=pcv->off[k][0]*ex[i]+pcv->off[k][1]*ey[i]+pcv->off[k][2]*ez[i];
        dant[i]=C1*dant1+C2*dant2;
    }
}
```

### 5、satantpcv()：卫星 PCV 改正

$$
\theta=\arccos \frac{\boldsymbol{e}_{r}^{s T} \boldsymbol{r}^{s}}{\left|\boldsymbol{r}^{s}\right|}
$$

```c
static void satantpcv(int sat, const double *rs, const double *rr, const pcv_t *pcv,
	double *dant)
{
	double ru[3],rz[3],eu[3],ez[3],nadir,cosa;
	int i;

	for (i=0;i<3;i++) {
		ru[i]=rr[i]-rs[i];
		rz[i]=-rs[i];
	}
	if (!normv3(ru,eu)||!normv3(rz,ez)) return;

	cosa=dot(eu,ez,3);
	cosa=cosa<-1.0?-1.0:(cosa>1.0?1.0:cosa);
	nadir=acos(cosa);							// (E.8.10)

	antmodel_s(sat,pcv,nadir,dant);
}
```

## 四、接收机天线相位中心改正

### 1、原理

和卫星相似，接收机端也存在由天线相位中心引起的误差 PCO 和 PCV。GNSS观测量是相对于**接收机天线的平均相位中心**而言的，而接收机天线对中是相对于几何中也而言的，这两种中心一般不重合，两者之差就称为**平均相位中心偏差（PCO）**，其大小可达**毫米级或厘米级**。且接收机天线的相位中也会随卫星信号输入的方向和强度的变化而变化，此时观测时刻的瞬时相位中也与平均相位中心的偏差称为**平均相位中心变化（PCV）**，它与卫星的高度角和方位角有关。因此接收机天线相位偏差由接收机天线PCO和PCV两部分组成。

![image-20231103092827333](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103092827333.png)

对于常见型号的接收机，IGS 给出了 GPS 和 GLONASS 的改正信息，可从天线改正文件中获取。由于缺少 BDS 的接收机天线相位中心改正值，在 PPP 数据处理中，一般将 GPS 的接收机 PCO 和 PCV 信息用于 BDS。

NGS 提供的 ANTEX 格式天线模型，包含了卫星天线模型以及部分接收机天线模型。使用天线模型的目的包括： 

* 修正天线参考点和天线相位中心的之间的偏差。

* 修正和仰角有关的误差。

* 修正 L1 和 L2 之间的相位中心偏差（这个误差可能对整周模糊度固定造成影响）。

  * RTKLIB 支持 NGS PCV 以及 ANTEX 格式的天线模型，其中包括了 PCO 和 PCV 修正参数。通过手册 E.8 章节可知，接收机天线修正如下：

    * **PCO修正**通常是当地坐标系 ENU 参数，因此需要利用转换矩阵转到 ECEF 坐标系:
      $$
      \boldsymbol{d}_{r, p c o, i}=\boldsymbol{E}_{r}{ }^{T} \boldsymbol{d}_{r, p c o, i, e n u}
      $$

    * **PCV修正**则通过对高度角进行插值得到：
      $$
      \boldsymbol{d}_{r, p c o, i}=\boldsymbol{E}_{r}{ }^{T} \boldsymbol{d}_{r, p c o, i, e n u}d_{r, p c v, i}(E l)=\frac{\left(E l-E l_{i}\right) d_{r, p c v, i}\left(E l_{i}\right)+\left(E l_{i+1}-E l\right) d_{r, p c v, i}\left(E l_{i+1}\right)}{E l_{i+1}-E l_{i}}
      $$

### 2、antmodel()：接收机 PCO、PCV 改正

* 计算 LOS 视向量在 ENU 中的单位矢量`e`
* 频段不同，天线的相位中心偏移(PCO)不同，先计算出每个频段天线在东、北、天三个方向总的偏移,即相位中心偏移`pcv->off`与`del[j]`之和。
* 计算相位中心偏移(PCO)在观测单位矢量 `e` 上的投影 `dot(off,e,3)`
* 计算天线相位中心变化量(PCV)：不同的高度角，相位中心变化不同，因此根据高度角对`pcv->var[i]`进行插值计算。
* PCO 和 PCV 两部分求和为 `dant[]`

> `del`为相对天线参考点偏移值
>
> `azel`为方位角和俯仰角，
>
> `pcv->off`为 phase center offset（PCO）
>
> `pcv->var`为 phase center variations （PCV）

```c
extern void antmodel(int sat, const pcv_t *pcv, const double *del, const double *azel,
                     int opt, double *dant)
{
	double e[3],off[3],cosel=cos(azel[1]);
	int i,j,ii=0,sys;

    // 计算视线向量在 ENU 中的单位矢量 e
	e[0]=sin(azel[0])*cosel;
	e[1]=cos(azel[0])*cosel;
	e[2]=sin(azel[1]);

	sys=PPP_Glo.sFlag[sat-1].sys;
	if(strlen(pcv->type)==0) {
		for (i=0;i<NFREQ;i++) {
			if (sys==SYS_GPS||sys==SYS_CMP||sys==SYS_GAL||sys==SYS_QZS) {
				ii = i;
				if (i==2) ii=1;
			}
			else if (sys==SYS_GLO) {
				ii = i+NFREQ;
				if (i==2) ii=1+NFREQ;
			}
            // 相位中心偏移(PCO)，pcv->off[i][j] 中的值来自于天线 PCV 文件
			for (j=0;j<3;j++) off[j]=pcv->off[ii][j]+del[j];

			if (norm(pcv->off[ii],3)>0.0) {
				sprintf(PPP_Glo.chMsg,"norm(pcv->off[ii],3)>0.0\n");
				outDebug(OUTWIN,OUTFIL,0);
			}

            // 相位中心偏移(PCO)在观测单位矢量 e 上的投影 dot(off,e,3)
            // 计算天线相位中心变化量(PCV)：不同的高度角，相位中心变化不同，因此根据高度角对 pcv->var[i] 进行插值计算。
            // dant[] 为上面两部分相加
			dant[i]=-dot(off,e,3)+(opt?interpvar0(0,90.0-azel[1]*R2D,pcv->var[ii],0):0.0);
		}
		return ;
	}

	// 频段不同，天线的相位中心偏移(PCO)不同。
    // 先计算出每个频段天线在东、北、天三个方向总的偏移，即相位中心偏移 pcv->off[i][j] 与 del[j] 之和
	for (i=0;i<NFREQ;i++) {
		if (sys==SYS_GPS||sys==SYS_CMP||sys==SYS_GAL||sys==SYS_QZS) {
			ii = i;
			if (i==2) ii=1;
		}
		else if (sys==SYS_GLO) {
			ii = i+NFREQ;
			if (i==2) ii=1+NFREQ;
		}
        // 相位中心偏移(PCO)，pcv->off[i][j] 中的值来自于天线 PCV 文件
		for (j=0;j<3;j++) off[j]=pcv->off[ii][j]+del[j];

		if (pcv->dazi!=0.0) 
			dant[i]=-dot(off,e,3)+interpvar1(azel[0]*R2D,90-azel[1]*R2D,pcv,ii);
		else
			dant[i]=-dot(off,e,3)+interpvar0(0,90.0-azel[1]*R2D,pcv->var[ii],0);
	}
}
```

## 五、天线相位缠绕改正

GNSS 载波相位是右旋圆极化电磁波，当接收机天线和卫星天线发生相对旋转时，载波相位观测值会因此产生误差，即相位缠绕（phase wind-up）。相位缠绕误差最大可达载波相位的一个波长，需要进行改正。

> 这部分算法模型摘自：[GPS从入门到放弃（二十三） --- 相位缠绕](https://blog.csdn.net/tyst08/article/details/105205817)

如下图所示是卫星、地球与太阳的位置关系：

![image-20231103154023975](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103154023975.png)

在卫星天线上建立卫星天线坐标系，以卫星的天线相位中心为原点； $Z$ 轴沿卫星天线方向指向地心； $X$ 轴在地球、太阳和卫星组成的平面内，指向太阳；Y轴与Z轴和X轴构成右手系。于是可以求得三轴方向的单位矢量 $e_{x s}, e_{y s}, e_{z s}$ 分别为：
$$
\begin{array}{c}
e_{z s}=\frac{-\boldsymbol{r}_{s a t}}{\left|\boldsymbol{r}_{\text {sat }}\right|} \\
e_{y s}=e_{z s} \times \boldsymbol{e}_{s u n} \\
\boldsymbol{e}_{x s}=\boldsymbol{e}_{z s} \times \boldsymbol{e}_{y s}
\end{array}
$$
其中 $\boldsymbol{r}_{\mathrm{sat}}$ 和 $\boldsymbol{r}_{\mathrm{sun}}$ 分别是 ECEF 坐标系中卫星和太阳的位置矢量，而 $\boldsymbol{e}_{\mathrm{sun}}$ 是卫星至太阳方向的单位矢量：
$$
e_{\text {sun }}=\frac{\boldsymbol{r}_{\text {sun }}-\boldsymbol{r}_{\text {sat }}}{\left|\boldsymbol{r}_{\text {sun }}-\boldsymbol{r}_{\text {sat }}\right|}
$$
计算相位缠绕时，在卫星和接收机处各定义一个有效偶极 $\boldsymbol{D}_{s}$ 和 $\boldsymbol{D}_{\boldsymbol{r}}$ ，且分别对应于星固坐标系和接收机所在位置的站心坐标系，有：
$$
\begin{array}{c}
\boldsymbol{D}_{s}=e_{x s}-e_{k}\left(e_{k} \cdot e_{x s}\right)-e_{k} \times e_{y s} \\
\boldsymbol{D}_{r}=e_{x r}-e_{k}\left(e_{k} \cdot e_{x r}\right)+e_{k} \times e_{y r} \\
\phi_{\mathrm{f}}=\operatorname{sign}\left(e_{k} \cdot\left(\boldsymbol{D}_{s} \times \boldsymbol{D}_{r}\right)\right) \arccos \left(\frac{\boldsymbol{D}_{s} \cdot \boldsymbol{D}_{r}}{\left|\boldsymbol{D}_{s}\right|\left|\boldsymbol{D}_{r}\right|}\right)
\end{array}
$$
其中 $e_{k}$ 为卫星至接收机方向的单位向量， $e_{x r} 、 e_{y r}$ 为接收机所在位置的站心坐标系的坐标轴方向的单位矢量，而 $\phi_{\mathrm{f}}$ 为相位缠绕值的小数部分。
$$
e_{k}=\frac{\boldsymbol{r}_{r}-\boldsymbol{r}_{s a t}}{\left|\boldsymbol{r}_{\boldsymbol{r}}-\boldsymbol{r}_{s a t}\right|}
$$
总结可以得出完整公式如下：
$$
\begin{array}{l}\boldsymbol{E}_{r}=\left(\boldsymbol{e}_{r, x}{ }^{T}, \boldsymbol{e}_{r, y}{ }^{T}, \boldsymbol{e}_{r, z}{ }^{T}\right)^{T} \\ \boldsymbol{E}^{s}=\left(\boldsymbol{e}_{x}^{s T}, \boldsymbol{e}_{y}^{s T}, \boldsymbol{e}_{z}^{s T}\right)^{T} \\ \boldsymbol{D}^{s}=\boldsymbol{e}_{x}^{s}-\boldsymbol{e}_{u}^{s}\left(\boldsymbol{e}_{u}^{s} \cdot \boldsymbol{e}_{x}^{s}\right)-\boldsymbol{e}_{u}^{s} \times \boldsymbol{e}_{y}^{s} \\ \boldsymbol{D}_{r}=\boldsymbol{e}_{r, x}-\boldsymbol{e}_{r}^{s}\left(\boldsymbol{e}_{r}^{s} \cdot \boldsymbol{e}_{r, x}\right)+\boldsymbol{e}_{r}^{s} \times \boldsymbol{e}_{r, y} \\ \phi_{p w}=\operatorname{sign}\left(\boldsymbol{e}_{r}^{s} \cdot\left(\boldsymbol{D}^{s} \times \boldsymbol{D}_{r}\right)\right) \arccos \frac{\boldsymbol{D}^{s} \cdot \boldsymbol{D}_{r}}{\left\|\boldsymbol{D}^{s}\right\|\left\|\boldsymbol{D}_{r}\right\|} / 2 \pi+N\end{array}
$$
式中，$\mathbf{e}_{r}^{s}$ 表示卫星指向接收机的单位向量；$\mathbf{x}, \mathbf{y}$ 和 $\mathbf{x}^{\prime}, \mathbf{y}^{\prime}$ 分别表示接收机和卫星的两个有效偶极矢量；sign 表示符号函数。

### 1、model_phw()：计算天线相位缠绕改正

```c
static int model_phw(gtime_t time, int sat, const char *type, int opt,
                     const double *rs, const double *rr, double *phw)
{
    double exs[3],eys[3],ek[3],exr[3],eyr[3],eks[3],ekr[3],E[9];
    double dr[3],ds[3],drs[3],r[3],pos[3],cosp,ph;
    int i;
    
    if (opt<=0) return 1; /* no phase windup */
    
    // 首先调用 sat-yaw 函数，根据卫星的姿态模型计算出卫星本体坐标系 X,Y 方向的单位矢量exs、eys，即上面公式里的SX、SY
    /* satellite yaw attitude model */
    if (!sat_yaw(time,sat,type,opt,rs,exs,eys)) return 0;
    
    // 计算卫星至接收机的单位矢量
    /* unit vector satellite to receiver */
    for (i=0;i<3;i++) r[i]=rr[i]-rs[i];
    if (!normv3(r,ek)) return 0;
    
    // 计算接收机天线在当地坐标系的北向、西向单位矢量
    /* unit vectors of receiver antenna */
    ecef2pos(rr,pos);
    xyz2enu(pos,E);
    exr[0]= E[1]; exr[1]= E[4]; exr[2]= E[7]; /* x = north */
    eyr[0]=-E[0]; eyr[1]=-E[3]; eyr[2]=-E[6]; /* y = west  */
    
    // 根据公式以及前一次的相位缠绕误差计算当前时刻相位缠绕误差
    /* phase windup effect */
    cross3(ek,eys,eks);
    cross3(ek,eyr,ekr);
    for (i=0;i<3;i++) {
        ds[i]=exs[i]-ek[i]*dot(ek,exs,3)-eks[i];
        dr[i]=exr[i]-ek[i]*dot(ek,exr,3)+ekr[i];
    }
    cosp=dot(ds,dr,3)/norm(ds,3)/norm(dr,3);
    if      (cosp<-1.0) cosp=-1.0;
    else if (cosp> 1.0) cosp= 1.0;
    ph=acos(cosp)/2.0/PI;
    cross3(ds,dr,drs);
    if (dot(ek,drs,3)<0.0) ph=-ph;
    
    *phw=ph+floor(*phw-ph+0.5); /* in cycle */
    return 1;
}
```

### 2、sat_yaw()：卫星姿态

由于不同类型卫星制造商的星固坐标系定义不同，为保持一致性，IGS 定义星固系如下

* Z 轴平行于卫星天线信号发射方向并指向地心。
* Y 轴平行于太阳帆板并垂直于太阳、地球和卫星构成的平面。
* X 轴垂直于 Y 轴和 Z 轴并构成右手坐标系并指向太阳入射方向。

GNSS 卫星名义姿态在星固系下 3 轴单位向量 $\boldsymbol{e}_{x} 、 \boldsymbol{e}_{y} 、 \boldsymbol{e}_{z}$ 可由下式确定：
$$
\left.\begin{array}{l}
e_{x}=e_{y} \times e_{z} \\
e_{y}=\frac{e_{\otimes} \times r}{\left|e_{\otimes} \times r\right|} \\
e_{z}=-\frac{r}{|r|}
\end{array}\right\}
$$
式中， $e_{\otimes}$ 为卫星至太阳方向的单位向量； $r$ 为地心指向卫星方向的单位向量； $\mid*\mid$ 表示向量取模运算符。GNSS卫星偏航角 $\varphi$ 定义为沿轨道切线方向与星固系 $X$ 轴之间的夹角：
$$
\varphi=\arccos \left(e_{T} \cdot e_{x}\right) 
$$
式中， $\boldsymbol{e}_{T} 、 \boldsymbol{e}_{X}$ 分别沿轨道切线方向、卫星星固系 X 轴单位向量；$\arccos (\cdot)$ 为反余弦函数。根据太阳高度角、轨道角与下式的几何关系，名义姿态偏航角可以表示为：
$$
\varphi=\arctan 2(-\tan \beta, \sin \mu)
$$
式中， $\beta$ 为太阳高度角； $\mu$ 为轨道角 (以远日点为起点)。对 GNSS 卫星而言，由于卫星的信号发射方向始终指向地心，因此不存在俯仰角与横滚角，卫星姿态仅用偏航姿态角 $\varphi$ 确定 ，如图所示，将卫星在轨切线方向 $\boldsymbol{e}_{T}$ 绕星固系的 $Z$ 轴旋转 $\varphi$ 角度，即可确定星固系 $X$ 轴的指向，因此，卫星在姿态异常时期，偏航姿态模型的建立主要是确定偏航姿态角 $\varphi$ 的变化。

![image-20231103155511844](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103155511844.png)

代码中：

* 调用 `sunmoonpos()` 计算日月 ECEF 坐标 `rs`、`rm`。
* 计算太阳高度角 `beta`、轨道角 `mu`。
* 调用 `yaw_angle()` 计算名义姿态偏航角 `yaw`。
* 计算卫星名义姿态在星固系下 `exs`、`eys`。

```c
static int sat_yaw(gtime_t time, int sat, const char *type, int opt,
	const double *rs, double *exs, double *eys)
{
	double rsun[3],ri[6],es[3],esun[3],n[3],p[3],en[3],ep[3],ex[3],E,beta,mu;
	double yaw,cosy,siny,erpv[5]={0};
	int i;

	// 调用 sunmoonpos() 计算日月 ECEF 坐标 rs、rm
	sunmoonpos(gpst2utc(time),erpv,rsun,NULL,NULL);

	// 计算太阳高度角 beta、轨道角 mu
	/* beta and orbit angle */
	matcpy(ri,rs,6,1);
	ri[3]-=OMGE*ri[1];
	ri[4]+=OMGE*ri[0];
	cross3(ri,ri+3,n);
	cross3(rsun,n,p);
	if (!normv3(rs,es)||!normv3(rsun,esun)||!normv3(n,en)||
		!normv3(p,ep)) return 0;
	beta=PI/2.0-acos(dot(esun,en,3));
	E=acos(dot(es,ep,3));
	mu=PI/2.0+(dot(es,esun,3)<=0?-E:E);
	if      (mu<-PI/2.0) mu+=2.0*PI;
	else if (mu>=PI/2.0) mu-=2.0*PI;

	// 调用 yaw_angle() 计算名义姿态偏航角 yaw
	/* yaw-angle of satellite */
	if (!yaw_angle(sat,type,opt,beta,mu,&yaw)) return 0;

	// 计算卫星名义姿态在星固系下 exs、eys
	/* satellite fixed x,y-vector */
	cross3(en,es,ex);
	cosy=cos(yaw);
	siny=sin(yaw);
	for (i=0;i<3;i++) {
		exs[i]=-siny*en[i]+cosy*ex[i];
		eys[i]=-cosy*en[i]-siny*ex[i];
	}
	return 1;
}
```

### 3、sunmoonpos()：计算 ECEF 下日月坐标

* 根据 ERP 参数计算 UT1 时间
* 调用 sunmoonpos_eci() 计算日月 ECI 坐标
* 调用 eci2ecef() 计算 ECI 到 ECEF 的转换矩阵 U
* 用转换矩阵 U 将日月坐标转到 ECEF

```c
extern void sunmoonpos(gtime_t tutc, const double *erpv, double *rsun,
                       double *rmoon, double *gmst)
{
    gtime_t tut;
    double rs[3],rm[3],U[9],gmst_;
    // 根据 ERP 参数计算 UT1 时间
    tut=timeadd(tutc,erpv[2]); /* utc -> ut1 */
    
    // 调用 sunmoonpos_eci() 计算日月 ECI 坐标
    /* sun and moon position in eci */
    sunmoonpos_eci(tut,rsun?rs:NULL,rmoon?rm:NULL);
    
    // 调用 eci2ecef() 计算 ECI 到 ECEF 的转换矩阵 U
    /* eci to ecef transformation matrix */
    eci2ecef(tutc,erpv,U,&gmst_);
    
    // 用转换矩阵 U 将日月坐标转到 ECEF
    /* sun and moon postion in ecef */
    if (rsun ) matmul("NN",3,1,3,1.0,U,rs,0.0,rsun );
    if (rmoon) matmul("NN",3,1,3,1.0,U,rm,0.0,rmoon);
    if (gmst ) *gmst=gmst_;
}
```

### 4、sunmoonpos_eci()：计算 ECI 下日月坐标

太阳的位置是通过历书时、太阳的视运动、恒星际位置以及太阳辐射量等信息来计算的；而月亮的位置则是通过观察月亮的视运动等信息来计算的。

1. 确定观测时间：通过输入的时间变量（如`tut`）确定观测时间。
2. 计算天文参数：通过函数`ast_args(t, f)`等计算与观测时间相关的天文参数。
3. 确定赤经和赤纬：根据观测时间和相关天文参数计算出太阳或月亮的赤经和赤纬。
4. 计算ECI坐标：使用计算出的赤经和赤纬，结合ECI坐标系的定义，计算出太阳或月亮在ECI坐标系中的位置。

```c
static void sunmoonpos_eci(gtime_t tut, double *rsun, double *rmoon)
{
    const double ep2000[]={2000,1,1,12,0,0};
    double t,f[5],eps,Ms,ls,rs,lm,pm,rm,sine,cose,sinp,cosp,sinl,cosl;
    
    // 从2000年1月1日12时到输入时间当前
    t=timediff(tut,epoch2time(ep2000))/86400.0/36525.0;
    
    // 天文参数计算
    /* astronomical arguments */
    ast_args(t,f);
    
    /* obliquity of the ecliptic */
    eps=23.439291-0.0130042*t;              // 黄赤交角
    sine=sin(eps*D2R); cose=cos(eps*D2R);
    
    /* sun position in eci */
    if (rsun) {
        Ms=357.5277233+35999.05034*t;
        ls=280.460+36000.770*t+1.914666471*sin(Ms*D2R)+0.019994643*sin(2.0*Ms*D2R);
        rs=AU*(1.000140612-0.016708617*cos(Ms*D2R)-0.000139589*cos(2.0*Ms*D2R));
        sinl=sin(ls*D2R); cosl=cos(ls*D2R);
        rsun[0]=rs*cosl;
        rsun[1]=rs*cose*sinl;
        rsun[2]=rs*sine*sinl;
    }
    /* moon position in eci */
    if (rmoon) {
        lm=218.32+481267.883*t+6.29*sin(f[0])-1.27*sin(f[0]-2.0*f[3])+
           0.66*sin(2.0*f[3])+0.21*sin(2.0*f[0])-0.19*sin(f[1])-0.11*sin(2.0*f[2]);
        pm=5.13*sin(f[2])+0.28*sin(f[0]+f[2])-0.28*sin(f[2]-f[0])-
           0.17*sin(f[2]-2.0*f[3]);
        rm=RE_WGS84/sin((0.9508+0.0518*cos(f[0])+0.0095*cos(f[0]-2.0*f[3])+
                   0.0078*cos(2.0*f[3])+0.0028*cos(2.0*f[0]))*D2R);
        sinl=sin(lm*D2R); cosl=cos(lm*D2R);
        sinp=sin(pm*D2R); cosp=cos(pm*D2R);
        rmoon[0]=rm*cosp*cosl;
        rmoon[1]=rm*(cose*cosp*sinl-sine*sinp);
        rmoon[2]=rm*(sine*cosp*sinl+cose*sinp);
    }
}
```

### 4、yaw_angle()、yaw_nominal()：计算卫星名义姿态航偏角

`yaw_angle()` 就是直接调用 `yaw_nominal()` 用下面公式计算：
$$
\varphi=\arctan 2(-\tan \beta, \sin \mu)+\pi
$$

```c
extern int yaw_angle(int sat, const char *type, int opt, double beta, double mu,
	double *yaw)
{
	*yaw=yaw_nominal(beta,mu);
	return 1;
}
```

```c
static double yaw_nominal(double beta, double mu)
{
	if (fabs(beta)<1E-12&&fabs(mu)<1E-12) return PI;
	return atan2(-tan(beta),sin(mu))+PI;
}
```

## 六、潮汐改正

> 潮汐改正这部分，公式的具体参数与 RTKLIB 略有不同，但原理和计算方法一样。

### 1、原理

由于地球不是理想刚体，形状会因其他天体的引力发生形变，所以即使将接收机固定在地球表面，接收机和地心的相位位置也会发生变化，接收机在地固系中的坐标随之改变，由此生的测量误差称为地球潮汐效应，影响如下：

![image-20231103140722802](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231103140722802.png)

GNSS 数据处理一般将潮汐分为三个部分，地球固体潮、海洋负荷潮和极潮。

* **地球固体潮**：是指地球固体在其他天体的引力作用下发生周期变化的现象，对接收机水平和高程方向的影响分别可达数厘米和数分米。
* **海洋负荷潮**：是指在太阳和月球引力作用下地球海洋发生的周期性变化，对接收机水平和高程方向的影响都在厘米级。
* **极潮**：是指地球自转轴发生的瞬时变化引起的测量误差，对接收机影响在厘米级。

关于以上潮汐改正，目前主要通过国际地球自转协议（International Earth Rotation Service，IERS）的 IERS Convention 技术文档中的模型改正。

### 2、tidedisp()：潮汐改正入口函数

```c
extern void tidedisp(gtime_t tutc, const double *rr, int opt, const erp_t *erp,
                     const double *odisp, double *dr)
{
    gtime_t tut;
    double pos[2],E[9],drt[3],denu[3],rs[3],rm[3],gmst,erpv[5]={0};
    int i;
    
    // 如果有 erp 参数，调用 geterp() 获取
    if (erp) {
        geterp(erp,utc2gpst(tutc),erpv);
    }
    // UTC 加上 erpv[2] 得到 UT 时间 tut
    tut=timeadd(tutc,erpv[2]);
    
    dr[0]=dr[1]=dr[2]=0.0;
    
    if (norm(rr,3)<=0.0) return;
    
    pos[0]=asin(rr[2]/norm(rr,3));
    pos[1]=atan2(rr[1],rr[0]);
    xyz2enu(pos,E);
    
    if (opt&1) { /* solid earth tides */
        // 调用 sunmoonpos() 计算日月 ECEF 坐标 rs、rm
        /* sun and moon position in ecef */
        sunmoonpos(tutc,erpv,rs,rm,&gmst);
        
        // 调用 tide_solid() 计算固体潮改正 drt，改正到 dr 
        tide_solid(rs,rm,pos,E,gmst,opt,drt);
        for (i=0;i<3;i++) dr[i]+=drt[i];
    }
    if ((opt&2)&&odisp) { /* ocean tide loading */
        // 调用 tide_oload() 计算海洋潮改正 drt，改正到 dr
        tide_oload(tut,odisp,denu);
        matmul("TN",3,1,3,1.0,E,denu,0.0,drt);
        for (i=0;i<3;i++) dr[i]+=drt[i];
    }
    if ((opt&4)&&erp) { /* pole tide */
        // 调用 tide_pole() 极潮改正到 drt，改正到 dr
        tide_pole(tut,pos,erpv,denu);
        matmul("TN",3,1,3,1.0,E,denu,0.0,drt);
        for (i=0;i<3;i++) dr[i]+=drt[i];
    }
}
```

### 3、获取 ERP 参数

#### 1. ERP 参数介绍

> 内容摘自博客：[GPS从入门到放弃（二十一）地球自转参数](https://blog.csdn.net/tyst08/article/details/104701589)

**地球自转参数**（ERP: Earth rotation parameters）主要包括地球极点的位移和速率、UT1-UTC的时间差、以及由天文观测确定的一天的时间长度与 86400 秒之间的差值 LOD。

地球自转参数可以从ftp服务站[ ftp://cddis.nasa.gov/gnss/products/](https://blog.csdn.net/hltt3838/article/details/120526100?ops_request_misc=&request_id=&biz_id=102&utm_term=GNSSERP参数&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-2-120526100.142^v96^pc_search_result_base7&spm=1018.2226.3001.4187) 下载。IGS 提供的 ERP 数据与精密星历数据放在一个目录中。此路径下的数据是以 GPS 周数（GPS Week）为目录名整理放置的。比如想找 2020 年元旦的 ERP 数据，经过计算知道那一天是GPS周第 2086 周，所以进入2086 目录下去下载相应数据。

**类似于 IGS 精密星历，ERP 参数文件也分为三种：**最终 ERP 参数（IGS Final，标识为 IGS）、快速ERP参数（IGS Rapid，标识为 IGR）、以及超快速ERP参数（IGS Ultra-Rapid，标识为 IGU）。其中超快速ERP参数又分为观测的部分和预测的部分。他们的延时、精度等指标如下表所示。在实际工作中，我们可以根据项目对时间及精度的要求，选取不同类型的文件来使用。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20201006165022218.png)

#### 2. readerp()：读取 ERP 参数

 ERP格式非常简单，几乎不用解释，一看就明白。这里附上2020年1月1日的快速ERP参数文件**igr20863.erp**如下：

![image-20231104150645757](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231104150645757.png)

文件中除了说明，就是一个表格，每一行表示一天的ERP数据。当然，上面这个文件中表格内容只有一行。第一列是时间，这个时间是用简化的儒略日来表示的，**儒略日** 58849.50 就是 2020 年 1 月 1 日。

```c
extern int readerp(const char *file, erp_t *erp)
{
    FILE *fp;
    erpd_t *erp_data;
    double v[14]={0};
    char buff[256];
    
    // 以读的方式打开文件
    if (!(fp=fopen(file,"r"))) {
        printf("erp file open error: file=%s\n",file);
        return 0;
    }
    // 循环读取每一行
    while (fgets(buff,sizeof(buff),fp)) {
        // sscanf 格式化取值，文件头不符合格式，自然就被跳过
        if (sscanf(buff,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   v,v+1,v+2,v+3,v+4,v+5,v+6,v+7,v+8,v+9,v+10,v+11,v+12,v+13)<5) {
            continue;
        }
        // 如果 erp 参数量超出容量，就扩大一倍容量
        if (erp->n>=erp->nmax) {
            erp->nmax=erp->nmax<=0?128:erp->nmax*2;
            erp_data=(erpd_t *)realloc(erp->data,sizeof(erpd_t)*erp->nmax);
            if (!erp_data) {    // 空间开辟失败，就退出不继续读取
                free(erp->data); erp->data=NULL; erp->n=erp->nmax=0;
                fclose(fp);
                return 0;
            }
            erp->data=erp_data;
        }
        erp->data[erp->n].mjd=v[0];
        erp->data[erp->n].xp=v[1]*1E-6*AS2R;
        erp->data[erp->n].yp=v[2]*1E-6*AS2R;
        erp->data[erp->n].ut1_utc=v[3]*1E-7;
        erp->data[erp->n].lod=v[4]*1E-7;
        erp->data[erp->n].xpr=v[12]*1E-6*AS2R;
        erp->data[erp->n++].ypr=v[13]*1E-6*AS2R;
    }
    fclose(fp);
    return 1;
}
```

#### 3. geterp()：插值获取当前时间的 ERP 参数

1. 计算当前 mjd。
2. 若当前时间早于 ERP 参数中最早的时间，采用最早的时间来计算。
3. 若当前时间晚于 ERP 参数中最晚的时间，采用最晚的时间来计算。
4. 若当前时间在 ERP 参数中最早与最晚的时间之间，则先找到最接近的两个时间，然后用插值。

```c
extern int geterp(const erp_t *erp, gtime_t time, double *erpv)
{
	const double ep[]={2000,1,1,12,0,0};
	double mjd,day,a;
	int i=0,j,k;

    // 如果没有 ERP 参数直接返回
	if (erp->n<=0) return 0;

    // 计算当前 mjd
	mjd=51544.5+(timediff(gpst2utc(time),epoch2time(ep)))/86400.0;

    // 若当前时间早于 ERP 参数中最早的时间，采用最早的时间来计算
	if (mjd<=erp->data[0].mjd) {
		day=mjd-erp->data[0].mjd;
		erpv[0]=erp->data[0].xp     +erp->data[0].xpr*day;
		erpv[1]=erp->data[0].yp     +erp->data[0].ypr*day;
		erpv[2]=erp->data[0].ut1_utc-erp->data[0].lod*day;
		erpv[3]=erp->data[0].lod;
		return 1;
	}
    // 若当前时间晚于 ERP 参数中最晚的时间，采用最晚的时间来计算
	if (mjd>=erp->data[erp->n-1].mjd) {
		day=mjd-erp->data[erp->n-1].mjd;
		erpv[0]=erp->data[erp->n-1].xp     +erp->data[erp->n-1].xpr*day;
		erpv[1]=erp->data[erp->n-1].yp     +erp->data[erp->n-1].ypr*day;
		erpv[2]=erp->data[erp->n-1].ut1_utc-erp->data[erp->n-1].lod*day;
		erpv[3]=erp->data[erp->n-1].lod;
		return 1;
	}
    // 若当前时间在 ERP 参数中最早与最晚的时间之间，则先找到最接近的两个时间，然后用插值。
	for (j=0,k=erp->n-1;j<=k;) {
		i=(j+k)/2;
		if (mjd<erp->data[i].mjd) k=i-1; 
		else if (mjd>erp->data[i+1].mjd) j=i+1;
		else break;
	}
	if (erp->data[i].mjd==mjd-erp->data[i+1].mjd) {
		a=0.5;
	}
	else {
		a=(mjd-erp->data[i+1].mjd)/(erp->data[i].mjd-mjd-erp->data[i+1].mjd);

		if (i+1>=erp->n||i<0) {
			printf("i+1>=erp->n || i<0  %d\n",i);
			getchar();
		}
	}
	erpv[0]=(1.0-a)*erp->data[i].xp     +a*erp->data[i+1].xp;
	erpv[1]=(1.0-a)*erp->data[i].yp     +a*erp->data[i+1].yp;
	erpv[2]=(1.0-a)*erp->data[i].ut1_utc+a*erp->data[i+1].ut1_utc;
	erpv[3]=(1.0-a)*erp->data[i].lod    +a*erp->data[i+1].lod;
	return 1;
}
```

### 4、tide_solid()：计算固体潮

固体潮为在太阳、月球等星体的引潮力的作用下，固体地球产生的同期性形变现象；虽然太阳的质量比月球大，但是由于太阳距离地球比月球距离地球远，因此太阳引起的固体潮相对月球较小。其他距离地球更远的天体，其固体潮效应也可忽略不计。随着作用点的位置的变化和太阳、月球位置的变化，固体潮的大小和方向也在不断变化，垂直方向最高可达 30～40 cm。在距离较短的相对定位中，固体潮的影响可以通过差分的方式消除，但精密单点定位不能通过差分的方式消除固体潮的影响，因此需要利用模型对其进行改正。固体潮二阶潮汐项对测站坐标影响的计算公式为：
$$
\Delta \boldsymbol{r}=\sum_{j=2} \frac{G M}{G M} \frac{\boldsymbol{r}^{4}}{\boldsymbol{R}_{j}^{3}}\left\{\left[3 l_{2}(\hat{\boldsymbol{R}}, \cdot \hat{\boldsymbol{r}})\right] \hat{\boldsymbol{R}}_{,}+\left[3\left(\frac{h_{2}}{2}-l_{2}\right)(\hat{\boldsymbol{R}}, \cdot \hat{\boldsymbol{r}})^{2}-\frac{h_{2}}{2}\right] \hat{\boldsymbol{r}}\right\}
$$
式中：

* $G M$：地球的引力常数

* $G M_{2}$：月球的引力常数

* $G M_{3}$：太阳的引力常数

* $r$：地球在地心坐标系中的视线向量的模

* $\hat{r}$ ：$r$ 的単位向量

* $\boldsymbol{R}_{2}$：月球在地心坐标系中的位置向量的模，$\hat{R}_{2}$ 为 $R_{2}$ 的单位向量

* $\boldsymbol{R}_{3}$：月球在地心坐标系中的位置向量的模，$\hat{R}_{3}$ 为 $R_{3}$ 的单位向量

* $l_{2}，h_{2}$：二阶 Love 数和 Shida 数：
  $$
  \left.\begin{array}{l}
  h_{2}=0.6087-0.0006 P_{2}(\sin \varphi) \\
  l_{2}=0.0847-0.0002 P_{2}(\sin \varphi)
  \end{array}\right\}
  $$
  式中，$\varphi$ 为测站的地心纬度；$P_{2}(\sin \varphi)=\frac{3 \sin ^{2} \varphi-1}{2}$ 。固体潮二阶项为固体潮改正的主要部分,最大可达分米级。


固体潮三阶潮汐项对测站位置影响的公式为：
$$
\begin{aligned}
\Delta \overline{\boldsymbol{r}}=\sum_{j=2}^{3} \frac{\mathrm{GM}}{\mathrm{GM}} \frac{\boldsymbol{r}^{5}}{\boldsymbol{R}_{j}^{4}}\left\{\left[h_{3} \hat{\boldsymbol{r}}\left(\frac{5}{2}\left(\hat{\boldsymbol{R}}_{j} \cdot \hat{\boldsymbol{r}}\right)^{3}-\frac{3}{2}(\hat{\boldsymbol{R}}, \hat{\boldsymbol{r}})\right] \hat{\boldsymbol{R}}_{j}+\right.\right. \\
l_{3}\left(\frac{15}{2}\left(\hat{\boldsymbol{R}}_{j} \cdot \hat{\boldsymbol{r}}^{2}-\frac{3}{2}\right)\left[\hat{\boldsymbol{R}}_{j}-\left(\hat{\boldsymbol{R}}_{j} \cdot \hat{\boldsymbol{r}}\right) \hat{\boldsymbol{r}}\right]\right\}
\end{aligned}
$$
式中，$h_{3}=0.292 ; l_{3}=0.015$。

一般由上式计算出的由太阳引起的固体湖改正较小，可以忽略不计，月球引起的固体潮改正为毫米级。

此外，固体潮的影响除了使测站产生周期性的位移外，其引湖位的零频项还会引起测站的永久性位移。设测站在径向和北向的永久性位移分别为 $U_{\mathrm{r}}$ 和 $U_{\mathrm{n}}$，则有：
$$
\left.\begin{array}{l}
U_{\mathrm{r}}=\left[-0.1206+0.001 P_{2}(\sin \varphi)\right] P_{2}(\sin \varphi) \\
U_{\mathrm{n}}=\left[-0.0252-0.001 P_{2}(\sin \varphi)\right] \sin (2 \varphi)
\end{array}\right\}
$$

代码中改正项比上面介绍的好像多一点，先调用两次 `tide_pl()` 计算日月潮汐二阶三阶改正项，然后计算频域改正项，再计算永久形变项。

```c
static void tide_pl(const double *eu, const double *rp, double GMp,
                    const double *pos, double *dr)
{
    const double H3=0.292,L3=0.015;     // 三阶 Love、Shida 数
    double r,ep[3],latp,lonp,p,K2,K3,a,H2,L2,dp,du,cosp,sinl,cosl;
    int i;
    
    if ((r=norm(rp,3))<=0.0) return;
    
    for (i=0;i<3;i++) ep[i]=rp[i]/r;
    
    K2=GMp/GME*SQR(RE_WGS84)*SQR(RE_WGS84)/(r*r*r);
    K3=K2*RE_WGS84/r;
    latp=asin(ep[2]); lonp=atan2(ep[1],ep[0]);
    cosp=cos(latp); sinl=sin(pos[0]); cosl=cos(pos[0]);
    
    // 二阶项
    /* step1 in phase (degree 2) */
    p=(3.0*sinl*sinl-1.0)/2.0;
    H2=0.6078-0.0006*p;         // 二阶 Love 数
    L2=0.0847+0.0002*p;         // 二阶 Shida 数
    a=dot(ep,eu,3);
    dp=K2*3.0*L2*a;
    du=K2*(H2*(1.5*a*a-0.5)-3.0*L2*a*a);
    
    // 三阶项
    /* step1 in phase (degree 3) */
    dp+=K3*L3*(7.5*a*a-1.5);
    du+=K3*(H3*(2.5*a*a*a-1.5*a)-L3*(7.5*a*a-1.5)*a);
    
    /* step1 out-of-phase (only radial) */
    du+=3.0/4.0*0.0025*K2*sin(2.0*latp)*sin(2.0*pos[0])*sin(pos[1]-lonp);
    du+=3.0/4.0*0.0022*K2*cosp*cosp*cosl*cosl*sin(2.0*(pos[1]-lonp));
    
    dr[0]=dp*ep[0]+du*eu[0];
    dr[1]=dp*ep[1]+du*eu[1];
    dr[2]=dp*ep[2]+du*eu[2];
}
```

```c
static void tide_solid(const double *rsun, const double *rmoon,
                       const double *pos, const double *E, double gmst, int opt,
                       double *dr)
{
    double dr1[3],dr2[3],eu[3],du,dn,sinl,sin2l;
    
    // 时域
    /* step1: time domain */
    eu[0]=E[2]; eu[1]=E[5]; eu[2]=E[8];
    tide_pl(eu,rsun, GMS,pos,dr1);
    tide_pl(eu,rmoon,GMM,pos,dr2);
    
    // 频域，只有 K1 半径
    /* step2: frequency domain, only K1 radial */
    sin2l=sin(2.0*pos[0]);
    du=-0.012*sin2l*sin(gmst+pos[1]);
    
    dr[0]=dr1[0]+dr2[0]+du*E[2];
    dr[1]=dr1[1]+dr2[1]+du*E[5];
    dr[2]=dr1[2]+dr2[2]+du*E[8];
    
    // 消除永久形变
    /* eliminate permanent deformation */
    if (opt&8) {
        sinl=sin(pos[0]); 
        du=0.1196*(1.5*sinl*sinl-0.5);
        dn=0.0247*sin2l;
        dr[0]+=du*E[2]+dn*E[1];
        dr[1]+=du*E[5]+dn*E[4];
        dr[2]+=du*E[8]+dn*E[7];
    }
}
```

### 5、tide_oload()：计算海潮负荷

海洋在日月引力等作用下产生潮汐变化，使得实际海平面相对于平均海平面发生周期性涨落,称为海洋潮。固体地球对海水质量潮汐分布产生的弹性响应称为海洋负荷效应。海洋潮的影响随测站与海洋距离的增加而不断减弱，近海地区可达厘米级，离海洋较远的地区约为毫米级。对于高精度精密单点定位而言，沿海地区应考虑海洋潮的影响，距离海洋 $1000 \mathrm{~km}$ 以上的测站，海洋潮影响可不予考虑。

由 IERS 2010 可知，测站的海洋潮瞬时位移 $\Delta c_{k}(k=1$ 表示为东方向、 $k=2$ 为北方向、 $k=3$ 为垂直方向) 可表示为 11 个潮波 (4 个半日潮波 $M_{2}, S_{2}, N_{2}, K_{2}, 4$ 个周日潮波 $K_{1}$, $O_{1}, P_{1}, Q_{1}$ 和 3 个长周期潮波 $\left.M_{f}, M_{m}, S_{s a}\right)$ 海洋潮共同影响的矢量叠加：
$$
\Delta c_{k}=\sum_{j=1}^{11} f_{j} A_{k, j} \cos \left(\omega_{j} t+\chi_{j}+\mu_{j}-\Phi_{k, j}\right)
$$
式中，$A_{k, j}$ 和 $\Phi_{k, j}$ 分别表示潮波 $j$ 在 $k$ 方向的振幅和 Greenwich 相位；$\omega_{j}$ 和 $\chi_{j}$ 分别为潮波 $j$ 的角频率和天文幅角；$f_{j}$ 和 $\mu_{j}$ 为顾及月亮轨道升交点调节作用的参数 (周期约为 $18.6 \mathrm{a}$ )，分别称为节点因数和天文相角。在 $1 \sim 3 \mathrm{~mm}$ 的海洋负荷位移改正精度下，可以认为 $f_{j}=1$ 和 $\mu_{j}=0$，则式可以简化为：
$$
\Delta c_{k}=\sum_{j=1}^{11} A_{k, j} \cos \left(\omega_{j} t+\chi_{j}-\Phi_{k, j}\right)
$$
不同研究机构根据卫星测高、船测等数据建立了多个海洋潮汐模型，如 CSR、OSU、FES、GOT 等都有多个版本，不同模型之间的差异可以达到 3mm 左右。

GAMP 中先定义了 11 个潮波参数 args，然后计算当前时间的天文参数，再计算由 11 个潮波引起的位移。

```c
static void tide_oload(gtime_t tut, const double *odisp, double *denu)
{
    // 11 个潮波定义
    const double args[][5]={
        {1.40519E-4, 2.0,-2.0, 0.0, 0.00},  /* M2 */
        {1.45444E-4, 0.0, 0.0, 0.0, 0.00},  /* S2 */
        {1.37880E-4, 2.0,-3.0, 1.0, 0.00},  /* N2 */
        {1.45842E-4, 2.0, 0.0, 0.0, 0.00},  /* K2 */
        {0.72921E-4, 1.0, 0.0, 0.0, 0.25},  /* K1 */
        {0.67598E-4, 1.0,-2.0, 0.0,-0.25},  /* O1 */
        {0.72523E-4,-1.0, 0.0, 0.0,-0.25},  /* P1 */
        {0.64959E-4, 1.0,-3.0, 1.0,-0.25},  /* Q1 */
        {0.53234E-5, 0.0, 2.0, 0.0, 0.00},  /* Mf */
        {0.26392E-5, 0.0, 1.0,-1.0, 0.00},  /* Mm */
        {0.03982E-5, 2.0, 0.0, 0.0, 0.00}   /* Ssa */
    };
    const double ep1975[]={1975,1,1,0,0,0};
    double ep[6],fday,days,t,t2,t3,a[5],ang,dp[3]={0};
    int i,j;
    
    // 角度参数
    /* angular argument: see subroutine arg.f for reference [1] */
    time2epoch(tut,ep);
    fday=ep[3]*3600.0+ep[4]*60.0+ep[5];
    ep[3]=ep[4]=ep[5]=0.0;
    days=timediff(epoch2time(ep),epoch2time(ep1975))/86400.0+1.0;
    t=(27392.500528+1.000000035*days)/36525.0;
    t2=t*t; t3=t2*t;
    
    a[0]=fday;
    a[1]=(279.69668+36000.768930485*t+3.03E-4*t2)*D2R; /* H0 */
    a[2]=(270.434358+481267.88314137*t-0.001133*t2+1.9E-6*t3)*D2R; /* S0 */
    a[3]=(334.329653+4069.0340329577*t-0.010325*t2-1.2E-5*t3)*D2R; /* P0 */
    a[4]=2.0*PI;
    
    // 计算由 11 个潮波引起的位移
    /* displacements by 11 constituents */
    for (i=0;i<11;i++) {
        ang=0.0;
        for (j=0;j<5;j++) ang+=a[j]*args[i][j];
        for (j=0;j<3;j++) dp[j]+=odisp[j+i*6]*cos(ang-odisp[j+3+i*6]*D2R);
    }
    denu[0]=-dp[1];
    denu[1]=-dp[2];
    denu[2]= dp[0];
}
```

### 6、tide_pole()：计算极潮

极移 (Polar Motion) 是地球瞬时自转轴在地球本体内的运动，表现为以极点为中心，在 $\pm 4^{\prime \prime}$ (相当于 $24 \mathrm{~m} \times 24 \mathrm{~m}$ ) 范围内，按与地球自转相同的方向形成一条时伸时缩的螺旋形曲线，即由于地壳对极移的弹性响应造成的地球自转轴变化。
假设测站的坐标为 $(\varphi, \lambda, h)$，极潮在径向与平面对该测站的影响分别为：
$$
\left.\begin{array}{l}
S_{r}=-32 \sin (2 \theta)\left(m_{1} \cos \lambda+m_{2} \sin \lambda\right) \\
S_{\theta}=-9 \cos (2 \theta)\left(m_{1} \cos \lambda+m_{2} \sin \lambda\right) \\
S_{\lambda}=9 \cos \theta\left(m_{1} \sin \lambda-m_{2} \cos \lambda\right)
\end{array}\right\}
$$
式中，$S$，为径向变形 (向上为正)，$S_{\theta}$ 为平面变形 (南北方向，向南为正) 、 $S_{\lambda}$ 为平面变形 (东西方向，向东为正)，单位均为毫米。其中 $\theta=\pi / 2-\varphi$,称为余纬。假设计算极潮改正时刻对应的 ERP 极移参数为 $\left(x_{p}, y_{p}\right)$，则有如下辅助公式：
$$
\left.\begin{array}{c}
m_{1}=x_{p}-\bar{x}_{p}, m_{2}=-\left(y_{p}-\bar{y}_{p}\right) \\
\bar{x}_{p}(t)=\bar{x}_{p}\left(t_{0}\right)+\left(t-t_{0}\right) \dot{\bar{x}}_{p}\left(t_{0}\right) \\
\bar{y}_{p}(t)=\bar{y}_{p}\left(t_{0}\right)+\left(t-t_{0}\right) \dot{\bar{y}}_{p}\left(t_{0}\right)
\end{array}\right\}
$$
式中：
$$
\left.\begin{array}{l}
\bar{x}_{p}\left(t_{0}\right)=0.054, \dot{\bar{x}}_{p}\left(t_{0}\right)=0.00083 \\
\bar{y}_{p}\left(t_{0}\right)=0.357, \dot{\bar{y}}_{p}\left(t_{0}\right)=0.00395
\end{array}\right\}
$$
式中，$\bar{x}_{p}, \bar{y}_{p}$ 单位为弧秒，对应的速度 $\bar{x}_{p}, \bar{y}_{p}$ 单位为弧秒/年，$m_{1}, m_{2}$ 单位也为弧秒，$t_{0}=$ 2000.0 ，对应的时间为 2000 年 1 月 1 日 12 时 0 分 0 秒。

考虑到 $m_{1} 、 m_{2}$ 最大可能至 0.8 弧秒，因此极潮改正在径向最大可到 $25 \mathrm{~mm}$，而在平面上变形最大约为 $7 \mathrm{~mm}$ 。因此，在高精度精密单点定位中应考虑此项极潮的影响。

实际应用中，如需将以站心地平坐标系中表示的极潮位移改正转化到空间直角坐标系中，可以采用如下公式进行：
$$
\left[\begin{array}{lll}
\mathrm{d} X & \mathrm{~d} Y & \mathrm{~d} Z
\end{array}\right]^{\mathrm{T}}=\boldsymbol{R}^{\mathrm{T}}\left[\begin{array}{lll}
S_{\theta} & S_{\lambda} & S_{r}
\end{array}\right]^{\mathrm{T}}
$$
式中：
$$
\boldsymbol{R}=\left[\begin{array}{ccc}
\cos \theta \cos \lambda & \cos \theta \sin \lambda & -\sin \theta \\
-\sin \lambda & \cos \lambda & 0 \\
\sin \theta \cos \lambda & \sin \theta \sin \lambda & \cos \theta
\end{array}\right]
$$

GAMP 中先调用 iers_mean_pole() 计算平均极点坐标 xp_bar、yp_bar，后面用的时候会再乘 1E-3，平均极点坐标计算与上面公式略有不同：

* 使用三次多项式来拟合 2000 年到 2010 年的平均极坐标
* 线性函数模型，拟合 2010 年以后的平均极坐标

然后用 ERP 参数和平均极点坐标计算 m1、m2，再套潮汐改正公式。

```c
static void iers_mean_pole(gtime_t tut, double *xp_bar, double *yp_bar)
{
    const double ep2000[]={2000,1,1,0,0,0};
    double y,y2,y3;
    
    y=timediff(tut,epoch2time(ep2000))/86400.0/365.25;
    // 使用三次多项式来拟合 2000 年到 2010 年的平均极坐标
    if (y<3653.0/365.25) { /* until 2010.0 */
        y2=y*y; y3=y2*y;
        *xp_bar= 55.974+1.8243*y+0.18413*y2+0.007024*y3; /* (mas) */
        *yp_bar=346.346+1.7896*y-0.10729*y2-0.000908*y3;
    }
    // 线性函数模型，拟合 2010 年以后的平均极坐标
    else { /* after 2010.0 */
        *xp_bar= 23.513+7.6141*y; /* (mas) */
        *yp_bar=358.891-0.6287*y;
    }
}
```

```c
static void tide_pole(gtime_t tut, const double *pos, const double *erpv,
                      double *denu)
{
    double xp_bar,yp_bar,m1,m2,cosl,sinl;
    // 计算平均极点坐标 xp_bar、yp_bar，后面用的时候会再乘 1E-3
    /* iers mean pole (mas) */
    iers_mean_pole(tut,&xp_bar,&yp_bar);
    
    // 用 ERP 参数和平均极点坐标计算 m1、m2
    /* ref [7] eq.7.24 */
    m1= erpv[0]/AS2R-xp_bar*1E-3; /* (as) */
    m2=-erpv[1]/AS2R+yp_bar*1E-3;
    
    /* sin(2*theta) = sin(2*phi), cos(2*theta)=-cos(2*phi) */
    cosl=cos(pos[1]);
    sinl=sin(pos[1]);
    denu[0]=  9E-3*sin(pos[0])    *(m1*sinl-m2*cosl); /* de= Slambda (m) */
    denu[1]= -9E-3*cos(2.0*pos[0])*(m1*cosl+m2*sinl); /* dn=-Stheta  (m) */
    denu[2]=-33E-3*sin(2.0*pos[0])*(m1*cosl+m2*sinl); /* du= Sr      (m) */
}
```

## 七、地球自转效应改正

GNSS 的 MEO 卫星轨道高度大约 20000km。导航信号由卫星发出到接收机接受需要数十微秒，对于 GEO 卫星信号传播时间更长。在导航信号传播过程中，由于地球自转的影响，地固坐标系已随地球旋转了一定角度，由此给观测值造成的误差可达数十米，其影响不可忽略。因此需要对观测值进行距离改正，改正数的计算方法如下：
$$
\Delta D=\frac{\omega}{c}\left[y^{S}\left(x_{R}-x^{S}\right)-x^{S}\left(y_{R}-y^{S}\right)\right]
$$
式中，$x^{S}, y^{S}, x_{R}, y_{R}$ 分别表示信号发射时刻卫星和接收机在地固坐标系下的坐标，$\omega$ 表示地球自转角速度，单位为弧度每秒。

地球自转效应改正在 `geodist()` 函数中计算站心几何距离的时候就已经改正了。GAMP 又写了 `sagnac()` 函数专门计算地球自转效应改正量，并不用于定位解算，只是赋值到 PPP_Info 里，用于生成 RCVEX 文件，代码与公式完全对应：

```c
extern double sagnac(const double *rs, const double *rr)
{
	return OMGE*(rs[0]*rr[1]-rs[1]*rr[0])/CLIGHT;
}
```

## 八、引力延迟改正

### 1、原理

广义相对论效应还会产生引力延迟。这里不加推导，直接给出引力延迟的计算公式如下：
$$
\Delta \mathrm{D}_{\mathrm{g}}=\frac{2 \mu}{\mathrm{c}^{2}} \ln \frac{\mathrm{r}+\mathrm{R}+\rho}{\mathrm{r}+\mathrm{R}-\rho}
$$
式中：$\mu$ 为万有引力常数 $\mathrm{G}$ 与地球总质量 $\mathrm{M}$ 之乘积；$\mathrm{c}$ 为真空中的光速；$\mathrm{r}$ 为卫星至地心的距离；$R$ 为测站至地心的距离；$\rho$ 为测站至卫星的距离。当卫星接近地平面时引力延迟取得最大值，约为 $19 \mathrm{~mm}$ 。当卫星在测站天顶方向时引力延迟取得最小值，约为 $13 \mathrm{~mm}$ 。引力延迟引起的相对测距误差不足 $10^{-9}$，一般的单点定位中无需顾及，但在精密单点定位 (PPP) 中应予以考虑。在相对定位中，当站间距离为 $1000 \mathrm{~km}$ 时，两站的引力延迟之差最大可达 $1.3 \mathrm{~mm}$；当站间距离为 $3000 \mathrm{~km}$ 时，两站的引力延迟之差最大可达 $3.6 \mathrm{~mm}$ 。只有在高精度相对定位中才需顾及此项改正。

GAMP 中不同系统的引力常数定义如下，在小数点后 13 位有区别：

```c
#define MU_GPS   3.9860050E14     /* gravitational constant         ref [1] */
#define MU_GLO   3.9860044E14     /* gravitational constant         ref [2] */
#define MU_GAL   3.986004418E14   /* earth gravitational constant   ref [7] */
#define MU_CMP   3.986004418E14   /* earth gravitational constant   ref [9] */
```

### 2、gravitationalDelayCorrection()：引力延迟改正

`gravitationalDelayCorrection()` 代码中，先计算接收机到地心的距离 `receiverModule`、卫星到地心的距离 `satelliteModule`、卫星到接收机距离 `distance`，然后套公式计算。

> 因为函数传入的是 ECEF 坐标，地心坐标就是原点(0,0,0)，所以到地心距离直接就是 ECEF 坐标的模。

```c
extern double gravitationalDelayCorrection (const int sys, const double *receiverPosition, 
	                                        const double *satellitePosition) 
{
	double	receiverModule;		// 接收机到地心的距离
	double	satelliteModule;	// 卫星到地心的距离
	double	distance;			// 接收机到卫星的矩阵
	double  MU=MU_GPS;

	// 先计算接收机到地心的距离 receiverModule、卫星到地心的距离 satelliteModule、卫星到接收机距离 distance
	// 传入的是 ECEF 坐标，地心坐标是原点(0,0,0)，所以到地心距离直接就是 ECEF 坐标的模
	receiverModule=sqrt(receiverPosition[0]*receiverPosition[0]+receiverPosition[1]*receiverPosition[1]+
		receiverPosition[2]*receiverPosition[2]);
	satelliteModule=sqrt(satellitePosition[0]*satellitePosition[0]+satellitePosition[1]*satellitePosition[1]+
		satellitePosition[2]*satellitePosition[2]);
	distance=sqrt((satellitePosition[0]-receiverPosition[0])*(satellitePosition[0]-receiverPosition[0])+
		(satellitePosition[1]-receiverPosition[1])*(satellitePosition[1]-receiverPosition[1])+
		(satellitePosition[2]-receiverPosition[2])*(satellitePosition[2]-receiverPosition[2]));

	// 不同系统引力常数在小数点后 13 位有区别
	switch (sys) {
	case SYS_GPS:
		MU=MU_GPS;
		break;
	case SYS_GLO:
		MU=MU_GLO;
		break;
	case SYS_GAL:
		MU=MU_GAL;
		break;
	case SYS_CMP:
		MU=MU_CMP;
		break;
	default:
		MU=MU_GPS;
		break;
	}

	// 套公式计算
	return 2.0*MU/(CLIGHT*CLIGHT)*log((satelliteModule+receiverModule+distance)/(satelliteModule+receiverModule-distance));
}
```
