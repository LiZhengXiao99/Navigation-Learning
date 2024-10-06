[TOC]

## 一、导航电文

### 1、GNSS卫星信号的组成
GPS卫星信号由**载波**、**伪码**、**导航电文**（数据码）三个层次组成。数据码首先与伪码异或相加实现扩频，然后二者的组合码通过双向移位键控（BPSK）对载波进行调制。用户接收机首先对载波信号进行BPSK（调相调制）解调，使卫星信号的中心频率从L1下变频为0，之后再将载波解调后的卫星信号与接收机内部复制的C/A码Gi做自相关运算，剥离卫星信号中的C/A码，使信号频宽变回到只含数据码的基带，以得到 50bps 数据码，再按导航电文的格式最终将数据码编译成导航电文。
### 2、导航电文的编排
* 卫星将导航电文以帧与子帧的形式编排成数据流，每帧导航电文长1500比特，计30s，依次由5个子帧组成。每个子帧长300比特，计6s，依次由10个字组成，每字长30比特，最高为比特先发，每个子帧中的每个字均以6比特的奇偶校验码结束。每比特长20ms，C/A码重复20个周期。 
* 每一个子帧的前两个字分别是遥测字（TLW）、交接字（HOW），后8个字组成数据块。第1子帧中的数据块称为第一数据库块，2和3子帧数据块合称为第二数据块，剩下的称为第三数据块。当卫星出现故障时，会在各大数据块的8个字中交替发射1和0。 

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/052a3f30c4b243299dfeba7d0695cde9.png)
* GPS对第三数据块采用了分页结构，即一帧中的第4子帧和第5子帧为一页，然后下一帧的第4和第5子帧继续发送下一页，第三数据块内容共25页，所以发送完一整套导航电文需要750s（12.5min） ，整个导航电文每12.5min重复一次。
*  在一个新GPS周开始时，导航电文从第一子帧开始播发，子帧4和5从第三数据块的首页开始播发。子帧123需要更新时，新的导航电文从帧的延边处开始播发（对应GPS时间是30s的整数倍）。子帧45需要更新时，新导航电文可以在子帧45的任何一页开始播发。
### 3、遥测字（TLW）
每一个子帧的第一个字均为遥测字，在导航电文中每6s出现一次，内部组成情况。1~8位是二进制固定值10001011的同步码，9–22位提供特许用户所需信息，23、24位保留，最后6位是奇偶校验码。 
### 4、交接字（HOW）
1–17位是截短的周内时计数，18位是警告标志，为1时提醒非特许用户自己承担使用该卫星信号的风险，该卫星第一数据块所提供的URA值有可能比其真实值大。19比特A–S标志，其值为1时表示对该卫星实施了反电子欺骗措施。20–22比特是子帧识别标志，共5个有效二进制：001表示第1子帧，010表示第2子帧，依次类推。后面是奇偶校验码。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/abbef60979c444d2930f721fe994145e.png)



### 5、第一数据块
也称时钟数据块，由第一子帧构成，包含如下内容： 

  * **星期数WN**：GPS星期，来自Z计数值的高10比特，最大值1023约19年，若1023+1则返0，上次返0在2019年4月6号。

    > 我国的北斗系统，也存在BD周数翻转问题，但在设计时，其周计数用13bit表示，翻转周期是8192周，大概是160年 。

  * **用户测距精度（URA）**：对所有由GPS地面监控部分和空间星座部分引起的测距误差大小的一个统计值，通过4比特表示用户测距精度因子N，0<=N <=6，URA = 2^(1+N/2)，6<N<15，URA = 2的（N-2）次方， URA值越大，表示卫星信号得到的GPS距离测量值精度越低。

  * **卫星健康状况**：6比特，最高位0表示正常1表示出错，低5位表示具体出错情况。

  * **时钟校正参数（$a_{f0}、a_{f1}、a_{f2}$）**：卫星时钟模型校正方程的三个系数。

  * **群波延时校正值（$T_{GD}$）**：针对单频接收机。单频接收机之所以有此项校正因为$a_{f0}$是针对双频测量值而言的。

  * **时钟数据期号（IODC）**：10比特表示的时钟数据块期号，一个IODC对应一套时钟校正参数，可用于快速检测时钟参数是否发生改变，IODC不变说明时钟参数没更新，就不用再重复读取。

### 6、第二数据块
提供卫星自身的星历参数、由子帧 2 3 组成，内容如下：

  卫星星历是描述卫星运动轨道的信息。也可以说卫星星历就是一组对应某一时刻的轨道参数及其变率。有了卫星星历就可以计算出任意时刻的卫星位置及其速度。GPS 广播星历参数共有 16 个，其中包括 1 个参考时刻，6 个对应参考时刻的开普勒轨道参数和 9 个反映摄动力影响的参数 ：

  > 1. 星历参考时刻 ：$t_{oe}$ 
  > 2. 长半轴平方根：$\sqrt{a_s}$
  > 3. 轨道偏心率：$e_s$
  > 4. 参考历元$t_{oe}$下的轨道倾角：$M_0$
  > 5. 本周初始历元的升交点赤经：$\Omega _0$
  > 6. 轨道近地点角距：$\omega$
  > 7. 参考历元$t_{oe}$下的平近点角：$M_0$
  > 8. 平运动差（由精密星历计算得到的卫星平均角速度与按给定参数计算所得的平均角速度之差）：$\Delta n$
  > 9. 轨道倾角变化率（弧度/秒）：$\dot{i}$
  > 10. 升交点赤经变化率（弧度/秒）：$\dot{\Omega}$
  > 11. 纬度幅角的余弦调和项改正的振幅（弧度）：$C_{uc}$
  > 12. 纬度幅角的正弦调和项改正的振幅（弧度）：$C_{us}$
  > 13. 轨道半径的余弦调和项改正的振幅（m）：$C_{rc}$
  > 14. 轨道半径的正弦调和项改正的振幅（m）：$C_{rs}$
  > 15. 轨道倾角的余弦调和项改正的振幅（弧度）：$C_{ic}$
  > 16. 轨道倾角的正弦调和项改正的振幅（弧度）：$C_{is}$

### 7、第三数据块
子帧4 5组成的第三数据块提供所有卫星的历书参数、电离层延时校正参数、GPS时与UTC间的关系及卫星健康状况等信息。
![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/76d5d9a8a63f422e8a8dd0aeb16f3023.png)


  > 1. 历书参考时间：$t_{oa}$
  > 2. 卫星轨道长半轴$a_s$的平方根：$\sqrt{a_s}$
  > 3. 轨道偏心率：$e_s$
  > 4. 相当于0.3π的轨道倾角：$\delta _i$
  > 5. 本周初始历元的升交点赤经：$\Omega _0$
  > 6. 轨道近地点角距：$\omega$
  > 7. 参考历元$t_{oe}$下的平近点角：$M_0$
  > 8. 升交点赤经变化率（弧度/秒）：$\dot{\Omega}$
  > 9. 卫星时钟校正参数：$a_{f0}$
  > 10. 卫星时钟校正参数：$a_{f1}$

> 历书参数与星历参数比较：
>
> * 历书与星历都是表示卫星运行的参数。**历书包括全部卫星的大概位置**，用于卫星预报；**星历只是当前接收机观测到的卫星**的精确位置，用于定位。 
> * 为了缩短卫星锁定时间，GPS 接收机需利用历书、当地位置的时间来预报卫星运行状态。 
> * 历书是从导航电文中提取的，每 12.5 分钟的导航电文才能得到一组完整的历书。历书的有效期为半年。 
> * 利用历书和当地的位置， 我们可以计算出卫星的方位和高度角，由此可以计算出当地能观测到的卫星和持续时间，即卫星高度角大于 5° 的出现时间。 
> * GPS 卫星星历参数包含在导航电文的第二和第三子帧中。从有效的星历中，我们可解得卫星的较准确位置和速度，从而用于接收机定位和测速。GPS 卫星星历每 30 秒重复一次，有效期为以星历参考时间为中心的 4 小时内 。
> * 历书信息存在 alm_t 中，星历存在 eph_t 中



## 二、NAV 星历文件读取



### 1、decode_navh()、decode_gnavh()、decode_hnavh()

以decode_navh()为例：![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/819ea1b1474442a09edd951a7d9409dd.png)


```c
static void decode_navh(char *buff, nav_t *nav) 
{
    int i,j; 
    char *label=buff+60; 
    
    trace(4,"decode_navh:\n"); 
    
    //读取电离层模型参数
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
            else if (!strncmp(buff,"IRNA",4)) { /* v.3.03 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_irn[i]=str2num(buff,j,12);
            }
            else if (!strncmp(buff,"IRNB",4)) { /* v.3.03 */
                for (i=0,j=5;i<4;i++,j+=12) nav->ion_irn[i+4]=str2num(buff,j,12);
            }
        }
    }
    
    //读取时间系统改正参数
    else if (strstr(label,"TIME SYSTEM CORR"    )) { /* opt ver.3 */
        if (nav) {
            if (!strncmp(buff,"GPUT",4)) {
                nav->utc_gps[0]=str2num(buff, 5,17);
                nav->utc_gps[1]=str2num(buff,22,16);
                nav->utc_gps[2]=str2num(buff,38, 7);
                nav->utc_gps[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"GLUT",4)) {
                nav->utc_glo[0]=-str2num(buff,5,17); /* tau_C */
            }
            else if (!strncmp(buff,"GLGP",4)) {
                nav->utc_glo[1]=str2num(buff, 5,17); /* tau_GPS */
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
                nav->utc_sbs[0]=str2num(buff, 5,17);
                nav->utc_sbs[1]=str2num(buff,22,16);
                nav->utc_sbs[2]=str2num(buff,38, 7);
                nav->utc_sbs[3]=str2num(buff,45, 5);
            }
            else if (!strncmp(buff,"IRUT",4)) { /* v.3.03 */
                nav->utc_irn[0]=str2num(buff, 5,17);
                nav->utc_irn[1]=str2num(buff,22,16);
                nav->utc_irn[2]=str2num(buff,38, 7);
                nav->utc_irn[3]=str2num(buff,45, 5);
                nav->utc_irn[8]=0.0; /* A2 */
            }
        }
    }
    //读取跳秒参数
    else if (strstr(label,"LEAP SECONDS"        )) { /* opt */
        if (nav) {
            nav->utc_gps[4]=str2num(buff, 0,6);
            nav->utc_gps[7]=str2num(buff, 6,6);
            nav->utc_gps[5]=str2num(buff,12,6);
            nav->utc_gps[6]=str2num(buff,18,6);
        }
    }
}
```



### 2、readrnxnav()：读取星历文件，添加到nav结构体中

* **add_eph**()：nav->eph[] 中添加eph星历数据，nav->n 表示NAV数量。
* **add_geph**()：nav->geph[] 中添加GLONASS星历数据，nav->ng 表示geph数量。
* **add_seph**()：nav->seph[] 中添加SBAS星历数据，nav->ns 表示seph数量。

```c
static int readrnxnav(FILE *fp, const char *opt, double ver, int sys,
                      nav_t *nav)
{
    eph_t eph;
    geph_t geph;
    seph_t seph;
    int stat,type;
    
    trace(3,"readrnxnav: ver=%.2f sys=%d\n",ver,sys);
    
    if (!nav) return 0;
    //调用readrnxnavb读取文件，然后根据星历类型选择函数保存
    /* read RINEX navigation data body */
    while ((stat=readrnxnavb(fp,opt,ver,sys,&type,&eph,&geph,&seph))>=0) {
        
        /* add ephemeris to navigation data */
        if (stat) {
            switch (type) {
                case 1 : stat=add_geph(nav,&geph); break;
                case 2 : stat=add_seph(nav,&seph); break;
                default: stat=add_eph (nav,&eph ); break;
            }
            if (!stat) return 0;
        }
    }
    return nav->n>0||nav->ng>0||nav->ns>0;
}
```



### 3、readrnxnavb()：读取一个历元的星历数据，添加到eph结构体中

1. nav数据文件体结构：卫星PRN号、卫星时间TOC、参数

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/7c62093881b847b8943b0910e78b23b8.png)


2. **执行流程**：

   * 调用`set_sysmask()`获取卫星系统掩码

   * 循环读取一行行，记录TOC，读取到`data[]`，i记录读取的数据数量，读够数量调用`decode_eph()`等函数赋值给`eph_t`结构体 

   ```c
   static int readrnxnavb(FILE *fp, const char *opt, double ver, int sys,
                          int *type, eph_t *eph, geph_t *geph, seph_t *seph)
   {
       gtime_t toc;
       double data[64];
       int i=0,j,prn,sat=0,sp=3,mask;
       char buff[MAXRNXLEN],id[8]="",*p;
       
       trace(4,"readrnxnavb: ver=%.2f sys=%d\n",ver,sys);
       
       /* set system mask */
       mask=set_sysmask(opt);  //设置卫星系统掩码
       
       //循环读取一行行，读取到data[]，i记录读取的数据数量，读够数量进入decode_eph()赋值给eph_t结构体
       while (fgets(buff,MAXRNXLEN,fp)) {
           
           if (i==0) {
               
               /* decode satellite field */
               if (ver>=3.0||sys==SYS_GAL||sys==SYS_QZS) { /* ver.3 or GAL/QZS */
                   sprintf(id,"%.3s",buff);
                   sat=satid2no(id);
                   sp=4;               //3以上版本，GALileo，QZSS sp都为4
                   if (ver>=3.0) {
                       sys=satsys(sat,NULL);
                       if (!sys) {
                           sys=(id[0]=='S')?SYS_SBS:((id[0]=='R')?SYS_GLO:SYS_GPS);
                       }
                   }
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
               /* decode Toc field */
               if (str2time(buff+sp,0,19,&toc)) {      //读取卫星钟时间TOC
                   trace(2,"rinex nav toc error: %23.23s\n",buff);
                   return 0;
               }
               /* decode data fields */
               for (j=0,p=buff+sp+19;j<3;j++,p+=19) {  //首行数据读3列，除了TOC还有3列
                   data[i++]=str2num(p,0,19);
               }
           }
           else {
               /* decode data fields */
               for (j=0,p=buff+sp;j<4;j++,p+=19) { //其它行数据都读4列
                   data[i++]=str2num(p,0,19);
               }
               
               /* decode ephemeris */
               if (sys==SYS_GLO&&i>=15) {
                   if (!(mask&sys)) return 0;
                   *type=1;
                   return decode_geph(ver,sat,toc,data,geph);
               }
               else if (sys==SYS_SBS&&i>=15) {
                   if (!(mask&sys)) return 0;
                   *type=2;
                   return decode_seph(ver,sat,toc,data,seph);
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

3. **调用函数**：

   * **decode_eph()**：将`data[]`中信息赋值到`eph_t` 类型结构体`eph`
   * **decode_geph()**：将`data[]`中信息赋值到`geph_t`  类型结构体`geph`
   * **decode_seph()**：将`data[]`中信息赋值到`seph_t`  类型结构体`seph`



## 三、卫星钟差钟漂改正

### 1、时钟校正参数（$a_{f0}、a_{f1}、a_{f2}$）改正

相对于 GPS 时间，卫星上作为时间和频率信号来源的原子钟也存在时间偏差和频率漂移。为确保各颗卫星的时钟与GPS时间同步，GPS地面监控部分通过对卫星信号进行检测，将卫星时钟在GPS时间t的卫星钟差$\Delta t^{(s)}$描述为如下二项式：
$$
\Delta t^{(s)}=a_{f0}+a_{f1}(t-t_{oc})+a_{f2}(t-t_{oc})^2
$$

### 2、相对论效应校正$\Delta t_r$

综合狭义相对论和广义相对论，在高空中高速运行的卫星原子钟比地面上一模一样的原子钟每天要快 38000ns ，每秒快 0.44ns 。如果不考虑相对论效应，GPS发上天两分钟内，卫星原子钟就会失去定位作用。在地面上设计原子钟时可以减小一点点它的频率，上天以后其时钟频率在地面上看来正好等于设计值。同时因为GPS运行轨道是椭圆而不是圆，地面上计算机还有根据卫星当前位置做相对论效应的校如下：
$$
\Delta t_r=Fe_s\sqrt{a_s} \sin E_k
$$

### 3、群波延迟校正$T_{GD}$

由第一数据块给出，只适用于单频。这样对于 L1 单频接收机，卫星时钟总钟差值如下：
$$
\delta t^{(s)}=\Delta t^{(s)}+\Delta t_{r}-T_{G D}
$$

### 4、钟漂校正

对上面卫星时钟总钟差值求导得：
$$
\delta f^{(s)}=a_{f 1}+2 a_{f 2}\left(t-t_{o c}\right)+\Delta \dot{t}_r
$$
群波延迟校正$T_{GD}$的导数为0，相对论效应校正$\Delta t_r$如下：
$$
\Delta \dot{t}_r=F e_s \sqrt{a_s} \dot{E}_k \cos E_k
$$



## 四、satposs()调用流程

### 1、调用流程图

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/4d2f221d133145e295736f82472a436f.png)

### 2、传入参数

```
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

### 3、执行流程

```c
rs [(0:2)+i*6]= obs[i] sat position {x,y,z} (m)			卫星位置
rs [(3:5)+i*6]= obs[i] sat velocity {vx,vy,vz} (m/s)	卫星速度
dts[(0:1)+i*2]= obs[i] sat clock {bias,drift} (s|s/s)	卫星钟差钟漂
```

* 遍历每一个OBS观测数据：
* 首先初始化，将对当前观测数据的 rs、dts、var 和 svh 数组的元素置 0 
* 通过判断某一频率下信号的伪距是否为 0，来得到此时所用的频率个数j 
* 用数据接收时刻减去伪距信号传播时间，得到卫星信号的发射时刻 `time[i]`
* 调用 `ephclk()` 函数，由广播星历计算出当前观测卫星与 GPS 时间的钟差 `dt` ,此时的钟差是没有考虑相对论效应和 TGD 的 ，`dt` 仅作为`satpos()`的参数，为了后面计算的卫星位置更准确，不作为最终定位方程计算的钟差。
* 信号发射时刻减去钟差 dt，得到 GPS 时间下的卫星信号发射时刻 
* 调用 `satpos()` 函数，计算信号发射时刻卫星的位置(ecef,m)、速度(ecef,m/s)、钟差((s|s/s)) ,这里计算出的钟差是考虑了相对论效应的了，只是还没有考虑 TGD 
* 如果没有精密钟差，则`ephclk()`用广播星历的钟差替代，猜测可能是选了 EPHOPT_PREC 精密星历，但精密钟差计算出问题，用广播星历重新计算钟差。



### 4、RTKLIB中卫星和卫星系统的表示

#### 1.卫星系统

  * 表示卫星系统的字母：G：GPS、R：GLONASS、E：GALILEO、C：BDS、J：QZSS，I：IRNSS、S：SBAS

  * 7位二进制码表示，对应位写1表示有对应的系统，做与运算可加系统。

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

  

#### 2.卫星
可以表示为各系统的卫星ID（系统缩写+PRN）：B02、C21，也可表示为连续的satellite number ，各种转换函数如下：

  * **satno()**：传入卫星系统(SYS_GPS,SYS_GLO,...) ，和PRN码，转换为连续的satellite number。
  * **satsys()**：传入satellite number ，返回卫星系统(SYS_GPS,SYS_GLO,...) ，通过传入的指针prn传出PRN值。
  * **satid2no()**：传入卫星ID，返回satellite number。
  * **satno2id()**：传入卫星系统，和PRN，返回卫星ID(Gxx,Cxx)
  * **sat2code()**：传入satellite number，返回卫星ID(Gxx,Cxx)
  * **code2sys()**：传入卫星系统缩写，返回系统二进制码SYS_XXX。
  * **satexclude()**：检测某颗卫星在定位时是否需要将其排除

  

### 5、调用函数

#### 1.satpos()：单颗卫星计算入口函数
针对单颗卫星，根据星历选项调用对应的解算函数`ephpos()`、`satpos_sbas()`、`satpos_ssr()`、`peph2pos()`，解算信号发射时刻卫星的 P(ecef,m)、V(ecef,m/s)、C((s|s/s)) 

  ```c
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

  

#### 2.ephclk()：钟差解算
根据卫星系统调用对应的函数`eph2clk()`、`geph2clk()`、`seph2clk()`，通过广播星历来确定卫星钟差 、钟漂

  ```c
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


#### 3.ephpos()：位置解算
由satpos()调用，执行流程如下：

  * 根据卫星系统，先调用对应的星历选择函数，seleph()、selgeph()、seph2pos()。
  * 调用对应的解算函数，eph2pos()、geph2pos()、seph2pos()，计算位置、钟差。
  * 增加一个极短的时间tt，再调用对应的解算函数计算位置、钟差。
  * 两次的位置、钟差相减再除以tt，得速度、钟漂。

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
      //在信号发射时刻的基础上给定一个微小的时间间隔，再次计算新时刻的 P、V、C。与3结合，通过扰动法计算出卫星的速度和频漂。
      //并没有使用那些位置和钟差公式对时间求导的结果
      /* satellite velocity and clock drift by differential approx */
      for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;    //卫星速度rs[i+3]
      dts[1]=(dtst[0]-dts[0])/tt;                     //钟漂dts[1]
  
      return 1;
  }
  ```

  

#### 4.seleph()、selgeph()：选择用于解算星历数据
传入sattle number，时间或 IDOE，如果传入 IDOE>=0 ,按 IDOE 找星历数据，否则取最接近时间的星历 


  ```c
  static eph_t *seleph(gtime_t time, int sat, int iode, const nav_t *nav)
  {
      double t,tmax,tmin;
      int i,j=-1,sys,sel;
      
      trace(4,"seleph  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);
      
      //根据传入的sattle number，调用satsys()判断卫星系统，赋值tmax，tmin，sel
      sys=satsys(sat,NULL);
      switch (sys) {
          case SYS_GPS: tmax=MAXDTOE+1.0    ; sel=eph_sel[0]; break;
          case SYS_GAL: tmax=MAXDTOE_GAL    ; sel=eph_sel[2]; break;
          case SYS_QZS: tmax=MAXDTOE_QZS+1.0; sel=eph_sel[3]; break;
          case SYS_CMP: tmax=MAXDTOE_CMP+1.0; sel=eph_sel[4]; break;
          case SYS_IRN: tmax=MAXDTOE_IRN+1.0; sel=eph_sel[5]; break;
          default: tmax=MAXDTOE+1.0; break;
      }
      tmin=tmax+1.0;
      
      //遍历nav->eph[]
      for (i=0;i<nav->n;i++) {
          if (nav->eph[i].sat!=sat) continue; //eph[i]不是需要的卫星，进入下一次循环
          if (iode>=0&&nav->eph[i].iode!=iode) continue;  //如果传入了idoe时间不符合，也进行下一次循环
          
          if (sys==SYS_GAL) {     //若是伽利略卫星还要判断是I/NAV、F/NAV
              sel=getseleph(SYS_GAL);
              if (sel==0&&!(nav->eph[i].code&(1<<9))) continue; /* I/NAV */
              if (sel==1&&!(nav->eph[i].code&(1<<8))) continue; /* F/NAV */
              if (timediff(nav->eph[i].toe,time)>=0.0) continue; /* AOD<=0 */
          }
          if ((t=fabs(timediff(nav->eph[i].toe,time)))>tmax) continue;    //时间差过大，也进行下一次循化
          if (iode>=0) return nav->eph+i;     //传入IDOE>=0，直接返回符合条件的星历
          if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */   //存下时间差最小的星历下标和时间差
      }
      if (iode>=0||j<0) {
          trace(3,"no broadcast ephemeris: %s sat=%2d iode=%3d\n",
                time_str(time,0),sat,iode);
          return NULL;
      }
      return nav->eph+j;
  }
  ```

  ```c
  static geph_t *selgeph(gtime_t time, int sat, int iode, const nav_t *nav)
  {
      double t,tmax=MAXDTOE_GLO,tmin=tmax+1.0;
      int i,j=-1;
      
      trace(4,"selgeph : time=%s sat=%2d iode=%2d\n",time_str(time,3),sat,iode);
      
      //变量nav->geph[]，
      for (i=0;i<nav->ng;i++) {
          if (nav->geph[i].sat!=sat) continue;            //卫星不同，进行下一次循环
          if (iode>=0&&nav->geph[i].iode!=iode) continue; //传入IDOE>0，IDOE不同，进行下一次循环
          if ((t=fabs(timediff(nav->geph[i].toe,time)))>tmax) continue;
          if (iode>=0) return nav->geph+i;    //传入IDOE>=0，直接返回符合条件的星历
          if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */   //存下时间差最小的星历下标和时间差
      }
      if (iode>=0||j<0) {
          trace(3,"no glonass ephemeris  : %s sat=%2d iode=%2d\n",time_str(time,0),
                sat,iode);
          return NULL;
      }
      return nav->geph+j;
  }
  ```

  

## 五、卫星位置、钟差的具体计算函数

> 公式模型见manual的142面，我在对应的函数语句后面标注了对应的公式编号。

### 1、alm2pos()：由历书计算卫星
历书信息计算卫星位置、钟差。（这个函数在RTKLIB里好像没怎么被调用过）

  ```c
  extern void alm2pos(gtime_t time, const alm_t *alm, double *rs, double *dts)
  {
      double tk,M,E,Ek,sinE,cosE,u,r,i,O,x,y,sinO,cosO,cosi,mu;
      int n;
      
      trace(4,"alm2pos : time=%s sat=%2d\n",time_str(time,3),alm->sat);
      
      tk=timediff(time,alm->toa);
      
      if (alm->A<=0.0) {
          rs[0]=rs[1]=rs[2]=*dts=0.0;
          return;
      }
      mu=satsys(alm->sat,NULL)==SYS_GAL?MU_GAL:MU_GPS;
      
      M=alm->M0+sqrt(mu/(alm->A*alm->A*alm->A))*tk;
      for (n=0,E=M,Ek=0.0;fabs(E-Ek)>RTOL_KEPLER&&n<MAX_ITER_KEPLER;n++) {
          Ek=E; E-=(E-alm->e*sin(E)-M)/(1.0-alm->e*cos(E));
      }
      if (n>=MAX_ITER_KEPLER) {
          trace(2,"alm2pos: kepler iteration overflow sat=%2d\n",alm->sat);
          return;
      }
      sinE=sin(E); cosE=cos(E);
      u=atan2(sqrt(1.0-alm->e*alm->e)*sinE,cosE-alm->e)+alm->omg;
      r=alm->A*(1.0-alm->e*cosE);
      i=alm->i0;
      O=alm->OMG0+(alm->OMGd-OMGE)*tk-OMGE*alm->toas;
      x=r*cos(u); y=r*sin(u); sinO=sin(O); cosO=cos(O); cosi=cos(i);
      rs[0]=x*cosO-y*cosi*sinO;
      rs[1]=x*sinO+y*cosi*cosO;
      rs[2]=y*sin(i);
      *dts=alm->f0+alm->f1*tk;
  }
  ```

  

### 2、eph2clk()：由广播星历计算卫星钟差

根据信号发射时间和广播星历，计算卫星钟差,不考虑相对论效应和 TGD ，二项式校正公式用了三遍。

### 3、geph2clk()：由广播星历计算GLONASS卫星钟差钟差
与eph2clk() 类似

  ```c
  extern double eph2clk(gtime_t time, const eph_t *eph)
  {
      double t,ts;
      int i;
      
      trace(4,"eph2clk : time=%s sat=%2d\n",time_str(time,3),eph->sat);
      
      t=ts=timediff(time,eph->toc);   // 计算与星历参考时间的偏差 dt = t-toc
      //利用二项式校正计算出卫星钟差，从 dt中减去这部分，然后再进行一次上述操作，得到最终的 dt
      for (i=0;i<2;i++) {
          t=ts-(eph->f0+eph->f1*t+eph->f2*t*t);       //(E.4.16)
      }
      //使用二项式校正得到最终的钟差
      return eph->f0+eph->f1*t+eph->f2*t*t;
  }
  ```

  ```c
  extern double geph2clk(gtime_t time, const geph_t *geph)
  {
      double t,ts;
      int i;
      
      trace(4,"geph2clk: time=%s sat=%2d\n",time_str(time,3),geph->sat);
      
      t=ts=timediff(time,geph->toe);
      
      for (i=0;i<2;i++) {
          t=ts-(-geph->taun+geph->gamn*t);    //(E.4.26)
      }
      return -geph->taun+geph->gamn*t;
  }
  ```

  

### 4、eph2pos()：由广播星历计算卫星位置钟差
根据广播星历计算出算信号发射时刻卫星的位置和钟差 ，gps, galileo, qzss, bds，由开普勒轨道参数和摄动改正参数计算，考虑的相对论效应，但没考虑TGD。

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
      
      //北斗的MEO、IGSO卫星计算方法与GPS, Galileo and QZSS相同，只是一些参数不同
      //GEO卫星的 O 和最后位置的计算稍有不同 
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
      
      *dts=eph->f0+eph->f1*tk+eph->f2*tk*tk;  //利用三个二项式模型系数af0、af1、af2计算卫星钟差
      
      /* relativity correction */ 
      *dts-=2.0*sqrt(mu*eph->A)*eph->e*sinE/SQR(CLIGHT);  //相对论效应改正卫星钟差
      
      /* position and clock error variance */
      *var=var_uraeph(sys,eph->sva);  //用 URA 值来标定方差
  }
  ```

  

### 5、var_uraeph()：用URA用户测距精度标定卫星位置方差。

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/15d6fc165600428784189bbf00456c13.png)

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

  > GLONASS URA设置为常数5m 
  >
  > ```c
  > extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
  >                   double *var)
  > {
  > ...    
  >  *var=SQR(ERREPH_GLO);
  > }
  > ```



### 6、GLONASS卫星位置计算： geph2pos() -> glorbit() -> deq()

由 GLONASS 星历计算卫星坐标。GLONASS 卫星播发的是 PZ-90 坐标系下参考时刻的卫星状态向量，每半个小时广播一次。如果需要得到某个时间的卫星位置必须通过运动模型积分得到。

#### 1.glorbit()：龙格库塔迭代

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/0904f7d7214b46ccb92ee3fb8b9c4578.png)



#### 2.deq()：微分方程计算

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/d5b839cb6f4c4470b41cb26132fe8a3a.png)

```c
extern void geph2pos(gtime_t time, const geph_t *geph, double *rs, double *dts,
                     double *var)
{
    double t,tt,x[6];
    int i;
    
    trace(4,"geph2pos: time=%s sat=%2d\n",time_str(time,3),geph->sat);
    
    t=timediff(time,geph->toe);
    
    *dts=-geph->taun+geph->gamn*t;  //计算钟差dts(E.4.26)
    
    for (i=0;i<3;i++) {
        x[i  ]=geph->pos[i];
        x[i+3]=geph->vel[i];
    }

    //步长TSTEP:60s
    for (tt=t<0.0?-TSTEP:TSTEP;fabs(t)>1E-9;t-=tt) {
        if (fabs(t)<TSTEP) tt=t;
        glorbit(tt,x,geph->acc);
    }
    for (i=0;i<3;i++) rs[i]=x[i];
    
    *var=SQR(ERREPH_GLO);   //glonass卫星的方差直接定为 5*5
}
static void glorbit(double t, double *x, const double *acc)
{
    double k1[6],k2[6],k3[6],k4[6],w[6];
    int i;
    
    deq(x,k1,acc); for (i=0;i<6;i++) w[i]=x[i]+k1[i]*t/2.0;
    deq(w,k2,acc); for (i=0;i<6;i++) w[i]=x[i]+k2[i]*t/2.0;
    deq(w,k3,acc); for (i=0;i<6;i++) w[i]=x[i]+k3[i]*t;
    deq(w,k4,acc); for (i=0;i<6;i++) x[i]+=(k1[i]+2.0*k2[i]+2.0*k3[i]+k4[i])*t/6.0;
}
static void deq(const double *x, double *xdot, const double *acc)
{
    double a,b,c,
    r2=dot(x,x,3),  //r平方
    r3=r2*sqrt(r2), //r三次方
    omg2=SQR(OMGE_GLO); //omg平方
    
    if (r2<=0.0) {  //计算出错
        xdot[0]=xdot[1]=xdot[2]=xdot[3]=xdot[4]=xdot[5]=0.0;
        return;
    }
    /* ref [2] A.3.1.2 with bug fix for xdot[4],xdot[5] */
    a=1.5*J2_GLO*MU_GLO*SQR(RE_GLO)/r2/r3; /* 3/2*J2*mu*Ae^2/r^5 */
    b=5.0*x[2]*x[2]/r2;                    /* 5*z^2/r^2 */
    c=-MU_GLO/r3-a*(1.0-b);                /* -mu/r^3-a(1-b) */
    xdot[0]=x[3]; xdot[1]=x[4]; xdot[2]=x[5];       //(E.4.22)
    xdot[3]=(c+omg2)*x[0]+2.0*OMGE_GLO*x[4]+acc[0]; //(E.4.23)
    xdot[4]=(c+omg2)*x[1]-2.0*OMGE_GLO*x[3]+acc[1]; //(E.4.24)
    xdot[5]=(c-2.0*a)*x[2]+acc[2];                  //(E.4.25)
}
```



## 六、精密星历

### 1、精密星历读取流程

  > `nav->peph[]` 存精密星历数据，`nav->ne` 精密钟差数量。
  >
  > `nav->pclk[]` 存精密钟差数据，`nav->nc` 精密钟差数量。

  * **execses_b**() 中调用`readpreceph()`。
  * **readpreceph**() 中：`readsp3()`读取精密星历，`readrnxc()` 读取精密钟差
  * **readsp3**() 中：`readsp3h() `读文件头，`readsp3b()` 读文件体，`combpeph() `对精密星历按时间、index 排序，再将相同星历合并。
  * **readrnxc**() 中：`readrnxfile()` 读取精密星历文件，`combpclk()  `排序合并精密钟差。

### 2、peph2pos()：精密星历计算卫星位置、钟差、速度、钟漂

执行流程如下：
  * 调用`pephpos()`、`pephclk() `，计算卫星位置、钟差。
  * 增加一个极短的时间 tt 再计算卫星位置、钟差。
  * 调用`satantoff()`计算天线相位中心改正量 `dant[]`。
  * 卫星位置`rs[i]`：`pephpos()`计算值 + `satantoff()`天线相位中心改正 。
  * 卫星速度`rs[i+3]`：两次的位置相减再除以tt。
  * 钟差`dts[0]`：相对论效应改正 。
  * 钟漂`dts[1]`：两次的钟差相减再除以 tt
  * 没有精密钟差，`dts`赋值 0，`satposs()`中会 `ephclk()` 广播星历的钟差替代 

  ```c
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
          dts[1]=(dtst[0]-dtss[0])/tt;
      }
      else { /* no precise clock */
          dts[0]=dts[1]=0.0;          //没有精密钟差，dts赋值0，satposs中会ephclk()广播星历的钟差替代
      }
      if (var) *var=vare+varc;
      
      return 1;
  }
  
  ```

  

### 2、pephpos()：精密星历计算卫星位置，钟差

  ```c
  static int pephpos(gtime_t time, int sat, const nav_t *nav, double *rs,
                     double *dts, double *vare, double *varc)
  {
      double t[NMAX+1],p[3][NMAX+1],c[2],*pos,std=0.0,s[3],sinl,cosl;
      int i,j,k,index;
      
      trace(4,"pephpos : time=%s sat=%2d\n",time_str(time,3),sat);
      
      rs[0]=rs[1]=rs[2]=dts[0]=0.0;
      
      if (nav->ne<NMAX+1||        //如果时间早于第一个精密星历时间，或迟于最后一个超过15分钟，return 0
          timediff(time,nav->peph[0].time)<-MAXDTE||
          timediff(time,nav->peph[nav->ne-1].time)>MAXDTE) {
          trace(3,"no prec ephem %s sat=%2d\n",time_str(time,0),sat);
          return 0;
      }
      /* binary search */     // 二分查找nav->peph[]中时间差最接近的精密星历的下标index
      for (i=0,j=nav->ne-1;i<j;) {
          k=(i+j)/2;
          if (timediff(nav->peph[k].time,time)<0.0) i=k+1; else j=k;
      }
      index=i<=0?0:i-1;
      
      /* polynomial interpolation for orbit */    //轨道多项式插值
      i=index-(NMAX+1)/2;     
      if (i<0) i=0; else if (i+NMAX>=nav->ne) i=nav->ne-NMAX-1;
      
      for (j=0;j<=NMAX;j++) {
          t[j]=timediff(nav->peph[i+j].time,time);
          if (norm(nav->peph[i+j].pos[sat-1],3)<=0.0) {
              trace(3,"prec ephem outage %s sat=%2d\n",time_str(time,0),sat);
              return 0;
          }
      }
      for (j=0;j<=NMAX;j++) {
          pos=nav->peph[i+j].pos[sat-1];
          /* correciton for earh rotation ver.2.4.0 */        //地球自转改正
          sinl=sin(OMGE*t[j]);
          cosl=cos(OMGE*t[j]);
          p[0][j]=cosl*pos[0]-sinl*pos[1];
          p[1][j]=sinl*pos[0]+cosl*pos[1];
          p[2][j]=pos[2];
      }
      for (i=0;i<3;i++) {
          rs[i]=interppol(t,p[i],NMAX+1);     //内维尔多项式插值获取卫星位置
      }
      if (vare) {
          for (i=0;i<3;i++) s[i]=nav->peph[index].std[sat-1][i];
          std=norm(s,3);
          
          /* extrapolation error for orbit */
          if      (t[0   ]>0.0) std+=EXTERR_EPH*SQR(t[0   ])/2.0;
          else if (t[NMAX]<0.0) std+=EXTERR_EPH*SQR(t[NMAX])/2.0;
          *vare=SQR(std);
      }
      /* linear interpolation for clock */        //线性插值获取钟差
      t[0]=timediff(time,nav->peph[index  ].time);
      t[1]=timediff(time,nav->peph[index+1].time);
      c[0]=nav->peph[index  ].pos[sat-1][3];
      c[1]=nav->peph[index+1].pos[sat-1][3];
      
      //计算标准差
      if (t[0]<=0.0) {
          if ((dts[0]=c[0])!=0.0) {
              std=nav->peph[index].std[sat-1][3]*CLIGHT-EXTERR_CLK*t[0];
          }
      }
      else if (t[1]>=0.0) {
          if ((dts[0]=c[1])!=0.0) {
              std=nav->peph[index+1].std[sat-1][3]*CLIGHT+EXTERR_CLK*t[1];
          }
      }
      else if (c[0]!=0.0&&c[1]!=0.0) {
          dts[0]=(c[1]*t[0]-c[0]*t[1])/(t[0]-t[1]);
          i=t[0]<-t[1]?0:1;
          std=nav->peph[index+i].std[sat-1][3]+EXTERR_CLK*fabs(t[i]);
      }
      else {
          dts[0]=0.0;
      }
      if (varc) *varc=SQR(std);
      return 1;
  }
  ```



### 3、interppol()：Neville 插值

由两个n-1次插值多项式构造一个n次多项式的线性逐次插值方法 

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/3ff9e1ba7fca4004a534aaf8f7e835b9.png)


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

  

### 5、pephclk()：精密钟差计算卫星钟差

  ```c
  static int pephclk(gtime_t time, int sat, const nav_t *nav, double *dts,
                     double *varc)
  {
      double t[2],c[2],std;
      int i,j,k,index;
      
      trace(4,"pephclk : time=%s sat=%2d\n",time_str(time,3),sat);
      
      if (nav->nc<2||
          timediff(time,nav->pclk[0].time)<-MAXDTE||
          timediff(time,nav->pclk[nav->nc-1].time)>MAXDTE) {
          trace(3,"no prec clock %s sat=%2d\n",time_str(time,0),sat);
          return 1;
      }
      /* binary search */     //二分查找nav->peph[]中时间差最接近的精密星历的下标index
      for (i=0,j=nav->nc-1;i<j;) {
          k=(i+j)/2;
          if (timediff(nav->pclk[k].time,time)<0.0) i=k+1; else j=k;
      }
      index=i<=0?0:i-1;
      
      /* linear interpolation for clock */        //钟差线性插值
      t[0]=timediff(time,nav->pclk[index  ].time);
      t[1]=timediff(time,nav->pclk[index+1].time);
      c[0]=nav->pclk[index  ].clk[sat-1][0];
      c[1]=nav->pclk[index+1].clk[sat-1][0];
      
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
          std=nav->pclk[index+i].std[sat-1][0]*CLIGHT+EXTERR_CLK*fabs(t[i]);
      }
      else {
          trace(3,"prec clock outage %s sat=%2d\n",time_str(time,0),sat);
          return 0;
      }
      if (varc) *varc=SQR(std);
      return 1;
  }
  

		[0]=timediff(time,nav->pclk[index  ].time);
      t[1]=timediff(time,nav->pclk[index+1].time);
      c[0]=nav->pclk[index  ].clk[sat-1][0];
      c[1]=nav->pclk[index+1].clk[sat-1][0];
      
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
          std=nav->pclk[index+i].std[sat-1][0]*CLIGHT+EXTERR_CLK*fabs(t[i]);
      }
      else {
          trace(3,"prec clock outage %s sat=%2d\n",time_str(time,0),sat);
          return 0;
      }
      if (varc) *varc=SQR(std);
      return 1;
  }