## 一、程序概述



> * 网址：http://www.gnssbook.cn/book2/index.html
>
> * 程序下载：http://www.gnssbook.cn/book2/GnssRcvr_V14.rar
>
> * 实例北斗GPS双模中频数据文件下载：[UTREK210_16369000_70s.DAT](https://pan.baidu.com/s/1EWB0oQxDneNDk9iqExLuqQ)，提取码: 829c 







### 8、UTREK210_16369000_70s.DAT

示例数据

* UTREK210：本书配套的GPS/北斗双模中频数据采集系统 
* 16369000：采样频率
* 70s：数据总时长70s





## 二、程序使用







## 三、startRun.m

执行流程：

1. `clc;clear all;close all;`：清理工作区
2. `addpath()`：添加工作路径`gui`、`Acquisition`、`Tracking`、`Navmsg`、`PVT`，之后可调用里面的函数
3. `global`定义全局变量，这些变量主要用于控制程序执行
4. 为`buffLSResult`、`buffKFResult`开辟空间，用于存最小二乘解和卡尔曼滤波解
5. 调用`gnssInit()`，初始化和GNSS接收机相关的全局变量
6. 调用`main_fig()`，生成主界面
7. `fopen()`以读的方式打开或创建文件`debug.txt`
8. 为`sigBuff`开辟空间，
9. 





## 五、gnssInit()：初始化

执行流程：

1. `global`定义**全局变量**
2. 为一些**全局变量**开辟空间、赋初值
3. 调用`createValueMapping()`，产生**电平转换和串并转换的查找表**`sampleValueTable`
4. 调用`ReadGnssConfig()`，获取**配置选项**
5. 计算**每一次读取中频数据的长度**， 目前默认一次读取50毫秒的中频数据 
   * 
6. 初始化**物理常量**（载波频率、光速等）作为`PhyConstant`结构体的成员
7. 调用`GpsCodeGen()`、`BdCodeGen()`，初始化**伪随机码**`gpsPrnTbl`、`bdPrnTbl`
8. 对GPS、BDS的**伪随机码**`gpsPrnTbl`、`bdPrnTbl`**重采样**，得到`gpsCode2048`、`bdCode4096`
9. 初始化GPS和北斗卫星**捕获跟踪状态**结构体`gpsSvStatus`、`bdSvStatus`，成员赋值0
10. 初始化中频数据**文件句柄**`ifFileHdle`为`-1`
11. 初始化**正弦、余弦表**`Carrier_I_tbl`、`Carrier_Q_tbl`
12. 初始化**乘法结果表**`MuxTbl`
13. 调用`ResetChannel`，初始化24个**跟踪通道**，
14. 初始化**观测量结构体**`measGnss`
15. 初始化**本地时间和位置、速度结构体**`localTime`、`localPosvel`、`localPosvelKF`
16. 初始化**星历参数结构体**`GpsEph`、`BdEph`

```matlab
function []=gnssInit();
%% 初始化和GNSS接收机相关的全局变量
%% 包括物理常量

global gnssConfig;
global ifFileLength;
global sampleValueTable; 
global gpsPrnTbl;
global bdPrnTbl;
global PhyConstant;
global ifFileHdle;
global Carrier_I_tbl;
global Carrier_Q_tbl;
global gpsCode2048;
global bdCode4096;
global BufferIfLength;
global gpsSvStatus;
global bdSvStatus;
global trackingChannel;
global measChannel; 
global MuxTbl;
global halfChipPhaseStep;
global sysTimeMs;
global prevSysTimeMs;
global sampCnt3;
global Bitmask31;
global bitmapvalue;
global ncoNorm;
global BD_NH_code;
global GpsEph;
global BdEph;
global MeasTicThrd;
global MeasTicCounter;
global TakeMeasFlag;
global measGnss; 
global localPosvel;
global localTime;
global kalmanFilter;
global guidisp;
global localPosvelKF;
global Re2t;
global g_ifdat;
global b_ifdat;

gpsPrnTbl  = zeros(33,1023);
bdPrnTbl = zeros(33,2046);
gpsCode2048 = zeros(33,2048);
bdCode4096 = zeros(33,4096);
sysTimeMs = 0;
prevSysTimeMs = 0;
sampCnt3 = 0;
ifFileLength = 0;
Bitmask31 = 2^31 -1;
guidisp = 1; %显示GUI界面
Re2t = zeros(3,3);
bitmapvalue=[1 3 -1 -3];
cdvalue =[-1 1];
BD_NH_code = [1 1 1 1 1 -1 1 1 -1 -1 1 -1 1 -1 1 1 -1 -1 -1 1];

%% 产生采样值电平转换表
sampleValueTable = createValueMapping(sampleValueTable);
%% 读取软件接收机配置信息
gnssConfig = ReadGnssConfig(gnssConfig);
ncoNorm = 2^31/gnssConfig.fs;

MeasTicThrd = uint32(0.1*gnssConfig.fs);  % 设定观测量提取频率为10Hz
if gnssConfig.div3==1,
    ncoNorm = 3*ncoNorm;
    MeasTicThrd = uint32(MeasTicThrd/3);
end;
MeasTicCounter = uint32(0);
TakeMeasFlag = uint32(0);

%% 计算每一次读取中频数据的长度， 目前默认一次读取50毫秒的中频数据 
if gnssConfig.FileFormat==1  % 如果是GPS+BDS模式
    BufferIfLength = ceil(50*gnssConfig.fs/2000);  % 一个字节包含两个采样点
    if gnssConfig.div3==1,
        g_ifdat = uint8(zeros(1,ceil(BufferIfLength*2/3)+1000));
        b_ifdat = uint8(zeros(1,ceil(BufferIfLength*2/3)+1000));        
    else        
        g_ifdat = uint8(zeros(1,ceil(BufferIfLength*2)+1000));
        b_ifdat = uint8(zeros(1,ceil(BufferIfLength*2)+1000));
    end;
else    % 如果是单GPS模式
    BufferIfLength = ceil(50*gnssConfig.fs/4000);  % 一个字节包含四个采样点 
    if gnssConfig.div3==1,
        g_ifdat = uint8(zeros(1,ceil(BufferIfLength*4/3)+1000));
        b_ifdat = uint8(zeros(1,ceil(BufferIfLength*4/3)+1000));        
    else        
        g_ifdat = uint8(zeros(1,ceil(BufferIfLength*4)+1000));
        b_ifdat = uint8(zeros(1,ceil(BufferIfLength*4)+1000));
    end;    
end;

%% 初始化物理常量
PhyConstant.L1_freq = 1575.42E6;
PhyConstant.B1_freq = 1561.098E6;
PhyConstant.C = 2.99792458E8;
PhyConstant.C_INV =3.33564095198152e-9;
PhyConstant.L1_lambda = PhyConstant.C/PhyConstant.L1_freq;
PhyConstant.B1_lambda = PhyConstant.C/PhyConstant.B1_freq;
PhyConstant.PI = 3.141592653589793238;
PhyConstant.F_RC = -4.442807633E-10; %相对论效应校正参数 
PhyConstant.GM = 3.986005e14;
PhyConstant.OMEGAE84  = 7.2921151467e-5; % 地球自转角速度
PhyConstant.WGS84_A =  6378137.0;  %WGS84 地球半长轴
PhyConstant.SEC_IN_WEEK = 604800;  %周内秒计数总数
PhyConstant.HalfSEC_IN_WEEK = 302400;%周内秒计数一半数

%% 初始化GPS伪随机码
for i=1:33,
    gpsPrnTbl(i,:) = GpsCodeGen(i);
end;
%% 初始化北斗伪随机码
for i=1:33,
    bdPrnTbl(i,:) = BdCodeGen(i);
end;


%% 对GPS和北斗伪随机码进行重采样
     % GPS code case
for i=1:2048,
    k=floor(i*1.023E6/2.048E6);
    for m=1:33
        gpsCode2048(m,i) = gpsPrnTbl(m,( mod(k,1023)+1 ) );
    end;
end;   
    % BD code case
for i=1:4096,
    k=floor(i*2.046E6/4.096E6);
    for m=1:33,
        bdCode4096(m,i) = bdPrnTbl(m,( mod(k,2046)+1 ) );
    end;
end;


%% 初始化GPS和北斗卫星捕获跟踪状态
for i=1:32,
    gpsSvStatus(i).state  = 0;
    gpsSvStatus(i).doppler = 0;
    gpsSvStatus(i).code = 0;
end;
for i=1:14,
    bdSvStatus(i).state  = 0;
    bdSvStatus(i).doppler = 0;
    bdSvStatus(i).code = 0;
    bdSvStatus(i).NHidx = 0;
end;

%% 初始化中频数据文件句柄
ifFileHdle = -1;

%% 初始化正弦、余弦表 
Carrier_I_tbl = [ 0, 6,11,15,16,15, 11,  6,  0, -6,-11,-15,-16,-15,-11,-6];
Carrier_Q_tbl = [16,15,11, 6, 0,-6,-11,-15,-16,-15,-11, -6,  0,  6, 11,15];

%% 初始化乘法结果表
for i=1:128,
    cdIdx = bitget(i,7);
    smpIdx = bitand(bitshift(i,-4),3);
    phIdx = bitand(i,15);
    MuxTbl(2*i-1) = cdvalue(cdIdx+1)*bitmapvalue(smpIdx+1)*Carrier_I_tbl(phIdx+1);
    MuxTbl(2*i) = cdvalue(cdIdx+1)*bitmapvalue(smpIdx+1)*Carrier_Q_tbl(phIdx+1);
end;
halfChipPhaseStep = 2^30; % 0.5 chip spacing

%% 初始化跟踪通道
for i=1:24,  % 24个跟踪通道
    ResetChannel(i);
end;
%% 初始化观测量
measGnss.num = 0;
measGnss.timetag = 0.0;
measGnss.prn = zeros(1,24,'uint8');
measGnss.gnssId = zeros(1,24,'uint8');
measGnss.CN0 = zeros(1,24,'uint8');
measGnss.pseudo_cd_ph = zeros(1,24);
measGnss.doppler = zeros(1,24);    
%% 初始化本地时间和位置、速度
localTime.accu = 0;
localTime.second = 0.0;
localTime.week =0;
localTime.timetag = 0;

localPosvel.pos = zeros(1,3);
localPosvel.llh = zeros(1,3);
localPosvel.dops = zeros(1,4);
localPosvel.bias = 0;
localPosvel.gbto = 0;
localPosvel.vel = zeros(1,3);
localPosvel.velENU = zeros(1,3);
localPosvel.drift = 0;
localPosvel.mode = uint8(0);
localPosvel.valid = uint8(0);
localPosvel.svnum = uint8(0);
localPosvel.fixSvSet = uint32(0);
localPosvel.fixSvSetBD = uint32(0);
localPosvel.localT = 0;

localPosvelKF.pos = zeros(1,3);
localPosvelKF.llh = zeros(1,3);
localPosvelKF.dops = zeros(1,4);
localPosvelKF.bias = 0;
localPosvelKF.gbto = 0;
localPosvelKF.vel = zeros(1,3);
localPosvelKF.velENU = zeros(1,3);
localPosvelKF.drift = 0;
localPosvelKF.mode = uint8(0);
localPosvelKF.valid = uint8(0);
localPosvelKF.svnum = uint8(0);
localPosvelKF.fixSvSet = uint32(0);
localPosvelKF.fixSvSetBD = uint32(0);
localPosvelKF.localT = 0;

%% 初始化卡尔曼滤波器
kalmanFilter.state = 0;  %初始化标志
kalmanFilter.x = zeros(1,9);
kalmanFilter.dx = zeros(1,9);
kalmanFilter.P = zeros(9,9);
kalmanFilter.phi = zeros(9,9);
kalmanFilter.Q = zeros(9,9);
kalmanFilter.measNum = 0;
kalmanFilter.Hvec = zeros(24,3);
kalmanFilter.measRange = zeros(1,24);
kalmanFilter.measDoppler = zeros(1,24);
kalmanFilter.Rrange = zeros(1,24);
kalmanFilter.Rdoppler = zeros(1,24);
kalmanFilter.localT = 0;
kalmanFilter.gbto_init = 0;
kalmanFilter.progagationTime = 0;
kalmanFilter.lsinitState = 0;

%% 初始化GPS 星历参数
for k=1:32,
    GpsEph(k).valid = 0;
    GpsEph(k).health = uint8(255);
    GpsEph(k).toc = 0;
    GpsEph(k).toe = 0;
    GpsEph(k).dn = 0;
    GpsEph(k).cuc = 0;
    GpsEph(k).cus = 0;
    GpsEph(k).cic = 0;
    GpsEph(k).cis = 0;
    GpsEph(k).crc = 0;
    GpsEph(k).crs = 0;
    GpsEph(k).ety = 0;
    GpsEph(k).sqra = 0;
    GpsEph(k).ma = 0;
    GpsEph(k).w0 = 0;
    GpsEph(k).inc0 = 0;
    GpsEph(k).omega = 0;
    GpsEph(k).omegadot = 0;
    GpsEph(k).idot = 0;
    GpsEph(k).af0 = 0;
    GpsEph(k).af1 = 0;
    GpsEph(k).af2 = 0;
    GpsEph(k).tgd = 0;
    GpsEph(k).iodc = 0;
    GpsEph(k).iode = 0;
    GpsEph(k).iode2 = 0;
    GpsEph(k).tow = 0;
    GpsEph(k).ura = 0;
    GpsEph(k).week = 0;
end;
%初始化北斗星历数据
for k=1:32,
    BdEph(k).valid = 0;
    BdEph(k).health = uint8(255);
    BdEph(k).toc = 0;
    BdEph(k).toe = 0;
    BdEph(k).dn = 0;
    BdEph(k).cuc = 0;
    BdEph(k).cus = 0;
    BdEph(k).cic = 0;
    BdEph(k).cis = 0;
    BdEph(k).crc = 0;
    BdEph(k).crs = 0;
    BdEph(k).ety = 0;
    BdEph(k).sqra = 0;
    BdEph(k).ma = 0;
    BdEph(k).w0 = 0;
    BdEph(k).inc0 = 0;
    BdEph(k).omega = 0;
    BdEph(k).omegadot = 0;
    BdEph(k).idot = 0;
    BdEph(k).af0 = 0;
    BdEph(k).af1 = 0;
    BdEph(k).af2 = 0;
    BdEph(k).tgd = 0;
    BdEph(k).tgd2 = 0;
    BdEph(k).iodc = 0;
    BdEph(k).iode = 0;
    BdEph(k).iode2 = 0;
    BdEph(k).tow = 0;
    BdEph(k).ura = 0;
    BdEph(k).week = 0;
end;

% MuxTbl = double(MuxTbl);
% gpsPrnTbl = double(gpsPrnTbl);
% bdPrnTbl = double(bdPrnTbl);
```



## 六、main_fig()：主界面





## 七、AcquisitionEngine()：信号捕获

执行流程：

1. `global`全局变量
2. 计算参与信号捕获的采样点数目
3. 调用`DownSampling()`，完成降采样过程和串并转换过程，降采样到2.048MHz(GPS)或4.096MHz(BDS)
4. for循环遍历GPS信号：
   * 遍历每一个跟踪通道，检测当前GPS信号是否被跟踪
   * 如果没有被跟踪，调用`AcquisitionByFFT()`利用FFT方法实现伪码相位纬度的并行搜索
   * 













## 八、AllocateTrackingChannel()：分配跟踪通道 









## 九、SignalTracking()：信号跟踪





## 十、gnssPvt()：伪距和多普勒PVT解算

1. 

















