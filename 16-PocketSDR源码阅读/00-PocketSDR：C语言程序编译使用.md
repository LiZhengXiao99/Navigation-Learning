

[TOC]

## 一、编译







## 二、pocket_acq：信号捕获

```c
pocket_aqc [-sig sig] [-prn prn[,...]] [-tint tint] [-toff toff]
    [-f freq] [-fi freq] [-d freq] [-nz] file
```

从数字中频信号中查找 GNSS 信号，并绘绘制出捕获结果。



**命令行参数**：

```c
-sig 		sig 			GNSS 信号类型 (L1CA, L2CM, ...)
-prn 		prn[,...]		PRN 码，用英文逗号分隔，可以填单独 PRN 码，也可填 PRN 码范围 (1-32)
-tint 		tint			搜索 GNSS 的间隔 (ms) [code cycle]
-toff 		toff			中频信号起始位置 (ms) [0.0]
-f 			freq 			中频信号采样频率 (MHz) [12.0]
-fi 		freq			中频信号频率 (MHz) [0.0]
-d 			freq			最大多普勒频移 (Hz) [5000.0]
-nz							信号捕获的相关结果阈值
-h							输出帮助信息：使用方式、信号 ID
file						输入数字中频信号文件的路径，格式为连续的 int8_t，
    						可以存实采样(I)，也可以存复采样(IQ)
```





## 三、pocket_trk：信号跟踪

```c
pocket_trk [-sig sig] [-prn prn[,...]] [-toff toff] [-f freq]
    [-fi freq] [-IQ] [-ti tint] [-log path] [-out path] [-q] [file]
```



**命令行参数**：

```c
-sig 		sig			GNSS 信号类型 (L1CA, L2CM, ...)
-prn 		prn[,...] 	[file]
-toff 		toff		中频信号起始位置 (ms) [0.0]
-f 			freq		中频信号采样频率 (MHz) [12.0]
-fi 		freq		中频信号频率 (MHz) [0.0]
-IQ						
-ti 		tint		信号跟踪状态的更新间隔 (s) [0.05]
-log 		path		日志输出路径，记录导航电文解码信息、码偏差，路径要求如下：
    						1. 本地文件路径 (不含":")，可以包含通配符 (%Y, %m, %d, %h, %M)
    						2. TCP 服务端，需要端口号 [:port]
    						3. TCP 客户段，需要地址和端口号 [address:port]
-out 		path		输出数据流用来输出特殊信息，包括 UBX-RXM-QZSSL6 信息。
-q						Suppress showing signal tracking status
[file]					输入数字中频信号文件的路径，格式为连续的 int8_t，
    					可以存实采样(I)，也可以存复采样(IQ)
```







## 四、pocket_scan：串口扫描

```c
pocket_scan [-e]
```









## 五、pocket_conf：射频前端配置

```c
pocket_conf [-s] [-a] [-h] [-p bus[,port]] [conf_file]
```





**命令行参数**：

```c
-s								保存配置到射频前端的 EEPROM
-a
-h
-p 			[bus[,port]]		USB 总线和端口号。
conf_file
```





## 六、pocket_dump：射频前端采集数字中频信号

```c
pocket_dump [-t tsec] [-r] [-p bus[,port]] [-c conf_file] [-q]
            [file [file]]
```







**命令行参数**：

```c
-t 				tsec			采集数据的时间 (s)
-r								Dump raw SDR device data without channel separation and quantization.
-p 				bus[,port]		USB 总线和端口号。
-c 				conf_file		配置文件。
-q 								Suppress showing data dump status.
[file [file]]					输出中频信号文件路径，可以双通道输出两个文件。
    								* 如果用了 -r 参数，那只会用第一个文件路径。
    								* 如果文件路径传 ""，不输出。
    								* 如果文件路径传 -，输出到 stdout。
    							默认文件名：
    								* ch1_YYYYMMDD_hhmmss.bin
    								* ch2_YYYYMMDD_hhmmss.bin
```

























