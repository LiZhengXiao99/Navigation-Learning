[TOC]

## 一、程序获取和编译







## 二、rnx2rtkp：基于 RINEX 后处理定位解算

```
rnx2rtkp [option ...] file file [...] 
```

### 1、简介

rnx2rtkp 全称 RINEX to RTK pos，通过原始 RINEX 文件，输出 RTKLIB 的定位坐标，下图可以很好的表示整个程序的逻辑：

![image-20231016123911526](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016123911526.png)

* 使用方式：`rnx2rtkp [option]... file file [...] `
* 读取 RINEX：OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。
* 对于相对定位，第一个 OBS 观测值文件需含接收机、流动站观测值，第二个 OBS 文件需含基准站观测值。
* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。
* 想用 SP3 精密星历文件，需提供 .sp3/.eph 文件的路径。
* 输入文件路径可包含通配符 *，为了防止与命令行命令冲突，要用 `"..."`  括起带通配符符路径。



### 2、配置选项

* `filopt_t`：**文件选项**，存结果输出、Trace、各种改正文件路径，不包括星历文件和观测文件。
* `solopt_t`：**结果选项**，可以设置结果输出形式（ENU、ECEF、NMEA、BLH），小数位数，是否输出文件头，是否输出速度等。
* `prcopt_t`：**处理选项**，是配置的重头戏，可以先看 [postpos 的用法](https://www.bilibili.com/video/BV1m5411Y7xV) 学习界面程序的配置方式。写代码配置和界面程序需要配置的东西是一样的，只是从在界面上选，换成在了代码里给对应字段赋值，或者在配置文件中设置。

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB%25E9%2585%258D%25E7%25BD%25AE%25E9%2580%2589%25E9%25A1%25B9.png)

### 3、配置文件





### 4、命令行参数

* **-？**：打印 help
* **-k** file：配置文件的输入选项，默认值是 [off]
* **-o** file：输出文件选项，默认值是 [stdout]
* **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是 [obs start time] 
* **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是 [obs end time] 
* **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
* **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是 [2]
* **-m** mask：设置截止高度角，`(deg) `,默认值是 [15]
* **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是 [G|R] ，想用除 GPS 以外的系统，还得加宏定义 ENAGLO、ENACMP、ENAGAL
* **-f** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是 [2]
* **-v** thres：设置整周模糊度 Ratio 值，写 0.0 为不固定整周模糊度，默认值是 [3.0] 
* **-b**：后向滤波
* **-c**：前后向滤波组合
* **-i**：单历元模糊度固定 instantaneous 
* **-h**：fix and hold 模糊度固定
* **-e**：输出 XYZ-ecef 坐标
* **-a**：输出 ENU-baseline
* **-n**：输出 NMEA-0183 GGA
* **-g**：输出经纬度格式为 ddd mm ss.ss ，默认为 [ddd.ddd] 
* **-t**：输出时间格式为 yyyy/mm/dd hh:mm:ss.ss ，默认为 [sssss.ss] 
* **-u**：输出为 UTC 时间，默认为 [gpst] 
* **-d** col：设置时间的小数位数，默认为 [3] 
* **-s** sep：设置文件分隔符，要写在单引号中，默认为 [' '] 
* **-r** x y z：基站位置 ECEF-XYZ (m)，默认 [average of single pos] ，流动站位置用于 fixed 模式
* **-l** lat lon hgt：基站位置 LLH (deg/m)，默认 [average of single pos]，流动站位置用于 fixed模式
* **-y** level：输出结果信息 (**0**:off,**1**:states,**2**:residuals) ，默认为 [0] 
* **-x** level：输出 debug trace 等级，默认为 [0] 

### 5、函数调用关系

![image-20231012212121216](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012212121216.png)

### 6、执行流程

![image-20231025155540386](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025155540386.png)

---

## 三、str2str：数据流存储转发

```
str2str -in stream[#...] -out stream[#...] [-out stream[#...]...] [options] 
```

```
OPTIONS 
	-in stream[#format] input stream path and format 
	-out stream[#format] output stream path and format 
```

### 1、简介

从数据流中输入数据，并将其分割和输出到多个数据流中，输入流可以是串行、tcp 客户端、tcp 服务器、ntrip 客户端或文件。输出流可以是串行、tcp 客户端、tcp 服务器、ntrip 服务器或文件。str2str 是常驻应用程序。要停止它：

* 如果运行在前台，则在控制台中键入 ctr-c；
* 如果运行在后台，则向后台进程发送 SIGINT 信号。

如果输入流和输出流都遵循 #format 输入信息的格式将被转换为输出格式。要指定输出使用 -msg 选项。如果省略选项 -in 或 -out，则输入为 stdin，"... "输出为 stdout、输入使用stdin，输出使用 stdout。如果选项 -in 或 -out 中的流为空，也会使用 stdin 或 stdout

### 2、命令行参数

使用方法：`str2str [-in stream] [-out stream [-out stream...]] [options]`

* 数据流路径：

  ```c
  serial       : serial://port[:brate[:bsize[:parity[:stopb[:fctr]]]]]
  tcp server   : tcpsvr://:port
  tcp client   : tcpcli://addr[:port]
  ntrip client : ntrip://[user[:passwd]@]addr[:port][/mntpnt]
  ntrip server : ntrips://[:passwd@]addr[:port]/mntpnt[:str] (only out)
  ntrip caster : ntripc://[user:passwd@][:port]/mntpnt[:srctbl] (only out)
  file         : [file://]path[::T][::+start][::xseppd][::S=swap]
  ```

* 数据格式：

  ```c
  rtcm2        : RTCM 2 (only in)
  rtcm3        : RTCM 3
  nov          : NovAtel OEMV/4/6,OEMStar (only in)
  oem3         : NovAtel OEM3 (only in)
  ubx          : ublox LEA-4T/5T/6T (only in)
  ss2          : NovAtel Superstar II (only in)
  hemis        : Hemisphere Eclipse/Crescent (only in)
  stq          : SkyTraq S1315F (only in)
  javad        : Javad (only in)
  nvs          : NVS BINR (only in)
  binex        : BINEX (only in)
  rt17         : Trimble RT17 (only in)
  sbf          : Septentrio SBF (only in)
  ```

* 选项：

  ```c
  -sta sta          测站 ID
  -opt opt          receiver dependent options
  -s  msec          timeout time (ms) [10000]
  -r  msec          reconnect interval (ms) [10000]
  -n  msec          nmea request cycle (m) [0]
  -f  sec           file swap margin (s) [30]
  -c  file          input commands file [no]
  -c1 file          output 1 commands file [no]
  -c2 file          output 2 commands file [no]
  -c3 file          output 3 commands file [no]
  -c4 file          output 4 commands file [no]
  -p  lat lon hgt   station position (latitude/longitude/height) (deg,m)
  -px x y z         station position (x/y/z-ecef) (m)
  -a  antinfo       antenna info (separated by ,)
  -i  rcvinfo       receiver info (separated by ,)
  -o  e n u         antenna offset (e,n,u) (m)
  -l  local_dir     ftp/http local directory []
  -x  proxy_addr    http/ntrip proxy address [no]
  -b  str_no        relay back messages from output str to input str [no]
  -t  level         trace level [0]
  -fl file          log file [str2str.trace]
  -h                print help
  ```

  

---

## 四、rtkrcv：实时定位解算

```
rtkrcv [-s][-p port|-d dev][-o file][-t level]
```

### 1、简介

在一台电脑上开 rtkrcv 实时定位解算，然后可以用另一台电脑通过 IP 地址和端口号连接，



### 2、命令行参数

*     `-s`：程序启动的时候开启 RTK 定位解算。
*     `-p port`：远程终端的端口号。
*     `-m port`：远程监控的端口号。
*     `-d dev`：终端设备。
*     `-o file`：处理选项文件，与 RNX2RTKP 相同，可以在界面程序 RTKPOST、RTKNAVI 上设置然后导出。
*     `-w pwd`：远程终端连接密码 ("": 无密码)。
*     `-r level`：输出解算状态文件 (0:off,1:states,2:residuals)。
*     `-t level`：Trace 调试级别 (0:off,1-5:on)。
*     `-sta sta`：接收机测站名。

### 3、终端命令

* `start`：开启实时解算，如果程序执行时传入了 -s 参数就不需要再用这个命令。
* `stop`：停止定位解算。
* `restart`：重启定位解算，如果处理选项重新设置了，发送这个命令使能新选项。
* 输出命令：如果不加 cycle 就是只输出一个结果，加了 cycle 就是按指定频率输出。
  * `solution [cycle]`：输出实时定位结果。
  * `status [cycle]`：输出解算状态。
  * `satellite [-n] [cycle]`：输出卫星状态。
  * `observ [-n] [cycle]`：输出观测数据。
  * `navidata [cycle]`：输出星历数据。
  * `stream [cycle]`：输出数据流状态。
* `error`：输出错误和警告信息，Ctrl-C 停止。
* `option [opt]`：输出处理选项，如果不跟 opt 就输出所有选项，加了 opt 就输出指定选项。
* `set opt [val]`：Save current processing optons to file. Without option, default file rtkrcv.conf is used.
* `load [file]`：导入处理选项文件，如果不加选项文件路径，默认读取 rtkrcv.conf；restart 重启定位解算来启用新的处理选项。
* `save [file]`：保存当前处理选项到文件，如果不加文件路径，默认存到 rtkrcv.conf。
* `log [file|off]`：log file 存下终端的 log 到文件，log off 停止记录。
* `help|? [path]`：输出命令列表，
* `exit`：退出终端连接，不影响定位解算解算。
* `shutdown`：停止定位解算，退出程序。
* `!command [arg...]`：执行操作系统的 shell 命令，不能使用需要交互的命令。

### 4、函数调用关系







### 5、程序执行流程

![image-20231024203128852](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024203128852.png)

---

## 五、convbin：数据文件格式转换

```c
convbin [-ts y/m/d h:m:s] [-te y/m/d h:m:s] [-ti tint] [-r format] [-ro opts] 
 		[-f freq] [-hc comment] [-hm marker] [-hn markno] [-ht marktype] 
	 	[-ho observ] [-hr rec] [-ha ant] [-hp pos] [-hd delta] [-v ver] [-od] 
 		[-os] [-x sat] [-y sys] [-d dir] [-c satid] [-o ofile] [-n nfile] 
 		[-g gfile] [-h hfile] [-q qfile] [-s sfile] file 
```







---

## 六、pos2kml：定位结果转谷歌地图格式

```c
pos2kml [option ...] file [...] 
```









