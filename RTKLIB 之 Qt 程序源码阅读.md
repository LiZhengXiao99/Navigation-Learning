[TOC]

## 一、RTKLIB Qt 程序简介

RTKLIB 是知名的全球导航卫星系统 GNSS 开源定位解算程序包，日本东京海洋大学的高须知二（Tomoji Takasu）开发，由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好。

RTKLIB 功能很齐全，GNSS 数据处理所需的基本功能都有，支持的数据格式很多，既可以实时解算也可以后处理，既可以接自己的 GNSS 模块也可以连 IGS 的数据流，既可以解算自己采集的数据也可以算 IGS 测站的数据，既可以 RTK 也可以 PPP；许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来。它的项目结构如下所示：

![RTKLIB](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB.png)





![image-20231016114907087](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231016114907087.png)

* **rtklaunch**：界面程序启动器，界面如上，用来启动另外的界面程序
* **rtkget**：下载 GNSS 数据，包括 OBS、EPH、ATX、CLK、DCB 等多种文件，可同时下载起止时间内多个机构、多个测站的数据，但可能下载速度很慢。
* **rtkcov**：GNSS 数据转换，把采集的接收机原始数据转成 RINEX。
* **rtkplot**：原始数据绘图、结果绘图，可以用来做原始数据质量分析、结果精度分析、结果轨迹绘图。
* **rtkpost**：后处理定位解算，传入观测文件、星历文件和其它改正信息文件，设置好解算选项进行后处理解算。
* **rtknavi**：实时定位解算，接通导航数据流，实时定位解算绘图。
* **strsvr**：数据流转换播发。
* **srctblbrows**：NTRIP  资源列表浏览器。





与命令行程序相比，界面程序更容易上手操作，





Qt 工程以 Qmake 的方式组织，打开 `app\qtapp\qtapp.pro`，

Qt 程序就是写了一些 ui，核心的数据处理通过调用 `src` 目录下的那些核心代码，

界面程序基本上就是





<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB-Qt%E5%B7%A5%E7%A8%8B.png" alt="RTKLIB-Qt工程" style="zoom: 40%;" />





## 二、src 核心代码库介绍

> 代码库是 RTKLIB 的核心，要不咋能叫"LIB"呢？

RTKLIB 提供许多代码库和 API，包括：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM 函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

头文件 rtklib.h 是库的核心 ，主要有三大部分：**宏定义**、**结构体定义**、**全局变量**、**函数定义**

>  需要注意并非所有函数都可以直接调用，只有加了 EXPORT 前缀，而且在 RTKLIB.h 中声明了才行；想用 static 前缀的函数也很简单，只需要把前缀改成 EXPORT，然后在 rtklib.h 中加上声明。

### 1、宏定义



![rtklib.h 宏定义](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%2520%25E5%25AE%258F%25E5%25AE%259A%25E4%25B9%2589.png)



各种 ifdef

* **WIN32、WIN_DLL**：用 Windows 下的代码，
* **ENAGLO、ENAGAL、ENAQZS、ENACMP、ENAIRN、ENALEO**：启用除 GPS 外的卫星系统，
* **OBS_100HZ**：判定时间重合的阈值 DTTOL

* **MKL**：
* **LAPACK**：
* **IERS_MODE**L：
* **CPUTIME_IN_GPST**：
* **CLOCK_MONOTONIC_RAW**：
* **RRCENA**：
* **SVR_REUSEADDR**：
* **TIME_64BIT**：



---

### 2、结构体定义

![rtklib.h结构体](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/rtklib.h%25E7%25BB%2593%25E6%259E%2584%25E4%25BD%2593.png)

---

### 3、全局变量

* `extern const double chisqr[];`：**卡方检验表**
* `extern const prcopt_t prcopt_default;`：**默认处理选项**
* `extern const solopt_t solopt_default;`：**默认结果选项**
* `extern const sbsigpband_t igpband1[9][8];`：**SBAS IGP 波段 0-8**
* `extern const sbsigpband_t igpband2[2][5];`：**SBAS IGP 波段 9-10**
* `extern const char *formatstrs[];`：**数据流格式字符串**
* `extern opt_t sysopts[];`：**系统选项表**

---

### 4、基础函数定义

#### 1. 矩阵、向量、最小二乘、卡尔曼滤波

* RTKLIB 中用 double 类型一维数组表示矩阵，不能自动识别矩阵的行列数，每次传矩阵的时候都要传入行数 n、列数 m；

* 用矩阵的时候要先 malloc 开辟空间，用完记得 free 释放空间；

* 想改 RTKLIB 算法得能熟练计算矩阵加减乘除转置求逆；

* RTKLIB 没有实现矩阵加减的函数，用的时候直接写 for 循环，比如把三维向量 dx 加到 X 上：

  ```c
  for (i=0;i<3;i++) X += dx;
  ```

* 矩阵求逆用的 LU 分解法，时间复杂度 $O^3$ ，对于大规模的矩阵，如果利用矩阵的稀疏性和对称性等特性，而且当使用不完全分解方法（例如，只计算到一定程度或使用截断技术）时，LU 分解的效率会更高。

* `matprint()` 很常用，调试的时候很难直接看的矩阵元素的值（都是指针），得输出到终端或者文件再看。

![image-20231021091639437](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091639437.png)

---

#### 2. 时间和字符串

* RTKLIB 中时间一般都以 `gtime_t` 类型存储，为了提高时间表示的精度，分开存 GPST 时间的整秒数和不足一秒的部分。
* 经常需要做年月日时分秒、周+周内秒、GPST 三种时间之间的转换；输出北京时间要在 UTC 基础上加 8 小时。
* BDT、GLONASST 不用于计算，读完文件就转为 GPS 时间了。

![image-20231021090651319](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021090651319.png)

---

#### 3. 坐标转换

* ECI 在定位程序中用的很少，只在 `sunmoonpos()` 函数中计算日月坐标时候用到了，无需关注。
* ENU、ECEF、LLH 三套坐标系都频繁使用，要熟练掌握他们之间的转换，包括协方差的转换
* ENU 是局部相对坐标系，以某一个 LLH 坐标为原点，坐标转换的时候要传入这个 LLH 坐标。
* ENU 常用 `e`表示、ECEF 常用 `r` 表示、LLH 常用 `pos` 表示。

![image-20231021091756265](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231021091756265.png)

---

#### 4. 卫星系统、观测值

* **卫星系统表示**：
  * 表示卫星系统的字母：GRECJIS。
  * 7 位二进制码表示，对应位写 1 表示有对应的系统，做与运算可加系统。
* **卫星的表示**：
  * 可以表示为各系统的卫星 ID（系统缩写+PRN）：B02、C21。
  * 也可表示为连续的卫星编号 satellite number，断点调试或者看 Trace 文件的时候，经常只能看到卫星编号。
* **观测值类型**：
  * **C**：伪距、**D**：多普勒、**L**：载波相位、**S**：载噪比。
  * `CODE_XXX`：观测值类型定义，用一串连续的数字表示。
  * `sigind_t`：表示每种卫星系统的载波类型和观测值类型 ，每种类型的系统其实对应的就是一个 `sigind_t` 结构体。
* **观测值优先级**：

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231024191221181.png)

---

### 5、选项调试输出

#### 1. 配置选项读取

* 选择主要存在 `prcopt_t`、`solopt_t`、`filopt_t` 三个结构体中。
* 后处理解算程序 rnx2rtkp 和实时解算程序 rtksvr 读取结果文件流程是一样的：
  * 先调用 `resetsysopts()` 重置所有配置为默认。
  * 调用 `loadopts()` 读取配置文件内容，存入 `opt_t` 的 `sysopt` 中。
  * 最后调用 `getsysopts()` 将 `opt_t` 转到 `porcopt_t`/`solopt_t`/`filopt_t`。

![image-20231025205106288](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205106288.png)

---

#### 2. Trace 调试

* 在 rtklib.h 中加入 #define TRACE，启用 trace ，不定义则将 trace 函数全赋空值。
* Trace 信息分五个等级，从 1-5 重要性逐渐降低，通过 tracelevel() 函数可以设置输出的最高等级，设置 2 级意味着只输出 1/2 级信息。
  * **一级 Trace 是致命错误**，出现一级错误基本上意味着程序无法继续执行，比如观测星历文件读取错误、内存分配错误。
  * **二级 Trace 是警告**，出现二级警告程序可能依然能继续执行，但也可能无法进行解算，比如改正文件读取失败，数据解析出错，二进制数据校验出错，某一历元解算失败，缺失解算所需的星历或改正参数等。
  * **三级  Trace 是程序主要执行流程**，主要在函数的开头，告诉我们执行到了这个函数。
  * **四级 Trace 是比三级更深入的程序执行流程**，主要在三级  Trace 函数的中间或者调用的子函数开头，告诉我们执行到了这个操作。
  * **五级 Trace 是解算的中间过程**，具体到每颗卫星，每个频点，每次循环。
* 看 Trace 文件可以辅助断点调试，甚至替代断点调试。程序执行出错，开 2/3 级 Trace，看 Trace 文件里的 error、warring 就能，知道大致出了啥问题，定位出问题的函数，断点调试的时候你就知道该在哪设置断点了。

![image-20231025205158762](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205158762.png)

---

#### 3. 结果输入输出、NMEA

* 输出的结果有两套：
  * 定位结果：坐标、协方差、有效果卫星数、差分龄期
  * 解算中间结果：
* NMEA 读取：

![image-20231025205301277](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205301277.png)

---

### 5、量测数据

#### 1. 导航数据输入

* 用完数据记得释放内存。

![image-20231025205602582](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205602582.png)

---

#### 2. RINEX 文件读写

* `readrnx()` 函数可以实现读取单个的 RINEX 文件，这里没列出来的 readobsnav() 可以根据传入的输入文件数组 `infile` 来读取；
* 输出 RINEX 文件的函数主要用与转换；

![image-20231025205701553](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205701553.png)

---

#### 3. 二进制数据读写

* 用于数据流解析。

![image-20231025205800413](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205800413.png)

---

#### 4. 星历数据解析





---

#### 5. RTCM 读写

支持的 RTCM 消息包括：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6a7fd3b4bcb908dfff7b08d7e438bf83.png" alt="6a7fd3b4bcb908dfff7b08d7e438bf83" style="zoom:50%;" />

想进行定位解释至少要有星历，要有观测数据，常用 NAV 配 MSM4（伪距载波信噪比）、MSM7（伪距载波多普勒信噪比）

![image-20231025205952192](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025205952192.png)

---

#### 6. 接收机自定义格式读写





---

### 6、解算相关

#### 1. 定位解算入口



![image-20231025210531296](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210531296.png)

---

#### 2. 实时解算



![image-20231025210624595](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210624595.png)

---

### 7、数据流相关

#### 1. 数据流

* 数据流函数用的大部分在 Window 和 Linux 各有一套，涉及到很多系统库，好在现在 AI 发达，可以用来辅助理解。
* 每种数据流关注四个函数：打开、关闭、写数据、读数据。

![image-20231025210846191](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210846191.png)

---

#### 2. 数据流线程管理



![image-20231025210938225](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025210938225.png)

---

### 8、模型改正

#### 1. 星历、钟差、DCB、FCB



![image-20231025223050670](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223050670.png)

---

#### 2. SBAS



![image-20231025223134482](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223134482.png)

---

#### 3. 定位模型：计算高度角、方位角、卫地距、DOP 值



![image-20231025223220415](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223220415.png)

---

#### 4. 对流层、电离层模型



![image-20231025223257219](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223257219.png)

---

#### 5. 天线改正：读取天线文件



![image-20231025223330203](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223330203.png)

---

#### 6. 潮汐改正



![image-20231025223344099](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223344099.png)

---

#### 7. 水准面模型



![image-20231025223357868](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223357868.png)

---

#### 8. 高程转换

都是转日本的高程系统，咱们用不到。

![image-20231025223409859](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025223409859.png)

---

### 9、其它杂项函数

#### 1. 下载函数

![image-20231025212102845](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212102845.png)

---

#### 2. 结果格式转换

把定位结果转为 KML、GPX 格式：

* **KML**：
* **GPX**：

![image-20231025212144253](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212144253.png)

---

#### 3. GIS 数据读取

可以读取 shapfile 矢量数据

![image-20231025212339125](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212339125.png)

---

#### 4. 平台相关函数

在 Windows 和 Linux 有完全不同的两套实现

![image-20231025212504258](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231025212504258.png)

---

#### 5. 用户自定义函数

在 rtklib.h 中声明了，在代码库中使用了，但没有实现，用代码库的时候需要我们自己实现，一般写个空函数就行。编译的时候经常会在这报错，如果说未定义就写三个空实现，如果重定义就把写的实现注释掉。





---

### 10、代码库的使用总结

总结一些命名和函数定义习惯：

* 矩阵做参数时一点要带上维度，矩阵 m 为行、n 为列，一般先传 n 后传 m。
* 带 `const` 的指针一定是输入参数，不带 `const` 的指针是输出参数或者既是输入也是输出。
* 类型命名结尾都带 `_t`，类型传参用的指针名不带 `_t`
* 用指针实现了顺序表，
* 很多读文件的函数都有结尾带 t 和结尾不带 t 两种，带 t 表示要传入开始时间和结束时间的。

* 





## 四、appcmn_qt

### 1、aboutdlg：



### 2、cmdoptdlg：



### 3、console：



### 4、fileoptdlg：



### 5、freqdlg：



### 6、ftpoptdlg：



### 7、glofcndlg：



### 8、gmview：



### 9、keydlg：



### 10、mapview：



### 11、mapviewopt：



### 12、maskoptdlg：



### 13、mntpoptdlg：



navi_post_opt：



pntdlg：



refdlg：



serioptdlg：



tcpoptdlg：



timedlg：



tspandlg：



viewer：



vieweropt：





