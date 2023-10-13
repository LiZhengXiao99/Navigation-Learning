> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、RTKLIB 简介

### 1、程序概述



![image-20231012215849180](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012215849180.png)

### 2、学习建议



* 有一点点 C 语言基础就可以了，不需要特意的再学语法，没见过的语法查一下，下次就会了；现在人工智能越来越强，把 RTKLIB 的代码段复制给AI，基本都能给你解释解释。
* 学的时候从后处理开始，把从读取RINEX文件到算出定位结果整个过程看明白，
* 入门建议看硕博论文，比如我的老师陈健和杨旭的硕士论文，写的简单，算法几乎和RTKLIB一模一样。
* 代码量很大，直接看可能会一头雾水；可以流程图、函数调用关系图、思维导图，
* 网上 RTKLIB 的资料很丰富，基本上能把每一行代码的意思都给你讲明白了，可以照着
* 算法的学习：
  * **矩阵运算**：
  * **参数估计**：最小二乘、卡尔曼滤波
  * **时间系统**：理解 gtime_t 类型，知道
  * **坐标系统**：
  * **数据读取**：RINEX、RTCM、NMEA，不用太细看，对数据格式有个基本的认识，知道读进来的数据以什么形式，存到什么变量里就OK。
  * **卫星位置钟差计算**：
  * **电离层、对流层改正**：
  * **伪距单点定位**：
  * **周跳检测**：
  * **模糊度固定**：
  * **差分定位**：
  * **精密单点定位**：

### 3、功能简介





### 4、代码分析





### 5、manual



![image-20231012203219194](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012203219194.png)

### 6、后处理程序执行流程图

![image-20231012211635669](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012211635669.png)

### 7、后处理函数调用关系

![image-20231012212121216](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231012212121216.png)





## 二、VS2022 下编译调试

### 1、下载RTKLIB

​	在[RTKLIB官网](https://www.rtklib.com/)选最新版 **2.4.3 b34**，点**Source Programs and Data**下面的**GitHub**进入**GitHub页面**，点开绿色的**Code**下拉菜单，再点**Download ZIP**，下载解压即可。

![1689207739592](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689207739592.png)

### 2、在VS2022中创建空C++项目、导入源码文件

1. 创建**C++空项目**，可以勾选“解决方案和项目放在统一目录中”，记住创建的项目目录。

2. 把 RTKLIB 源码文件中**整个src文件夹**复制到创建的项目文件目录中。

3. 把 RTKLIB 源码文件中 **\app\consapp** 中的 **rnx2rtkp.c** 放到刚刚复制过去的 **src文件夹**。

4. 在解决源文件中添加名为 “src” 的筛选器，再在 src 筛选器下面添加名为 “rcv” 的筛选器 。

   右键添加现有项目把 **src/rcv文件夹** 中的所有文件加到 **src/rcv筛选器** 中，src 中所有代码文件加到 src 筛选器中。

5. 把主函数 **rnx2rtkp.c** 文件中的 **#include "rtklib.h"** 修改为 **#include "./rtklib.h“** 

   把在 **src/rcv文件夹几个的.c文件** 中的 **#include "rtklib.h"** 修改为 **#include "../rtklib.h”** 

### 3、项目属性设置

1. 打开项目属性，在**链接器—输入—附加依赖项**中添加依赖库**winmm.lib**和**ws2_32.lib**。 

2. **配置属性—高级—字符集**中设置为用**使用多字节字符集** 。

3. **C/C++**中的**SDL检查设置为否**，**附加包含目录**添加**.\src** 、预编译头为**不使用预编译头** 。

   预处理器中添加如下内容： 

   ```
   _LIB
   _CRT_SECURE_NO_WARNINGS
   _WINSOCK_DEPRECATED_NO_WARNINGS             
   ENAGLO
   ENACMP
   DENAGAL
   DLL
   WIN32
   ```

   > 尤其主要加WIN32，好多博客都没加这一项，加了这一项后RTKLIB就不会用Linux下的<pthread.h>和<sys/select.h>，咱们项目要在Windows下编译运行的，不加会报”找不到<pthread.h>和<sys/select.h>“的错。

4. 将常规中的目标文件名改为rnx2rtkp 。

   > 改不改都行，默认目标文件名是项目名。

### 4、改代码的BUG

​	可能会报“使用了可能未初始化的本地指针变量“sbs”的错误，解决方式是对指针变量进行初始化，将ephemeris.c文件中的第579行改为“const sbssatp_t *sbs=NULL;”。

## 三、VScode + WSL 下编译调试

开发中经常要用到 Linux，WSL

### 1、项目文件

* 去 [GitHub](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3) 下载一份最新的 2.4.3 b34 源代码压缩包

* 创建文件夹 `my_rnx2rtkp` 做项目主文件夹。

* 在 `my_rnx2rtkp` 里新建文件夹 src\ 、include\、out_exe\

* 将 RTKLIB 源码中的 `src` 文件夹所有 `.c` 文件加的我们的 `src` 文件夹里，并将源码中的 `app/consapp/rnx2rtkp/rnx2rtkp.c` 也放入我们的 `src` 做主函数文件。

* 将源码中的 `src` 文件夹的 `rtklib.h` 文件加的我们的 `include` 文件夹里

* 在  `src` 文件夹中创建 `CMakeLists.txt` ，编写 CMake 文件

  ```cmake
  cmake_minimum_required( VERSION 3.0 )
  project(my_rnx2rtkp)
  LINK_LIBRARIES(m)	# 添加 math
  aux_source_directory(. src_list)
  add_subdirectory(./rcv)	# rcv 子目录
  include_directories(../include)
  add_executable(rnx2rtkp ${src_list})
  ```

* 在子目录 `src/rcv` 里也创建一个  `CMakeLists.txt`

  ```Cmake
  cmake_minimum_required( VERSION 3.0 )
  project(my_rnx2rtkp)
  LINK_LIBRARIES(m)   # 添加 math
  aux_source_directory(. src_list)
  include_directories(../../include)
  ```

  



### 2、WSL 环境配置

* 安装 WSL：建议安装 WSL2，先在系统设置里启用虚拟机，然后在微软商城安装



* 更新软件列表

  ```bash
  sudo apt update
  ```

* 安装编译器和调试器 GCC、G++、GDB

  ```bash
  sudo apt install build-essential gdb
  ```

* 安装 CMake

  ```bash
  sudo apt install cmake
  ```

* 检测安装是否成功

  ```bash
  cmake -version
  ```

* * 

* 安装 ssh

  ```bash
  sudo apt install ssh
  ```

* 其它推荐软件的安装

  - vim：
  - git：
  - ranger：
  - mc：
  - exa：代替 ls
  - tree：
  - cloc：统计代码行数
  - ack：替代grep
  - glances：系统监控工具
  - colordiff：替代bat
  - bat：替代cat
  - dstat：


### 3、VScode 插件安装









## 四、rnx2rtkp 命令行参数

### 1、rnx2rtkp 命令行程序使用方式

* 使用方式：`rnx2rtkp [option]... file file [...] `

* 读取 RINEX：OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。

* 对于相对定位，第一个 OBS 观测值文件需含接收机、流动站观测值，第二个 OBS 文件需含基准站观测值。

* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。

* 想用 SP3 精密星历文件，需提供.sp3/.eph文件的路径。

* 输入文件路径可包含通配符 *，为了防止与命令行命令冲突，要用 `"..."`  括起带通配符符路径。

### 2、命令行参数

1. **-？**：打印 help
2. **-k** file：配置文件的输入选项，默认值是 [off]
3. **-o** file：输出文件选项，默认值是 [stdout]
4. **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是 [obs start time] 
5. **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是 [obs end time] 
6. **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
7. **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是 [2]
8. **-m** mask：设置截止高度角，`(deg) `,默认值是 [15]
9. **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是 [G|R] ，想用除GPS以为的系统，还得加宏定义 ENAGLO、ENACMP、ENAGAL
10. **-f ** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是 [2]
11. **-v ** thres：设置整周模糊度 Ratio 值，写 0.0 为不固定整周模糊度，默认值是 [3.0] 
12. **-b：**后向滤波
13. **-c**：前后向滤波组合
14. **-i**：单历元模糊度固定 instantaneous 
15. **-h**：fix and hold 模糊度固定
16. **-e**：输出 XYZ-ecef 坐标
17. **-a**：输出 ENU-baseline
18. **-n**：输出 NMEA-0183 GGA
19. **-g**：输出经纬度格式为 ddd mm ss.ss ，默认为 [ddd.ddd] 
20. **-t**：输出时间格式为 yyyy/mm/dd hh:mm:ss.ss ，默认为 [sssss.ss] 
21. **-u**：输出为 UTC 时间，默认为 [gpst] 
22. **-d** col：设置时间的小数位数，默认为 [3] 
23. **-s** sep：设置文件分隔符，要写在单引号中，默认为 [' '] 
24. **-r** x y z：基站位置 ECEF-XYZ (m)，默认 [average of single pos] ，流动站位置用于 fixed 模式
25. **-l** lat lon hgt：基站位置 LLH (deg/m)，默认 [average of single pos]，流动站位置用于 fixed模式
26. **-y** level：输出结果信息 (**0**:off,**1**:states,**2**:residuals) ，默认为 [0] 
27. **-x** level：输出 debug trace 等级，默认为 [0] 

### 3、stdarg库：解析命令行参数

stdarg.h 是 C 语言中C标准函数库的头文件，stdarg 是由 standard（标准）和 arguments（参数）简化而来，主要目的为让函数能够接收可变参数。 用法如下：

- 首先在函数里定义一具 VA_LIST 型的变量，这个变量是指向参数的指针 。
- 然后用 VA_START 宏初始化变量刚定义的 VA_LIST 变量 。
- 然后用 VA_ARG 返回可变的参数，VA_ARG 的第二个参数是你要返回的参数的类型（如果函数有多个可变参数的，依次调用 VA_ARG 获取各个参数） 
- 最后用 VA_END 宏结束可变参数的获取 

```c
#include <stdio.h>
#include <stdarg.h>
void printargs(int arg1, ...) /* 输出所有int类型的参数，直到-1结束 */
{
    va_list ap;	//定义一个va_list变量ap
    int i;
    va_start(ap, arg1);	//执行ap = (va_list)&v + _INTSIZEOF(v)，ap指向参数v之后的那个参数的地址，即 ap指向第一个可变参数在堆栈的地址。
    for (i = arg1; i != -1; i = va_arg(ap, int))	//va_arg(ap,t) ， ( *(t *)((ap += _INTSIZEOF(t)) - _INTSIZEOF(t)) )取出当前ap指针所指的值，并使ap指向下一个参数。 ap＋= sizeof(t类型)，让ap指向下一个参数的地址。然后返回ap-sizeof(t类型)的t类型*指针，这正是第一个可变参数在堆栈里的地址。然后 用*取得这个地址的内容。
    printf("%d ", i);
    va_end(ap);	//清空va_list ap
    putchar('\n');
}
int main(void)
{
    printargs(5, 2, 14, 84, 97, 15, 24, 48, -1);
    printargs(84, 51, -1);
    printargs(-1);
    printargs(1, -1);
    while(1);
    return 0;
}
```

