[TOC]

## 一、RTKLIB 简介

### 1、程序概述



### 2、资源获取





### 3、功能简介



### 4、代码分析



### 5、manual

1. **Overview**：概述、
2. **User Requirements**：用户要求、
   1. **System Requirements**：系统要求、
   2. **License**：许可、
3. **Instructions** ：说明书、
   1. **Installation and Uninstallation**：安装和卸载
   2. **Real‐Time Positioning with RTKNAVI**：用RTKNAVI实时定位
   3. **Configure Input, Output and Log Streams for RTKNAVI**：为RTKNAVI配置输入、输出和日志流
   4. **Post‐Processing Analysis with RTKPOS**：用RTKPOST后处理分析
   5. **Configure Positioning Options for RTKNAVI and RTKPOST**：配置RTKNAVI和RTKPOST的定位选项
   6. **Convert Receiver Raw Data to RINEX with RTKCONV**：用RTKCONV将接收器原始数据转换为RINEX
   7. **View and Plot Solutions with RTKPLOT**：用RTKPLOT查看和绘制结果
   8. **View and Plot Observation Data with RTKPLOT**：用RTKPLOT查看和绘制观数据
   9. **Download GNSS Products and Data with RTKGET**：用RTKGET下载GNSS数据
   10. **NTRIP Browser** ：NTRIP客户端
   11. **Use CUI APs of RTKLIB**：用RTKLIB的命令行程序
4. **Build APs or Develop User APs with RTKLIB**：用RTKLIB构建程序
   1. **Rebuild GUI and CUI APs on Windows**：在Windows上重建GUI和CUI程序
   2. **Build CUI APs**：构建CUI程序
   3. **Develop and Link User APs with RTKLIB**：开发用户程序
5. 附录A：**CUI程序命令参考**：RTKRCV、RNX2RTKP、POS2KML、CONVBIN、STR2STR
6. 附录B：**文件格式**：Positioning Solution File、SBAS Log File、Solution Status File、Configuration File、URL List File for GNSS Data
7. 附录C：**API参考**，RTKLIB函数的一句话简介
8. 附录D：**文件和信息**：Supported RINEX Files、Supported Receiver Messages、Supported Signal IDs/Observation Types、Default Priorities of Multiple Signals、Receiver Dependent Input Options
9. 附录E：**模型和算法**：
   * Time System：时间系统
   * Coordinates System：坐标系统
   * GNSS Signal Measurement Models：信号测量模型
   * GNSS Satellite Ephemerides and Clocks：卫星星历表和时钟
   * Troposphere and Ionosphere Models：对流层、电离层模型
   * Single Point Positioning：SPP
   * Kinematic, Static and Moving‐Baseline：差分定位
   * PPP (Precise Point Positioning)：PPP
10. 附录F：**GNSS信号详细说明**
11. **参考文献**





### 6、后处理程序执行流程图





## 二、VS2022 下编译调试

> 参考”珞珈山圈哥“的博客:[rtklib学习笔记1：在Visual Studio 2019中调试rtklib 2.4.3程序](https://blog.csdn.net/nowitzki9/article/details/126799857?spm=1001.2014.3001.5506)，博客是用VS2019调试，我用VS2022，按他的步骤调试也成功了。

### 1、下载RTKLIB

​	在[RTKLIB官网](https://www.rtklib.com/)选最新版 **2.4.3 b34**，点**Source Programs and Data**下面的**GitHub**进入**GitHub页面**，点开绿色的**Code**下拉菜单，再点**Download ZIP**，下载解压即可。

### 2、在VS2022中创建空C++项目、导入源码文件

1. 创建**C++空项目**，可以勾选“解决方案和项目放在统一目录中”，记住创建的项目目录。

2. 把RTKLIB源码文件中**整个src文件夹**复制到创建的项目文件目录中。

3. 把RTKLIB源码文件中**\app\consapp**中的**rnx2rtkp.c**放到刚刚复制过去的**src文件夹**。

4. 在解决源文件中添加名为“src”的筛选器，再在src筛选器下面添加名为“rcv”的筛选器 。

   右键添加现有项目把**src/rcv文件夹**中的所有文件加到**src/rcv筛选器**中，src中所有代码文件加到src筛选器中。

5. 把主函数**rnx2rtkp.c**文件中的**#include "rtklib.h"**修改为**#include "./rtklib.h“** 

   把在**src/rcv文件夹几个的.c文件**中的**#include "rtklib.h"**修改为**#include "../rtklib.h”** 

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
   DLL
   WIN32
   ```

   > 尤其主要加WIN32，好多博客都没加这一项，加了这一项后RTKLIB就不会用Linux下的<pthread.h>和<sys/select.h>，咱们项目要在Windows下编译运行的，不加会报”找不到<pthread.h>和<sys/select.h>“的错。

4. 将常规中的目标文件名改为rnx2rtkp 。

   > 改不改都行，默认目标文件名是项目名。

### 4、改代码的BUG

​	可能会报“使用了可能未初始化的本地指针变量“sbs”的错误，解决方式是对指针变量进行初始化，将ephemeris.c文件中的第579行改为“const sbssatp_t *sbs=NULL;”。

## 三、VScode + WSL 下编译调试





### 1、项目文件

* 去 [GitHub](https://github.com/tomojitakasu/RTKLIB/tree/rtklib_2.4.3) 下载一份最新的 2.4.3 b34 源代码压缩包

  ![1689207739592](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689207739592.png)

* 创建文件夹 `my_rnx2rtkp` 做项目主文件夹。

* 在 `my_rnx2rtkp` 里新建文件夹 src 、include、out_exe

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

  



## 四、rnx2rtkp 命令行参数

### 1、rnx2rtkp 命令行程序使用方式

* 使用方式：`rnx2rtkp [option]... file file [...] `

* 读取RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS 等文件，计算接收机、流动站坐标，并输出结果。

* 对与相对定位，第一个OBS观测值文件需含接收机、流动站观测值，第二个OBS文件需含基准站观测值。

* 输入文件至少要有一个星历文件，RINEX NAV/GNAV/HNAV 。

* 想用SP3精密星历文件，需提供.sp3/.eph文件的路径。

* 输入文件路径可包含通配符*，为了防止与命令行命令冲突，要用\"...\" 括起带通配符符路径。

### 2、命令行参数

1. **-？、-**：打印help
2. **-k** file：配置文件的输入选项，默认值是[off]
3. **-o** file：输出文件选项，默认值是[stdout]
4. **-ts** ds ts：设置开始解算时间`(ds=y/m/d ts=h:m:s) `，默认值是[obs start time] 
5. **-te** de ds：设置结束解算时间`(de=y/m/d te=h:m:s) `，默认值是[obs end time] 
6. **-ti** tint：设置解算时间间隔频率`(sec) `，默认值是[all]
7. **-p** mode：设置解算模式，(**0**:single,**1**:dgps,**2**:kinematic,**3**:static,**4**:moving-base,**5**:fixed,**6**:ppp-kinematic,**7**:ppp-static)，默认值是[2]
8. **-m** mask：设置截止高度角，`(deg) `,默认值是[15]
9. **-sys** s：设置用于计算的导航系统，`(s=G:GPS,R:GLO,E:GAL,J:QZS,C:BDS,I:IRN) `，默认值是[G|R] 
10. **-f ** freq：设置用于计算的频率，` (1:L1,2:L1+L2,3:L1+L2+L5) `，默认值是[2]
11. **-v ** thres：设置整周模糊度Ratio值，写0.0为不固定整周模糊度，默认值是[3.0] 
12. **-b：**后向滤波
13. **-c**：前后向滤波组合
14. **-i**：单历元模糊度固定instantaneous 
15. **-h**：fix and hold模糊度固定
16. **-e**：输出XYZ-ecef坐标
17. **-a**：输出ENU-baseline
18. **-n**：输出NMEA-0183 GGA
19. **-g**：输出经纬度格式为ddd mm ss.ss ，默认为[ddd.ddd] 
20. **-t**：输出时间格式为yyyy/mm/dd hh:mm:ss.ss ，默认为[sssss.ss] 
21. **-u**：输出为UTC时间，默认为[gpst] 
22. **-d** col：设置时间的小数位数，默认为[3] 
23. **-s** sep：设置文件分隔符，要写在单引号中，默认为[' '] 
24. **-r** x y z：基站位置ECEF-XYZ (m)，默认[average of single pos] ，流动站位置用于fixed模式
25. **-l** lat lon hgt：基站位置LLH (deg/m)，默认[average of single pos]，流动站位置用于fixed模式
26. **-y** level：输出结果信息(**0**:off,**1**:states,**2**:residuals) ，默认为[0] 
27. **-x** level：输出debug trace等级，默认为[0] 

### 3、stdarg库

stdarg.h是C语言中C标准函数库的头文件，stdarg是由standard（标准）和 arguments（参数）简化而来，主要目的为让函数能够接收可变参数。 用法如下：

- 首先在函数里定义一具VA_LIST型的变量，这个变量是指向参数的指针 。
- 然后用VA_START宏初始化变量刚定义的VA_LIST变量 。
- 然后用VA_ARG返回可变的参数，VA_ARG的第二个参数是你要返回的参数的类型（如果函数有多个可变参数的，依次调用VA_ARG获取各个参数） 
- 最后用VA_END宏结束可变参数的获取 

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













## 六、矩阵计算

### 1、矩阵计算相关函数

> GNSS处理的数据都是矩阵数据，RTKLIB的**rtkcmn.c**中写了一些矩阵运算的函数。
>
> 也可看**manual**的**Appendix C API References**，里面有RTKLIB函数功能的简单描述

1. **mat()** ：New matrix、创建新矩阵，传入行数n，列数m，malloc开辟n*m个**double**空间，返回此空间的首地址。

   ```C++
   extern double *mat(int n, int m)
   {
       double *p;
       
       if (n<=0||m<=0) return NULL;
       if (!(p=(double *)malloc(sizeof(double)*n*m))) {
           fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
       return p;
   }
   ```

2. **imat()、**New integer matrix、与mat类似，区别在于iamt创建的矩阵是**int**类型的。**（RTKLIB里的矩阵一般都是double类型的二维数组）**

   ```C++
   extern int *imat(int n, int m)
   {
       int *p;
       
       if (n<=0||m<=0) return NULL;
       if (!(p=(int *)malloc(sizeof(int)*n*m))) {
           fatalerr("integer matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
       return p;
   }
   ```

3. **zeros():**New zero matrix、创建0矩阵、**double**类型。它比mat多了`\#if NOCALLOC if ((p=mat(n,m))) for (n=n*m-1;n>=0;n--) p[n]=0.0;`，如果预编译指令有NOCALLOC，会把所有位赋值0.0

   ```C++
   extern double *zeros(int n, int m)
   {
       double *p;
       
   #if NOCALLOC
       if ((p=mat(n,m))) for (n=n*m-1;n>=0;n--) p[n]=0.0;
   #else
       if (n<=0||m<=0) return NULL;
       if (!(p=(double *)calloc(sizeof(double),n*m))) {
           fatalerr("matrix memory allocation error: n=%d,m=%d\n",n,m);
       }
   #endif
       return p;
   }
   ```

4. **eye()**：New identity matrix、创建单位矩阵，**对角线元素全赋值1.0** 。

   ```C++
   extern double *eye(int n)
   {
       double *p;
       int i;
       
       if ((p=zeros(n,n))) for (i=0;i<n;i++) p[i+i*n]=1.0;
       return p;
   }
   ```

5. **dot()**：Inner Product、点乘、求两个向量的**内积**（对应位元素相乘再相加）

   ```C++
   extern double dot(const double *a, const double *b, int n)
   {
       double c=0.0;
       
       while (--n>=0) c+=a[n]*b[n];
       return c;
   }
   ```

6. **norm()**：Euclid norm、求向量的I2范数（向量自己的内积再开方 ）

   ```c++
   extern double norm(const double *a, int n)
   {
       return sqrt(dot(a,a,n));
   }
   ```

   . **cross3()**	、Outer product of 3D vectors、三维向量的外积

   ```c++
   extern void cross3(const double *a, const double *b, double *c)
   {
       c[0]=a[1]*b[2]-a[2]*b[1];
       c[1]=a[2]*b[0]-a[0]*b[2];
       c[2]=a[0]*b[1]-a[1]*b[0];
   }
   ```

   . **normv3()**	Normalize 3D vector、规格化三维向量（各元素除以向量的I2范数），a为传入向量，b为传出向量。

   ```c++
   extern int normv3(const double *a, double *b)
   {
       double r;
       if ((r=norm(a,3))<=0.0) return 0;
       b[0]=a[0]/r;
       b[1]=a[1]/r;
       b[2]=a[2]/r;
       return 1;
   }
   ```

7. **matcpy()：**Copy matrix、将B矩阵的所有元素赋值给A矩阵 

   ```C++
   extern void matcpy(double *A, const double *B, int n, int m)
   {
       memcpy(A,B,sizeof(double)*n*m);
   }
   ```

8. **matmul()：**Multiply matrix、矩阵A、B相乘返回C

   - tr：两个元素的字符数组，Tr、Tr[1]分别标识A、B的状态，取值N正常、T转置，可直接写TN、NN。
   - n、k、m：A为nm矩阵， B为mk矩阵、A的列数与B的行数相等，所有只用三个参数。作者之所以把n和k放在m前面，看起来顺序反逻辑，其实是矩阵相乘一共要循环n*k次，从编程的顺序来排的 。
   - 总体流程为： C=alphaAB+beta*C 

   ```c++
   extern void matmul(const char *tr, int n, int k, int m, double alpha,
                      const double *A, const double *B, double beta, double *C)
   {
       double d;
       int i,j,x,f=tr[0]=='N'?(tr[1]=='N'?1:2):(tr[1]=='N'?3:4);
       
       for (i=0;i<n;i++) for (j=0;j<k;j++) {
           d=0.0;
           switch (f) {
               case 1: for (x=0;x<m;x++) d+=A[i+x*n]*B[x+j*m]; break;
               case 2: for (x=0;x<m;x++) d+=A[i+x*n]*B[j+x*k]; break;
               case 3: for (x=0;x<m;x++) d+=A[x+i*m]*B[x+j*m]; break;
               case 4: for (x=0;x<m;x++) d+=A[x+i*m]*B[j+x*k]; break;
           }
           if (beta==0.0) C[i+j*n]=alpha*d; else C[i+j*n]=alpha*d+beta*C[i+j*n];
       }
   }
   ```

9. **matinv()：**Inverse of matrix、矩阵求逆

   ```C++
   extern int matinv(double *A, int n)
   {
       double d,*B;
       int i,j,*indx;
       
       indx=imat(n,1); B=mat(n,n); matcpy(B,A,n,n);
       if (ludcmp(B,n,indx,&d)) {free(indx); free(B); return -1;}
       for (j=0;j<n;j++) {
           for (i=0;i<n;i++) A[i+j*n]=0.0;
           A[j+j*n]=1.0;
           lubksb(B,n,indx,A+j*n);
       }
       free(indx); free(B);
       return 0;
   }
   ```

10. **solve()：**Solve linear equation、求方程线性解 、AX=Y，求X，A为nn，Y为nm

    ```c++
    extern int solve(const char *tr, const double *A, const double *Y, int n,
                     int m, double *X)
    {
        double *B=mat(n,n);
        int info;
        
        matcpy(B,A,n,n);
        if (!(info=matinv(B,n))) matmul(tr[0]=='N'?"NN":"TN",n,m,n,1.0,B,Y,0.0,X);
        free(B);
        return info;
    }
    ```

    

11. **ludcmp ()**：LU分解，即把矩阵A分解LU乘积的形式，U为单位上三角矩阵和L单位为下三角矩阵两部分 。

    ```c++
    static int ludcmp(double *A, int n, int *indx, double *d)
    {
        double big,s,tmp,*vv=mat(n,1);
        int i,imax=0,j,k;
        
        *d=1.0;
        for (i=0;i<n;i++) {
            big=0.0; for (j=0;j<n;j++) if ((tmp=fabs(A[i+j*n]))>big) big=tmp;
            if (big>0.0) vv[i]=1.0/big; else {free(vv); return -1;}
        }
        for (j=0;j<n;j++) {
            for (i=0;i<j;i++) {
                s=A[i+j*n]; for (k=0;k<i;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
            }
            big=0.0;
            for (i=j;i<n;i++) {
                s=A[i+j*n]; for (k=0;k<j;k++) s-=A[i+k*n]*A[k+j*n]; A[i+j*n]=s;
                if ((tmp=vv[i]*fabs(s))>=big) {big=tmp; imax=i;}
            }
            if (j!=imax) {
                for (k=0;k<n;k++) {
                    tmp=A[imax+k*n]; A[imax+k*n]=A[j+k*n]; A[j+k*n]=tmp;
                }
                *d=-(*d); vv[imax]=vv[j];
            }
            indx[j]=imax;
            if (A[j+j*n]==0.0) {free(vv); return -1;}
            if (j!=n-1) {
                tmp=1.0/A[j+j*n]; for (i=j+1;i<n;i++) A[i+j*n]*=tmp;
            }
        }
        free(vv);
        return 0;
    }
    ```

    

12. **lubksb ()**：LU回代,即把单位上三角矩阵U和单位下三角矩阵L矩阵回代为一个整体矩阵 

    ```c++
    static void lubksb(const double *A, int n, const int *indx, double *b)
    {
        double s;
        int i,ii=-1,ip,j;
        
        for (i=0;i<n;i++) {
            ip=indx[i]; s=b[ip]; b[ip]=b[i];
            if (ii>=0) for (j=ii;j<i;j++) s-=A[i+j*n]*b[j]; else if (s) ii=i;
            b[i]=s;
        }
        for (i=n-1;i>=0;i--) {
            s=b[i]; for (j=i+1;j<n;j++) s-=A[i+j*n]*b[j]; b[i]=s/A[i+i*n];
        }
    }
    ```

    

13. **matprint()：**Print matrix、打印矩阵到stdout

    ```c++
    extern void matprint(const double A[], int n, int m, int p, int q)
    {
        matfprint(A,n,m,p,q,stdout);
    }
    ```

    

14. **matfprint()：**Print matrix to file、打印矩阵到文件中

```c++
extern void matfprint(const double A[], int n, int m, int p, int q, FILE *fp)
{
    int i,j;
    
    for (i=0;i<n;i++) {
        for (j=0;j<m;j++) fprintf(fp," %*.*f",p,q,A[i+j*n]);
        fprintf(fp,"\n");
    }
}
```



### 2、最小二乘与Kalman滤波

1. **lsq()：**Least square estimation、最小二乘估计

   - **A**：nm阶设计矩阵的转置，m<n则无法计算。
   - **y**：m阶观测残差，**y=v=l-HX** 。
   - **X**：传出参数、待估计的n阶参数向量的增量。
   - **Q**：传出参数、nn协方差阵。

   ```c++
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

2. **filter_()、filter():**卡尔曼滤波器

   - X：n阶参数向量，就是参数，不再是最小二乘中的参数的增量 
   - P：nn阶协方差阵。X、P在filter()函数中既是传入参数，也是传出参数，迭代。
   - H：nm阶设计矩阵的转置
   - V：m阶新息向量
   - R：mm阶量测噪声协方差阵
   - Xp、Pp：预测参数和预测协方差

   ```c++
   extern  int filter_(const double *x, const double *P, const double *H,
                      const double *v, const double *R, int n, int m,
                      double *xp, double *Pp)
   {
       double *F=mat(n,m),*Q=mat(m,m),*K=mat(n,m),*I=eye(n);
       int info;
       
       matcpy(Q,R,m,m);
       matcpy(xp,x,n,1);
       matmul("NN",n,m,n,1.0,P,H,0.0,F);       /* Q=H'*P*H+R */
       matmul("TN",m,m,n,1.0,H,F,1.0,Q);
       if (!(info=matinv(Q,m))) {
           matmul("NN",n,m,m,1.0,F,Q,0.0,K);   /* K=P*H*Q^-1 */
           matmul("NN",n,1,m,1.0,K,v,1.0,xp);  /* xp=x+K*v */
           matmul("NT",n,n,m,-1.0,K,H,1.0,I);  /* Pp=(I-K*H')*P */
           matmul("NN",n,n,n,1.0,I,P,0.0,Pp);
       }
       free(F); free(Q); free(K); free(I);
       return info;
   }
   ```

   ```c++
   extern int filter(double *x, double *P, const double *H, const double *v,
                     const double *R, int n, int m)
   {
       double *x_,*xp_,*P_,*Pp_,*H_;
       int i,j,k,info,*ix;
       
       ix=imat(n,1); for (i=k=0;i<n;i++) if (x[i]!=0.0&&P[i+i*n]>0.0) ix[k++]=i;
       x_=mat(k,1); xp_=mat(k,1); P_=mat(k,k); Pp_=mat(k,k); H_=mat(k,m);
       for (i=0;i<k;i++) {
           x_[i]=x[ix[i]];
           for (j=0;j<k;j++) P_[i+j*k]=P[ix[i]+ix[j]*n];
           for (j=0;j<m;j++) H_[i+j*k]=H[ix[i]+j*n];
       }
       info=filter_(x_,P_,H_,v,R,k,m,xp_,Pp_);
       for (i=0;i<k;i++) {
           x[ix[i]]=xp_[i];
           for (j=0;j<k;j++) P[ix[i]+ix[j]*n]=Pp_[i+j*k];
       }
       free(ix); free(x_); free(xp_); free(P_); free(Pp_); free(H_);
       return info;
   }
   ```

3. **smoother()：**Kalman smoother 、结合前向滤波和后向滤波的卡尔曼滤波平滑

   - xf、Qf：前向滤波n阶结果和nn阶协方差
   - xb、Qb：后向滤波n阶结果和nn阶协方差
   - xs、Qs：平滑n阶结果和nn阶协方差

   ```c++
   extern int smoother(const double *xf, const double *Qf, const double *xb,
                       const double *Qb, int n, double *xs, double *Qs)
   {
       double *invQf=mat(n,n),*invQb=mat(n,n),*xx=mat(n,1);
       int i,info=-1;
       
       matcpy(invQf,Qf,n,n);
       matcpy(invQb,Qb,n,n);
       if (!matinv(invQf,n)&&!matinv(invQb,n)) {
           for (i=0;i<n*n;i++) Qs[i]=invQf[i]+invQb[i];
           if (!(info=matinv(Qs,n))) {
               matmul("NN",n,1,n,1.0,invQf,xf,0.0,xx);
               matmul("NN",n,1,n,1.0,invQb,xb,1.0,xx);
               matmul("NN",n,1,n,1.0,Qs,xx,0.0,xs);
           }
       }
       free(invQf); free(invQb); free(xx);
       return info;
   }
   ```

