[TOC]

## 一、配置 Qt Creater + Qt 5.15.2 + MinGW 开发环境

> * **Qt Creater**：
> * **Qt 5.15.2**：
> * **MinGW **：

### 1、配置 MinGW 





### 2、在镜像网站下载 Qt Creater





### 3、注册 Qt 账号





### 4、安装 Qt 5.15.2











## 二、下载 RTKLIB_Demo5_b34j

> * **RTKLIB**：
> * **Demo5**：
> * **b34j**：

#### 1. 下载 RTKLIB_Demo5_b34j 源码





#### 2. 下载编译好的二进制可执行文件放到 bin 目录





## 三、编译调试

### 1、在 Qt Creater 中打开 RTKLIB_Demo5_b34j 的 Qt 工程





### 2、加宏定义

在【项目】【构建设置】中，【添加构建配置】

在【构建步骤】里的qmake中的额外参数一栏填入："DEFINES += XXXX"，也就是对应的宏变量

* 启用 Trace：
* 启用多系统：
* Windows 下编译：



### 3、构建程序





### 4、编译运行程序





## 四、Qt 程序使用测试

### 1、使用 RTKGET 下载 GNSS 观测数据和改正产品





### 2、使用 RTKPOST 进行后处理 PPK、PPP 解算

以安理工陈超老师采集的 cpt 数据为例，比较

#### 1. 单 GPS 测试





#### 2. 单 BDS 测试





#### 3. 四系统测试





### 3、使用 RTKPLOT 进行原始数据和结果数据绘制







### 4、使用 STRSVR进行数据转发和采集





### 5、使用 RTKCONV 进行测试将 RTCM3、OEM7 转为 RINEX





### 6、使用 RTKNAVI 进行实时 RTK、PPP 解算





 



