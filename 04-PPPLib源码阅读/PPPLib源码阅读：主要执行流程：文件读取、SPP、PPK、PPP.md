PPPLib 基于 RTKLib 以 C++ 为主要开发语言编写，支持后处理 PPP、PPK、INS/GNSS 松组合和紧组合 ，作者是我的老师陈超。

## 一、主函数及文件读取

![1689749750172](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689749750172.png)

### main

main 函数很短，调用 ParsePara 处理传入的程序参数，尤其是读取的配置文件，然后调用 Processer 进行下一步处理。程序参数如下：

- `-C`：Configuration 文件路径，必须有。
- `-M`：定位模式，用下面的字符串：`SPP-KINE`、`PPP-KINE`、`PPP-STATIC`、`PPK-KINE`、`PPK-STATIC`、`IGLC-GSOF`、`IGLC-PPP`、`IGLC-PPK`、`IGTC`、`PPP`、`IGTC-PPK`
- `-S`：启用的系统：比如GPS、BDS：`GB`
- `-L`：调试等级，有以下三个：1 ：Debug、32：Warning、128：Info

### ParsePara

先是一个 for 循环，对传入参数进行处理，配置文件存入 `conf_file` 并判断是否存在、定位模式存入 mode、卫星系统存入 mask、调试等级存入 level。然后根据参数中的模式对选项结构体赋值，调用 InitLog 初始化配置文件，调用 LoadConf 加载配置文件到选项结构体。

### Processer

打开数据文件夹到 dir、根据定位模式实例化对应的求解器（cSppSolver、cPppSolver、cPpkSolver、cFusionSolver）



然后是一个 while 循环，读取文件夹的文件，经过一系列判断之后，调用 AutoMatchFile 





### AutoMatchFile



### InitReader 













## 二、SPP 流程

![1689750091423](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689750091423.png)















### 三、PPK 流程

![1689750166573](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689750166573.png)









## 四、PPP 流程

![image-20230809072817153](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230809072817153.png)















