> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、KF-GINS 简介

### 1、程序概述

KF-GINS 是武大 i2Nav 实验室开源的一套松组合导航程序；可以读取 IMU 数据文件、GNSS 结果文件，进行松组合解算。代码量小，有详细的文档、注释和讲解，代码结构很好理解，但里面具体的算法并不是完全按公式来，有一定的工程技巧。

### 2、相关资料

* **项目开源地址**：https://github.com/i2Nav-WHU
* **i2NAV组合导航讲义、数据集**：http://www.i2nav.cn/index/newList_zw?newskind_id=13a8654e060c40c69e5f3d4c13069078
* **介绍视频**：https://www.bilibili.com/video/BV1Zs4y1B7m2/

### 3、代码分析

![image-20230925154842096](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925154842096.png)

![image-20230925155625914](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925155625914.png)

用 cloc 对 src 目录进行统计，结果如下。可以看出代码量很小，只有1412行，注释很详细，足有804行。

![image-20230924121001944](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230924121001944.png)

### 4、第三方库

* **abseil-cpp**：Google的开源C++库，提供了一系列实用的工具和功能，例如字符串处理、时间处理、错误处理、日志记录等。
* **eigen**：用于线性代数、矩阵和向量操作、数值计算和转换。
* **yaml-cpp**：YAML解析器和生成器库。

无需自己配置，作者把它们放到 ThirdParty 文件夹，并在 CMakeLists 文件中引入了：

```cmake
# Eigen3
include_directories(ThirdParty/eigen-3.3.9)

# yaml-cpp-0.7.0
add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format
        absl::time)
```

## 二、编译、调试

基于 WSL + VScode，

 

## 三、类型定义

### 1、核心类：GIEngine

![image-20230925154656020](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925154656020.png)

![image-20230925183808857](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925183808857.png)

### 2、文件读写类型

![image-20230925190030672](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925190030672.png)

### 3、配置选项类：GINSOptions

![image-20230925183309602](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925183309602.png)

### 4、大地参数计算静态类：Earth

![image-20230925182052936](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925182052936.png)

### 5、角度弧度转换静态类：Angle

![image-20230925182624597](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925182624597.png)

## 四、程序执行流程

![image-20230925181044694](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181044694.png)

函数调用关系：

![image-20230925181228939](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181228939.png)

### 2、主函数

首先判断命令行参数，如果不为 2（可执行程序名算第一个参数 `argv[0]`）即没传入配置文件路径，输出提示并退出程序：

```cpp
if (argc != 2) {
    std::cout << "usage: KF-GINS kf-gins.yaml" << std::endl;
    return -1;
}
```

创建 t1、t2、t3 用于计时：

```cpp
long t1,t2,t3; // 用于计时
t1=clock();
std::cout << std::endl << "KF-GINS: An EKF-Based GNSS/INS Integrated Navigation System" << std::endl << std::endl;
auto ts = absl::Now();
```

读取配置文件：

```cpp
// 加载配置文件
// load configuration file
YAML::Node config;
try {
    config = YAML::LoadFile(argv[1]);
} catch (YAML::Exception &exception) {
    std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
              << std::endl;
    return -1;
}

// 读取配置参数到GINSOptioins中，并构造GIEngine
// load configuration parameters to GINSOptioins
GINSOptions options;
if (!loadConfig(config, options)) {
    std::cout << "Error occurs in the configuration file!" << std::endl;
    return -1;
}

// 读取文件路径配置
// load filepath configuration
std::string imupath, gnsspath, outputpath;
try {
    imupath    = config["imupath"].as<std::string>();
    gnsspath   = config["gnsspath"].as<std::string>();
    outputpath = config["outputpath"].as<std::string>();
} catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check the file path and output path!" << std::endl;
    return -1;
}

// imu数据配置，数据处理区间
// imudata configuration， data processing interval
int imudatalen, imudatarate;
double starttime, endtime;
try {
    imudatalen  = config["imudatalen"].as<int>();
    imudatarate = config["imudatarate"].as<int>();
    starttime   = config["starttime"].as<double>();
    endtime     = config["endtime"].as<double>();
} catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check the data length, data rate, and the process time!"
              << std::endl;
    return -1;
}
```

根据读进来的配置，构造解算用到的几个对象：

* 文件读取对象：`gnssfile`、`imufile`
* 松组合解算核心类：`giengine`
* 构造输出文件对象：`navfile`、`imuerrfile`、`stdfile`

```cpp
// 加载 GNSS 文件和 IMU 文件
// load GNSS file and IMU file
GnssFileLoader gnssfile(gnsspath);
ImuFileLoader imufile(imupath, imudatalen, imudatarate);

t2 =clock();

// 构造GIEngine
// Construct GIEngine
GIEngine giengine(options);

// 构造输出文件
// construct output file
// navfile: gnssweek(1) + time(1) + pos(3) + vel(3) + euler angle(3) = 11
// imuerrfile: time(1) + gyrbias(3) + accbias(3) + gyrscale(3) + accscale(3) = 13
// stdfile: time(1) + pva_std(9) + imubias_std(6) + imuscale_std(6) = 22
int nav_columns = 11, imuerr_columns = 13, std_columns = 22;
FileSaver navfile(outputpath + "/KF_GINS_Navresult.nav", nav_columns, FileSaver::TEXT);
FileSaver imuerrfile(outputpath + "/KF_GINS_IMU_ERR.txt", imuerr_columns, FileSaver::TEXT);
FileSaver stdfile(outputpath + "/KF_GINS_STD.txt", std_columns, FileSaver::TEXT);

// 检查文件是否正确打开
// check if these files are all opened
if (!gnssfile.isOpen() || !imufile.isOpen() || !navfile.isOpen() || !imuerrfile.isOpen() || !stdfile.isOpen()) {
    std::cout << "Failed to open data file!" << std::endl;
    return -1;
}
```

检查处理起止时间是否合理：

```cpp
if (endtime < 0) {
    endtime = imufile.endtime();
}
if (endtime > 604800 || starttime < imufile.starttime() || starttime > endtime) {
    std::cout << "Process time ERROR!" << std::endl;
    return -1;
}
```

循环调用 `imufile.next()`、`gnssfile.next()` 读取 IMU、GNSS 数据，直到时间戳在解算时间范围内。循环结束后 `imu_cur`、`gnss` 分别存解算时间内第一个IMU、GNSS量测，且文件指针指向的位置也到达解算时间内数据的开头：

```cpp
IMU imu_cur;
do {
    imu_cur = imufile.next();
} while (imu_cur.time < starttime);

GNSS gnss;
do {
    gnss = gnssfile.next();
} while (gnss.time <= starttime);
```

调用 `addImuData()`、`addGnssData()` 将刚刚读取到解算时间范围内第一个 IMU、GNSS 数据加入 `giengine`，并对 IMU 数据进行误差补偿，减去零偏、除以加上单位阵后的比例：

```cpp
// 添加IMU数据到GIEngine中，补偿IMU误差
// add imudata to GIEngine and compensate IMU error
giengine.addImuData(imu_cur, true);

// 添加GNSS数据到GIEngine
// add gnssdata to GIEngine
giengine.addGnssData(gnss);
```

定义变量，用于保存处理结果、显示处理进程：
```cpp
// 用于保存处理结果
// used to save processing results
double timestamp;
NavState navstate;
Eigen::MatrixXd cov;

// 用于显示处理进程
// used to display processing progress
int percent = 0, lastpercent = 0;
double interval = endtime - starttime;
```

接下来是一个大 while 死循环，每次循环都会读取一个 IMU 数据，只有当前 IMU 状态时间新于 GNSS 时间时，才会读取 GNSS 数据：

```cpp
// 当前IMU状态时间新于GNSS时间时，读取并添加新的GNSS数据到GIEngine
// load new gnssdata when current state time is newer than GNSS time and add it to GIEngine
if (gnss.time < imu_cur.time && !gnssfile.isEof()) {
    gnss = gnssfile.next();
    giengine.addGnssData(gnss);
}

// 读取并添加新的IMU数据到GIEngine
// load new imudata and add it to GIEngine
imu_cur = imufile.next();
if (imu_cur.time > endtime || imufile.isEof()) {
    break;
}
giengine.addImuData(imu_cur);
```

调用 `newImuProcess()` 根据当前 IMU、GNSS 数据进行解算，下面会重点介绍：

```cpp
giengine.newImuProcess();
```

解算之后，获取当前时间，IMU状态和协方差、保存并输出处理结果，输出结果的时间戳与 IMU 时间戳一致：

```cpp
// 获取当前时间，IMU状态和协方差
// get current timestamp, navigation state and covariance
timestamp = giengine.timestamp();
navstate  = giengine.getNavState();
cov       = giengine.getCovariance();

// 保存处理结果
// save processing results
writeNavResult(timestamp, navstate, navfile, imuerrfile);
writeSTD(timestamp, cov, stdfile);
```

显示处理进展：

```cpp
percent = int((imu_cur.time - starttime) / interval * 100);
   if (percent - lastpercent >= 1) {
   std::cout << " - Processing: " << std::setw(3) << percent << "%\r" << std::flush;
   lastpercent = percent;
}
```

循环处理完成之后，关闭打开的文件、输出结束信息、return 0：

```cpp
// 关闭打开的文件
// close opened file
imufile.close();
gnssfile.close();
navfile.close();
imuerrfile.close();
stdfile.close();

// 处理完毕
// process finish
auto te = absl::Now();
std::cout << std::endl << std::endl << "KF-GINS Process Finish! ";
std::cout << "From " << starttime << " s to " << endtime << " s, total " << interval << " s!" << std::endl;
std::cout << "Cost " << absl::ToDoubleSeconds(te - ts) << " s in total" << std::endl;

return 0;
```

### 3、配置文件读取

KF-GINS 使用 YMAL 格式的配置文件，通过配置文件可以设置数据文件路径、处理时间段、初始PVA、初始比例零偏、杆臂等。KF-GINS 的配置都是键值对形式的： `键 :值`，设置的时候改后面的值即可。程序执行的时候要把配置文件路径作为命令行参数。下面简单介绍读取流程：

在主函数中先调用 yaml-cpp 的接口 `YAML::LoadFile()` 通过 YMAL 配置文件路径，将配置导入为 YMAL 节点 `config`：

```cpp
YAML::Node config;
try {
    config = YAML::LoadFile(argv[1]);
} catch (YAML::Exception &exception) {
    std::cout << "Failed to read configuration file. Please check the path and format of the configuration file!"
              << std::endl;
    return -1;
}
```

然后调用 `loadConfig()` 从 YMAL 根节点 `config` 读取配置参数到 `GINSOptions` 类型对象 `options` 中：

```cpp
GINSOptions options;
if (!loadConfig(config, options)) {
    std::cout << "Error occurs in the configuration file!" << std::endl;
    return -1;
}
```

需要注意 `loadConfig()` 并没有把所有配置信息都读进来，它读取的只是取初始位置、IMU零偏、比例和对应的标准差；大部分参数都是三维的，读取的时候先存成 `vector<double>` 然后进行量纲、单位转换，再存到 `options` 对应的 `vector3d` 类型字段中。以初始PVA为例：

```cpp
    // 读取初始位置(纬度 经度 高程)、(北向速度 东向速度 垂向速度)、姿态(欧拉角，ZYX旋转顺序, 横滚角、俯仰角、航向角)
    // load initial position(latitude longitude altitude)
    //              velocity(speeds in the directions of north, east and down)
    //              attitude(euler angle, ZYX, roll, pitch and yaw)
    std::vector<double> vec1, vec2, vec3, vec4, vec5, vec6;
    try {
        vec1 = config["initpos"].as<std::vector<double>>();
        vec2 = config["initvel"].as<std::vector<double>>();
        vec3 = config["initatt"].as<std::vector<double>>();
    } catch (YAML::Exception &exception) {
        std::cout << "Failed when loading configuration. Please check initial position, velocity, and attitude!"
                  << std::endl;
        return false;
    }
    for (int i = 0; i < 3; i++) {   // 单位转换
        options.initstate.pos[i]   = vec1[i] * D2R;
        options.initstate.vel[i]   = vec2[i];
        options.initstate.euler[i] = vec3[i] * D2R;
    }
    options.initstate.pos[2] *= R2D;    // 高程不用转
```

文件路径和IMU处理配置是在主函数中读取：

```cpp
// 读取文件路径配置
// load filepath configuration
std::string imupath, gnsspath, outputpath;
try {
    imupath    = config["imupath"].as<std::string>();
    gnsspath   = config["gnsspath"].as<std::string>();
    outputpath = config["outputpath"].as<std::string>();
} catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check the file path and output path!" << std::endl;
    return -1;
}

// imu数据配置，数据处理区间
// imudata configuration， data processing interval
int imudatalen, imudatarate;
double starttime, endtime;
try {
    imudatalen  = config["imudatalen"].as<int>();
    imudatarate = config["imudatarate"].as<int>();
    starttime   = config["starttime"].as<double>();
    endtime     = config["endtime"].as<double>();
} catch (YAML::Exception &exception) {
    std::cout << "Failed when loading configuration. Please check the data length, data rate, and the process time!"
              << std::endl;
    return -1;
}
```

### 4、数据文件读取

KF-GINS 中没有一次性把整个文件都读进来；而是先打开文件，获取文件描述符；然后计算一点，读一点，来模仿实时解算。

在主函数中先构造 `GnssFileLoader`、`ImuFileLoader` 类的对象 `gnssfile`、`imufile`：

```cpp
GnssFileLoader gnssfile(gnsspath);
ImuFileLoader imufile(imupath, imudatalen, imudatarate);
```

构造函数中调用 `open()` 函数，将文件打开，获得文件指针，并记录下文件列数和 IMU 采样间隔。

```cpp
explicit GnssFileLoader(const string &filename, int columns = 7) {
    open(filename, columns, FileLoader::TEXT);
}
```

```cpp
ImuFileLoader(const string &filename, int columns, int rate = 200) {
    open(filename, columns, FileLoader::TEXT);

    dt_ = 1.0 / (double) rate;

    imu_.time = 0;
}
```

```cpp
bool FileLoader::open(const string &filename, int columns, int filetype) {
    auto type = filetype == TEXT ? std::ios_base::in : (std::ios_base::in | std::ios_base::binary);
    filefp_.open(filename, type);

    columns_  = columns;
    filetype_ = filetype;
    return isOpen();
}
```







### 5、newImuProcess()：松组合

![image-20230925154404051](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925154404051.png)

首先将当前 IMU 时间作为系统当前状态时间，也就是说这个函数执行完之后，得到的系统状态向量和协方差阵是当前 IMU 时间的：

```cpp
timestamp_ = imucur_.time;
```

如果GNSS有效，则将量测更新时间设置为 GNSS 时间：

```cpp
double updatetime = gnssdata_.isvalid ? gnssdata_.time : -1;
```

先调用 `isToUpdate()`，根据当前 GNSS 与当前和先前两 IMU 量测的时间关系，判断是否需要进行 GNSS 更新，有四种情况，分别返回不同的值：

![image-20230925120000711](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925120000711.png)

```cpp
int GIEngine::isToUpdate(double imutime1, double imutime2, double updatetime) const {
    if (abs(imutime1 - updatetime) < TIME_ALIGN_ERR) {
        // 更新时间靠近imutime1
        // updatetime is near to imutime1
        return 1;
    } else if (abs(imutime2 - updatetime) <= TIME_ALIGN_ERR) {
        // 更新时间靠近imutime2
        // updatetime is near to imutime2
        return 2;
    } else if (imutime1 < updatetime && updatetime < imutime2) {
        // 更新时间在imutime1和imutime2之间, 但不靠近任何一个
        // updatetime is between imutime1 and imutime2, but not near to either
        return 3;
    } else {
        // 更新时间不在imutimt1和imutime2之间，且不靠近任何一个
        // updatetime is not bewteen imutime1 and imutime2, and not near to either.
        return 0;
    }
}
```

> 根据更新时间对齐误差 `TIME_ALIGN_ERR` 评定是否靠近，默认为 0.001，也就是说时间差距在 1ms 内，认为是靠近的。文档和注释都用“靠近”这个词，起始与其说是“靠近”，更像是在说是“重合”。

返回 0，表示 GNSS 不在两个 IMU 之间，在当前 IMU 量测之后，那么只进行捷联惯导递推，调用 `insPropagation()` 根据两帧 IMU 量测将状态递推到当前 IMU 时间戳：

```cpp
if (res == 0) {
    // 只传播导航状态
    // only propagate navigation state
    insPropagation(imupre_, imucur_);
```

返回 1，表示 GNSS 在两个 IMU 之间，更靠近前一个 IMU，那么先调用 `gnssUpdate()` 进行 GNSS 量测更新，再调用 `stateFeedback()` 进行系统状态反馈，最后调用 `insPropagation()` 根据两帧 IMU 量测将状态递推到当前 IMU 时间戳：

```cpp
} else if (res == 1) {
    // GNSS数据靠近上一历元，先对上一历元进行GNSS更新
    // gnssdata is near to the previous imudata, we should firstly do gnss update
    gnssUpdate(gnssdata_);
    stateFeedback();

    pvapre_ = pvacur_;
    insPropagation(imupre_, imucur_);
```

返回 2，表示 GNSS 在两个 IMU 之间，更靠近后一个 IMU，那么先调用 `insPropagation()` 根据两帧 IMU 量测将状态递推到当前 IMU 时间戳，再调用 `gnssUpdate()` 进行 GNSS 量测更新，最后调用 `stateFeedback()` 进行系统状态反馈：

```cpp
} else if (res == 2) {
    // GNSS数据靠近当前历元，先对当前IMU进行状态传播
    // gnssdata is near current imudata, we should firstly propagate navigation state
    insPropagation(imupre_, imucur_);
    gnssUpdate(gnssdata_);
    stateFeedback();

```

返回 3，表示 GNSS 在两个 IMU 之间，不靠近任何一个，那么先调用 imuInterpolate() 根据两帧 IMU 量测插值到 GNSS 时间戳，得到 GNSS 时刻 IMU 量测值 `midimu`；调用 insPropagation 根据前一个 IMU 和 `midimu` 将状态递推到当前 GNSS 时间戳；再调用 `gnssUpdate()` 进行 GNSS 量测更新，调用 `stateFeedback()` 进行系统状态反馈；最后再调用一次 `insPropagation()`  根据 midimu 和当前时刻 IMU 量测将状态递推到当前 IMU 时间戳：

```cpp
} else {
    // GNSS数据在两个IMU数据之间(不靠近任何一个), 将当前IMU内插到整秒时刻
    // gnssdata is between the two imudata, we interpolate current imudata to gnss time
    IMU midimu;
    imuInterpolate(imupre_, imucur_, updatetime, midimu);
    // 对前一半IMU进行状态传播
    // propagate navigation state for the first half imudata
    insPropagation(imupre_, midimu);
    // 整秒时刻进行GNSS更新，并反馈系统状态
    // do GNSS position update at the whole second and feedback system states
    gnssUpdate(gnssdata_);
    stateFeedback();
    // 对后一半IMU进行状态传播
    // propagate navigation state for the second half imudata
    pvapre_ = pvacur_;
    insPropagation(midimu, imucur_);
}
```

几种情况可总结如下图：

![image-20230925120109838](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925120109838.png)

处理完之后调用 `checkCov() `检查协方差对角线元素是否都为正，更新上一时刻的状态和 IMU 数据：
```cpp
// 检查协方差对角线元素是否都为正
// check diagonal elements of current covariance matrix
checkCov();
// 更新上一时刻的状态和IMU数据
// update system state and imudata at the previous epoch
pvapre_ = pvacur_;
imupre_ = imucur_;
```

## 六、Earth 类：地球参数和坐标转换

`Earth` 类里都是静态函数，使用的时候直接`类名::成员函数()`，文件的开头定义了一些椭球参数：

```cpp
/* WGS84椭球模型参数
   NOTE:如果使用其他椭球模型需要修改椭球参数 */
const double WGS84_WIE = 7.2921151467E-5;       /* 地球自转角速度*/
const double WGS84_F   = 0.0033528106647474805; /* 扁率 */
const double WGS84_RA  = 6378137.0000000000;    /* 长半轴a */
const double WGS84_RB  = 6356752.3142451793;    /* 短半轴b */
const double WGS84_GM0 = 398600441800000.00;    /* 地球引力常数 */
const double WGS84_E1  = 0.0066943799901413156; /* 第一偏心率平方 */
const double WGS84_E2  = 0.0067394967422764341; /* 第二偏心率平方 */
```

### 1、gravity()：正常重力计算

重力是万有引力与离心力共同作用的结果，随纬度升高离心力增大但引力减小、随高程升高引力减小，共同作用下重力的计算公式如下：

$$
g_{L}=9.7803267715 \times\left(1+0.0052790414 \times \sin ^{2} L-0.0000232718 \times \sin ^{2} 2 L\right) \\
+h\times(0.0000000043977311\times\sin ^{2} L-0.0000030876910891)+0.0000000000007211\times\sin ^{4} 2 L
$$

```cpp
static double gravity(const Vector3d &blh) {
    double sin2 = sin(blh[0]);
    sin2 *= sin2;
    return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) +
         blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2];
}
```

### 2、meridianPrimeVerticalRadius()：计算子午圈半径 RM、卯酉圈半径 RN

返回值是 `Vector2d`，第一个是子午圈主曲率半径 RM、第二个是卯酉圈主半径 RN：
$$
R_{M}=\frac{R_{e}\left(1-e^{2}\right)}{\left(1-e^{2} \sin ^{2} L\right)^{3 / 2}}、R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```cpp
static Eigen::Vector2d meridianPrimeVerticalRadius(double lat) {
    double tmp, sqrttmp;

    tmp = sin(lat);	
    tmp *= tmp;
    tmp     = 1 - WGS84_E1 * tmp;
    sqrttmp = sqrt(tmp);

    return {WGS84_RA * (1 - WGS84_E1) / (sqrttmp * tmp), WGS84_RA / sqrttmp};
}
```

### 3、RN()：计算卯酉圈主半径 RN

$$
R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```cpp
static double RN(double lat) {
   double sinlat = sin(lat);
   return WGS84_RA / sqrt(1.0 - WGS84_E1 * sinlat * sinlat);
}
```

### 4、cne()：n系(导航坐标系)到e系(地心地固坐标系)转换矩阵

$$
C_{e}^{n}=\left[\begin{array}{ccc}-\sin \varphi & 0 & \cos \varphi \\ 0 & 1 & 0 \\ -\cos \varphi & 0 & -\sin \varphi\end{array}\right]\left[\begin{array}{ccc}\cos \lambda & \sin \lambda & 0 \\ -\sin \lambda & \cos \lambda & 0 \\ 0 & 0 & 1\end{array}\right]=\\ \left[\begin{array}{ccc}-\sin \varphi \cos \lambda & -\sin \varphi \sin \lambda & \cos \varphi \\ -\sin \lambda & \cos \lambda & 0 \\ -\cos \varphi \cos \lambda & -\cos \varphi \sin \lambda & -\sin \varphi\end{array}\right]
$$

```cpp
static Matrix3d cne(const Vector3d &blh) {
    double coslon, sinlon, coslat, sinlat;

    sinlat = sin(blh[0]);
    sinlon = sin(blh[1]);
    coslat = cos(blh[0]);
    coslon = cos(blh[1]);

    Matrix3d dcm;
    dcm(0, 0) = -sinlat * coslon;
    dcm(0, 1) = -sinlon;
    dcm(0, 2) = -coslat * coslon;

    dcm(1, 0) = -sinlat * sinlon;
    dcm(1, 1) = coslon;
    dcm(1, 2) = -coslat * sinlon;

    dcm(2, 0) = coslat;
    dcm(2, 1) = 0;
    dcm(2, 2) = -sinlat;

    return dcm;
}
```

### 5、qne()：n系(北东地)到e系(ECEF)转换四元数

$$
\boldsymbol{q}_{n}^{e}=\left[\begin{array}{c}\cos (-\pi / 4-\varphi / 2) \cos (\lambda / 2) \\ -\sin (-\pi / 4-\varphi / 2) \sin (\lambda / 2) \\ \sin (-\pi / 4-\varphi / 2) \cos (\lambda / 2) \\ \cos (-\pi / 4-\sin / 2) \sin (\lambda / 2)]\end{array}\right]
$$

```cpp
/* n系(导航坐标系)到e系(地心地固坐标系)转换四元数 */
static Quaterniond qne(const Vector3d &blh) {
    Quaterniond quat;

    double coslon, sinlon, coslat, sinlat;

    coslon = cos(blh[1] * 0.5);
    sinlon = sin(blh[1] * 0.5);
    coslat = cos(-M_PI * 0.25 - blh[0] * 0.5);
    sinlat = sin(-M_PI * 0.25 - blh[0] * 0.5);

    quat.w() = coslat * coslon;
    quat.x() = -sinlat * sinlon;
    quat.y() = sinlat * coslon;
    quat.z() = coslat * sinlon;

    return quat;
}
```

### 6、blh()：从n系到e系转换四元数得到纬度和经度



```cpp
/* 从n系到e系转换四元数得到纬度和经度 */
static Vector3d blh(const Quaterniond &qne, double height) {
    return {-2 * atan(qne.y() / qne.w()) - M_PI * 0.5, 2 * atan2(qne.z(), qne.w()), height};
}
```

### 7、blh2ecef()：大地坐标(纬度、经度和高程)转地心地固坐标

$$
\begin{array}{l}x=\left(R_{N}+h\right) \cos L \cos \lambda \\ y=\left(R_{N}+h\right) \cos L \sin \lambda \\ z=\left[R_{N}\left(1-e^{2}\right)+h\right] \sin L\end{array}
$$

```cpp
/* 大地坐标(纬度、经度和高程)转地心地固坐标 */
static Vector3d blh2ecef(const Vector3d &blh) {
    double coslat, sinlat, coslon, sinlon;
    double rnh, rn;

    coslat = cos(blh[0]);
    sinlat = sin(blh[0]);
    coslon = cos(blh[1]);
    sinlon = sin(blh[1]);

    rn  = RN(blh[0]);
    rnh = rn + blh[2];

    return {rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * WGS84_E1) * sinlat};
}
```

### 7、ecef2blh()：地心地固坐标转大地坐标

$$
\begin{array}{c}B_{0}=\arctan \left(\frac{Z}{\left(1-e^{2}\right) p}\right) \\ N_{k}=\frac{a}{\sqrt{1-e^{2} \sin ^{2} B_{k-1}}} \\ H_{k}=\frac{p}{\cos B_{k-1}}-N_{k} \\ B_{k}=\arctan \left(\frac{z}{\left(1-\frac{e^{2} N_{k}}{N_{k}}\right)\ p }\right)\end{array}
$$

```cpp
static Vector3d ecef2blh(const Vector3d &ecef) {
    double p = sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1]);
    double rn;
    double lat, lon;
    double h = 0, h2;

    // 初始状态
    lat = atan(ecef[2] / (p * (1.0 - WGS84_E1)));
    lon = 2.0 * atan2(ecef[1], ecef[0] + p);

    do {
        h2  = h;
        rn  = RN(lat);
        h   = p / cos(lat) - rn;
        lat = atan(ecef[2] / (p * (1.0 - WGS84_E1 * rn / (rn + h))));
    } while (fabs(h - h2) > 1.0e-4);

    return {lat, lon, h};
}
```

### 8、DRi()：n系相对位置转大地坐标相对位置

$$
\left[\begin{array}{l}\delta \varphi \\ \delta L \\ \delta H\end{array}\right]=\left[\begin{array}{ccc}\left(R_{M}+H\right)^{-1} & 0 & 0 \\ 0 & \left(R_{N}+H\right)^{-1} & 0 \\ 0 & 0 & -1 \end{array}\right]\left[\begin{array}{l}\delta \boldsymbol{p}_{N} \\ \delta \boldsymbol{p}_{E} \\ \delta \boldsymbol{p}_{B}\end{array}\right]
$$

```cpp
/* n系相对位置转大地坐标相对位置 */
static Matrix3d DRi(const Vector3d &blh) {
    Matrix3d dri = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dri(0, 0) = 1.0 / (rmn[0] + blh[2]);
    dri(1, 1) = 1.0 / ((rmn[1] + blh[2]) * cos(blh[0]));
    dri(2, 2) = -1;
    return dri;
}
```

### 9、DR()：大地坐标相对位置转n系相对位置

$$
\left[\begin{array}{l}\delta \varphi \\ \delta L \\ \delta H\end{array}\right]=\left[\begin{array}{ccc}\left(R_{M}+H\right)^{-1} & 0 & 0 \\ 0 & \left(R_{N}+H\right)^{-1} & 0 \\ 0 & 0 & -1 \end{array}\right]\left[\begin{array}{l}\delta \boldsymbol{p}_{N} \\ \delta \boldsymbol{p}_{E} \\ \delta \boldsymbol{p}_{B}\end{array}\right]
$$

```cpp
/* 大地坐标相对位置转n系相对位置 */
static Matrix3d DR(const Vector3d &blh) {
    Matrix3d dr = Matrix3d::Zero();

    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(blh[0]);

    dr(0, 0) = rmn[0] + blh[2];
    dr(1, 1) = (rmn[1] + blh[2]) * cos(blh[0]);
    dr(2, 2) = -1;
    return dr;
}
```

### 10、local2global()：局部坐标(在origin处展开)转大地坐标





### 11、global2local()：大地坐标转局部坐标(在origin处展开)





### 12、iewe()：地球自转角速度投影到e系

$$
\boldsymbol{\omega}_{i e}^{e}=\left[\begin{array}{lll}0 & 0 & \omega_{e}\end{array}\right]^{T}
$$

```cpp
static Vector3d iewe() {
    return {0, 0, WGS84_WIE};
}
```

### 13、iewn()：地球自转角速度投影到n系

$$
\boldsymbol{\omega}_{i e}^{n}=\left[\begin{array}{lll}\omega_{e} \cos \varphi & 0 & -\omega_{e} \sin \varphi\end{array}\right]^{T}
$$

```cpp
static Vector3d iewn(double lat) {
    return {WGS84_WIE * cos(lat), 0, -WGS84_WIE * sin(lat)};
}
```

也可以直接传入北东地（n 系）坐标计算：

```cpp
static Vector3d iewn(const Vector3d &origin, const Vector3d &local) {
    Vector3d global = local2global(origin, local);
    return iewn(global[0]);
}
```

### 14、enwn()：n系相对于e系转动角速度投影到n系

由载体运动线速度和地球曲率引起，与东向、北向速度有关，与天向速度无关
$$
\boldsymbol{\omega}_{e n}^{n}=\left[\begin{array}{lll}\frac{v_{E}}{R_{N}+h} & \frac{-v_{N}}{R_{M}+h} & -\frac{v_{E} \tan \varphi}{R_{N}+h}\end{array}\right]^{T}
$$

```cpp
static Vector3d enwn(const Eigen::Vector2d &rmn, const Vector3d &blh, const Vector3d &vel) {
    return {vel[1] / (rmn[1] + blh[2]), -vel[0] / (rmn[0] + blh[2]), -vel[1] * tan(blh[0]) / (rmn[1] + blh[2])};
}
```

同样也可以直接传入北东地（n 系）坐标计算：

```cpp
static Vector3d enwn(const Vector3d &origin, const Vector3d &local, const Vector3d &vel) {
    Vector3d global     = local2global(origin, local);
    Eigen::Vector2d rmn = meridianPrimeVerticalRadius(global[0]);

    return enwn(rmn, global, vel);
}
```

## 七、捷联惯导更新：insPropagation()

![image-20230922181230280](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230922181230280.png)

### 1、insPropagation()：捷联惯导递推

根据两帧的 IMU 量测，将系统状态和误差状态从前一个 IMU 时间递推到后一个 IMU 时间；主要有三个步骤：IMU 误差补偿、状态更新（机械编排）、噪声传播。

先调用 `imuCompensate()`，补偿当前时刻 IMU 量测，就是减去零偏、除以加上单位阵后的比例：

```cpp
imuCompensate(imucur);
```

调用 `insMech()` 依次进行速度更新、位置更新、姿态更新：

```cpp
INSMech::insMech(pvapre_, pvacur_, imupre, imucur);
```

之后一大段是噪声传播，后面详细介绍。

### 2、imuCompensate()：IMU数据误差补偿

减去零偏、除以加上单位阵后的比例：
$$
\begin{array}{l}\operatorname{diag}\left(\boldsymbol{I}+\overline{\boldsymbol{s}}_{g}\right)^{-1}\left(\tilde{\boldsymbol{\omega}}_{i b}^{b}-\overline{\boldsymbol{b}}_{g}\right)=\hat{\boldsymbol{\omega}}_{i b}^{b} \\ \operatorname{diag}\left(\boldsymbol{I}+\overline{\boldsymbol{s}}_{a}\right)^{-1}\left(\tilde{\boldsymbol{f}}_{i b}^{b}-\overline{\boldsymbol{b}}_{a}\right)=\hat{\boldsymbol{f}}_{i b}^{b}\end{array}
$$

```cpp
void GIEngine::imuCompensate(IMU &imu) {
    // 补偿IMU零偏
    // compensate the imu bias
    imu.dtheta -= imuerror_.gyrbias * imu.dt;
    imu.dvel -= imuerror_.accbias * imu.dt;
    // 补偿IMU比例因子
    // compensate the imu scale
    Eigen::Vector3d gyrscale, accscale;
    gyrscale   = Eigen::Vector3d::Ones() + imuerror_.gyrscale;
    accscale   = Eigen::Vector3d::Ones() + imuerror_.accscale;
    imu.dtheta = imu.dtheta.cwiseProduct(gyrscale.cwiseInverse());
    imu.dvel   = imu.dvel.cwiseProduct(accscale.cwiseInverse());
}
```

### 3、insMech()：IMU 状态更新（机械编排）

依次进行速度更新、位置更新、姿态更新，不可调换顺序。

![image-20230925155325909](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925155325909.png)

```cpp
void INSMech::insMech(const PVA &pvapre, PVA &pvacur, const IMU &imupre, const IMU &imucur) {
    // perform velocity update, position updata and attitude update in sequence, irreversible order
    // 依次进行速度更新、位置更新、姿态更新, 不可调换顺序
    velUpdate(pvapre, pvacur, imupre, imucur);
    posUpdate(pvapre, pvacur, imupre, imucur);
    attUpdate(pvapre, pvacur, imupre, imucur);
}
```

值得一提的是：

* PVA 更新都是先计算中间时刻，再由此计算当前时刻。
* 位置更新中：先计算n系到e系旋转四元数，再调用 `blh()`计算经纬度。
* 我觉得因为PVA写成三个函数，部分代码有重复，PVA更新在同一个函数实现能更简洁一些。

### 4、velUpdate()：速度更新

![image-20230925191706967](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925191706967.png)

先定义解算过程中涉及的中间变量：

```cpp
Eigen::Vector3d d_vfb, d_vfn, d_vgn, gl, midvel, midpos;
Eigen::Vector3d temp1, temp2, temp3;
Eigen::Matrix3d cnn, I33 = Eigen::Matrix3d::Identity();
Eigen::Quaterniond qne, qee, qnn, qbb, q1, q2;
```

调用 `meridianPrimeVerticalRadius()`，根据上一时刻位置计算**子午圈、卯酉圈半径**：
$$
R_{M}=\frac{R_{e}\left(1-e^{2}\right)}{\left(1-e^{2} \sin ^{2} L\right)^{3 / 2}}、R_{N}=\frac{R_{e}}{\sqrt{1-e^{2} \sin ^{2} L}}
$$

```cpp
Eigen::Vector2d rmrn = Earth::meridianPrimeVerticalRadius(pvapre.pos(0));
```

计算地球自转引起的导航系旋转 wie_n：
$$
\boldsymbol{\omega}_{i e}^{n}=\left[\begin{array}{lll}\omega_{e} \cos \varphi & 0 & -\omega_{e} \sin \varphi\end{array}\right]^{\mathrm{T}}
$$

```cpp
wie_n << WGS84_WIE * cos(pvapre.pos[0]), 0, -WGS84_WIE * sin(pvapre.pos[0]);
```

计算载体在地球表面移动因地球曲率引起的导航系旋转 wen_n：
$$
\boldsymbol{\omega}_{e n}^{n}=\left[\begin{array}{c}v_{E} /\left(R_{N}+h\right) \\ -v_{N} /\left(R_{M}+h\right) \\ -v_{E} \tan \varphi /\left(R_{N}+h\right)\end{array}\right]
$$

```cpp
wen_n << pvapre.vel[1] / (rmrn[1] + pvapre.pos[2]), -pvapre.vel[0] / (rmrn[0] + pvapre.pos[2]),
    -pvapre.vel[1] * tan(pvapre.pos[0]) / (rmrn[1] + pvapre.pos[2]);
```

调用 `gravity()`，根据上一时刻位置计算**重力 ** gravity：
$$
g_{L}=9.7803267715 \times\left(1+0.0052790414 \times \sin ^{2} L-0.0000232718 \times \sin ^{2} 2 L\right) \\
+h\times(0.0000000043977311\times\sin ^{2} L-0.0000030876910891)+0.0000000000007211\times\sin ^{4} 2 L
$$

```cpp
double gravity = Earth::gravity(pvapre.pos);
```

计算 b 系比力积分项 d_vfb，单子样+前一周期补偿划桨效应：
$$
\underbrace{\Delta \boldsymbol{v}_{f, k-1)}^{b(k-1)}}_{b \text { 系比力积分项 }}=\Delta \boldsymbol{v}_{k}+\frac{1}{2} \Delta \boldsymbol{\theta}_{k} \times \Delta \boldsymbol{v}_{k}+\frac{1}{12}\left(\Delta \boldsymbol{\theta}_{k-1} \times \Delta \boldsymbol{v}_{k}+\Delta \boldsymbol{v}_{k-1} \times \Delta \boldsymbol{\theta}_{k}\right)
$$

```cpp
// 旋转效应和双子样划桨效应
// rotational and sculling motion
temp1 = imucur.dtheta.cross(imucur.dvel) / 2;
temp2 = imupre.dtheta.cross(imucur.dvel) / 12;
temp3 = imupre.dvel.cross(imucur.dtheta) / 12;
// b系比力积分项
// velocity increment due to the specific force
d_vfb = imucur.dvel + temp1 + temp2 + temp3;
```

比力积分项投影到 n 系，三行代码分别对应的公式为：
$$
{\omega}_{i n}^{n}={\omega}_{i e}^{n}+{\omega}_{e n}^{n}
$$

$$
\left.\boldsymbol{\zeta}_{n(k-1) n(k)} \approx \boldsymbol{\omega}_{i n}^{n}\right|_{t=t_{k-1 / 2}} \Delta t_{k}
$$

$$
\underbrace{\left[\boldsymbol{I}-\frac{1}{2}\left(\boldsymbol{\zeta}_{n(k-1) n(k)} \times\right)\right] \boldsymbol{C}_{b(k-1)}^{n(k-1)} \Delta \boldsymbol{v}_{f, k}^{b(k-1)}}_{n \text { 系比力积分项 }}
$$

```cpp
// 比力积分项投影到n系
// velocity increment dut to the specfic force projected to the n-frame
temp1 = (wie_n + wen_n) * imucur.dt / 2;
cnn   = I33 - Rotation::skewSymmetric(temp1);
d_vfn = cnn * pvapre.att.cbn * d_vfb;
```

计算重力/哥式积分项：
$$
\underbrace{\boldsymbol{g}_{l}^{n} \Delta t_{k}-\left(2 \boldsymbol{\omega}_{i}^{n}+\boldsymbol{\omega}_{e n}^{n}\right) \times\left.\boldsymbol{v}_{e b}^{n}\right|_{t=t_{k-1 / 2}} \Delta t_{k}}_{\text {重力/哥氏积分项 }}
$$

```cpp
// 计算重力/哥式积分项
// velocity increment due to the gravity and Coriolis force
gl << 0, 0, gravity;
d_vgn = (gl - (2 * wie_n + wen_n).cross(pvapre.vel)) * imucur.dt;
```

上一时刻速度加上一半比力积分项和比力积分项，得到中间时刻速度：

```cpp
// 得到中间时刻速度
// velocity at k-1/2
midvel = pvapre.vel + (d_vfn + d_vgn) / 2;
```

外推得到中间时刻位置：

* 计算两时刻n系旋转四元数 qnn
* 根据地球自转角速率，计算两时刻e系旋转四元数 qee
* 调用 qne 根据先前时刻位置，计算先前时刻n系到e系旋转四元数 qne
* 当前时刻n系到e系旋转四元数 qne = 两时刻e系旋转四元数 * 先前n系到e系旋转四元数 * 两时刻n系旋转四元数
* 中间时刻高程 = 先前高程 - 高程方向速度 * 一半采样周期（因为北东地，计算出的速度时地向的，所以减）
* 调用 `blh()` 根据 n系到e系旋转四元数计算经纬度

```cpp
// 外推得到中间时刻位置
// position extrapolation to k-1/2
qnn = Rotation::rotvec2quaternion(temp1);
temp2 << 0, 0, -WGS84_WIE * imucur.dt / 2;
qee = Rotation::rotvec2quaternion(temp2);
qne = Earth::qne(pvapre.pos);
qne = qee * qne * qnn;
midpos[2] = pvapre.pos[2] - midvel[2] * imucur.dt / 2;
midpos    = Earth::blh(qne, midpos[2]);
```

基于用中间时刻的位置，重新做一遍之前的操作：

* 重新计算中间时刻的地理参数：rmrn、wie_n、wen_n（重力没重算）
* 重新计算 n 系下平均比力积分项：d_vfn
* 重新计算重力、哥式积分项：d_vgn

```cpp
// 重新计算中间时刻的 rmrn, wie_e, wen_n
// recompute rmrn, wie_n, and wen_n at k-1/2
rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
wen_n << midvel[1] / (rmrn[1] + midpos[2]), -midvel[0] / (rmrn[0] + midpos[2]),
    -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);

// 重新计算n系下平均比力积分项
// recompute d_vfn
temp3 = (wie_n + wen_n) * imucur.dt / 2;
cnn   = I33 - Rotation::skewSymmetric(temp3);
d_vfn = cnn * pvapre.att.cbn * d_vfb;

// 重新计算重力、哥式积分项
// recompute d_vgn
gl << 0, 0, Earth::gravity(midpos);
d_vgn = (gl - (2 * wie_n + wen_n).cross(midvel)) * imucur.dt;
```

最后，用上一时刻的速度，加上n系下平均比力积分项、重力/哥式积分项，得到当前时刻速度：
$$
\begin{aligned} \boldsymbol{v}_{e b, k}^{n}=\boldsymbol{v}_{e b, k-1}^{n} & +\underbrace{\left[\boldsymbol{I}-\frac{1}{2}\left(\boldsymbol{\zeta}_{n(k-1) n(k)} \times\right)\right] \boldsymbol{C}_{b(k-1)}^{n(k-1)} \Delta \boldsymbol{v}_{f, k}^{b(k-1)}}_{n \text { 系比力积分项 }} \\ & +\underbrace{\boldsymbol{g}_{l}^{n} \Delta t_{k}-\left(2 \boldsymbol{\omega}_{i}^{n}+\boldsymbol{\omega}_{e n}^{n}\right) \times\left.\boldsymbol{v}_{e b}^{n}\right|_{t=t_{k-1 / 2}} \Delta t_{k}}_{\text {重力/哥氏积分项 }}\end{aligned}
$$

```cpp
pvacur.vel = pvapre.vel + d_vfn + d_vgn;
```

### 5、posUpdate()：位置更新

![image-20230925191759173](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925191759173.png)

先定义解算过程中涉及的中间变量：

```cpp
Eigen::Vector3d temp1, temp2, midvel, midpos;
Eigen::Quaterniond qne, qee, qnn;
```

重新计算中间时刻的速度 `midvel` 和位置 `midpos`：

* 中间时刻速度：取两时刻的平均。
* 中间时刻位置：上一时刻位置 + 平均速度 * 一半采样间隔。

```cpp
// 重新计算中间时刻的速度和位置
// recompute velocity and position at k-1/2
midvel = (pvacur.vel + pvapre.vel) / 2;
midpos = pvapre.pos + Earth::DRi(pvapre.pos) * midvel * imucur.dt / 2;
```

根据中间时刻位置， 重新计算中间时刻地理参数（除了重力）：

```cpp
// 重新计算中间时刻地理参数
// recompute rmrn, wie_n, wen_n at k-1/2
Eigen::Vector2d rmrn;
Eigen::Vector3d wie_n, wen_n;
rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
wen_n << midvel[1] / (rmrn[1] + midpos[2]), -midvel[0] / (rmrn[0] + midpos[2]),
    -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);
```

重新计算 k 时刻到 k-1 时刻 n 系旋转矢量：
$$
{\omega}_{i n}^{n}={\omega}_{i e}^{n}+{\omega}_{e n}^{n}
$$

$$
\left.\boldsymbol{\zeta}_{n(k-1) n(k)} \approx \boldsymbol{\omega}_{i n}^{n}\right|_{t=t_{k-1 / 2}} \Delta t_{k}
$$

```cpp
// 重新计算 k时刻到k-1时刻 n系旋转矢量
// recompute n-frame rotation vector (n(k) with respect to n(k-1)-frame)
temp1 = (wie_n + wen_n) * imucur.dt;
qnn   = Rotation::rotvec2quaternion(temp1);
```

e系转动等效旋转矢量 (k-1时刻k时刻，所以取负号)，直接就是地球自转角速率乘以时间差：

```cpp
// e系转动等效旋转矢量 (k-1时刻k时刻，所以取负号)
// e-frame rotation vector (e(k-1) with respect to e(k)-frame)
temp2 << 0, 0, -WGS84_WIE * imucur.dt;
qee = Rotation::rotvec2quaternion(temp2);
```

由先前时刻位置，调用 qne()，得到先前n系到e系旋转四元数：

```cpp
qne = Earth::qne(pvapre.pos);
```

当前时刻n系到e系旋转四元数 = 两时刻e系旋转四元数 * 先前n系到e系旋转四元数 * 两时刻n系旋转四元数：

```cpp
qne = qee * qne * qnn;
```

当前时刻高程 = 先前高程 - 高程方向速度 * 采样周期（因为北东地，计算出的速度时地向的，所以减）：

``` C++
pvacur.pos[2] = pvapre.pos[2] - midvel[2] * imucur.dt;
```

调用 `blh()` 根据 n 系到 e 系旋转四元数计算经纬度：

```C++
pvacur.pos = Earth::blh(qne, pvacur.pos[2]);
```

### 6、attUpdate()：姿态更新

![image-20230925191854741](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925191854741.png)

先定义解算过程中涉及的中间变量：

```cpp
Eigen::Quaterniond qne_pre, qne_cur, qne_mid, qnn, qbb;
Eigen::Vector3d temp1, midpos, midvel;
```

重新计算中间时刻的速度和位置：

* 根据两时刻速度，计算平均速度 midvel
* 根据上一时刻位置计算n系到e系转换四元数 qne_pre
* 根据当前时刻位置计算n系到e系转换四元数 qne_cur
* 根据两时刻转换四元数，计算n系到e系平均转换四元数 qne_mid（注意得通过等效旋转矢量，并非直接插值）
* 计算当前中间时刻位置 midpos

```cpp
// 重新计算中间时刻的速度和位置
// recompute velocity and position at k-1/2
midvel = (pvapre.vel + pvacur.vel) / 2;
qne_pre   = Earth::qne(pvapre.pos);
qne_cur   = Earth::qne(pvacur.pos);
temp1     = Rotation::quaternion2vector(qne_cur.inverse() * qne_pre);
qne_mid   = qne_pre * Rotation::rotvec2quaternion(temp1 / 2).inverse();
midpos[2] = (pvacur.pos[2] + pvapre.pos[2]) / 2;
midpos    = Earth::blh(qne_mid, midpos[2]);
```

重新计算中间时刻地理参数：

```cpp
// 重新计算中间时刻地理参数
// recompute rmrn, wie_n, wen_n at k-1/2
Eigen::Vector2d rmrn;
Eigen::Vector3d wie_n, wen_n;
rmrn = Earth::meridianPrimeVerticalRadius(midpos[0]);
wie_n << WGS84_WIE * cos(midpos[0]), 0, -WGS84_WIE * sin(midpos[0]);
wen_n << midvel[1] / (rmrn[1] + midpos[2]), -midvel[0] / (rmrn[0] + midpos[2]),
    -midvel[1] * tan(midpos[0]) / (rmrn[1] + midpos[2]);
```

计算 n 系的旋转四元数 k-1时刻到k时刻系旋转：
$$
{\omega}_{i n}^{n}={\omega}_{i e}^{n}+{\omega}_{e n}^{n}
$$

$$
\left.\boldsymbol{\zeta}_{n(k-1) n(k)} \approx \boldsymbol{\omega}_{i n}^{n}\right|_{t=t_{k-1 / 2}} \Delta t_{k}
$$

```cpp
// 重新计算 k时刻到k-1时刻 n系旋转矢量
// recompute n-frame rotation vector (n(k) with respect to n(k-1)-frame)
temp1 = (wie_n + wen_n) * imucur.dt;
qnn   = Rotation::rotvec2quaternion(temp1);
```

计算 b 系旋转四元数补偿二阶圆锥误差：
$$
\boldsymbol{q}_{b_{k}}^{b_{k-1}} \leftarrow \boldsymbol{\phi}_{k}=\Delta \boldsymbol{\theta}_{k}+\frac{1}{12} \Delta \boldsymbol{\theta}_{k-1} \times \Delta \boldsymbol{\theta}_{k}
$$

```cpp
// 计算b系旋转四元数 补偿二阶圆锥误差
// b-frame rotation vector (b(k) with respect to b(k-1)-frame)
// compensate the second-order coning correction term.
temp1 = imucur.dtheta + imupre.dtheta.cross(imucur.dtheta) / 12;
qbb   = Rotation::rotvec2quaternion(temp1);
```

两时刻n系的旋转四元数 * 上一时刻姿态四元数 * 两时刻b系旋转四元数得到当前姿态：
$$
\boldsymbol{q}_{b_{k}}^{n_{k}}=\boldsymbol{q}_{n_{k-1}}^{n_{k}} \boldsymbol{q}_{b_{k-1}}^{n_{k-1}} \boldsymbol{q}_{b_{k}}^{b_{k-1}}
$$

```cpp
// 姿态更新完成
// attitude update finish
pvacur.att.qbn   = qnn * pvapre.att.qbn * qbb;
pvacur.att.cbn   = Rotation::quaternion2matrix(pvacur.att.qbn);
pvacur.att.euler = Rotation::matrix2euler(pvacur.att.cbn);
```

### 7、噪声传播

在 `insPropagation()` 函数中，IMU状态更新之后进行。

先要构建 F 矩阵：
$$
\mathbf{F}=\left[\begin{array}{ccccccc}\mathbf{F}_{r r} & \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{F}_{v r} & \mathbf{F}_{v v} & {\left[\left(\mathbf{C}_{b}^{n} \boldsymbol{f}^{b}\right) \times\right]} & \mathbf{0} & \mathbf{C}_{b}^{n} & \mathbf{0} & \mathbf{C}_{b}^{n} \operatorname{diag}\left(\boldsymbol{f}^{b}\right) \\ \mathbf{F}_{\phi r} & \mathbf{F}_{\phi v} & -\left(\boldsymbol{\omega}_{i n}^{n} \times\right) & -\mathbf{C}_{b}^{n} & \mathbf{0} & -\mathbf{C}_{b}^{n} \operatorname{diag}\left(\boldsymbol{\omega}_{i b}^{b}\right) & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \frac{-1}{T_{g b}} \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \frac{-1}{T_{a b}} \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \frac{-1}{T_{g s}} \mathbf{I}_{3 \times 3} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \frac{-1}{T_{a s}} \mathbf{I}_{3 \times 3}\end{array}\right]
$$
其中：
$$
\mathbf{F}_{r r}=\left[\begin{array}{ccc}-\frac{v_{D}}{R_{M}+h} & 0 & \frac{v_{N}}{R_{M}+h} \\ \frac{v_{E} \tan \varphi}{R_{N}+h} & -\frac{v_{D}+v_{N} \tan \varphi}{R_{N}+h} & \frac{v_{E}}{R_{N}+h} \\ 0 & 0 & 0\end{array}\right]
$$

```C++
temp.setZero();
temp(0, 0) = -pvapre_.vel[2] / rmh;
temp(0, 2) = pvapre_.vel[0] / rmh;
temp(1, 0) = pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
temp(1, 1) = -(pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
temp(1, 2) = pvapre_.vel[1] / rnh;
F.block(P_ID, P_ID, 3, 3) = temp;
F.block(P_ID, V_ID, 3, 3) = Eigen::Matrix3d::Identity();
```

、
$$
\mathbf{F}_{v r}=\left[\begin{array}{ccc}\frac{-2 v_{E} \omega_{e} \cos \varphi}{R_{M}+h}-\frac{v_{E}^{2} \sec ^{2} \varphi}{\left(R_{M}+h\right)\left(R_{N}+h\right)} & 0 & \frac{v_{N} v_{D}}{\left(R_{M}+h\right)^{2}}-\frac{v_{E}^{2} \tan \varphi}{\left(R_{N}+h\right)^{2}} \\ \frac{2 \omega_{e}\left(v_{N} \cos \varphi-v_{D} \sin \varphi\right)}{R_{M}+h}+\frac{v_{N} v_{E} \sec ^{2} \varphi}{\left(R_{M}+h\right)\left(R_{N}+h\right)} & 0 & \frac{v_{E} v_{D}+v_{N} v_{E} \tan \varphi}{\left(R_{N}+h\right)^{2}} \\ \frac{2 \omega_{e} v_{E} \sin \varphi}{R_{M}+h} & 0 & -\frac{v_{E}^{2}}{\left(R_{N}+h\right)^{2}}-\frac{v_{N}^{2}}{\left(R_{M}+h\right)^{2}}+\frac{2 g_{p}}{\sqrt{R_{M} R_{N}+h}}\end{array}\right]
$$

```C++
temp.setZero();
temp(0, 0) = -2 * pvapre_.vel[1] * WGS84_WIE * cos(pvapre_.pos[0]) / rmh -
             pow(pvapre_.vel[1], 2) / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
temp(0, 2) = pvapre_.vel[0] * pvapre_.vel[2] / rmh / rmh - pow(pvapre_.vel[1], 2) * tan(pvapre_.pos[0]) / rnh / rnh;
temp(1, 0) = 2 * WGS84_WIE * (pvapre_.vel[0] * cos(pvapre_.pos[0]) - pvapre_.vel[2] * sin(pvapre_.pos[0])) / rmh +
             pvapre_.vel[0] * pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
temp(1, 2) = (pvapre_.vel[1] * pvapre_.vel[2] + pvapre_.vel[0] * pvapre_.vel[1] * tan(pvapre_.pos[0])) / rnh / rnh;
temp(2, 0) = 2 * WGS84_WIE * pvapre_.vel[1] * sin(pvapre_.pos[0]) / rmh;
temp(2, 2) = -pow(pvapre_.vel[1], 2) / rnh / rnh - pow(pvapre_.vel[0], 2) / rmh / rmh +
             2 * gravity / (sqrt(rmrn[0] * rmrn[1]) + pvapre_.pos[2]);
F.block(V_ID, P_ID, 3, 3) = temp;
```


$$
\mathbf{F}_{v v}=\left[\begin{array}{ccc}\frac{v_{D}}{R_{M}+h} & -2\left(\omega_{e} \sin \varphi+\frac{v_{E} \tan \varphi}{R_{N}+h}\right) & \frac{v_{N}}{R_{M}+h} \\ 2 \omega_{e} \sin \varphi+\frac{v_{E} \tan \varphi}{R_{N}+h} & \frac{v_{D}+v_{N} \tan \varphi}{R_{N}+h} & 2 \omega_{e} \cos \varphi+\frac{v_{E}}{R_{N}+h} \\ -\frac{2 v_{N}}{R_{M}+h} & -2\left(\omega_{e} \cos \varphi+\frac{v_{E}}{R_{N}+h}\right) & 0\end{array}\right]
$$

```C++
temp.setZero();
temp(0, 0) = pvapre_.vel[2] / rmh;
temp(0, 1) = -2 * (WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh);
temp(0, 2) = pvapre_.vel[0] / rmh;
temp(1, 0) = 2 * WGS84_WIE * sin(pvapre_.pos[0]) + pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh;
temp(1, 1) = (pvapre_.vel[2] + pvapre_.vel[0] * tan(pvapre_.pos[0])) / rnh;
temp(1, 2) = 2 * WGS84_WIE * cos(pvapre_.pos[0]) + pvapre_.vel[1] / rnh;
temp(2, 0) = -2 * pvapre_.vel[0] / rmh;
temp(2, 1) = -2 * (WGS84_WIE * cos(pvapre_.pos(0)) + pvapre_.vel[1] / rnh);
F.block(V_ID, V_ID, 3, 3)   = temp;
F.block(V_ID, PHI_ID, 3, 3) = Rotation::skewSymmetric(pvapre_.att.cbn * accel);
F.block(V_ID, BA_ID, 3, 3)  = pvapre_.att.cbn;
F.block(V_ID, SA_ID, 3, 3)  = pvapre_.att.cbn * (accel.asDiagonal());
```


$$
\mathbf{F}_{\phi r}=\left[\begin{array}{ccc}-\frac{\omega_{e} \sin \varphi}{R_{M}+h} & 0 & \frac{v_{E}}{\left(R_{N}+h\right)^{2}} \\ 0 & 0 & -\frac{v_{N}}{\left(R_{M}+h\right)^{2}} \\ -\frac{\omega_{e} \cos \varphi}{R_{M}+h}-\frac{v_{E} \sec ^{2} \varphi}{\left(R_{M}+h\right)\left(R_{N}+h\right)} & 0 & -\frac{v_{E} \tan \varphi}{\left(R_{N}+h\right)^{2}}\end{array}\right]
$$

```C++
temp.setZero();
temp(0, 0) = -WGS84_WIE * sin(pvapre_.pos[0]) / rmh;
temp(0, 2) = pvapre_.vel[1] / rnh / rnh;
temp(1, 2) = -pvapre_.vel[0] / rmh / rmh;
temp(2, 0) = -WGS84_WIE * cos(pvapre_.pos[0]) / rmh - pvapre_.vel[1] / rmh / rnh / pow(cos(pvapre_.pos[0]), 2);
temp(2, 2) = -pvapre_.vel[1] * tan(pvapre_.pos[0]) / rnh / rnh;
F.block(PHI_ID, P_ID, 3, 3) = temp;
```


$$
\mathbf{F}_{\phi v}=\left[\begin{array}{ccc}0 & \frac{1}{R_{N}+h} & 0 \\ -\frac{1}{R_{M}+h} & 0 & 0 \\ 0 & -\frac{\tan \varphi}{R_{N}+h} & 0\end{array}\right]
$$

```C++
temp.setZero();
temp(0, 1) = 1 / rnh;
temp(1, 0) = -1 / rmh;
temp(2, 1) = -tan(pvapre_.pos[0]) / rnh;
F.block(PHI_ID, V_ID, 3, 3)   = temp;
F.block(PHI_ID, PHI_ID, 3, 3) = -Rotation::skewSymmetric(wie_n + wen_n);
F.block(PHI_ID, BG_ID, 3, 3)  = -pvapre_.att.cbn;
F.block(PHI_ID, SG_ID, 3, 3)  = -pvapre_.att.cbn * (omega.asDiagonal());
```

IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程：
$$
\frac{-1}{T_{g s}} \mathbf{I}_{3 \times 3}
$$

```C++
// IMU零偏误差和比例因子误差，建模成一阶高斯-马尔科夫过程
// imu bias error and scale error, modeled as the first-order Gauss-Markov process
F.block(BG_ID, BG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
F.block(BA_ID, BA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
F.block(SG_ID, SG_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
F.block(SA_ID, SA_ID, 3, 3) = -1 / options_.imunoise.corr_time * Eigen::Matrix3d::Identity();
```

系统噪声驱动矩阵：
$$
\underset{21 \times 18}{\mathbf{G}_{18}}=\left[\begin{array}{cccccc}\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{C}_{b}^{n} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{C}_{b}^{n} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_{3 \times 3} & \mathbf{0} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_{3 \times 3} & \mathbf{0} \\ \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I}_{3 \times 3}\end{array}\right]
$$

```C++
// 系统噪声驱动矩阵
// system noise driven matrix
G.block(V_ID, VRW_ID, 3, 3)    = pvapre_.att.cbn;
G.block(PHI_ID, ARW_ID, 3, 3)  = pvapre_.att.cbn;
G.block(BG_ID, BGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
G.block(BA_ID, BASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
G.block(SG_ID, SGSTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
G.block(SA_ID, SASTD_ID, 3, 3) = Eigen::Matrix3d::Identity();
```

系统传播噪声的状态转移矩阵：
$$
\boldsymbol{\Phi}_{k / k-1}=\boldsymbol{I}+\boldsymbol{F}_{k-1} \Delta t_{k}
$$

```C++
// 状态转移矩阵
// compute the state transition matrix
Phi.setIdentity();
Phi = Phi + F * imucur.dt; 
```

系统传播噪声：
$$
\boldsymbol{Q}_{k}=\left(\begin{array}{c}\boldsymbol{\Phi}_{k / k-1} \boldsymbol{G}_{k-1} \boldsymbol{q}_{k-1} \boldsymbol{G}_{k-1}^{T} \boldsymbol{\Phi}_{k / k-1}^{T} \\ +\boldsymbol{G}_{k} \boldsymbol{q}_{k} \boldsymbol{G}_{k}^{T}\end{array}\right) \Delta t_{k} / 2
$$

```C++
// 计算系统传播噪声
// compute system propagation noise
Qd = G * Qc_ * G.transpose() * imucur.dt;
Qd = (Phi * Qd * Phi.transpose() + Qd) / 2;
```

 EKF 预测传播系统协方差和系统误差状态：
$$
\begin{aligned} \boldsymbol{x}_{k / k-1} & =\boldsymbol{\Phi}_{k / k-1} \boldsymbol{x}_{k-1} \\ \boldsymbol{P}_{k / k-1} & =\boldsymbol{\Phi}_{k / k-1} \boldsymbol{P}_{k-1} \boldsymbol{\Phi}_{k / k-1}^{T}+\boldsymbol{Q}_{k}\end{aligned}
$$

```C++
EKFPredict(Phi, Qd);
```

```C++
void GIEngine::EKFPredict(Eigen::MatrixXd &Phi, Eigen::MatrixXd &Qd) {

    assert(Phi.rows() == Cov_.rows());
    assert(Qd.rows() == Cov_.rows());

    // 传播系统协方差和误差状态
    // propagate system covariance and error state
    Cov_ = Phi * Cov_ * Phi.transpose() + Qd;
    dx_  = Phi * dx_;
}
```

## 八、GNSS 量测更新、系统状态反馈

### 1、gnssUpdate()：GNSS 量测更新













### 2、EKFUpdate()：EKF更新协方差和误差状态







### 3、stateFeedback()：状态反馈







## 九、KF-GINS常见问题

> 复制自PPT

### KF-GINS能够达到怎么样的定位精度？

组合导航算法精度**更受IMU等级、以及测试时GNSS定位质量**影响。组合导航算法相对成熟，对于同样的设备只要算法正确实现，算法几乎不会对定位精度产生较大影响。

### 初始导航状态和初始导航状态标准差如何给定？

* 初始位置(和速度)可由**GNSS给定**，初始姿态(和速度)可从**参考结果中获取**；

* 位置速度标准差可由GNSS给定，姿态标准差可给经验值，车载领域一般横滚俯仰小，航向大一些

### IMU数据输入到程序之前，需要扣除重力加速度吗？

不需要，**惯性导航算法中补偿了重力加速度**

### INS机械编排中旋转效应等补偿项，对于低端IMU是否需要补偿？

低端IMU测量噪声更大，**简化的IMU积分算法对低端IMU精度产生的影响较小**

### 组合导航中GNSS信号丢失期间进行纯惯导解算，这时IMU误差项可以补偿吗？

GNSS丢失期间IMU误差项不更新，但是可以**利用之前估计的IMU误差项继续补偿**

### IMU数据，如何从速率形式转到增量形式？

一般采用更高频率速率数据积分得到增量数据，参考：[新手入门系列4——MEMS IMU**原始数**](http://www.i2nav.cn/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=2e05f5cdac6b4725b8bfe54a689c7add)

**据采集和时间同步**的那些坑(i2Nav网站)

### IMU零偏和比例因子建模时相关时间如何给定？

建模为一阶高斯-马尔可夫过程，实际中一般**根据经验设置相关时间，MEMS设置1h**

### GNSS/INS组合导航中是否需要考虑惯性系和车体系的转换？

**不需要，GNSS/INS组合导航不受载体限制**，不需要考虑车体系

### 初始化拓展

* 动态初始对准，GNSS位置差分速度(或者GNSS直接输出速度)，位置差分计算初始航向

* 快速航向初始化 ，轨迹匹配方法快速获取准确初始航向

* 静态粗对准，适用于高精度惯导，双矢量法找到初始姿态

###  观测信息拓展

* 直接构建观测向量、观测模型和噪声矩阵，调用EKFUpdate更新和stateFeedback反馈

* GNSS速度观测信息、NHC约束信息(对于车载)、零速信息修正

### 状态信息拓展

* 如果需要增广系统状态(如里程计增广比例因子[2])，则修改RANK(NOISERANK)，添加StateID, NoiseID
* 协方差、状态转移矩阵、观测信息对应修改；添加观测信息，进行更新反馈
