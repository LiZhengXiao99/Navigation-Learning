> 原始 Markdown文档、Visio流程图、XMind思维导图见：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、OB-GINS 简介

### 1、程序概述



基于图优化的 IMU/GNSS 松组合解算，IMU 预积分算法相比以视觉为主的 ORB-SLAM3、VINS 要精细一些，

开源地址：https://github.com/i2Nav-WHU/OB_GINS

### 2、相关论文

* Hailiang Tang, Tisheng Zhang, Xiaoji Niu, Jing Fan, and Jingnan Liu, “Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization,” *IEEE Sensors Journal*, 2022. [下载](http://www.i2nav.com/ueditor/jsp/upload/file/20220801/1659348408510061111.pdf)

  > **摘要翻译**：
  >
  > * 在基于滤波的 GNSS/INS 组合导航系统中，精密的 IMU 机械编排需要考虑到地球自转、牵连角速度、科氏加速度等的影响；然而大多数的图优化框架中的 IMU 预积分模型都没有考虑这些因素。
  > * 我们提出了一种进行了地球自转补偿的滑动窗口图优化 GNSS/INS 组合导航算法，并且评估了地球自转等对MEMS-IMU预积分的影响。
  > * 测试了 GNSS 量测缺失，采用了有效的方法对预积分结果进行评价。结果表明，此方法预积分的精度与精密机械编排的精度相当；相比之下，IMU 在不补偿地球自转的情况下，预积分的精度显著低于机械编排的精度，有显著的精度降级。
  > * 当 GNSS 中断时间为 60 秒时，工业级 MEMS 的降级可能为 200% 模块，消费级 MEMS 芯片超过 10%。此外，如果 GNSS 中断时间更长，精度降级还会更显著。

* Junxiang Jiang, Xiaoji Niu, and Jingnan Liu, “Improved IMU Preintegration with Gravity Change and Earth Rotation for Optimization-Based GNSS/VINS,” *Remote Sensing*, vol. 12, no. 18, p. 3048, Sep. 2020, doi: [10.3390/rs12183048](https://doi.org/10.3390/rs12183048). [下载](https://sci-hub.se/10.3390/rs12183048)

  > **摘要翻译**：
  >
  > * IMU 预积分技术广泛用于图优化框架的多源融合定位解算中，它可以避免高频率的 IMU 量测数据在迭代计算中重复积分，并且在偏差估计发生变化时保持偏差校正的能力。
  > * 自从 IMU 预积分被提出后，陆续有一些改变姿态参数和数值积分方法增强版本的算法被设计出来，然而这些版本都没有考虑到重力和地球自转。
  > * 在本文提出的算法中，我们利用了基于大地经纬高算出的重力向量，且预积分 IMU 测量的不确定性没有使用协方差矩阵，而是以平方根信息矩阵（SRIM）的形式传播，以获得更好的数值稳定性，并且易于在基于优化的框架中使用。
  > * 为了评估算法的精度，我们使用了两种级别的 IMU 数据进行测试，测试结果表明，改进的IMU预集成算法能够很好地应对重力变化和地球自转。当处理能有效感知地球自转的高级 IMU 的数据时，必须考虑地球自转。
  > * 如果忽略重力的变化，水平姿态的均方根误差（RMSE）大约是 1.38  倍的大地位移；此外，定位 RMSE 也不在有限的范围内；这意味着几十公里和几百米的实验中分别使用的低级别和高级 IMU。

* Le Chang, Xiaoji Niu, and Tianyi Liu, “GNSS/IMU/ODO/LiDAR-SLAM Integrated Navigation System Using IMU/ODO Pre-Integration,” *Sensors*, vol. 20, no. 17, p. 4702, Aug. 2020, doi: [10.3390/s20174702](https://doi.org/10.3390/s20174702). [下载](https://www.mdpi.com/1424-8220/20/17/4702/pdf)

  > **摘要翻译**：
  >
  > * 在本文，我们提出一套 GNSS/IMU/ODO/Lidar-SLAM 融合导航定位算法。
  > * **前端**进行航位推算，进行 IMU/ODO 预积分。**后端**通过图优化对 GNSS 坐标、IMU/ODO 预积分结果、LiDAR-SLAM 得出的相对位置和姿态进行融合。
  > * **里程计**信息用于降低 IMU 预积分算法中的**漂移率**。**滑动窗口**算法可以通过减少图优化的参数，来**降低**图优化的**计算量**。
  > * 分别在露天区域和隧道情况下进行车辆数据测试；测试结果表明，所提出的导航系统可以有效地提高导航的精确度和鲁棒性。
  > * 在模拟两分钟 GNSS 中断期间，相比传统的 GNSS/INS（惯性导航系统）/ODO组合算法， 北东地的 RMS 分别下降62.8%、72.3%、52.1%。此外，与GNSS/IMU/激光雷达SLAM集成相比，误差降低了62.1%导航系统，里程表和非完整约束的辅助下减少垂直误差为 72.3%。
  > * 在实际隧道案例中的测试表明，在弱环境特征区域激光雷达 SLAM 几乎不能工作，里程计在预集成中的辅助作用至关重要，并且可以有效地减少沿正向方向的定位漂移并保持 SLAM 短期内有效。
  > * 因此，提出的 GNSS/IMU/ODO/Lidar-SLAM 组合导航系统可以有效地融合来自多个来源的信息，以维护 SLAM 过程显著降低导航误差，尤其是在 GNSS 信号严重的恶劣地区退化和环境特征不足以用于激光雷达 SLAM。
  >



### 3、代码分析



用 cloc 对 src 目录下 factors、fileio、preintegration 进行统计，结果如下：

![image-20230927173131452](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230927173131452.png)





### 4、第三方库



### 5、重点函数



## 二、编译、调试





## 三、类型定义





## 四、程序主要执行流程

### 1、函数调用关系图





### 2、main 函数

OB-GINS 主函数很长，将近 500 行，可以分为三部分：

* while 循环解算前：读取配置文件、创建解算对象，赋值初始的状态量，进行一次初始预积分。
* while 循环解算：
* while 循环解算后：释放创建的对象，输出解算时间。



判断参数数量，如果不为 2，说明没传入配置文件，提醒用户传配置文件路径，然后退出程序。输出程序信息，记录开始时间。

```cpp
if (argc != 2) {
    std::cout << "usage: ob_gins ob_gins.yaml" << std::endl;
    return -1;
}

std::cout << "\nOB_GINS: An Optimization-Based GNSS/INS Integrated Navigation System\n\n";

auto ts = absl::Now();
```

导入配置选项

```cpp
// 读取配置
// load configuration
YAML::Node config;
std::vector<double> vec;
try {
    config = YAML::LoadFile(argv[1]);
} catch (YAML::Exception &exception) {
    std::cout << "Failed to read configuration file" << std::endl;
    return -1;
}

// 时间信息
// processing time
int windows   = config["windows"].as<int>();
int starttime = config["starttime"].as<int>();
int endtime   = config["endtime"].as<int>();

// 迭代次数
// number of iterations
int num_iterations = config["num_iterations"].as<int>();

// 进行GNSS粗差检测
// Do GNSS outlier culling
bool is_outlier_culling = config["is_outlier_culling"].as<bool>();

// 初始化信息
// initialization
vec = config["initvel"].as<std::vector<double>>();
Vector3d initvel(vec.data());
vec = config["initatt"].as<std::vector<double>>();
Vector3d initatt(vec.data());
initatt *= D2R;

vec = config["initgb"].as<std::vector<double>>();
Vector3d initbg(vec.data());
initbg *= D2R / 3600.0;
vec = config["initab"].as<std::vector<double>>();
Vector3d initba(vec.data());
initba *= 1.0e-5;

// 数据文件
// data file
std::string gnsspath   = config["gnssfile"].as<std::string>();
std::string imupath    = config["imufile"].as<std::string>();
std::string outputpath = config["outputpath"].as<std::string>();
int imudatalen         = config["imudatalen"].as<int>();
int imudatarate        = config["imudatarate"].as<int>();

// 是否考虑地球自转
// consider the Earth's rotation
bool isearth = config["isearth"].as<bool>();

// 安装参数
// installation parameters
vec = config["antlever"].as<std::vector<double>>();
Vector3d antlever(vec.data());
vec = config["odolever"].as<std::vector<double>>();
Vector3d odolever(vec.data());
vec = config["bodyangle"].as<std::vector<double>>();
Vector3d bodyangle(vec.data());
bodyangle *= D2R;

// IMU噪声参数
// IMU noise parameters
auto parameters          = std::make_shared<IntegrationParameters>();
parameters->gyr_arw      = config["imumodel"]["arw"].as<double>() * D2R / 60.0;
parameters->gyr_bias_std = config["imumodel"]["gbstd"].as<double>() * D2R / 3600.0;
parameters->acc_vrw      = config["imumodel"]["vrw"].as<double>() / 60.0;
parameters->acc_bias_std = config["imumodel"]["abstd"].as<double>() * 1.0e-5;
parameters->corr_time    = config["imumodel"]["corrtime"].as<double>() * 3600;

bool isuseodo       = config["odometer"]["isuseodo"].as<bool>();
vec                 = config["odometer"]["std"].as<std::vector<double>>();
parameters->odo_std = Vector3d(vec.data());
parameters->odo_srw = config["odometer"]["srw"].as<double>() * 1e-6;
parameters->lodo    = odolever;
parameters->abv     = bodyangle;

// GNSS仿真中断配置
// GNSS outage parameters
bool isuseoutage = config["isuseoutage"].as<bool>();
int outagetime   = config["outagetime"].as<int>();
int outagelen    = config["outagelen"].as<int>();
int outageperiod = config["outageperiod"].as<int>();
auto gnssthreshold = config["gnssthreshold"].as<double>();
```

根据读进来的配置，构造解算用到的几个文件对象：

* 文件读取对象：`gnssfile`、`imufile`
* 构造输出文件对象：`navfile`、`errfile`

```cpp
GnssFileLoader gnssfile(gnsspath);
ImuFileLoader imufile(imupath, imudatalen, imudatarate);
FileSaver navfile(outputpath + "/OB_GINS_TXT.nav", 11, FileSaver::TEXT);
FileSaver errfile(outputpath + "/OB_GINS_IMU_ERR.bin", 7, FileSaver::BINARY);
if (!imufile.isOpen() || !navfile.isOpen() || !navfile.isOpen() || !errfile.isOpen()) {
    std::cout << "Failed to open data file" << std::endl;
    return -1;
}
```

循环调用 `imufile.next()`、`gnssfile.next()` 读取 IMU、GNSS 数据，直到时间戳在解算时间范围内。循环结束后 `imu_cur`、`gnss` 分别存解算时间内第一个 IMU、GNSS 量测，且文件指针指向的位置也到达解算时间内数据的开头：

```cpp
// 数据文件调整
// data alignment
IMU imu_cur, imu_pre;
do {
    imu_pre = imu_cur;
    imu_cur = imufile.next();
} while (imu_cur.time < starttime);

GNSS gnss;
do {
    gnss = gnssfile.next();
} while (gnss.time < starttime);
```

设置初始状态：

* station_origin：
* 

```cpp
// 初始位置, 求相对
Vector3d station_origin = gnss.blh;
parameters->gravity     = Earth::gravity(gnss.blh);
gnss.blh                = Earth::global2local(station_origin, gnss.blh);

// 站心坐标系原点
parameters->station = station_origin;

std::vector<IntegrationState> statelist(windows + 1);
std::vector<IntegrationStateData> statedatalist(windows + 1);
std::deque<std::shared_ptr<PreintegrationBase>> preintegrationlist;
std::deque<GNSS> gnsslist;      // GNSS 数据双端对列
std::deque<double> timelist;    // 时间戳双端对列

Preintegration::PreintegrationOptions preintegration_options = Preintegration::getOptions(isuseodo, isearth);

// 赋值初始状态到 state_curr
// initialization
IntegrationState state_curr = {
    .time = round(gnss.time),
    .p    = gnss.blh - Rotation::euler2quaternion(initatt) * antlever,
    .q    = Rotation::euler2quaternion(initatt),
    .v    = initvel,
    .bg   = initbg,
    .ba   = initba,
    .sodo = 0.0,
    .abv  = {bodyangle[1], bodyangle[2]},
};
std::cout << "Initilization at " << gnss.time << " s " << std::endl;

statelist[0]     = state_curr;
statedatalist[0] = Preintegration::stateToData(state_curr, preintegration_options);
gnsslist.push_back(gnss);

double sow = round(gnss.time);
timelist.push_back(sow);
```

初始预积分，根据设置预积分模式，调用 createPreintegration() 创建对应的预积分器，放入 preintegrationlist：

```cpp
preintegrationlist.emplace_back(
    Preintegration::createPreintegration(parameters, imu_pre, state_curr, preintegration_options));
```











下面进入 while 循环解算，如果在解算时间内，且文件还有数据，调用 addNewImu() 加入IMU数据，其中会执行 integrationProcess() 预积分，然后再读取下一个 IMU 数据：

```cpp
// 如果当前 IMU 时间戳超过结束时间，或文件读完，退出循环解算完成
if ((imu_cur.time > endtime) || imufile.isEof()) {
    break;
}

// 加入IMU数据，其中会执行 integrationProcess() 预积分
// Add new imu data to preintegration
preintegrationlist.back()->addNewImu(imu_cur);

imu_pre = imu_cur;
imu_cur = imufile.next(); 
```

判断读进来 IMU 时间是否等于设置的 GNSS 时间，不等于就直接调用 writeNavResult() 输出解算信息：

```cpp
auto integration = *preintegrationlist.rbegin();
writeNavResult(integration->endTime(), station_origin, integration->currentState(), navfile, errfile);
```













## 五、IMU 预积分





















