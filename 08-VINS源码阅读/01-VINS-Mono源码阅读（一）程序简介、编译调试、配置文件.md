[TOC]

## 一、VINS-Mono 简介

### 1、程序概述

与双目相机和 RGB-D 相机相比，单目相机具有结构简单、成本低和处理速度快的优点。然而，单目 VSLAM 存在尺度不确定性、无法对齐位姿和重力方向的自身缺点和快速运动导致的运动模糊的环境下容易跟踪丢失等不足。为弥补此问题，可将单目相机和 IMU 相结合的传感器融合，这种融合方案被称为单目视觉惯性里程计（Visual Inertial Odometry，VIO）或单目视觉惯性 SLAM（Visual-inertial SLAM，VINS）。

* IMU 也可以弥补视觉 SLAM 在短时间、快速运动上的不足，另外由于 IMU 不依赖外界环境信

  息，对环境变化不敏感，也可以在少纹理、明暗变化较大或者光线较弱场景内提供短期的定位方案以及位姿估计方案。

* 较之惯性信息，视觉里程计可以提供丰富的外界信息，在低速平稳的运动中位姿估计稳定，而且视觉里程计在长时间运行后的漂移较小，并且可以通过回环检测修正自身位置以减小累积误差。

因此，在 SLAM 的众多分支中，无论在理论还是实践上，视觉-惯性融合为导航定位的研究工作提供了一个十分有前景的、小型化和低成本化的改进方案。

香港科技大学沈劭劼团队开发的 VINS 系统，用了一种紧耦合的非线性优化方法。该团队在 2017 年发布的 VINS-Mono 通过在四元数上进行 IMU 的预积分，并且采用滑动窗口法融合 IMU 信息和相机观测到的特征数据，实现了数据的紧耦合。并且采用四自由度的图优化方法实现了回环检测模块，来得到全局约束。在 2019 年，该团队又发布了 VINS-Fusion，在 VINS-Mono 的基础上又加入了双目、双目+IMU 等更多的传感器类型，以及支持了 VINS 和 GPS 的融合。它支持在线标定相机及 IMU 参数及鱼眼相机模型，并且支持保存当前地图和加载过往地图。在与 IMU 的结合上，它采用了四元数积分方案，与视觉信息进行紧耦合，具有很强的鲁棒性和定位精度。

![VINS-mono 组成](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/VINS-mono%20%E7%BB%84%E6%88%90.png)

### 2、资源获取

* GitHub 上开源了 VINS-mono，VINS-Fusion 和 VINS-mobile



### 3、代码分析





文件结构如下：

* ar_demo：一个ar应用demo
* benchmark_publisher：接收并发布数据集的基准值
* camera_model
  * calib：相机参数标定
  * camera_models：各种相机模型类
  * chessboard：检测棋盘格
  * gpl
  * sparse_graph
  * intrinsic_calib.cc：相机标定模块main函数

* config：系统配置文件存放处
* feature_trackers：
  * feature_tracker_node.cpp ROS 节点函数，回调函数
  * feature_tracker.cpp 图像特征光流跟踪

* pose_graph：
  * keyframe.cpp 关键帧选取、描述子计算与匹配
  * pose_graph.cpp 位姿图的建立与图优化
  * pose_graph_node.cpp ROS 节点函数，回调函数，主线程

* support_files：帮助文档、Bow字典、Brief模板文件

* vins_estimator：
  * factor：实现IMU、camera等残差模型
  * initial：系统初始化，外参标定，SFM
  * utility：相机可视化，四元数等数据转换
  * estimator.cpp：紧耦合的VIO状态估计器实现
  * estimator_node.cpp：ROS 节点函数，回调函数，主线程
  * feature_manager.cpp：特征点管理，三角化，关键帧等
  * parameters.cpp：读取参数



### 4、程序功能

程序主要分为五部分：

* 传感器数据处理：Camera 特征提取追踪、IMU 预积分
* 初始化：
* 基于滑动窗口的非线性优化：
* 闭环检测：
* 全局位姿图优化：











### 5、程序执行流程图





ROS 节点图如下：

![VINS-mono node 节点](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/VINS-mono%20node%20%E8%8A%82%E7%82%B9.png)





## 二、编译调试









## 三、launch 启动文件

在文件夹 VINS-Mono/vins_estimator/launch 中，启动时直接读取 euroc.launch 同时打开三个节点







## 四、YAML 配置文件







