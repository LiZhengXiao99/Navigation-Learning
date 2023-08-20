[TOC]

## 一、光流追踪原理

获取摄像头的图像帧，并按照事先设定的频率，把 cur 帧上满足要求的特征点以sensor_msg::PointCloudPtr 的格式发布出去，以便 RVIZ 和 vins--estimator接收

前端主要就是光流追踪，

主要目的就行向后端提供特征点信息，包括：

* **像素坐标**：特征点提取算法
* **去畸变后的归一化坐标**：特征点去畸变算法
* **特征点 ID**：光流追踪算法
* **特征点速度**，用于 IMU 和 Camera 时间戳校正

与 ORB-SLAM 中基于特征点和描述子的视觉里程计不同，VINS 不计算描述子。同时，使用光流法 (Optical Flow) 来跟踪特征点的运动。这样可以回避计算和匹配描述子带来的时间，但光流本身的计算需要一定时间。



光流是一种描述像素随着时间，在图像之间运动的方法。随着时间的经过，同一个像素会在图像中运动，而我们希望追踪它的运动过程。计算部分像素运动的称为稀疏光流，计算所有像素的称为稠密光流。稀疏光流以 Lucas-Kanade 光流为代表，并可以在 SLAM 中用于跟踪特征点位置。

在 LK 光流中，我们认为相机的图像是随着时间变化的，图像可以看做关于位置和时间的函数。考虑空间中某个固定点，由于相机的运动，它的坐标将发生变化；我们希望估计这个点在其它时刻图像里的位置，引入**灰度不变假设**，即：同一点点所在区域灰度值在连续两帧之间灰度变化极小，









转化为用优化问题，为防止局部最小，用图像金字塔提高光流追踪的稳定性，图像缩放了更容易追踪，最终还要把找到的特征点返回到实际图像上的位置，上一次金字塔最终的结果作为下一次金字塔最终的初值。





主要三个源程序，feature_tracker_node是特征跟踪线程的系统入口，feature_tracker是特征跟踪算法的具体实现，parameters是设备等参数的读取和存放

* 对于每一幅新图像，KLT稀疏光流算法对现有特征进行跟踪；

* 检测新的角点特征以保证每个图像特征的最小数目(100-300)

* 通过设置两个相邻特征之间像素的最小间隔来执行均匀的特征分布；

* 利用基本矩阵模型的RANSAC算法进行外点剔除；

* 对特征点进行去畸变矫正，然后投影到一个单位球面上(对于cata-fisheye camera)。

* 关键帧选取

  * 当前帧相对最近的关键帧的特征平均视差大于一个阈值就为关键帧（因为视差可以根据平移和旋转共同得到，而纯旋转则导致不能三角化成功，所以这一步需要IMU预积分进行补偿）
  * 当前帧跟踪到的特征点数量小于阈值视为关键帧；






网上这张图有好几个水印，不知道是谁画的😂：

![feature_tracker 类](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/feature_tracker%20%E7%B1%BB.png)





feature_trackers可被认为是一个单独的模块：

![image-20230820152338754](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230820152338754.png)

* **输入**：图像，即订阅了传感器或者rosbag发布的topic：“/cam0/image_raw”

  ```C++
  ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
  ```

* **输出**：

  * 发布topic：“/feature_trackers/feature_img” 即跟踪的特征点图像，主要是之后给RVIZ用和调试用
  * 发布topic：“/feature_trackers/feature” 即跟踪的特征点信息，由/vins_estimator订阅并进行优化
  * 发布topic：“/feature_trackers/restart” 即判断特征跟踪模块是否出错，若有问题则进行复位，由/vins_estimator订阅

  ```C++
  pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
  pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
  pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
  ```

  















