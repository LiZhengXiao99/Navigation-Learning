[TOC]

## 一、程序简介

### 1、SVO 概述

SVO 全称 **S**emi-direct monocular **V**isual **O**dometry（[半直接视觉里程计](https://www.zhihu.com/search?q=半直接视觉里程计&search_source=Entity&hybrid_search_source=Entity&hybrid_search_extra={"sourceType"%3A"answer"%2C"sourceId"%3A138644975})），是苏黎世大学机器人感知组的克里斯蒂安.弗斯特等人，于 2014 年 ICRA 会议上发表的工作：[SVO: Fast Semi-Direct Monocular Visual Odometry](https://rpg.ifi.uzh.ch/docs/ICRA14_Forster.pdf)，随后在 GitHub 开源：[uzh-rpg/rpg_svo](https://link.zhihu.com/?target=https%3A//github.com/uzh-rpg/rpg_svo)。2016 年扩展了多相机和 IMU之后，写成期刊论文，称为 SVO 2.0，在IEEE Trans. on Robotics 上发表论文：[SVO: Semi-Direct Visual Odometry for Monocular and Multi-Camera Systems](https://rpg.ifi.uzh.ch/docs/TRO17_Forster-SVO.pdf)，并于 2021 年开源：[rpg_svo_pro_open](https://github.com/uzh-rpg/rpg_svo_pro_open)。作者弗斯特也在乔治亚理工的 gtsam 组呆过一段时间，参与了 gtsam 中 IMU 部分，亦是VIO当中的著名工作。

SVO 的核心特点是



SVO 另一特点是实现了一种特有的**深度滤波器**（Depth Filter）。这里一种基于均匀—高斯混合分布的深度滤波器，由弗吉亚兹于 2011 年提出并推导。单目 SLAM 中，刚提的特征点是没有深度的，所以必须用新来的帧的信息，去更新这些特征点的深度分布，也就是所谓的“深度滤波器”。 SVO 将这种滤波器用于关键点的深度估计，并使用了逆深度作为参数化形式，使之能够更好地计算特征点位置。这里 SVO 在建图线程中的主要任务。



### 2、线程设计

![image-20230817142653202](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230817142653202.png)

**SVO整体上分为两个线程**：

* **SVO整体上分为两个线程**：估计当前帧的位姿

  * 第一步是通过基于模型的稀疏**图像对齐**进行位姿的初始化，具体一点说就是通过最小化相同路标点投影位置对应的像素之间的**光度误差**，得到相对于前一帧的相机姿态（相对位姿）。注意这里优化的对象是帧间的相对位姿**（直接法）**。
  * 第二步是通过相应的**特征(块)对齐**，对重投影点对应的二维坐标进行细化。意思是说，由于先前估计的位姿和路标点位置可能有误差，当前帧中的特征块与参考关键帧中相应的特征块之间会存在光度误差(灰度不变假设)，我们在这一步中不考虑相机位姿，而是直接对当前帧中每一个特征块的2D位置进行单独优化以**进一步最小化**特征块的**光度误差**，这就建立了重投影点误差，也得到了相同路标点对应的重投影点的较好估计。这一步中优化的对象是当前帧中特征块的2D位置（重投影位置）。
  * 最后一步是对相机位姿和路标点位置进行优化估计，这一步中重投影位置是使用上一步中优化后的，参考帧是地图中的关键帧，因此均认为是准确的。优化的对象是路标点位置和相机位姿。

* **Mapping 线程**：估计特征点的深度。

  为每一个要被估计的3D点对应的2D特征初始化概率深度滤波器，当深度滤波器的不确定性足够小（收敛）时，在地图中插入相应的3D点（更新地图），并用来估计位姿。

  * 第一步是从图像帧队列中读入图像帧，并检测图像帧是否是关键帧。
  * 如果该帧图像是关键帧的话，就对图像进行特征提取，并对每个2D特征**初始化概率深度滤波器**，这些滤波器的初始化有很大的不确定性，在随后的图像帧中深度估计都将以贝叶斯方式更新。
  * 如果该帧图像不是关键帧的话，就用来更新深度滤波器，当深度滤波器的不确定性足够小时（收敛时），在地图中插入相应的3D点（更新地图），并用来估计位姿。



## 二、SVO 算法原理

























## 三、编译调试







