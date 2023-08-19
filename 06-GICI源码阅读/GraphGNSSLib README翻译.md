[TOC]

# GraphGNSSLib

## 一个使用因子图优化的GNSS定位开源程序包

此仓库是开源软件包的实现，GraphGNSSLib 利用因子图优化（FGO）来执行GNSS定位和实时动态（RTK）定位.在这个软件包中，历史和当前时期的测量结果被结构化为因子图，然后通过非线性优化进行求解。该软件包基于C++，与机器人操作系统（ROS）平台兼容。同时，该软件包结合了RTKLIB (**[version: 2.4.3 b33](http://www.rtklib.com/)**) 去读取解析 [RINEX](https://en.wikipedia.org/wiki/RINEX) 文件. 机器人领域的用户可以很容易地访问全球导航卫星系统的原始数据，以便进一步研究。

**重要提示**：

- **GNSS Positioning**方法基于FGO的伪距和多普勒测量的组合来估计全球导航卫星卫星系统接收器的位置。
-  **GNSS-RTK Positioning**方法基于双差伪距、载波相位和使用FGO的多普勒测量的组合来估计GNSS接收器的浮点解。最后，使用LAMBDA算法模糊性固定。

**作者**: [Weisong Wen](https://weisongwen.wixsite.com/weisongwen), [Li-ta Hsu](https://www.polyu-ipn-lab.com/) from the [Intelligent Positioning and Navigation Laboratory](https://www.polyu-ipn-lab.com/), The Hong Kong Polytechnic University. 

**相关论文：** (代码中的算法并不和论文完全一致)

- Wen, W., & Hsu, L. T. (2021, May). [Towards robust GNSS positioning and Real-time kinematic using factor graph optimization](https://ieeexplore.ieee.org/abstract/document/9562037). In 2021 IEEE International Conference on Robotics and Automation (ICRA) (pp. 5884-5890). IEEE. 
- Wen, W., Zhang, G., & Hsu, L. T. (2021). [GNSS outlier mitigation via graduated non-convexity factor graph optimization](https://ieeexplore.ieee.org/abstract/document/9627801). IEEE Transactions on Vehicular Technology, 71(1), 297-310.
- Zhong, Y., Wen, W., Ng, H. F., Bai, X., & Hsu, L. T. (2022, September). [Real-time Factor Graph Optimization Aided by Graduated Non-convexity Based Outlier Mitigation for Smartphone Decimeter Challenge](https://www.ion.org/publications/abstract.cfm?articleID=18382). In Proceedings of the 35th International Technical Meeting of the Satellite Division of The Institute of Navigation (ION GNSS+ 2022) (pp. 2339-2348).

*如果你用GraphGNSSLib做学术研究，请引用我们的论文 [论文](https://ieeexplore.ieee.org/abstract/document/9562037)*


  ![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/6d3f11d9293c4bb2976c46f767efe303.png)

<center> GraphGNSSLib执行流程, 更多信息请参考manual和论文.</center>

## 0. Docker 支持

如果你不熟悉ROS，我们强烈推荐推荐你用我们的Docker容器来使用GraphGNSSLib，具体见下文

## 1. 前置条件

### 1.1 **Ubuntu** and **ROS**

Ubuntu 64-bit 16.04, ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation). 我们只在 Ubuntu 16.04 with ROS Kinetic环境中测试过。

### 1.2. **Ceres Solver**

请按照下面的步骤安装Ceres-solver，而不是用最新版的。

**Step 1**: 下载与GraphGNSSLib兼容的 [Ceres-solver](https://github.com/weisongwen/GraphGNSSLib/tree/master/support_files)

**Step 2**: make and install

```bash
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# make Ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
sudo make -j4
sudo make test
sudo make install
```

### 1.3. **扩展库**

```bash
sudo apt-get install ros-kinetic-novatel-msgs
```

## 2. Build GraphGNSSLib

克隆这个仓库和catkin_make:

```bash
mkdir GraphGNSSLib/src
cd ~/GraphGNSSLib/src
mkdir result
git clone https://github.com/weisongwen/GraphGNSSLib.git
cd ../
# if you fail in the last catkin_make, please source and catkin_make again
catkin_make
source ~/GraphGNSSLib/devel/setup.bash
catkin_make
```

(**如果你这一步失败了，尝试去用其它的有纯净操作系统的电脑或重新安装Ubuntu和ROS**)

## 3. 用 [UrbanNav](https://www.polyu-ipn-lab.com/download)数据集执行FGO的GNSS定位

在香港TST附件采集的动态数据验证通过FGO的GNSS定位

- GPS second span: **46701** to **47185**

- 卫星系统：**GPS/BeiDou**

- 窗口大小: **Batch**

- 使用的测量值: 双差伪距、载波相位、多普勒

- 默认结果文件

  ```c++
  ~/GraphGNSSLib/trajectory_psr_dop_fusion.csv
  ```

请确保在rtklib.h中启用以下内容：

```bash
#define RTK_FGO 0
```

- Solution 1 to run the GNSS positioning Demo

  ```bash
  source ~/GraphGNSSLib/devel/setup.bash
  # read GNSS raw data and publish as ROS topic
  # we provide several datasets, enjoy it!
  roslaunch global_fusion dataublox_TST20190428.launch
  # run pseudorange and doppler fusion
  roslaunch global_fusion psr_doppler_fusion.launch
  ```

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/050bf95f4dd94106a05f0c02db0dde81.png)

  <center> 在整个测试过程中，三种方法的轨迹（红色曲线为使用WLS和的GNSS定位、绿色曲线为使用EKF的GNSS定位、蓝色曲线为使用FGO和的GNSS定位。x轴和y轴分别表示东向和北向</center>

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/c06db7c9b1b44d9ab741207b2d535558.png)

<center> 智能手机收集的TST数据：WLS为红点，FGO为紫色曲线</center>

## 4. GNSS RTK-FGO处理静态数据集

在香港TST附件采集的静态数据验证通过FGO的GNSS定位

- GPS时间跨度: **270149** to **270306**

- 卫星系统: **GPS/BeiDou**

- 窗口大小: **Batch**

- 使用的测量值: 双差伪距、载波相位、多普勒

- 默认结果文件

  ```c++
  ~/GraphGNSSLib/FGO_trajectoryllh_pdrtk.csv
  ```

请确保在rtklib.h中启用以下内容：

```bash
#define RTK_FGO 1
```

- Solution 1 to run the RTK-FGO Demo

  ```bash
  source ~/GraphGNSSLib/devel/setup.bash
  # read GNSS raw data and publish as ROS topic
  roslaunch global_fusion dataublox_TST20200603.launch
  # run GNSS RTK
  roslaunch global_fusion psr_doppler_car_rtk.launch
  ```

![在这里插入图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/3407ae9876c64ba58de14280a092d584.png)



  <center> 红点为RTK-EKF 、蓝点为RTK-FGO；x轴和y轴分别表示东向和北向。</center>

## 5. Docker 支持

想用docker运行GraphGNSSLib，请先确保 [docker](https://docs.docker.com/install/linux/docker-ce/ubuntu/)已经安装在你的电脑上。如果你想用docker运行global_fusion：

```bash
cd ~/catkin_ws/src/GraphGNSSLib/docker
make build
sudo -E ./start.bash #Do not delete " -E "
source devel/setup.bash
# run pseudorange and doppler fusion
roslaunch global_fusion psr_doppler_fusion.launch
# you should open another ternimal to enter the docker.
# read GNSS raw data and publish as ROS topic
roslaunch global_fusion dataublox_TST20190428.launch
```

除此以外，这个[视频](https://www.youtube.com/watch?v=WMM2de_SxTw)演示了如何用建立GraphGNSSLib的docker文件

如果你想重启容器，请先将他关闭：

```bash
sudo ./stop.bash
#then restart it
sudo -E ./start.bash 
```

 目录 ~/shared_dir 用来连接容器和主机。在容器中位于 ~/graph1/shared_dir ，你可以下载代码到 ~/shared_dir ，并且在容器中编译（推荐给那些有兴趣更改源代码的人）。

## 6. 致谢

我们用 [Ceres-solver](http://ceres-solver.org/)实现非线性优化和 [RTKLIB](http://www.rtklib.com/)解析GNSS数据，有些函数源自 [VINS-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)。[rviz_satellite](https://github.com/nobleo/rviz_satellite)用于可视化。如果有做的不合适的地方，请与我联系：17902061r@connect.polyu.hk ([Weisong WEN](https://weisongwen.wixsite.com/weisongwen))，非常感谢Mr. Zhong Yihan的维护

## 7. License

源代码基于[GPLv3](http://www.gnu.org/licenses/)许可，我们依然在为提高代码可靠性工作. 有任何技术问题，可联系Weisong Wen<17902061r@connect.polyu.hk> 。有任何商业问题，可联系Li-ta Hsu <lt.hsu@polyu.edu.hk>.