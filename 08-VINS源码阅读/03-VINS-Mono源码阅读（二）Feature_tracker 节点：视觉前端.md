[TOC]

## 一、Feature_tracker 节点

feature_tracker 是 vins 的前端，主要作用是跟踪特征并三角化恢复深度以后发给后端去优化，它的目录在 src/ feature_tracker 下，功能主要是获取摄像头的图像帧，并按照事先设定的频率，把当前帧上满足要求的特征点以 sensor_msg::PointCloudPtr的格式发布出去，以便 RVIZ 和 estimator 节点接收。主要代码在三个源程序文件：

* feature_tracker_node 是特征跟踪线程的系统入口。
* feature_tracker 是特征跟踪算法的具体实现。
* parameters 是设备等参数的读取和存放。

![feature_tracker 类](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/feature_tracker%20%E7%B1%BB.png)



![feature_tracker_node 流程图](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/feature_tracker_node%20%E6%B5%81%E7%A8%8B%E5%9B%BE.png)

我也画了一个流程图：

![image-20230823133849092](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230823133849092.png)



### 2、feature_tracker 输入输出

![image-20230820152338754](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230820152338754.png)

* **输入**：图像，即订阅了传感器或者 rosbag 发布的topic：“/cam0/image_raw”

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


### 3、feature_tracker 主函数

主函数先初始化 ROS 节点、读取配置文件、相机内参，订阅和发布 ROS 消息，然后 `ros::spin();` 进入 spin 状态，不断循环。接收到图像数据后，会调用回调函数 `img_callback()` 进行处理。

```c++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");   // ros 节点初始化
    ros::NodeHandle n("~");     // 声明一个句柄，~ 代表这个节点的命名空间
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);    // 设置 ros log 级别
    readParameters(n);  // 读取配置文件

    for (int i = 0; i < NUM_OF_CAM; i++)    // 循环获取每个相机的内参
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]); 

    if(FISHEYE) // 如果是鱼眼相机
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // 向 roscore 注册订阅这个 topic，收到一次 message 就执行一次回调函数
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    // 注册一些 publisher
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000);
    /*
    if (SHOW_TRACK)
        cv::namedWindow("vis", cv::WINDOW_NORMAL);
    */
    ros::spin();    // spin 代表这个节点开始循环查询 topic 是否接收
    return 0;
}
```

### 4、img_callback()：图像处理回调函数

![前端光流追踪](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/%E5%89%8D%E7%AB%AF%E5%85%89%E6%B5%81%E8%BF%BD%E8%B8%AA.png)

收到一次原始图像消息，就执行一次此，进行图像处理。

* 先判断是否是第一帧图像，如果是，设置标志位、记录为起止图像时间戳，直接返回。

```c++

```

* 检查当前时间戳是否正常，这里默认超过 1s 或者当前时间戳早于上一帧就异常。VINS 中没有使用像 ORB-SLAM 中那样的描述子匹配，所以对时间戳要求就高，图像时间差太多光流追踪就会失效。如果时间戳不正常，发布 restart_flag 告诉其它模块（Estimater）要重启了。

```c++

```

* 控制发给后端的频率，保证不超过 FREQ。帧数量除以当前时间戳减去第一帧时间戳，得到发送的频率，发送频率过大就不向后端发送，但还要做光流追踪，保证连续性。频率和阈值频率很接近，就重启一下，避免 delta 太大，起始时间设为当前时间，发出帧数清零。

```c++

```

* 调用 toCvCopy() 把图像从 ros message 转成 cv::Mat，以便用 openCV 处理。

```c++

```

* 调用 readImage() 图像均衡化预处理、光流追踪、提取特征点（如果发布）、特征点去畸变、计算速度。

```c++

```

* 循环，把新特征点赋上 id。

```c++

```

* 如果可以发布给后端，ros 消息进行格式化，发布数据给后端。



### 5、readImage()：均衡化、光流追踪、特征提取、去畸变、计算速度

* 如果需要均衡化 EQUALIZE，调用 createCLAHE() 创建自适应直方图均衡化（CLAHE）对象，调用 apply() 均衡化。

  

* cur_img 和 forw_img 分别是光流跟踪的前后两帧，forw_img 才是真正的当前帧，cur_img 实际上是上一帧，prev_img 是上一次发布的帧。prev_img 的用处是：光流跟踪后用 prev_img 和 forw_img 根据本质矩阵做 RANSAC 剔除 outlier，也就是rejectWithF()函数。





### 6、undistortedPoints()：去畸变、计算特征点速度







## 二、自适应直方图均衡化

### 1、原理

图像太亮或太暗，提取特征点比较难，所以需要均衡化一下，提升对比度。





### 2、cv::createCLAHE()：创建自适应直方图均衡化（CLAHE）对象





### 3、cv::apply()：均衡化







## 三、LK 光流追踪、特征提取

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

### 2、cv::calcOpticalFlowPyrLK()：LK 光流追踪



### 3、reduceVector()：根据状态位矩阵瘦身

实现了两套，分别用于 `vector<cv::Point2f>`、`vector<int>`

```c++
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
```

```c++
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}
```

### 4、rejectWithF()：根据本质矩阵做 RANSAC 剔除跟踪错误

rejectWithF() 主要是利用了 cv::findFundamentalMat() 这个函数来进一步剔除outlier。这个函数的功能对应在《SLAM十四讲》第七讲 2D-2D 对极几何相关知识点，两帧上的一系列对应的特征点能够复原出两帧之间的相对位姿变化，也就是基础矩阵 E。但是这些特征点中肯定会有一些outlier，所以通过这个 opencv 的函数，能够巧妙地剔除这些 outlier。

```c++
void FeatureTracker::rejectWithF()
{
    // 保证当前追踪到的光流至少是 8 个点
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // 得到归一化的坐标值
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            
            // 虚拟相机焦距、虚拟尺寸
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        // 调用 findFundamentalMat() 计算本质矩阵，也是一种对极约束的粗差剔除
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();

        // 调用 reduceVector() 根据状态位瘦身
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}
```

### 5、cv::findFundamentalMat()

```c++
cv::Mat cv::findFundamentalMat(const cv::Mat& points1, const cv::Mat& points2, double* mask=0, int method=cv::FM_RANSAC, double param1=10.0, double param2=0.99, int minInliers=10)
```

参数说明如下：

- `points1`：第一个视图中的3D点（每行一个点）。
- `points2`：第二个视图中的3D点（每行一个点）。
- `mask`：输出掩码。如果mask为NULL，则不计算掩码。
- `method`：计算方法，有7种方法，分别是：FM_7POINTS、FM_8POINTS、FM_LMEDS、FM_RANSAC、FM_LAGRANGES、FM_EPNP、FM_EPnP_P3P、FM_EPnP。
- `param1`：RANSAC方法中的第一个参数。
- `param2`：RANSAC方法中的第二个参数。
- `minInliers`：RANSAC方法中的最小内点数。

补充一点，如果 cv::findFundamentalMat() 函数传入的是归一化坐标，那么得到的是本质矩阵 E，如果传入的是像素坐标，那么得到的是基础矩阵。

**基础矩阵**（Fundamental matrix）是三维空间中的一种变换矩阵，它描述了两个相机之间的相对位置和方向。在计算机视觉中，基础矩阵通常用于对齐两个图像，或者计算图像之间的对应点。基础矩阵的维度是3 x 3，它描述了从第一个相机坐标系到第二个相机坐标系的变换。

**本质矩阵**（Essential matrix）也是描述两个相机之间相对位置和方向的矩阵，但它只包含了旋转和平移信息，不包含缩放信息。因此，本质矩阵的维度是3 x 3。在计算机视觉中，本质矩阵通常用于计算两幅图像之间的旋转和平移信息，或者求解相机的内部参数。

基础矩阵和本质矩阵之间可以通过一些代数运算进行转换。例如，可以通过奇异值分解将基础矩阵分解为本质矩阵和单应性矩阵的乘积。

### 6、setMask()：对跟踪点进行排序并去除密集点

在跟踪过程中，为了保持跟踪到的特征点在当前帧图像中均匀分布（避免特征点扎堆的现象），会调用 FeatureTracker 类中的 FeatureTracker:;setMask() 函数，先对跟踪到的特征点 forw_pts 按照跟踪次数降序排列（认为特征点被跟踪到的次数越多越好），然后遍历这个降序排列，对于遍历的每一个特征点，在 mask 中将该点周围半径为 MIN_DIST 的区域设置为 0，在后续的遍历过程中，不再选择该区域内的点。

```c++
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // 倾向于保留追踪时间更长的特征点
    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    // 利用光流特点，追踪多稳定性好的，放前面，规则用 lambda 表达式
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {

        if (mask.at<uchar>(it.second.first) == 255)
        {
            // 把挑选剩下的迭代器重新放入容器
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // circle 把周围一个圈内全部置 0，这个区域不允许别的特征点存在，避免特征点太集中
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}
```

### 6、cv::goodFeaturesToTrack()：特征提取

由于跟踪过程中，上一帧特征点由于各种原因无法被跟踪，而且为了保证特征点均匀分布而剔除了一些特征点，如果不补充新的特征点，那么每一帧中特征点的数量会越来越少。所以，当前帧除了跟踪前一帧中的特征点，还会调用 cv::goodFeaturesToTrack() 在 mask 中不为 0 的区域提取新的特征点。







## 四、去畸变

### 1、去畸变原理

由于添加镜头，会改变光线的传播路径，导致投影成像的位置与实际的像坐标不一致。畸变一般分为径向畸变和切向畸变，**畸变的程度越靠近图像边缘越高**



### 2、针孔相机（PinholeCamera）去畸变







### 3、鱼眼相机（CataCamera）去畸变



























