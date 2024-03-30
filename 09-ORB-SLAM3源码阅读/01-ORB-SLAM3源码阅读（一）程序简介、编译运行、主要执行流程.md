> 原始Markdown文档、Visio流程图、XMind思维导图：https://github.com/LiZhengXiao99/Navigation-Learning

[TOC]

## 一、程序简介

### 1、ORB-SLAM 概述

ORB 指 **O**riented FAST and **r**otated **B**RIEF，是一种结合 FAST 和 BRIEF，并引入旋转不变性的一种特征点和描述子；SLAM 指 **S**imultaneous **L**ocalization **a**nd **M**apping，指的是同时进行实时定位和地图构建。

ORB-SLAM3 是**迄今为止，最完整的视觉惯性 SLAM 系统系统**，它是第一个集成了单目相机、双目相机、RGB-D相机，以及单目相机结合 IMU、双目相机结合 IMU 的 SLAM 系统。并且在 ORB-SLAM2 的基础上，改进了相机模型，使其不再局限于传统的小孔成像模型，而是可以**扩展到鱼眼模型**。在与 IMU 的结合上，它根据运动模型在流形上进行 **IMU 的预积分**的方式，然后采用非线性优化的思想，**将 IMU 的预积分结果和视觉 SLAM 的重投影模型一同进行图优化，使得预积分残差以及重投影误差共同达到最小**，以此来完成视觉信息和惯导系统的**紧耦合**。并且它采用了更为快速的**初始化**方法，以及丢失跟踪后利用惯导系统快速**重定位**方法。此外，它还采用**地图集**的方式，实现了对大场景的定位建图。这也是如今众多开源方案中，功能最强大、最精准的方法。系统框图如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815102741960.png" alt="image-20230815102741960" style="zoom: 50%;" />

### 2、ORB-SLAM3 历史与演变

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690952265203.png" alt="1690952265203" style="zoom: 33%;" />

### 3、代码分析

源码：https://github.com/UZ-SLAMLab/ORB_SLAM3

文件结构如下：

* Examples 和Exampleold 根据传感器类型，分别存放新的和旧的代码实例。
* include 和 src 分别存放代码的 .h 头文件和 cc/cpp 原文件。
* Thirdparty 存放了 DBOW2、Sophus 和 g2o。
  * DBOW2 是词袋模型，推荐博客：[DBoW2库介绍](https://www.cnblogs.com/luyb/p/6033196.html)
  * Sophus 是李代数库，
  * g2o 是图优化库，
* Vocabulary 存放 ORB 词典。





### 4、ORB-SLAM 论文

* Parallel Tracking and Mapping for Small AR Workspaces，下载

  > **摘要翻译**：
  >
  > * 本论文提出了一种在未知场景下估计相机位姿的方法。
  > * 尽管之前已经有了很多将 SLAM 应用于机器人的尝试，我们
  > * 我们将跟踪和建图分成两个单独的任务， 在双核计算机上以并行线程处理：
  >   * 跟踪线程
  >   * 建图线程根据之前观察到的视频帧生成点特征的三维地图。
  > * 这样就可以使用计算量大的批处理优化技术，对实时性要求没那么高。
  > * 该系统可绘制出包含数千个地标的详细地图，并可
  >   以帧速率进行跟踪，其准确性和鲁棒性可与最先进的基于模型的系统相媲美。
  >   先进的基于模型的系统
  >
  > 

* ORB-SLAM: a Versatile and Accurate Monocular SLAM System，下载

  > **摘要翻译**：
  >
  > 

* ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras，下载

  > **摘要翻译**：
  >
  > 

* ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM，下载

  > **摘要翻译**：
  >
  > 











## 三、编译使用









## 三、主要执行流程

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815102741960.png" alt="image-20230815102741960" style="zoom: 67%;" />

### 1、线程设计

* **Tracking 线程**：接受一个关键帧，计算相机位姿，决定何时给局部建图插入一个关键帧，生成地图点；作为主线程一直执行。

  在追踪线程，根据传感器的不同需要，选择不同的初始化方式。首先，对图像进行 ORB 特征提取与匹配，由于传统ORB 算法提取的特征点分布不均匀且误匹配率过高，因此本文基于 ORB-SLAM3 算法对特征匹配环节进行改进。首先在特征点提取过程利用四叉树策略将图像分为若干个网格,分别在每个网格中提取最佳特征点，然后在特征匹配阶段引入GMS 匹配方法，使用基于网格剔除误匹配的统计量筛选正确匹配，从而得到分布比较均匀且正确的匹配对；其次，利用恒速运动跟踪模型和参考帧跟踪模型计算初始位姿，两个模型分别用于正常跟踪模式与跟踪失败后的重定位；然后，跟踪局部地图以确定更多局部地图点与当前帧图像的特征匹配关系，进而基于图优化模型进一步优化相机位姿；最后，根据关键帧选取条件确定新关键帧。此外，IMU 模式中的跟踪线程还需要计算 IMU 预积分，从而使局部建图线程在视觉与IMU 结合后进行局部 BA 优化。

* **Localmapping 线程**：操作局部图，进行局部 BA

  局部建图线程的流程主要包含两方面，首先是对局部地图进行维护，即将新关键帧和地图点插入活动地图，同时剔除不满足条件的关键帧和地图点，然后进行局部 BA 进一步优化关键帧位姿和地图点空间坐标。因为 ORB-SLAM3 的传感器类型包含 IMU，所以该算法的局部地图构建线程还包括IMU 的初始化，该过程的目的是为了给局部 BA 和全局 BA提供一个更好的初始值以减少 IMU 噪声积累。

* **LoopClosing 线程**：对关键帧处理，执行图优化，做全局 BA

  回环检测线程主要包括回环检测和回环校正。首先在关键帧数据库中确认候选闭环帧，并与当前关键帧进行特征匹配，然后通过 Sim3 变换计算相似变换，其次当前帧的共视帧和候选闭环帧进行投影匹配，当匹配数满足相应条件时则可检测到回环，最后校正关键帧位姿和局部地图点三维坐标，并融合地图点和优化本质图。回环校正完成后 ORB-SLAM3 系统通过在独立线程中启动完整的 BA 细化地图映射，并且可保障实时性不受影响。

* **Viewer 线程**：可视化



### 2、主要执行流程

程序有很多的主文件，在 Examples 文件夹中，从网上找了张单目融合IMU的主文件流程图（Mono_inertial_tum_vi.cc）

![Mono_inertial_tum_vi.cc流程](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/Mono_inertial_tum_vi.cc%E6%B5%81%E7%A8%8B.png)



```c++
int main(int argc, char **argv)
{
    // 输出运行的序列数目
    const int num_seq = (argc-3)/3;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= ((argc % 3) == 1);

    string file_name;
    if (bFileName)
        file_name = string(argv[argc-1]);

    cout << "file name: " << file_name << endl;

    // 按照下面提示至少输入6个参数
    if(argc < 6)
    {
        cerr << endl << "Usage: ./mono_inertial_tum_vi path_to_vocabulary path_to_settings path_to_image_folder_1 path_to_times_file_1 path_to_imu_data_1 (path_to_image_folder_2 path_to_times_file_2 path_to_imu_data_2 ... path_to_image_folder_N path_to_times_file_N path_to_imu_data_N) (trajectory_file_name)" << endl;
        return 1;
    }


    // Load all sequences:
    // 准备加载所有序列的数据
    int seq;
    vector< vector<string> > vstrImageFilenames;    //图像文件名
    vector< vector<double> > vTimestampsCam;        //图像时间戳
    vector< vector<cv::Point3f> > vAcc, vGyro;      //加速度计，陀螺仪
    vector< vector<double> > vTimestampsImu;        //IMU时间戳
    vector<int> nImages;                            
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);

    int tot_images = 0;
    // 遍历每个序列
    for (seq = 0; seq<num_seq; seq++)
    {
        // Step 1 加载图像名和对应的图像时间戳
        cout << "Loading images for sequence " << seq << "...";
        LoadImages(string(argv[3*(seq+1)]), string(argv[3*(seq+1)+1]), vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        // Step 2 加载IMU数据
        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(string(argv[3*(seq+1)+2]), vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        //检查是否存在有效数目的图像和imu数据
        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first
        // Step 3 默认IMU数据早于图像数据记录，找到和第一帧图像时间戳最接近的imu时间戳索引，记录在first_imu[seq]中
        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0]){
            first_imu[seq]++;
            cout << "first_imu[seq] = "  << first_imu[seq] << endl;
        }
        // 因为上面退出while循环时IMU时间戳刚刚超过图像时间戳，所以这里需要再减一个索引    
        first_imu[seq]--; // first imu measurement to be considered

    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout << endl << "-------" << endl;
    cout.precision(17);

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // Step 4 SLAM系统的初始化，包括读取配置文件、字典，创建跟踪、局部建图、闭环、显示线程
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true, 0, file_name);

    //遍历所有数据
    int proccIm = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Main loop
        cv::Mat im;
        //存放imu数据容器,包含该加速度,角速度,时间戳
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        //直方图均衡化,直方图均衡化的思想就是这样的:
        //假设我有灰度级255的图像，但是都是属于［100，110］的灰度，图像对比度就很低，我应该尽可能拉到整个［0，255］
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            // Step 5 读取每一帧图像并转换为灰度图存储在im,seq表示第几个数据集,ni表示这个数据集的第几个数据
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_GRAYSCALE);

            // clahe
            //直方图均衡化
            clahe->apply(im,im);


            // 取出对应的图像时间戳
            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }


            // Load imu measurements from previous frame
            //清空imu测量
            vImuMeas.clear();

            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;
                // Step 6 把上一图像帧和当前图像帧之间的imu信息存储在vImuMeas里
                // 注意第一个图像帧没有对应的imu数据 //?是否存在一帧,因为之前是从最接近图像第一帧的imu算起,可能无效
                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    // cout << "t_imu = " << fixed << vImuMeas.back().t << endl;
                    first_imu[seq]++;
                }
            }

            // cout << "first imu: " << first_imu[seq] << endl;
            /*cout << "first imu time: " << fixed << vTimestampsImu[first_imu] << endl;
            cout << "size vImu: " << vImuMeas.size() << endl;*/
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
            // Step 7 跟踪线程作为主线程运行
            SLAM.TrackMonocular(im,tframe,vImuMeas); // TODO change to monocular_inertial

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            // 等待读取下一帧
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6

        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;
            // Step 8 更换数据集 
            SLAM.ChangeDataset();
        }

    }

    // cout << "ttrack_tot = " << ttrack_tot << std::endl;
    // Stop all threads
    // Step 9 关闭SLAM中所有线程
    SLAM.Shutdown();


    // Tracking time statistics

    // Save camera trajectory
    // Step 10 保存相机位姿（轨迹）
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages[0]; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages[0]/2] << endl;
    cout << "mean tracking time: " << totaltime/proccIm << endl;

    /*const string kf_file =  "kf_" + ss.str() + ".txt";
    const string f_file =  "f_" + ss.str() + ".txt";

    SLAM.SaveTrajectoryEuRoC(f_file);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);*/

    return 0;
}
```













