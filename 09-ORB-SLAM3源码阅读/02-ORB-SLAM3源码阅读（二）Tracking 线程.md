[TOC]

## 一、Tracking 线程作用

跟踪线程主要负责定位传感器并确定何时插入新的关键帧，主要由 ORB 特征提取、初始位姿估计、局部地图跟踪和关键帧选取 4 部分组成。

### 1、ORB 特征提取

由 FAST( features fromaccelerated segment test) 算法改到 ORB 利用 FAST 算法对插入的新图像帧进行 FAST 角点搜索，即搜索整个图像帧中所有与其周围邻域内足够多的像素点的灰度值相差较大的像素点。

![image-20230815110123866](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815110123866.png)

上图所示为图像帧中某一个角点和周围邻域内的像素点，给定比较的阈值以及圆圈大小(本文以圆圈大小 16为例)。 首先比较像素点 1 和 9，如果其和中心像素的灰度值差的绝对值均小于阈值，则该点不是角点，否则将它作为候选角点；再比较像素点 1、5、9 和 13与中心像素点灰度值差的绝对值，如果其中有3个绝对值大于阈值，则该点继续作为候点，否则舍弃；最后比较像素点 1 ~ 16 与中心像素的灰度值差，如果连续大于或小于阈值的像素点超过 8 个，则该点为特征点，否则舍弃。 由于此时搜索出的角点过多，因此本文对检测出来的角点群利用非极大值抑制方法进行 FAST 角点的筛选，并在保留的 FAST 角点上计算方向，以此来实现特征点的旋转不变性，最后计算当前图像帧中角点的 BRIEF( binary robustindependent elementary features)描述子用于与上一帧中的角点的 BRIEF 描述子进行特征匹配。

### 2、初始位姿估计

首先判断当前帧与上一帧的特征匹配是否成功，若成功则假设传感器为恒速运动模型来预测传感器当前位姿，并搜索与上一帧观察到的地图点进行对应，最后与对应的地图点进行位姿优化。 若没有足够的位姿与地图点的匹配对，则使用上一帧中更广泛的位姿附近的地图点的搜索范围。 若当前帧与上一帧的特征匹配失败，则需要全局重定位，首先将当前帧转化为词袋向量，利用 ORB 特征词典与所有关键帧进行匹配，然后对每个关键帧进行随机抽样一致性( random sample consensus，RANSAC)迭代，并使用 PnP 算法进行传感器位姿求解。 若传感器位姿有足够多可被使用的正确数据，则搜索与所有关键帧观察到的地图点进行对应，通过匹配对进行位姿优化。

### 3、局部地图跟踪

首先更新局部关键帧和局部地图点,将局部地图点与当前帧进行匹配,最后使用最小化重投影误差方法实现对传感器位姿的进一步优化。

### 4、关键帧选取

通过舍弃不满足条件的关键帧，保留满足以下条件的关键帧,保证系统具有鲁棒性。

* 自上次全局重定位后,必须已经过大于 20 帧。
* 地图构建部分处于空闲状态，或自上次插入关键帧后已经过大于 20 帧。
* 当前帧至少跟踪到 50 个点。
* 当前帧比与当前帧共享最多地图点的帧跟踪到少于 90% 的点。





## 三、初始化

运行 ORB-SLAM3 的第一步是初始化，从 System 的构造函数开始，

1. 检查 SLAM 系统**传感器初始化类型**，需要用到 **sensor** ;
2. 检查传感器配置文件，也就是**相机标定参数**等，需要用到 **strSettingsFile**；
3. 实例化 **ORB 词典**对象，并加载 ORB 词典；
4. 根据 ORB 词典实例对象，创建**关键帧 DataBase**；
5. 创建 Altas，也就是**多地图系统**；
6. **IMU 初始化**设置， 如果需要。
7. 利用 Atlas 创建 **Drawer**，包括 **FrameDrawer** 和 **MapDrawer**;
8. 初始化 **Tracking** 线程 ;
9. 初始化局部地图 **LocalMapping** 线程，并加载；
10. 初始化闭环 **LoopClosing** 线程，并加载；
11. 将上述实例化的**对象指针**，传入需要用到的线程，方便进行**数据共享**。
12. 初始化**用户可视化线程**，并加载；

```c++
/// @brief 构造函数，ORB-SLAM3 初始化，将会启动其它线程
/// @param strVocFile 词袋文件所在路径
/// @param strSettingsFile 配置文件所在路径
/// @param sensor 传感器类型（6种：单目、双目、RGB-D，和与惯导的结合）
/// @param bUseViewer 是否使用可视化界面（可以不使用界面，只生成轨迹文件）
/// @param initFr 表示初始化帧的id,开始 设置为 0
/// @param strSequence 序列名，在跟踪线程和局部建图线程用得到
System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const string &strSequence):
    mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    // 检查 SLAM 系统传感器初始化类型，需要用到 sensor
    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;
    else if(mSensor==IMU_MONOCULAR)
        cout << "Monocular-Inertial" << endl;
    else if(mSensor==IMU_STEREO)
        cout << "Stereo-Inertial" << endl;
    else if(mSensor==IMU_RGBD)
        cout << "RGB-D-Inertial" << endl;

    // 检查传感器配置文件，也就是相机标定参数等，需要用到 strSettingsFile；
    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);

        mStrLoadAtlasFromFile = settings_->atlasLoadFile();
        mStrSaveAtlasToFile = settings_->atlasSaveFile();

        cout << (*settings_) << endl;
    }
    else{
        settings_ = nullptr;
        cv::FileNode node = fsSettings["System.LoadAtlasFromFile"];
        if(!node.empty() && node.isString())
        {
            mStrLoadAtlasFromFile = (string)node;
        }

        node = fsSettings["System.SaveAtlasToFile"];
        if(!node.empty() && node.isString())
        {
            mStrSaveAtlasToFile = (string)node;
        }
    }

    node = fsSettings["loopClosing"];
    bool activeLC = true;
    if(!node.empty())
    {
        activeLC = static_cast<int>(fsSettings["loopClosing"]) != 0;
    }

    mStrVocabularyFilePath = strVocFile;

    bool loadedAtlas = false;

    if(mStrLoadAtlasFromFile.empty())
    {   
        // 实例化 ORB 词典对象，并加载 ORB 词典；
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        // 根据 ORB 词典实例对象，创建关键帧 DataBase
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        // 创建 Altas，也就是多地图系统
        //Create the Atlas
        cout << "Initialization of Atlas from scratch " << endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        cout << "Load File" << endl;

        // Load the file with an earlier session
        //clock_t start = clock();
        cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            cout << "Error to load the file, please try with other session file or vocabulary file" << endl;
            exit(-1);
        }
        //mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);


        //cout << "KF in DB: " << mpKeyFrameDatabase->mnNumKFs << "; words: " << mpKeyFrameDatabase->mnNumWords << endl;

        loadedAtlas = true;

        mpAtlas->CreateNewMap();

        //clock_t timeElapsed = clock() - start;
        //unsigned msElapsed = timeElapsed / (CLOCKS_PER_SEC / 1000);
        //cout << "Binary file read in " << msElapsed << " ms" << endl;

        //usleep(10*1000*1000);
    }

    // IMU 初始化设置, 如果需要
    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    // 利用 Atlas 创建 Drawer，包括 FrameDrawer 和 MapDrawer;
    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    // 初始化 Tracking 线程
    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    cout << "Seq. Name: " << strSequence << endl;
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_, strSequence);

    // 初始化局部地图 LocalMapping 线程，并加载
    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(this, mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, strSequence);
    mptLocalMapping = new thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mInitFr = initFr;
    if(settings_)
        mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    else
        mpLocalMapper->mThFarPoints = fsSettings["thFarPoints"];
    if(mpLocalMapper->mThFarPoints!=0)
    {
        cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    // 初始化闭环 LoopClosing 线程，并加载
    //Initialize the Loop Closing thread and launch
    // mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR, activeLC); // mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    // 将上述实例化的对象指针，传入需要用到的线程，方便进行数据共享
    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //usleep(10*1000*1000);

    // 初始化用户可视化线程，并加载
    //Initialize the Viewer thread and launch
    if(bUseViewer)
    //if(false) // TODO
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpLoopCloser->mpViewer = mpViewer;
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}
```



## 四、传感器输入

也从 System.h/cc 文件内的函数开始：

- 单目、单目-惯性：TrackMonocular()
- 双目、双目-惯性：TrackStereo()
- RGB-D、RGB-D-惯性：TrackRGBD()

**主要完成两件事**：

- 根据输入数据，生成一个关键帧
- 根据关键帧信息，调用 Track 的 GrabImage Monocular/Stereo/RGBD，进行帧间位姿估计

**通用步骤**：

1. 检查SLAM系统**传感器类型**；
2. 检查是否打开或关闭**定位模式状态**；
3. 检查**重置状态**，即重置Track线程或者Activate Map ;
4. 如果使用**IMU传感器**，需要加载IMU数据；
5. Track 线程计算**当前帧位姿参数**；
6. 更新状态参数和数据。



## 五、帧构建

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-22b62ea6bfe4880a5eadce19e421493b_b.png)







## 六、track



![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-fc30930e37d0479b1bc258816c20e0c1_b.png)









