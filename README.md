![Navigation-Learning-cover](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/Navigation-Learning-cover.png)

[TOC]

---

* 本仓库是我学习的记录，会长期更新，分享出来，希望有人用得上，也激励着自己坚持学下去。

- 有些程序会详细写（比如 RTKLIB、KF-GINS），有些只画个流程图、列举重点的函数。
- 水平不高，理解尚浅，列举的理论公式无法保证准确性，仅供参考，千万不要照着我的笔记来写代码写论文，更不要直接复制到论文里，希望小伙伴们注意。
- 不建议直接下载整个仓库，内容可能大部分都用不着，Markdown 和 PDF 可以在线看，想要的文件也可以[单独下载](https://zhuanlan.zhihu.com/p/578116206)。Github 上的 Markdown 公式可能显示不完全，可以下载下来用 [Typora](https://typoraio.cn/) 看。
- 本仓库所有内容都可以随意转载，可用于任何目的，不必征求我的意见。
- 有问题欢迎随时联系我：**电话** (15255291038)、**微信** (lizhengxiao99)、**QQ** (1482275402)、**Email** (dauger@126.com)。


---

记录一些我看到过的项目，其中加粗的项目会在本仓库介绍：

|       类型        |                          项目/网址                           |
| :---------------: | :----------------------------------------------------------: |
| **GNSS 数据处理** | **[RTKLIB](https://www.rtklib.com/)**、[RTKLIB-Demo5](https://github.com/rtklibexplorer/RTKLIB)、[rtklib-py](https://github.com/rtklibexplorer/rtklib-py)、[rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge)、[rtkrcv_ros](https://github.com/ajbfinesc/rtkrcv_ros)、**[GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)**、**[Ginan](https://github.com/GeoscienceAustralia/ginan)**、**[goGPS](https://github.com/goGPS-Project/goGPS_MATLAB)**、[BNC](https://igs.bkg.bund.de/ntrip/bnc)、[GFZRNX](https://dataservices.gfz-potsdam.de/panmetaworks/showshort.php?id=escidoc:1577894)、[GAMIT/GLOBK]()、[GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)、[Net_Diff](https://github.com/YizeZhang/Net_Diff)、[POSGO](https://github.com/lizhengnss/POSGO)、[Pride-PPPAR](https://github.com/PrideLab/PRIDE-PPPAR)、[PPPwizard](http://www.ppp-wizard.net/)、[GPSTk](https://github.com/SGL-UT/GPSTk)、[G-NUT/Anubis](https://www.pecny.cz/Joomla25/index.php/gnss/sw/anubis)、[Teqc](https://www.unavco.org/software/data-processing/teqc/teqc.html)、[Bernese](https://www.bernese.unibe.ch/)、[gLAB](https://github.com/valgur/gLAB?tab=readme-ov-file)、[gnssrefl](https://github.com/kristinemlarson/gnssrefl)、[RobustGNSS](https://github.com/wvu-navLab/RobustGNSS)、[GAMPII-GOOD](https://github.com/zhouforme0318/GAMPII-GOOD)、[ntrip](https://github.com/sevensx/ntrip)、[rtcm](https://github.com/Node-NTRIP/rtcm)、[cors](https://github.com/Erensu/cors)、[georinex](https://github.com/geospace-code/georinex)、[PPP-BayesTree](https://github.com/wvu-navLab/PPP-BayesTree)、[FAST](https://github.com/ChangChuntao/FAST)、[gnsspy](https://github.com/GNSSpy-Project/gnsspy)、[gnssgo](https://github.com/FengXuebin/gnssgo)、[raPPPid](https://github.com/TUW-VieVS/raPPPid)、[FCB-FILES](https://github.com/FCB-SGG/FCB-FILES)、[MG_APP](https://github.com/XiaoGongWei/MG_APP)、[gps_amcl](https://github.com/midemig/gps_amcl)、[RTKinGSS](https://github.com/shaolinbit/RTKinGSS)、[gnss_lib_py](https://github.com/Stanford-NavLab/gnss_lib_py)、[PNT-Integrity](https://github.com/cisagov/PNT-Integrity)、[GDDS](https://geodesy.noaa.gov/gps-toolbox/gdds.shtml)、[groops](https://github.com/groops-devs/groops)、 [OREKIT](https://link.zhihu.com/?target=https%3A//www.orekit.org/)、[gnss-tec](https://github.com/gnss-lab/gnss-tec)、[PW_from_GPS](https://github.com/ZiskinZiv/PW_from_GPS)、[PyGPSClient](https://github.com/semuconsulting/PyGPSClient)、[STM32-GNSS](https://github.com/SimpleMethod/STM32-GNSS)、[HASlib](https://github.com/nlsfi/HASlib)、[GNSSAMS](https://github.com/GanAHE/GNSSAMS)、[ppp-tools](https://github.com/aewallin/ppp-tools)、[cssrlib](https://github.com/hirokawa/cssrlib)、[gpstest](https://github.com/barbeau/gpstest)、[iSniff-GPS](https://github.com/hubert3/iSniff-GPS)、[hypatia](https://github.com/snkas/hypatia)、[gps-measurement-tools](https://github.com/google/gps-measurement-tools)、[FE-GUT](https://github.com/zhaoqj23/FE-GUT)、[gnss-RX](https://github.com/HeryMwenegoha/gnss-RX)、[Pypredict](https://github.com/spel-uchile/Pypredict)、[novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver)、[GPSToolbox 投稿](https://geodesy.noaa.gov/gps-toolbox/exist.htm)、[awesome-gnss 整理](https://github.com/barbeau/awesome-gnss)、[mcraymer 整理](https://mcraymer.github.io/geodesy/index.html) |
| **GNSS 信号处理** | **[GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)**、**[SoftGNSS](https://github.com/kristianpaul/SoftGNSS)**、[SoftGNSS-python](https://github.com/perrysou/SoftGNSS-python)、**[PocketSDR](https://github.com/tomojitakasu/PocketSDR)**、[GNSS-SDRLIB](https://github.com/taroz/GNSS-SDRLIB)、[nut2nt](https://github.com/amungo/nut2nt)、[Beagle_SDR_GPS](https://github.com/jks-prv/Beagle_SDR_GPS)、[FlyDog_SDR_GPS](https://github.com/flydog-sdr/FlyDog_SDR_GPS)、[GNSS-GPS-SDR](https://github.com/JiaoXianjun/GNSS-GPS-SDR)、[gnss-sdr-1pps](https://github.com/oscimp/gnss-sdr-1pps)、[SatDump](https://github.com/SatDump/SatDump)、[gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim)、[beidou-sdr-sim](https://github.com/yangfan852219770/beidou-sdr-sim)、[galileo-sdr-sim](https://github.com/harshadms/galileo-sdr-sim)、[gps-qzss-sdr-sim](https://github.com/iGNSS/gps-qzss-sdr-sim)、[multi-sdr-gps-sim](https://github.com/Mictronics/multi-sdr-gps-sim)、[SignalSim](https://github.com/globsky/SignalSim)、[GPS_GAL_SSS](https://github.com/domonforyou/GPS_GAL_SSS)、[greta-oto](https://github.com/globsky/greta-oto)、[BD3_FPGA](https://github.com/whc2uestc/BD3_FPGA)、[GNSS-matlab](https://github.com/danipascual/GNSS-matlab)、[SDR-GB-SAR](https://github.com/jmfriedt/SDR-GB-SAR)、[gps_rf_frontend_sim](https://github.com/iliasam/gps_rf_frontend_sim)、[GNSS-VHDL](https://github.com/danipascual/GNSS-VHDL)、[gnss-baseband](https://github.com/j-core/gnss-baseband)、[Analog-GPS-data-receiver](https://github.com/leaningktower/Analog-GPS-data-receiver)、[GNSS-DSP-tools](https://github.com/pmonta/GNSS-DSP-tools)、[hard_sydr](https://github.com/aproposorg/hard_sydr)、[B1C_Signals_Simulation](https://github.com/pandaclover/B1C_Signals_Simulation)、[CU-SDR-Collection](https://github.com/gnsscusdr/CU-SDR-Collection)、[ESP32_SDR_GPS](https://github.com/iliasam/ESP32_SDR_GPS)、[STM32F4_SDR_GPS](https://github.com/iliasam/STM32F4_SDR_GPS)、[snappergps-pcb](https://github.com/SnapperGPS/snappergps-pcb)、[Fast_GNSS_ReceiverMATLAB](https://github.com/JohnBagshaw/Fast_GNSS_ReceiverMATLAB)、[gnss-sdr-rs](https://github.com/kewei/gnss-sdr-rs) |
| **INS、组合导航** | **[PSINS](http://www.psins.org.cn/)**、**[KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)**、**[OB-GINS](https://github.com/i2Nav-WHU/OB_GINS)**、**[TGINS](https://github.com/heiwa0519/TGINS)**、[PPPLIB](https://geodesy.noaa.gov/gps-toolbox/PPPLib.htm)、[Compass](https://gitee.com/hw_cc/compass)、[GINav_v2.0](https://github.com/kaichen686/GINav_v2.0-test)、[GINAV](https://github.com/kaichen686/GINav)、[IGNAV](https://github.com/Erensu/ignav)、[MATLAB-Groves](https://github.com/zbai/MATLAB-Groves)、[imu_x_fusion](https://github.com/cggos/imu_x_fusion)、[OpenIMU](https://github.com/introlab/OpenIMU)、[eagleye](https://github.com/MapIV/eagleye)、[ai-imu-dr](https://github.com/mbrossar/ai-imu-dr)、[NaveCodePro](https://github.com/zelanzou/NaveCodePro)、[nav_matlab](https://github.com/yandld/nav_matlab)、[kalibr_allan](https://github.com/rpng/kalibr_allan)、[imu_data_simulation](https://github.com/robosu12/imu_data_simulation)、[GPS_IMU_Kalman_Filter](https://github.com/karanchawla/GPS_IMU_Kalman_Filter)、[TightlyCoupledINSGNSS](https://github.com/benzenemo/TightlyCoupledINSGNSS)、 [Wheel-INS ](https://github.com/i2Nav-WHU/Wheel-INS)、[GNSS-INS](https://github.com/hitleeleo/GNSS-INS)、[imu_tools](https://github.com/CCNYRoboticsLab/imu_tools)、[imu_utils](https://github.com/gaowenliang/imu_utils)、[GyroAllan](https://github.com/XinLiGH/GyroAllan)、[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)、[MEMS-IMU-Denoising](https://github.com/ansfl/MEMS-IMU-Denoising)、[nav_matlab](https://github.com/yandld/nav_matlab)、[agrobot](https://github.com/nesl/agrobot)、[IBG_EKF_TC](https://github.com/Dennissy23/IBG_EKF_TC)、 [ZCJ_GNSSINS_DeepIntegration ](https://github.com/kongtian-SiBu/ZCJ_GNSSINS_DeepIntegration)、[Smartphone_IMU_GPS](https://github.com/alexschultze/Smartphone_IMU_GPS)、[INSTINCT](https://github.com/UniStuttgart-INS/INSTINCT)、[Gait-Tracking](https://github.com/xioTechnologies/Gait-Tracking)、[Machine_Learning_GNSS_IMU_Integration](https://github.com/Akpozi/Machine_Learning_GNSS_IMU_Integration)、[ImuCalibration-Poistion](https://github.com/shenshikexmu/ImuCalibration-Poistion)、[IMU-study 整理](https://github.com/Staok/IMU-study) |
|   **视觉 SLAM**   | **[Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)/[Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**、**[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**/[2](https://github.com/raulmur/ORB_SLAM2)/[1](https://github.com/raulmur/ORB_SLAM)、[OpenVINS](https://github.com/rpng/open_vins)、[Openvslam](https://github.com/xdspacelab/openvslam)、[svo](https://github.com/uzh-rpg/rpg_svo)、[svo_pro](https://github.com/uzh-rpg/rpg_svo_pro_open)、[msckf_vio](https://github.com/KumarRobotics/msckf_vio)、[okvis](https://github.com/ethz-asl/okvis)、[DM-VIO](https://github.com/lukasvst/dm-vio)、[DSO](https://github.com/JakobEngel/dso)、[DSOL](https://github.com/versatran01/dsol)、[NeRF](https://github.com/bmild/nerf)、[Elasticfusion](https://github.com/mp3guy/ElasticFusion)、[OpenMVG](https://github.com/openMVG/openMVG)、[Meshroom](https://github.com/alicevision/Meshroom)、[Kintinuous](https://github.com/mp3guy/Kintinuous)、[Mvision](https://github.com/Ewenwan/MVision)、[rgbdslam_v2](https://github.com/felixendres/rgbdslam_v2)、[awesome-visual-slam 整理](https://github.com/tzutalin/awesome-visual-slam)、[Recent_SLAM_Research 整理](https://github.com/YiChenCityU/Recent_SLAM_Research)、[Awesome CV Works 整理](https://vincentqin.tech/posts/awesome-works/)、[Lee-SLAM-source 整理](https://github.com/AlbertSlam/Lee-SLAM-source)、[awesome-slam 整理](https://github.com/kanster/awesome-slam)、[awesome-NeRF 整理](https://github.com/awesome-NeRF/awesome-NeRF)、[visual-slam-roadmap 整理](https://github.com/changh95/visual-slam-roadmap)、[Visual_SLAM_Related_Research 整理](https://github.com/wuxiaolang/Visual_SLAM_Related_Research) |
|   **激光 SLAM**   | [Gmapping](https://github.com/ros-perception/slam_gmapping)、[Cartographer](https://github.com/cartographer-project/cartographer)、[hector-slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)、[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)、[LOAM-Livox](https://github.com/hku-mars/loam_livox)、[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)、[SuMa](https://github.com/jbehley/SuMa)、[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)、[FAST-LIO-SAM](https://github.com/kahowang/FAST_LIO_SAM)、[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[LIO-Mapping](https://github.com/hyye/lio-mapping)、[CT-ICP](https://github.com/jedeschaud/ct_icp)、[Coco-LIC](https://github.com/APRIL-ZJU/Coco-LIC)、[BoW3D](https://github.com/YungeCui/BoW3D)、[CloudViewer](https://github.com/nightn/CloudViewer)、、[LT-Mapper](https://github.com/gisbi-kim/lt-mapper)、[awesome-lidar 整理](https://github.com/szenergy/awesome-lidar)、[awesome-point-cloud-place-recognition 整理](https://github.com/kxhit/awesome-point-cloud-place-recognition)、[awesome-sar 整理](https://github.com/RadarCODE/awesome-sar)、[awesome-radar-perception 整理](https://github.com/ZHOUYI1023/awesome-radar-perception) |
|   **多源融合**    | **[GICI-LIB](https://github.com/chichengcn/gici-open)**、[GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS)、[GLIO](https://github.com/XikunLiu-huskit/GLIO)、[InGVIO](https://github.com/ChangwuLiu/InGVIO)、[Multi-Sensor-Fusion](https://github.com/2013fangwentao/Multi_Sensor_Fusion)、[MSF_developed](https://github.com/milkytipo/MSF_developed)、[MINS](https://github.com/rpng/MINS)、[mars_lib](https://github.com/aau-cns/mars_lib)、[imu_x_fusion](https://github.com/cggos/imu_x_fusion)、[IC_GVINS](https://github.com/i2Nav-WHU/IC-GVINS)、[FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)、[VINS-GPS-Wheel](https://github.com/Wallong/VINS-GPS-Wheel)、[sync_gps_lidar_imu_cam](https://github.com/nkliuhui/sync_gps_lidar_imu_cam)、[carvig](https://github.com/Erensu/carvig)、[LVI-SAM](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LVI-SAM)、[NaveGo](https://github.com/rodralez/NaveGo)、[YabLoc](https://github.com/tier4/YabLoc) |
|   **规划控制**    | [**navigation**](https://github.com/ros-planning/navigation)、**[navigation2 ](https://github.com/ros-planning/navigation2)**、[Apollo](https://github.com/ApolloAuto/apollo)、[Autoware](https://github.com/autowarefoundation/autoware)、[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)、[ardupilot](https://github.com/ArduPilot/ardupilot)、[mars](https://github.com/OPEN-AIR-SUN/mars)、[PathPlanning](https://github.com/zhm-real/PathPlanning)、[MissionPlanner](https://github.com/ArduPilot/MissionPlanner)、[navigator](https://github.com/Nova-UTD/navigator)、[self-driving-car](https://github.com/ndrplz/self-driving-car)、[Awesome-Self-Driving 整理](https://github.com/iGNSS/Awesome-Self-Driving)、[Autopilot-Notes 整理](https://github.com/gotonote/Autopilot-Notes) |
|      **AI**       | [tensorflow](https://github.com/tensorflow/tensorflow)、[keras](https://github.com/keras-team/keras)、[Paddle](https://github.com/PaddlePaddle/Paddle)、[pytorch](https://github.com/pytorch/pytorch)、[Theano](https://github.com/Theano/Theano)、[wekan](https://github.com/wekan/wekan)、[caffe](https://github.com/BVLC/caffe)、[torch7](https://github.com/torch/torch7)、[yolov5](https://github.com/ultralytics/yolov5)、[DeepSpeed](https://github.com/microsoft/DeepSpeed)、[transformers](https://github.com/huggingface/transformers)、[accelerate](https://github.com/huggingface/accelerate)、[mindspore](https://github.com/mindspore-ai/mindspore)、[jittor](https://github.com/Jittor/jittor)、[oneflow](https://github.com/Oneflow-Inc/oneflow)、[x-deeplearning](https://github.com/alibaba/x-deeplearning)、[MegEngine](https://github.com/MegEngine/MegEngine)、[ncnn](https://github.com/Tencent/ncnn)、[FinRL](https://github.com/AI4Finance-Foundation/FinRL)、[spinningup](https://github.com/openai/spinningup)、[baselines](https://github.com/openai/baselines)、[stable-baselines](https://github.com/hill-a/stable-baselines)、[mxnet](https://github.com/apache/mxnet)、[MegEngine](https://github.com/MegEngine/MegEngine)、[TensorRT](https://github.com/NVIDIA/TensorRT)、[darknet](https://github.com/pjreddie/darknet)、[darknet_ros](https://github.com/leggedrobotics/darknet_ros)、[mxnet](https://github.com/apache/mxnet)、[CNTK](https://github.com/microsoft/CNTK)、[matconvnet](https://github.com/vlfeat/matconvnet)、[chainer](https://github.com/chainer/chainer)、[onnx](https://github.com/onnx/onnx)、[Theano](https://github.com/Theano/Theano)、[ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning)、[DeepLearning 整理](https://github.com/Mikoto10032/DeepLearning)、[awesome-deep-learning 整理](https://github.com/ChristosChristofidis/awesome-deep-learning)、[Deep-Learning-Papers-Reading-Roadmap 整理](https://github.com/floodsung/Deep-Learning-Papers-Reading-Roadmap)、[awesome-deep-learning-papers 整理](https://github.com/terryum/awesome-deep-learning-papers)、[awesome-chatgpt-prompts-zh 整理](https://github.com/PlexPt/awesome-chatgpt-prompts-zh) |
|     **其它**      | [kalman](https://github.com/mherb/kalman)、[bayes-filters-lib](https://github.com/robotology/bayes-filters-lib)、[filterpy](https://github.com/rlabbe/filterpy)、[DynAdjust](https://github.com/icsm-au/DynAdjust)、[KalmanFilter](https://github.com/mannyray/KalmanFilter)、[ahrs](https://github.com/Mayitzin/ahrs)、[utm](https://github.com/sfegan/utm)、  [uwb-localization](https://github.com/lijx10/uwb-localization)、[Location](https://github.com/yyccR/Location)、[Indoor-Positioning](https://github.com/GXW19961104/Indoor-Positioning)、[positioning-algorithms-for-uwb-matlab](https://github.com/cliansang/positioning-algorithms-for-uwb-matlab)、[UWB_DualAntenna_AoA](https://github.com/ETH-PBL/UWB_DualAntenna_AoA)、[Pedometer](https://github.com/mirraico/Pedometer)、[GetSensorData_Android](https://github.com/lopsi/GetSensorData_Android)、[LoRa_2G4_localization](https://github.com/alphaLeporis/LoRa_2G4_localization)、[DeadReckoning](https://github.com/nisargnp/DeadReckoning)、[Smartlight_UWB](https://github.com/Zekke-e/Smartlight_UWB)、[senslogs](https://github.com/tyrex-team/senslogs)、[vrs](https://github.com/facebookresearch/vrs)、[Pedestrian-Dead-Reckoning](https://github.com/Zidane-Han/Pedestrian-Dead-Reckoning)、[UWBPositioning](https://github.com/Meihai/UWBPositioning)、[util-uwb-dataset](https://github.com/utiasDSL/util-uwb-dataset)、[RSSI-Dataset-for-Indoor-Localization-Fingerprinting](https://github.com/pspachos/RSSI-Dataset-for-Indoor-Localization-Fingerprinting)、[Awesome-Human-Activity-Recognition 整理](https://github.com/iGNSS/Awesome-Human-Activity-Recognition) |
|    **常用库**     | [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)、[OpenBLAS](https://github.com/OpenMathLib/OpenBLAS)、[Gflags](https://github.com/gflags/gflags)、[Glog](https://github.com/google/glog)、[easyloggingpp](https://github.com/abumq/easyloggingpp)、[Ceres-Solver](https://github.com/ceres-solver/ceres-solver)、[g2o](https://github.com/RainerKuemmerle/g2o)、[gtsam](https://github.com/borglab/gtsam)、[Yaml-Cpp](https://github.com/jbeder/yaml-cpp)、[OpenCV](https://github.com/opencv/opencv)、[PCL](https://pointclouds.org/)、[Boost](https://github.com/boostorg/boost)、[better-enums](http://github.com/aantron/better-enums)、[DBoW2](https://github.com/dorian3d/DBoW2)、[matplotlib-cpp](https://github.com/lava/matplotlib-cpp) |
|     **工具**      | [zotero](https://github.com/zotero/zotero)、[Captura](https://github.com/MathewSachin/Captura)、[ GitHub520](https://github.com/521xueweihan/GitHub520)、[free](https://github.com/zp-9696/free)、[carrot](https://github.com/xx025/carrot)、[aliyunpan](https://github.com/gaozhangmin/aliyunpan) |
| **开源程序作者**  | [池澄](https://github.com/chichengcn/gici-open)、[周峰](https://github.com/zhouforme0318)、[陈超](https://github.com/heiwa0519)、[严恭敏](https://psins.org.cn/sy)、[李政](https://github.com/lizhengnss)、[苏景岚](https://github.com/Erensu)、[陈凯](https://github.com/kaichen686)、[王晗](https://github.com/wh200720041)、[刘国庆](https://github.com/DreamWaterFound)、[高翔](https://github.com/gaoxiang12)、[杨熙](https://github.com/yandld)、[张春杰](https://github.com/kongtian-SiBu)、[魏源](https://github.com/hui-Scholarliness)、[布树辉](https://link.zhihu.com/?target=http%3A//www.adv-ci.com/blog/)、[谢立华](https://link.zhihu.com/?target=https%3A//wanghan.pro/)、[邹丹平](https://link.zhihu.com/?target=http%3A//drone.sjtu.edu.cn/dpzou/index.php)、[XikunLiu-huskit](https://github.com/XikunLiu-huskit)、[cggos](https://github.com/cggos)、[shaolinbit](https://github.com/shaolinbit)、[Welson WEN](https://www.zhihu.com/people/Welson-WEN)、[cggos](https://github.com/cggos)、[zarathustr](https://github.com/zarathustr)、[YizeZhang](https://github.com/YizeZhang)、[globsky](https://github.com/globsky)、[tomojitakasu](https://github.com/tomojitakasu)、[carlesfernandez](https://github.com/carlesfernandez)、[rtklibexplorer](https://github.com/rtklibexplorer)、[GeoscienceAustralia](https://github.com/GeoscienceAustralia)、[taroz](https://github.com/taroz)、[osqzss](https://github.com/osqzss) |

---

记录一些导航相关的网址、学习资源、导航设备：

|           类型           |                             网址                             |
| :----------------------: | :----------------------------------------------------------: |
|       **中文期刊**       | [测绘学报](http://xb.chinasmp.com/CN/1001-1595/home.shtml)、[武汉大学学报（信息科学版）](http://ch.whu.edu.cn/)、[测绘地理信息](http://chdlxx.whu.edu.cn/homeNav?lang=zh)、[北京航空航天大学学报](https://bhxb.buaa.edu.cn/bhzk/)、[电子学报](https://www.ejournal.org.cn/CN/home)、[测绘工程](https://www.xueshu.com.cn/cehgc/)、[测绘通报](http://tb.chinasmp.com/CN/0494-0911/home.shtml)、[信号处理](https://signal.ejournal.org.cn/)、[中国惯性技术](http://www.zggxjsxb.com/CN/1005-6734/home.shtml)、[地球物理学报](http://www.geophy.cn/)、[大地测量与地球动力学](http://www.jgg09.com/CN/volumn/current.shtml)、[全球定位系统](http://www.qqdwxt.cn/)、[导航定位学报](https://dhdwxb.chinajournal.net.cn/WKC/WebPublication/index.aspx?mid=chwz)、[地理空间信息](https://dxkj.chinajournal.net.cn/WKE2/WebPublication/index.aspx?mid=DXKJ)、[海洋预报](http://www.hyyb.org.cn/)、[仪器仪表学报](http://yqyb.etmchina.com/yqyb/home)、[导航定位与授时](http://pnt.ijournals.cn/dhdwyss/ch/index.aspx)、[地球科学与环境学报](http://jese.chd.edu.cn/)、[弹箭与制导学报](https://djzd.cbpt.cnki.net/WKD3/WebPublication/index.aspx?mid=djzd)、[微电子与计算机](http://www.journalmc.com/)、[西北工业大学学报](https://journals.nwpu.edu.cn/xbgydxxb/CN/volumn/current.shtml)、[宇航学报](https://www.yhxb.org.cn/homeNav?lang=zh)、[仪表技术与传感器](http://www.17sensor.com/#/)、[测控技术](http://ckjs.ijournals.cn/ckjs/ch/index.aspx)、[系统仿真学报](https://www.china-simulation.com/CN/1004-731X/home.shtml)、[航天控制](http://htkz.magtechjournal.com/CN/home)、[兵工自动化](http://bgzdh.ijournals.com.cn/bgzdh/home) |
|       **英文期刊**       | [GPS Solutions](https://link.springer.com/journal/10291)、[Remote Sensing](https://www.mdpi.com/journal/remotesensing)、[Measurement](https://www.sciencedirect.com/journal/measurement)、[Satellite Navigation](https://satellite-navigation.springeropen.com/)、 |
| **各系统、分析中心官网** | [BDS](http://www.beidou.gov.cn)、[GPS](http://www.gps.gov)、[GLONASS](https://www.glonass-iac.ru/en/)、[Galileo](http://ec.europa.eu/index_en.htm)、[QZSS](https://qzss.go.jp/en/technical/qzssinfo/)、[IRNSS](https://www.isro.gov.in/irnss-programme)、[MGEX](https://www.igs.org/mgex/)、[IGS](https://igs.org/)、[IGMAS](http://www.igmas.org/)、[ITRF](https://itrf.ign.fr/en/homepage)、[IGS标准](https://www.igs.org/formats-and-standards)、[IGS站](https://network.igs.org/)、[iGMAS](http://igmas.users.sgg.whu.edu.cn/home)、[IAG](https://www.iag-aig.org/)、[WDS](https://worlddatasystem.org/)、[IUGG](https://iugg.org/)、[GGOS](https://ggos.org/)、CNES、IAC、SHAO、CODE、[GFZ](https://www.gfz-potsdam.de/)、ESA、[WHU](http://www.igs.gnsswhu.cn/)、[BKG](https://igs.bkg.bund.de/)、[CDDIS](https://cddis.nasa.gov/)、ING、[中国卫星导航系统管理办公室测试评估研究中心](https://www.csno-tarc.cn/)、[gpsworld](https://www.gpsworld.com/)、[GNSS Calendar](https://www.gnsscalendar.com/)、[时间转换查询](http://www.leapsecond.com/java/gpsclock.htm)、[灰机wiki卫星百科](https://sat.huijiwiki.com/wiki/%E9%A6%96%E9%A1%B5) |
|     **导航设备厂商**     | [Ublox](https://www.u-blox.com/en/)、[诺瓦泰](https://novatel.com/)、[天宝](http://www.tianbaonet.com/)、[徕卡](https://leica-geosystems.com/)、[意法半导体](https://www.st.com/zh/positioning/gnss-ics.html)、[Javad](https://www.javad.com/)、[Septentrio](https://www.baidu.com/link?url=Tz1rvry1DTYCTiDvKIZyzBdtkn9o5lrNK_1Lm_QdjvQkTa0dDYD5asZOG8sp3z51&wd=&eqid=b98a38f0010d606a0000000665e53220)、[Spectra Geospatial](https://spectrageospatial.com/)、[SXblue](https://sxbluegps.com/products/vehicle-guidance-receivers/)、[NavCom](https://www.navcomtech.com/en/)、[Geneq](https://geneq.com/fr)、[和芯星通](https://www.unicorecomm.com/)、[北斗星通](https://www.bdstar.com/)、[创宇星通](http://www.cyxt.com/)、[华测导航](https://www.huace.cn/)、[思南导航](https://www.sinognss.com/)、[南方测绘](http://www.southsurvey.com/)、[合众思壮](https://www.unistrong.com/)、[移远通信](https://www.quectel.com.cn/)、[北云科技](https://www.bynav.com/)、[格林恩德](http://www.szgled.cn/)、[创新微](https://www.minewsemi.com/)、[天硕导航](https://www.tersus-gnss.cn/)、[超核电子](https://www.hipnuc.com/)、[原极科技](https://www.forsense.cn/)、[奥比中光](https://www.orbbec.com.cn/)、[浩如科技](https://www.haorutech.com/)、[凌思科技](https://www.lins-tech.com/)、[矽睿科技](http://www.siwisemi.com/)、[瑞芬科技](http://www.rion-tech.net/)、[北微传感](http://www.bwsensing.com.cn/)、[元生创新](https://www.yesense.com/)、[维特智能](https://wit-motion.cn/)、[智腾微电子](http://www.ztmicro.com/product/jzdz-zhdh/)、[北斗时代科技](http://www.bdstartimes.com/)、[清研讯科](https://www.tsingoal.com/)、[喜讯科技](https://www.xexun.com/)、[沃旭通讯](https://www.woxuwireless.com/)、[瑞达科讯](https://html.rdkx-iot.com/)、[美迪索科](https://html.rdkx-iot.com/)、[泰浩微](https://www.taihaowei.net/)、[RealSense](www.intelrealsense.com/)、[申稷光电](http://www.shsenky.com/)、[司岚光电](http://www.slamopto.com/index.php?lang=cn)、[奥比中光](https://www.orbbec.com.cn/)、[科力光电](http://www.sdkeli.com/)、[禾赛科技](https://www.hesaitech.com/cn/)、[基恩士](https://www.keyence.com.cn/)、[锐驰激光](https://www.richbeam.com/)、[EAI科技](https://ydlidar.cn/about.html)、[Kinect](https://learn.microsoft.com/zh-tw/azure/Kinect-dk/depth-camera)、[海川润泽](https://www.57iot.com/)、[四信物联网](https://www.four-faith.com.cn/)、[钦天导航](https://www.qinnav.com/)、[思为无线](https://www.nicerf.cn/)、[光鉴科技](https://learn.microsoft.com/zh-tw/azure/Kinect-dk/depth-camera)、[图漾科技](https://www.percipio.xyz/)、[微深联创](https://www.visenai.com/) |
|       **芯片厂商**       | [恩智浦](https://www.nxp.com.cn/)、[微芯](https://www.microchip.com/)、[瑞萨](https://www.renesas.cn/cn/zh)、[高通](https://www.qualcomm.cn/)、[意法半导体](https://www.st.com/content/st_com/zh.html)、[AMD](https://www.amd.com/zh-cn.html)、[ARM](https://www.arm.com/)、[英飞凌](https://www.infineon.com/)、[德州仪器](https://www.ti.com.cn/)、[新唐](https://www.nuvoton.com.cn/)、[罗姆](https://www.rohm.com.cn/)、[三星电子](https://www.samsung.com.cn/)、[东芝](https://www.toshiba.com.cn/)、[海力士](https://www.skhynix.com.cn/)、[美光](https://www.micron.cn/)、[赛普拉斯](https://www.infineon.com/?utm_source=cypress&utm_medium=referral&utm_campaign=202110_globe_en_all_integration-homepage&redirId=test_homepage)、[亚德诺](https://www.analog.com/cn/index.html)、[莱迪思](https://www.latticesemi.com/zh-CN/)、[思佳讯](https://www.skyworksinc.com/?Lang=zh-cn)、[博通](https://www.broadcom.cn/)、[Maevell](https://www.marvell.com/)、[菲菱科思](http://www.phoenixcompany.cn/)、[CISCO](https://www.cisco.com/)、[安森美](https://www.onsemi.cn/)、[兆易创新](https://www.gigadevice.com.cn/)、[乐鑫科技](http://espressif.cn/zh-hans)、[沁恒微电子](https://www.wch.cn/products/CH569.html)、[高云半导体](http://www.gowinsemi.com.cn/)、[复旦微电子](https://www.fmsh.com/products.shtml)、[宏晶科技](http://www.macrosilicon.com/default.asp)、[摩尔线程](https://www.mthreads.com/)、[芯动科技](https://www.innosilicon.cn/)、[海思半导体](https://www.hisilicon.com/en)、[平头哥](https://www.t-head.cn/)、[紫光集团](https://www.unigroup.com.cn/)、[长江存储](https://www.ymtc.com/)、[长鑫存储](https://www.cxmt.com/)、[易灵思](https://www.elitestek.com/index.html) |
|   **导航定位计算平台**   | [Jetson](https://www.nvidia.com/en-us/autonomous-machines/)、[Arduino](https://www.arduino.cc/)、[ESP32](https://www.espressif.com.cn/)、[STC89C51](https://www.stcmicro.com/index.html)、[STM32](https://www.st.com/zh/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html)、[GD32](https://www.gd32mcu.com/cn/product/)、[CH32](https://www.wch.cn/products/productsCenter/mcuInterface?categoryId=70)、[ZYNQ](https://china.xilinx.com/products/silicon-devices/soc/zynq-7000.html)、[香橙派](http://www.orangepi.cn/)、[RK3588](https://www.rock-chips.com/a/cn/product/RK35xilie/2022/0926/1656.html)、[NanoPi](https://www.friendlyelec.com/)、[泰山派](https://lckfb.com/project/detail/lctspi-2g-16g?param=baseInfo) |
|     **淘宝京东店铺**     | [北天](https://beitianznsb.tmall.com/)、[墨子号科技](https://mzhtek.taobao.com/)、[南方测绘](https://south.tmall.com/)、[体感中国](https://shop36983089.taobao.com/)、[移远](https://yiyuanznsb.tmall.com/)、[千寻位置](https://qianxunweizhi.tmall.com/)、[集思宝](https://jisibaohw.tmall.com/)、[深圳天工测绘](https://shop471758324.taobao.com/)、[思南导航](https://sinognss.tmall.com/)、[维特智能](https://weitezhineng.tmall.com/)、[北云科技](https://shop382665129.taobao.com/)、[原极科技](https://shop69172801.taobao.com/)、[凌思科技](https://shop578921739.taobao.com/)、[啊路比电子](https://shop264805684.taobao.com/)、[超核电子](https://hipnuc.tmall.com/)、[瑞芬科技](https://rion-tech.taobao.com/)、[蓝尊科技](https://shop433492811.taobao.com/)、[北微传感](https://bewis.taobao.com/)、[精讯畅通](https://shop116671741.taobao.com/)、[华信](https://hx002.taobao.com/)、[红心天线](https://shop278992037.taobao.com/)、[维信天线](https://shop106012054.taobao.com/)、[亚博智能](https://yabozhineng.tmall.com/)、[微雪电子](https://weixuesm.tmall.com/)、[轮趣科技](https://wheeltec.tmall.com/)、[幻尔机器人](https://lobot-zone.taobao.com/)、 [youyeetoo开源硬件商城](https://shop113286404.taobao.com/)、[天之博特](https://tianbot.taobao.com/)、[合宙](https://luat.taobao.com/)、[正点原子](https://zhengdianyuanzi.tmall.com/)、[野火](https://yehuosm.tmall.com/)、[Speed](https://sipeed.taobao.com/)、[seeed](https://seeedstudio.taobao.com/)、[芯板坊](https://shop599532105.taobao.com/)、[Abrobt](https://shop264518119.taobao.com/)、[创乐博](https://chuanglebo.tmall.com/)、[米尔科技](https://shop108478821.taobao.com/)、[嘉立创](https://shop400371330.taobao.com/)、[微相科技](https://esoc.taobao.com/)、[我爱开发板](https://shop580004668.taobao.com/)、[璞致电子](https://shop275566416.taobao.com/)、[香橙派](https://xiangchengpaidn.tmall.com/)、[EAI](https://eaibot.taobao.com/)、[北醒](https://jxlyt520.taobao.com/)、[司岚](https://shop152221712.taobao.com/)、[SenkyLaser](https://senkylaser.taobao.com/)、[宇树](https://unitree.tmall.com/)、[科恩光电](https://shop254986868.taobao.com/)、 [骆鸵互娱科技](https://shop148383984.taobao.com/)、[奥比中光](https://shop142544700.taobao.com/)、 [韦东山老师个人店 ](https://100ask.taobao.com/)、[WeAct Studio](https://weactstudio.taobao.com/?spm=a1z10.1-c.0.0.888c78f2VOna3u)、[OpenJumper](https://shop555818949.taobao.com/?spm=a1z10.1-c-s.0.0.5a0b778cLIkmxU)、[开源SDR实验室](https://opensourcesdrlab.taobao.com/)、[友善](https://nanopi.taobao.com/)、[alinx](https://alinx.tmall.com/) |
|       **知乎博主**       | [Random Walker](https://www.zhihu.com/people/dao-ge-92-60)、[任乾](https://www.zhihu.com/people/ren-gan-16)、[吴桐](https://www.zhihu.com/people/wu-tong-15-20)、[Hyperion](https://www.zhihu.com/people/TheTengda)、[yangongmin](https://www.zhihu.com/people/yangongmin)、[曹小白](https://www.zhihu.com/people/luchilushi)、[左家垅宋威龙](https://www.zhihu.com/people/wang-hao-nan-46-18)、[咖啡萝卜丝](https://www.zhihu.com/people/ka-pei-luo-bu)、[杨熙](https://www.zhihu.com/people/yang-xi-97-90)、[Welson WEN](https://www.zhihu.com/people/Welson-WEN)、[迷途小书童](https://www.zhihu.com/people/mei-xi-31-99)、[枯荣有常](https://www.zhihu.com/people/zun-yu-9)、[计算机视觉life](https://www.zhihu.com/people/cheng-xu-yuan-10)、[半闲居士](https://www.zhihu.com/people/gao-xiang-24-90)、[邱笑晨](https://www.zhihu.com/people/ji-zhi-de-xiao-chen)、[Churlaaaaaaa](https://www.zhihu.com/people/xi-men-dou-dou-65)、[刘国庆](https://www.zhihu.com/people/guoqingliu)、[泰伦斯-Ternence](https://www.zhihu.com/people/xiu-xue-chu-neng-ing)、[鱼香ROS](https://www.zhihu.com/people/fishros)、[行知SLAM](https://www.zhihu.com/people/yue-guang-qin-liao-cheng)、[Hyperion](https://www.zhihu.com/people/TheTengda)、[Yusheng](https://www.zhihu.com/people/wang-yu-sheng-84-10)、[李想](https://www.zhihu.com/people/li-xiang-62-60-61)、[清风](https://www.zhihu.com/people/qing-feng-77-90) |
|       **CSDN博主**       | [Random-Walker](https://blog.csdn.net/daoge2666)、[辉——书生意气](https://blog.csdn.net/qq_45391544)、[GISer.Wang](https://blog.csdn.net/LoveCarpenter)、[他人是一面镜子，保持谦虚的态度](https://blog.csdn.net/hltt3838)、[随心乐行](https://blog.csdn.net/qq_35099602)、[十八与她](https://blog.csdn.net/absll)、[Lucid_Sheep](https://blog.csdn.net/Lucid_Sheep)、[卡尔曼的BD SLAMer](https://blog.csdn.net/u011344545)、[梧桐Fighting](https://blog.csdn.net/dong20081991)、[路痴导航员](https://blog.csdn.net/weixin_42918498)、[魔方的块](https://blog.csdn.net/Pro2015)、[枯荣有常](https://blog.csdn.net/wuwuku123)、[WHU-学渣](https://blog.csdn.net/weixin_42474500)、[GNSS初学者](https://blog.csdn.net/qq_38607471)、[LZ_CUMT](https://blog.csdn.net/sinat_39238867)、[icebear](https://blog.csdn.net/icebear____?type=blog)、[tyst08](https://blog.csdn.net/tyst08)、[hyisoe](https://blog.csdn.net/hyisoe?type=blog)、[unbiliverbal](https://blog.csdn.net/unbiliverbal)、[My.科研小菜鸡](https://blog.csdn.net/qq_41861406)、[沃斯故我在](https://blog.csdn.net/weixin_45476865)、[什么都不会的小澎友](https://blog.csdn.net/weixin_45432823?type=blog)、[李太白lx](https://blog.csdn.net/tiancailx?type=blog)、[不学习就落后](https://blog.csdn.net/zhaolewen?type=blog)、[学测绘的小王](https://blog.csdn.net/wys3101492902)、[GNSS-by flying fish](https://blog.csdn.net/weixin_44294660)、[如阳光如你](https://blog.csdn.net/weixin_44126988?type=blog)、[Manii](https://blog.csdn.net/qq_41839222)、[Charmve](https://blog.csdn.net/Charmve)、[jr9910](https://blog.csdn.net/jr9910?type=blog)、[开源SDR实验室](https://blog.csdn.net/OpenSourceSDR)、[difudan4777](https://blog.csdn.net/difudan4777?type=blog)、[Amentia outsider](https://blog.csdn.net/cxy0711)、[JANGHIGH](https://blog.csdn.net/jppdss) |
|     **导航视频教程**     | [严恭敏-卡尔曼滤波与组合导航](https://www.bilibili.com/video/BV11K411J7gp)、[i2NAV-惯性导航](https://www.bilibili.com/video/BV1nR4y1E7Yj)、[i2NAV-组合导航](https://www.bilibili.com/video/BV1na411Z7rQ)、[吴德伟-导航原理](https://www.bilibili.com/video/BV1wx411d7PK)、[高成发-GNSS原理](https://www.bilibili.com/video/BV157411A72u)、[哈工大-GNSS原理](https://www.bilibili.com/video/BV1mB4y1V7zX)、[朱家海-惯性导航系统](https://www.bilibili.com/video/BV1Ar4y1Q7tr)、[机器人工匠阿杰-ROS入门](https://space.bilibili.com/411541289/channel/collectiondetail?sid=693700)、[杨旭-GNSS接收机](https://space.bilibili.com/286787541/channel/seriesdetail?sid=2980038)、[赵乐文-开源GNSS数据处理软件介绍](https://space.bilibili.com/479790048?spm_id_from=333.337)、[冰菓的RTKlib&GAMP](https://space.bilibili.com/199461274/channel/collectiondetail?sid=1088015)、[泰伦斯-GNSS伪距单点定位](https://space.bilibili.com/688837845/channel/seriesdetail?sid=3823979)、[大胡子刘师傅-组合导航入门](https://space.bilibili.com/8494354)、[武大-测绘学概论](https://www.bilibili.com/video/BV1y7411Z72b)、[计算机视觉life](https://cvlife.net/)、[深蓝学院](https://www.shenlanxueyuan.com/)、[硬禾学堂](https://class.eetree.cn/)、[古月居](https://www.guyuehome.com/) |
|   **国内知名研究团队**   | [武大牛小骥i2NAV](http://www.i2nav.com/)、[武大耿江辉Pride](http://pride.whu.edu.cn/index.shtml#)、[武大钟燕飞RSIDEA](http://rsidea.whu.edu.cn/index.html)、[武大姚剑CVRS](https://cvrs.whu.edu.cn/)、[武大郭迟BRAIN](https://zhiyuteam.com/html/web//index.html)、[武大张祖勋数字摄影测量与计算机视觉研究中心](https://dpcv.whu.edu.cn/jj/zxjj.htm)、[上交感知与导航研究所](https://isn.sjtu.edu.cn/web/index) |
|     **SLAM研究团队**     | [香港科技大学空中机器人实验室](https://uav.hkust.edu.hk/)、[香港科技大学机器人与多感知实验室 RAM-LAB](https://www.ram-lab.com/)、[香港中文大学天石机器人实验室](http://ri.cuhk.edu.hk/)、[浙江大学 CAD&CG 国家重点实验室](http://www.zjucvg.net/)、[美国密歇根大学感知机器人实验室（PeRL）](https://robots.engin.umich.edu/About/)、[美国卡耐基梅陇大学机器人研究所](https://www.ri.cmu.edu/)、 [美国加州大学圣地亚哥分校语境机器人研究所](https://existentialrobotics.org/index.html)、[美国特拉华大学人感知与导航组](https://sites.udel.edu/robot/)、[美国麻省理工学院航空航天实验室](http://acl.mit.edu/)、[美国麻省理工学院 SPARK 实验室](http://web.mit.edu/sparklab/)、[美国麻省理工学院海洋机器人组](https://marinerobotics.mit.edu/)、[美国明尼苏达大学多元自主机器人系统实验室](http://mars.cs.umn.edu/index.php)、[美国宾夕法尼亚大学 Vijay Kumar 实验室](https://www.kumarrobotics.org/)、[美国麻省理工学院 Robust Robotics Group]()、[美国佐治亚理工学院智能视觉与自动化实验室](https://link.zhihu.com/?target=http%3A//ivalab.gatech.edu/)、[加拿大蒙特利尔大学机器人与嵌入式 AI 实验室](http://groups.csail.mit.edu/rrg/index.php)、[加拿大舍布鲁克大学智能、交互、综合、跨学科机器人实验室](https://introlab.3it.usherbrooke.ca/)、[瑞士苏黎世大学机器人与感知课题组](http//3A//rpg.ifi.uzh.ch/index.html)、[瑞士苏黎世联邦理工 Vision for Robotics Lab](https://v4rl.ethz.ch/the-group.html)、[瑞士苏黎世联邦理工计算机视觉与几何实验室](https://link.zhihu.com/?target=http%3A//www.cvg.ethz.ch/index.php)、[瑞士苏黎世联邦理工自主系统实验室](https://cvg.ethz.ch/index)、[英国帝国理工学院戴森机器人实验室](https://www.imperial.ac.uk/dyson-robotics-lab/)、[英国牛津大学信息工程学](https://www.robots.ox.ac.uk/)、[德国慕尼黑工业大学计算机视觉组](https://github.com/tum-vision)、[德国马克斯普朗克智能系统研究所嵌入式视觉组](https://ev.is.mpg.de/)、[德国弗莱堡大学智能自主系统实验室](http://ais.informatik.uni-freiburg.de/index_en.php)、[德国波恩大学摄影测量与机器人实验室](https://www.ipb.uni-bonn.de/)、[西班牙萨拉戈萨大学机器人、感知与实时组 SLAM 实验室](http://robots.unizar.es/slamlab/)、[西班牙马拉加大学机器感知与智能机器人课题组](http://mapir.uma.es/mapirwebsite/)、[奥地利格拉茨技术大学计算机图形学与视觉研究所](https://www.tugraz.at/institutes/icg/home/)、[波兰波兹南工业大学移动机器人实验室](http://lrm.put.poznan.pl/)、[澳大利亚昆士兰科技大学机器人技术中心](https://www.qut.edu.au/research/centre-for-robotics)、[澳大利亚机器人视觉中心](https://www.roboticvision.org/)、[日本国立先进工业科学技术研究所](https://www.airc.aist.go.jp/en/intro/) |

## 📚 推荐书籍

![image-20240125155004417](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240125155004417.png)

---

## 01-RTKLIB：GNSS 数据处理

RTKLIB 是全球导航卫星系统 GNSS 开源定位解算程序包，由日本东京海洋大学的高须知二（Tomoji Takasu）开发，由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好，许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来，适合作为 GNSS 入门学习的代码。

![RTKLIB](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/RTKLIB.png)

* **支持多个 GNSS 系统的标准和精密定位算法**，包括 GPS，GLONASS，Beidou，Galileo，QZSS 和 SBAS。

* **支持 9 种GNSS实时和后处理定位模式**：
  * **single**：伪距单点定位
  * **DGPS/DGNSS**：伪距差分
  * **kinematic**：载波动态相对定位，动态RTK，假设流动站是移动的，可以做车载定位
  * **Static**：载波静态相对定位，静态RTK，两站都是静止的，可以得到很高的精度
  * **Moving-Baseline**：两站都动，双天线，主要用来定姿
  * **Fixed**：固定坐标，解算模糊度、对流层、电离层等参数
  * **PPP-Kinematic**：动态精密单点定位
  * **PPP-Static**：静态精密单点定位
  * **PPP-Fixed**：PPP 固定坐标，解算模糊度、对流层、电离层等参数。

* **支持多种GNSS标准格式和协议**：RINEX2.10、RINEX2.11、RINEX2.12、RINEX3.00、RINEX3.01、RINEX3.02、RTCM2.3、RTCM3.1、RTCM3.2、BINEX、NTRIP、NMEA0183、SP3、ANTEX1.4、IONEX1.0、NGS PCV、EMS 2.0。

* **支持多种GNSS接收机专有数据协议格式**：NovAtel:OEM4/V/6，OEM3, OEMStar、Superstar II、 Hemisphere、Crescent、u‐blox:LEA-4T/5T/6T、SkyTraq、JAVAD 、GW10-II/III 和 NVS。

* **支持外部通信**：Serial、TCP/IP、NTRIP、本地日志文件、FTP 和 HTTP。

* **提供许多代码库和API**：卫星和导航系统函数、矩阵和向量函数，时间和字符串函数、坐标的转换，输入和输出函数、调试跟踪函数、平台依赖函数、定位模型、大气模型、天线模型、地球潮汐模型、大地水准面模型、基准转换、RINEX函数、星历和时钟函数、精密星历和时钟、接收机原始数据函数、RTCM函数，解算函数、谷歌地球KML转换、SBAS函数、选项（option）函数、流数据输入和输出函数、整周模糊度解算、标准定位、精密定位、后处理定位（解算）、流服务器函数、RTK服务器函数、下载函数。

> 推荐阅读：[不迷途导航程序员：RTKLIB 源码阅读笔记](https://mp.weixin.qq.com/s/2D3V0qDh6fwt_tZ0225znw)

---

## 02-GAMP：基于 RTKLIB 的后处理双频 PPP

GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些些多余的函数、代码简洁化，精简出后处理双频 PPP 部分，并对算法进行改进增强。对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的，是用纯 C 语言编写，由于做了简化，代码比 RTKLIB 原版还要简单；使用也非常简单，软件包里直接有 VS 工程，和组织好的配置、数据文件，简单改改路径就能算出结果。

![GAMP](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GAMP.png)

---

## 03-PSINS：MATLAB-C++ 捷联惯导工具箱

PSINS（**P**recise **S**trapdown **I**nertial **N**avigation **S**ystem 高精度捷联惯导系统算法）工具箱由西北工业大学自动化学院惯性技术教研室严恭敏老师开发和维护。工具箱分为Matlab和C++两部分。主要应用于**捷联惯导**系统的数据处理和算法验证开发，它包括**惯性传感器数据分析**、**惯组标定**、**初始对准**、**惯导AVP**（姿态-速度-位置）更新解算、**组合导航**Kalman滤波等功能。C++部分采用 VC6 编写，可以用于嵌入式开发。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231224221345981.png" alt="image-20231224221345981" style="zoom:50%;" />

---

## 04-Ginan



澳大利亚

基于 RTKLIB 改写，做了面向对象封装，

大量使用 C++11/14/17 的新特性，使用了 Boost库，PEA 的主函数就有 500 行，读起来比较困难；

依赖的库较多，好在提供了 Docker；

可以运行在 Linux 和 MacOS，在 Windows 下需运行在 WSL 或 Docker 环境

文档很详细，而且可以生成 Doxygen，根据注释生成网站，可以方便的查看类型嵌套和程序调用

包括 PEA 定位和 POD 定轨程序，

PEA 用 C++、POD 部分用 Fortune，绘图和批处理脚本用 Python

矩阵运算主要用 Eigen 实现，也用 OpenBLAS 进行多线程矩阵运算

> 推荐阅读：[不迷途导航程序员：PEA源码阅读笔记](https://mp.weixin.qq.com/s/Z1-WT7ulJBPhN5wdVlZ17Q)

---

## 05-goGPS：MATLAB-GNSS 数据处理

**goGPS**是一个处理GNSS原始数据的软件，最初支持单频低成本GPS接收机数据，但现在也可以用来处理多频多系统GNSS数据。它实现了多种算法来解算，目前包括两个主要的最小二乘法（LS）引擎：一个基于于组合观测数据（例如无电离层观测）；另一个能够使用所有的频率和记录的信号数据，而不进行任何组合（电离层延迟是正常方程的参数）。组合和不组合的引擎都支持PPP、NET解算。目前只支持静态测站的解算，还不能动态解算。

<img src="https://gogps-project.github.io/wiki/images/goGPS_MainWindow.png?raw=true" alt="goGPS Main Window" style="zoom: 33%;" />

---

## 06-GICI-LIB：GNSS+INS+Camera 图优化融合定位

GICI-LIB 全称 **G**NSS/**I**NS/**C**amera **I**ntegrated Navigation Library，是上海交大最新开源的一套基于图优化的 GNSS+INS+Camera 集成导航定位库。基于 RTKLIB 处理 I/O 流、编解码；基于 OKVIS 因子图优化类型封装；基于 SVO 做特征提取。以 GNSS 为主，再加入 INS、Camera 做组合，支持相当多的数据格式、定位模式，包含很多 GNSS 因子、惯导因子、视觉因子及运动约束。以处理实时数据为主，后处理也采用模拟实时数据处理的方式进行。

- **论文**：[GICI-LIB: A GNSS/INS/Camera Integrated Navigation Library](https://arxiv.org/abs/2306.13268)，可以[在这](https://arxiv.org/pdf/2306.13268.pdf)下载
- **源码**：[https://github.com/chichengcn/gici-open](https://github.com/chichengcn/gici-open)
- **数据**：[https://github.com/chichengcn/gici-open-dataset](https://github.com/chichengcn/gici-open-dataset)

典型应用方式如下图：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1689512108793.png" alt="1689512108793"  />

包含以下估计器：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1690024555067.png" alt="1690024555067" style="zoom: 50%;" />

> 补充：GICI 还在持续更新，前段时间看作者还准备加 PPP 紧组合模式。

---

## 07-SoftGSS：MATLAB-GPS 软件接收机

SoftGNSS 是《软件定义的GPS和伽利略接收机》附带的程序，MATLAB 编写，实现了一套最简单的 GNSS 软件接收机功；输入经过天线接收，射频前端滤波下变频后的数字中频信号文件，进行 GPS L1 C/A 码的捕获跟踪，生成伪距观测值，解译导航电文，最小二乘定位解算；代码量很小也很简单，适合作为 GNSS 基带数字信号处理的入门阅读程序。主要执行流程如下：

![SoftGNSS流程图](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/SoftGNSS%25E6%25B5%2581%25E7%25A8%258B%25E5%259B%25BE.png)

---

## 08-VINS：

与双目相机和 RGB-D 相机相比，单目相机具有结构简单、成本低和处理速度快的优点。然而，单目 VSLAM 存在尺度不确定性、无法对齐位姿和重力方向的自身缺点和快速运动导致的运动模糊的环境下容易跟踪丢失等不足。为弥补此问题，可将单目相机和 IMU 相结合的传感器融合，这种融合方案被称为单目视觉惯性里程计（Visual Inertial Odometry，VIO）或单目视觉惯性 SLAM（Visual-inertial SLAM，VINS）。

* IMU 也可以弥补视觉 SLAM 在短时间、快速运动上的不足，另外由于 IMU 不依赖外界环境信息，对环境变化不敏感，也可以在少纹理、明暗变化较大或者光线较弱场景内提供短期的定位方案以及位姿估计方案。

* 较之惯性信息，视觉里程计可以提供丰富的外界信息，在低速平稳的运动中位姿估计稳定，而且视觉里程计在长时间运行后的漂移较小，并且可以通过回环检测修正自身位置以减小累积误差。

香港科技大学沈劭劼团队开发的 VINS 系统，用了一种紧耦合的非线性优化方法。该团队在 2017 年发布的 VINS-Mono 通过在四元数上进行 IMU 的预积分，并且采用滑动窗口法融合 IMU 信息和相机观测到的特征数据，实现了数据的紧耦合。并且采用四自由度的图优化方法实现了回环检测模块，来得到全局约束。在 2019 年，该团队又发布了 VINS-Fusion，在 VINS-Mono 的基础上又加入了双目、双目+IMU 等更多的传感器类型，以及支持了 VINS 和 GPS 的融合。它支持在线标定相机及 IMU 参数及鱼眼相机模型，并且支持保存当前地图和加载过往地图。在与 IMU 的结合上，它采用了四元数积分方案，与视觉信息进行紧耦合，具有很强的鲁棒性和定位精度。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-1939bbe6498166275bf55eec6b68542f_r.jpg" alt="img" style="zoom: 67%;" />

---

## 09-ORB-SLAM3：

ORB 指 **O**riented FAST and **r**otated **B**RIEF，是一种结合 FAST 和 BRIEF，并引入旋转不变性的一种特征点和描述子；SLAM 指 **S**imultaneous **L**ocalization **a**nd **M**apping，指的是同时进行实时定位和地图构建。

ORB-SLAM3 是**迄今为止，最完整的视觉惯性 SLAM 系统系统**，它是第一个集成了单目相机、双目相机、RGB-D相机，以及单目相机结合 IMU、双目相机结合 IMU 的 SLAM 系统。并且在 ORB-SLAM2 的基础上，改进了相机模型，使其不再局限于传统的小孔成像模型，而是可以**扩展到鱼眼模型**。在与 IMU 的结合上，它根据运动模型在流形上进行 **IMU 的预积分**的方式，然后采用非线性优化的思想，**将 IMU 的预积分结果和视觉 SLAM 的重投影模型一同进行图优化，使得预积分残差以及重投影误差共同达到最小**，以此来完成视觉信息和惯导系统的**紧耦合**。并且它采用了更为快速的**初始化**方法，以及丢失跟踪后利用惯导系统快速**重定位**方法。此外，它还采用**地图集**的方式，实现了对大场景的定位建图。这也是如今众多开源方案中，功能最强大、最精准的方法。系统框图如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230815102741960.png" alt="image-20230815102741960" style="zoom:50%;" />

---

## 10-GNSS-SDR：GNSS 软件接收机

GNSS-SDR（**GNSS:** **G**lobal **N**avigation **S**atellite **S**ystems、**SDR:** **S**oftware **D**efined **R**eceiver），在 GitHub 上搜索 “GNSS” 排第一的仓库，收藏量也高达 1.3k。与上面列举的 RTKLIB、GAMP 等 GNSS 数据处理软件不同，GNSS-SDR 直接对信号进行处理，是一个用 C++ 实现的 GNSS 软件接收机开源项目。有了 GNSS-SDR，用户可以通过创建一个图来构建 GNSS 软件接收器，图中的节点是信号处理块，线条代表它们之间的数据流。该软件为不同的合适射频前端提供接口，并实现从接收器一直到 PVT 解算的所有功能。它的设计允许任何形式的定制，包括信号源、信号处理算法、与其他系统的互操作性、输出格式的互换，并为所有中间信号、参数和变量提供接口。

软件旨在促进新信号处理技术的发展，提供一种简便的方法来衡量这些技术对接收机整体性能的影响。通过对每个软件模块进行系统功能验证，以及使用真实和合成信号对整个接收机进行实验验证，对所有流程进行测试。

目前的技术仍无法以卫星发射频率（约 1.5 GHz）对信号进行数字处理，因此我们仍需要一个射频前端，将信号降频到较低频率，在此过程中进行一些滤波和放大，并以一定的速率进行采样，将量化的数字原始采样流传输到计算平台（通过 USB、以太网等）。

软件接收机可在普通的 PC 中运行，并通过 USB 和以太网总线为各种市售或定制的射频前端提供接口，使处理算法适应不同的采样频率、中间频率和采样分辨率。它还可以处理存储在文件中的原始数据样本。软件对可用的卫星信号进行信号采集和跟踪，对导航信息进行解码，并计算定位算法所需的观测值，最终实现完整导航解决方案。处理输出可存储在 RINEX 文件中，或通过 TCP/IP 服务器以 RTCM 3.2 消息形式实时传输。导航结果以 KML 和 GeoJSON 格式存储。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GeneralBlockDiagram.png" alt="GeneralBlockDiagram" style="zoom:50%;" />

---

## 11-TGINS：







---

## 12-KF-GINS：

KF-GINS 是武大 i2Nav 实验室开源的一套松组合导航程序；可以读取 IMU 数据文件、GNSS 结果文件，进行松组合解算，计算位置、速度、姿态、陀螺仪零偏、加速度计零偏、陀螺仪比例、加速度计比力，共 21 维状态向量。代码量小，有详细的文档、注释和讲解，代码结构很好理解，有一些可以学习的工程技巧。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230928094541518.png" alt="image-20230928094541518" style="zoom: 33%;" />

![image-20230925181044694](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20230925181044694.png)

* **项目开源地址**：https://github.com/i2Nav-WHU
* **i2NAV组合导航讲义、数据集**：http://www.i2nav.cn/index/newList_zw?newskind_id=13a8654e060c40c69e5f3d4c13069078
* **介绍视频**：https://www.bilibili.com/video/BV1Zs4y1B7m2/

---

## 13-OB-GINS：

武大 I2NAV 开源，基于图优化的 IMU/GNSS 松组合解算，IMU 预积分算法相比以视觉为主的 ORB-SLAM3、VINS 要精细一些。

![OB-GINS 执行流程](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/OB-GINS%2520%25E6%2589%25A7%25E8%25A1%258C%25E6%25B5%2581%25E7%25A8%258B.png)

相关链接：

* 开源地址：https://github.com/i2Nav-WHU/OB_GINS，
* 相关论文：
  * Hailiang Tang, Tisheng Zhang, Xiaoji Niu, Jing Fan, and Jingnan Liu, “Impact of the Earth Rotation Compensation on MEMS-IMU Preintegration of Factor Graph Optimization,” *IEEE Sensors Journal*, 2022. [下载](http://www.i2nav.com/ueditor/jsp/upload/file/20220801/1659348408510061111.pdf)
  * Junxiang Jiang, Xiaoji Niu, and Jingnan Liu, “Improved IMU Preintegration with Gravity Change and Earth Rotation for Optimization-Based GNSS/VINS,” *Remote Sensing*, vol. 12, no. 18, p. 3048, Sep. 2020, doi: [10.3390/rs12183048](https://doi.org/10.3390/rs12183048). [下载](https://sci-hub.se/10.3390/rs12183048)
  * Le Chang, Xiaoji Niu, and Tianyi Liu, “GNSS/IMU/ODO/LiDAR-SLAM Integrated Navigation System Using IMU/ODO Pre-Integration,” *Sensors*, vol. 20, no. 17, p. 4702, Aug. 2020, doi: [10.3390/s20174702](https://doi.org/10.3390/s20174702). [下载](https://www.mdpi.com/1424-8220/20/17/4702/pdf)

---

## 14-ROS导航功能包

功能简单来说，就是根据输入的里程计等传感器的信息流和机器人的全局位置，通过导航算法，计算得出安全可靠的机器人速度控制指令。广泛用在一些对可靠性要求没那么高的自主导航机器人场景中，比如扫地机器人、物流机器人等。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20231111160631063.png" alt="image-20231111160631063" style="zoom:50%;" />

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1-13.png" alt="img" style="zoom:50%;" />

---

## 15-北斗GPS双模软件接收机

《北斗GPS双模软件接收机》书配套程序，MATLAB 编写，程序运行相当耗时，70s 的示例程序要算几个小时。

![GnssRcvr_V14程序文件结构](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/GnssRcvr_V14%E7%A8%8B%E5%BA%8F%E6%96%87%E4%BB%B6%E7%BB%93%E6%9E%84.png)

* 网址：http://www.gnssbook.cn/book2/index.html

* 程序下载：http://www.gnssbook.cn/book2/GnssRcvr_V14.rar

* 实例北斗GPS双模中频数据文件下载：[UTREK210_16369000_70s.DAT](https://pan.baidu.com/s/1EWB0oQxDneNDk9iqExLuqQ)，提取码: 829c 

---

## 16-PocketSDR

PocketSDR 是 RTKLIB 作者写的一款 GNSS 软件接收机，包含一个射频前端和一套后处理 GNSS 接收机程序（只支持后处理），实现了一整套完整的 GNSS 接收机功能，采用 C、Python 编写，支持几乎所有的 GNSS 信号（比商业接收机支持的还要多），目前 0.8 版本的程序支持的信号如下：

* **GPS**: L1C/A, L1CP, L1CD, L2CM, L5I, L5Q, 
* **GLONASS**: L1C/A, L2C/A, L3OCD, L3OCP,
* **Galileo**: E1B, E1C, E5aI, E5aQ, E5bI, E5bQ, E6B, E6C,
* **QZSS**: L1C/A, L1C/B, L1CP,L1CD, L1S, L2CM, L5I, L5Q, L5SI, L5SQ, L6D, L6E, 
* **BeiDou**: B1I, B1CP, B1CD, B2I,B2aD, B2aP, B2bI, B3I, 
* **NavIC**: L5-SPS, SBAS: L1C/A, L5I, L5Q

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240110092301571.png" alt="image-20240110092301571" style="zoom:50%;" />
