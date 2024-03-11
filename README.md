![Navigation-Learning-cover](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/Navigation-Learning-cover.png)

[TOC]

---

* 本仓库是我学习的记录，会长期更新，分享出来，希望有人用得上，也激励着自己坚持学下去。

- 有些程序会详细写（比如 RTKLIB、KF-GINS），有些只画个流程图、列举重点的函数。

- 水平不高，理解尚浅，列举的理论公式无法保证准确性，仅供参考，千万不要照着我的笔记来写代码写论文，更不要直接复制到论文里，希望小伙伴们注意。

- 不建议直接下载整个仓库，内容太多可能大部分都用不着，Markdown 和 PDF 可以在线看，想要的文件也可以[单独下载](https://zhuanlan.zhihu.com/p/578116206)。Github 上的 Markdown 公式可能显示不完全，可以下载下来用 [Typora](https://typoraio.cn/) 看。

- 本仓库所有内容都可以随意转载，可用于任何目的，不必征求我的意见。

- 如果您认可我的整理，并且想在该仓库更新时在 Github All activity 页面和 Github 通知页面看到本仓库更新，欢迎您 watch 本仓库而不是 fork 本仓库，fork 到自己名下的仓库只会保持您 fork 时的状态，不会自动更新。如果您愿意 star 一下那当然更好不过啦~

- 有问题欢迎随时联系我：**电话** (15255291038)、**微信** (lizhengxiao99)、**QQ** (1482275402)、**Email** (dauger@126.com)


---

记录一些我看到过的项目，其中加粗的项目会在本仓库介绍：

|       类型        |                          项目/网址                           |
| :---------------: | :----------------------------------------------------------: |
| **GNSS 数据处理** | **[RTKLIB](https://www.rtklib.com/)**、[RTKLIB-Demo5](https://github.com/rtklibexplorer/RTKLIB)、[rtklib-py](https://github.com/rtklibexplorer/rtklib-py)、[rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge)、[rtkrcv_ros](https://github.com/ajbfinesc/rtkrcv_ros)、**[GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)**、**[Ginan](https://github.com/GeoscienceAustralia/ginan)**、**[goGPS](https://github.com/goGPS-Project/goGPS_MATLAB)**、[BNC](https://igs.bkg.bund.de/ntrip/bnc)、[GFZRNX](https://dataservices.gfz-potsdam.de/panmetaworks/showshort.php?id=escidoc:1577894)、[GAMIT/GLOBK]()、[GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)、[Net_Diff](https://github.com/YizeZhang/Net_Diff)、[POSGO](https://github.com/lizhengnss/POSGO)、[Pride-PPPAR](https://github.com/PrideLab/PRIDE-PPPAR)、[PPPwizard](http://www.ppp-wizard.net/)、[G-NUT/Anubis](https://www.pecny.cz/Joomla25/index.php/gnss/sw/anubis)、[Teqc](https://www.unavco.org/software/data-processing/teqc/teqc.html)、[Bernese](https://www.bernese.unibe.ch/)、[gLAB](https://github.com/valgur/gLAB?tab=readme-ov-file)、[groops](https://github.com/groops-devs/groops)、[gnssrefl](https://github.com/kristinemlarson/gnssrefl)、[RobustGNSS](https://github.com/wvu-navLab/RobustGNSS)、[GAMPII-GOOD](https://github.com/zhouforme0318/GAMPII-GOOD)、[ntrip](https://github.com/sevensx/ntrip)、[rtcm](https://github.com/Node-NTRIP/rtcm)、[cors](https://github.com/Erensu/cors)、[georinex](https://github.com/geospace-code/georinex)、[PPP-BayesTree](https://github.com/wvu-navLab/PPP-BayesTree)、[FAST](https://github.com/ChangChuntao/FAST)、[gnsspy](https://github.com/GNSSpy-Project/gnsspy)、[gnssgo](https://github.com/FengXuebin/gnssgo)、[raPPPid](https://github.com/TUW-VieVS/raPPPid)、[FCB-FILES](https://github.com/FCB-SGG/FCB-FILES)、[MG_APP](https://github.com/XiaoGongWei/MG_APP)、[gps_amcl](https://github.com/midemig/gps_amcl)、[RTKinGSS](https://github.com/shaolinbit/RTKinGSS)、[PNT-Integrity](https://github.com/cisagov/PNT-Integrity)、[GDDS](https://geodesy.noaa.gov/gps-toolbox/gdds.shtml)、 [OREKIT](https://link.zhihu.com/?target=https%3A//www.orekit.org/)、[gnss-tec](https://github.com/gnss-lab/gnss-tec)、[PW_from_GPS](https://github.com/ZiskinZiv/PW_from_GPS)、[PyGPSClient](https://github.com/semuconsulting/PyGPSClient)、[STM32-GNSS](https://github.com/SimpleMethod/STM32-GNSS)、[HASlib](https://github.com/nlsfi/HASlib)、[GNSSAMS](https://github.com/GanAHE/GNSSAMS)、[ppp-tools](https://github.com/aewallin/ppp-tools)、[cssrlib](https://github.com/hirokawa/cssrlib)、[hypatia](https://github.com/snkas/hypatia)、[FE-GUT](https://github.com/zhaoqj23/FE-GUT)、[gnss-RX](https://github.com/HeryMwenegoha/gnss-RX)、[Pypredict](https://github.com/spel-uchile/Pypredict)、[GPSToolbox 投稿](https://geodesy.noaa.gov/gps-toolbox/exist.htm)、[awesome-gnss 整理](https://github.com/barbeau/awesome-gnss)、[mcraymer 整理](https://mcraymer.github.io/geodesy/index.html) |
| **GNSS 信号处理** | **[GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)**、**[SoftGNSS](https://github.com/kristianpaul/SoftGNSS)**、[SoftGNSS-python](https://github.com/perrysou/SoftGNSS-python)、**[PocketSDR](https://github.com/tomojitakasu/PocketSDR)**、[GNSS-SDRLIB](https://github.com/taroz/GNSS-SDRLIB)、[nut2nt](https://github.com/amungo/nut2nt)、[GNSS-GPS-SDR](https://github.com/JiaoXianjun/GNSS-GPS-SDR)、[SatDump](https://github.com/SatDump/SatDump)、[gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim)、[beidou-sdr-sim](https://github.com/yangfan852219770/beidou-sdr-sim)、[galileo-sdr-sim](https://github.com/harshadms/galileo-sdr-sim)、[gps-qzss-sdr-sim](https://github.com/iGNSS/gps-qzss-sdr-sim)、[SignalSim](https://github.com/globsky/SignalSim)、[GPS_GAL_SSS](https://github.com/domonforyou/GPS_GAL_SSS)、[greta-oto](https://github.com/globsky/greta-oto)、[BD3_FPGA](https://github.com/whc2uestc/BD3_FPGA)、[GNSS-matlab](https://github.com/danipascual/GNSS-matlab)、[SDR-GB-SAR](https://github.com/jmfriedt/SDR-GB-SAR)、[gps_rf_frontend_sim](https://github.com/iliasam/gps_rf_frontend_sim)、[GNSS-VHDL](https://github.com/danipascual/GNSS-VHDL)、[gnss-baseband](https://github.com/j-core/gnss-baseband)、[Analog-GPS-data-receiver](https://github.com/leaningktower/Analog-GPS-data-receiver)、[GNSS-DSP-tools](https://github.com/pmonta/GNSS-DSP-tools)、[hard_sydr](https://github.com/aproposorg/hard_sydr)、[B1C_Signals_Simulation](https://github.com/pandaclover/B1C_Signals_Simulation)、[CU-SDR-Collection](https://github.com/gnsscusdr/CU-SDR-Collection)、[ESP32_SDR_GPS](https://github.com/iliasam/ESP32_SDR_GPS)、[STM32F4_SDR_GPS](https://github.com/iliasam/STM32F4_SDR_GPS) |
| **INS、组合导航** | **[PSINS](http://www.psins.org.cn/)**、**[KF-GINS](https://github.com/i2Nav-WHU/KF-GINS)**、**[OB-GINS](https://github.com/i2Nav-WHU/OB_GINS)**、**[TGINS](https://github.com/heiwa0519/TGINS)**、[PPPLIB](https://geodesy.noaa.gov/gps-toolbox/PPPLib.htm)、[Compass](https://gitee.com/hw_cc/compass)、[GINav_v2.0](https://github.com/kaichen686/GINav_v2.0-test)、[GINAV](https://github.com/kaichen686/GINav)、[IGNAV](https://github.com/Erensu/ignav)、[MATLAB-Groves](https://github.com/zbai/MATLAB-Groves)、[imu_x_fusion](https://github.com/cggos/imu_x_fusion)、[OpenIMU](https://github.com/introlab/OpenIMU)、[eagleye](https://github.com/MapIV/eagleye)、[ai-imu-dr](https://github.com/mbrossar/ai-imu-dr)、[NaveCodePro](https://github.com/zelanzou/NaveCodePro)、[nav_matlab](https://github.com/yandld/nav_matlab)、[kalibr_allan](https://github.com/rpng/kalibr_allan)、[imu_data_simulation](https://github.com/robosu12/imu_data_simulation)、[GPS_IMU_Kalman_Filter](https://github.com/karanchawla/GPS_IMU_Kalman_Filter)、[TightlyCoupledINSGNSS](https://github.com/benzenemo/TightlyCoupledINSGNSS)、 [Wheel-INS ](https://github.com/i2Nav-WHU/Wheel-INS)、[imu_tools](https://github.com/CCNYRoboticsLab/imu_tools)、[imu_utils](https://github.com/gaowenliang/imu_utils)、[GyroAllan](https://github.com/XinLiGH/GyroAllan)、[gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim)、[MEMS-IMU-Denoising](https://github.com/ansfl/MEMS-IMU-Denoising)、[nav_matlab](https://github.com/yandld/nav_matlab)、[agrobot](https://github.com/nesl/agrobot)、[IBG_EKF_TC](https://github.com/Dennissy23/IBG_EKF_TC)、 [ZCJ_GNSSINS_DeepIntegration ](https://github.com/kongtian-SiBu/ZCJ_GNSSINS_DeepIntegration)、[Smartphone_IMU_GPS](https://github.com/alexschultze/Smartphone_IMU_GPS)、[INSTINCT](https://github.com/UniStuttgart-INS/INSTINCT)、[Gait-Tracking](https://github.com/xioTechnologies/Gait-Tracking)、[Machine_Learning_GNSS_IMU_Integration](https://github.com/Akpozi/Machine_Learning_GNSS_IMU_Integration)、[ImuCalibration-Poistion](https://github.com/shenshikexmu/ImuCalibration-Poistion)、[IMU-study 整理](https://github.com/Staok/IMU-study) |
|   **视觉 SLAM**   | **[Vins-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)/[Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**、**[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)**/[2](https://github.com/raulmur/ORB_SLAM2)/[1](https://github.com/raulmur/ORB_SLAM)、[OpenVINS](https://github.com/rpng/open_vins)、[Openvslam](https://github.com/xdspacelab/openvslam)、[svo](https://github.com/uzh-rpg/rpg_svo)、[svo_pro](https://github.com/uzh-rpg/rpg_svo_pro_open)、[msckf_vio](https://github.com/KumarRobotics/msckf_vio)、[okvis](https://github.com/ethz-asl/okvis)、[DM-VIO](https://github.com/lukasvst/dm-vio)、[DSO](https://github.com/JakobEngel/dso)、[DSOL](https://github.com/versatran01/dsol)、[NeRF](https://github.com/bmild/nerf)、[Elasticfusion](https://github.com/mp3guy/ElasticFusion)、[OpenMVG](https://github.com/openMVG/openMVG)、[Kintinuous](https://github.com/mp3guy/Kintinuous)、[Mvision](https://github.com/Ewenwan/MVision)、[rgbdslam_v2](https://github.com/felixendres/rgbdslam_v2)、[awesome-visual-slam 整理](https://github.com/tzutalin/awesome-visual-slam)、[Recent_SLAM_Research 整理](https://github.com/YiChenCityU/Recent_SLAM_Research)、[Awesome CV Works 整理](https://vincentqin.tech/posts/awesome-works/)、[Lee-SLAM-source 整理](https://github.com/AlbertSlam/Lee-SLAM-source)、[awesome-slam 整理](https://github.com/kanster/awesome-slam)、[awesome-NeRF 整理](https://github.com/awesome-NeRF/awesome-NeRF)、[visual-slam-roadmap 整理](https://github.com/changh95/visual-slam-roadmap)、[Visual_SLAM_Related_Research 整理](https://github.com/wuxiaolang/Visual_SLAM_Related_Research) |
|   **激光 SLAM**   | [Gmapping](https://github.com/ros-perception/slam_gmapping)、[Cartographer](https://github.com/cartographer-project/cartographer)、[hector-slam](https://github.com/tu-darmstadt-ros-pkg/hector_slam)、[LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)、[LOAM-Livox](https://github.com/hku-mars/loam_livox)、[A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM)、[SuMa](https://github.com/jbehley/SuMa)、[LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)、[FAST-LIO-SAM](https://github.com/kahowang/FAST_LIO_SAM)、[FAST-LIO](https://github.com/hku-mars/FAST_LIO)、[LIO-Mapping](https://github.com/hyye/lio-mapping)、[CT-ICP](https://github.com/jedeschaud/ct_icp)、[BoW3D](https://github.com/YungeCui/BoW3D)、[LT-Mapper](https://github.com/gisbi-kim/lt-mapper)、[awesome-lidar 整理](https://github.com/szenergy/awesome-lidar)、[awesome-point-cloud-place-recognition 整理](https://github.com/kxhit/awesome-point-cloud-place-recognition)、[awesome-sar 整理](https://github.com/RadarCODE/awesome-sar) |
|   **多源融合**    | **[GICI-LIB](https://github.com/chichengcn/gici-open)**、[GVINS](https://github.com/HKUST-Aerial-Robotics/GVINS)、[GLIO](https://github.com/XikunLiu-huskit/GLIO)、[Multi-Sensor-Fusion](https://github.com/2013fangwentao/Multi_Sensor_Fusion)、[MSF_developed](https://github.com/milkytipo/MSF_developed)、[MINS](https://github.com/rpng/MINS)、[mars_lib](https://github.com/aau-cns/mars_lib)、[imu_x_fusion](https://github.com/cggos/imu_x_fusion)、[IC_GVINS](https://github.com/i2Nav-WHU/IC-GVINS)、[FAST-LIVO](https://github.com/hku-mars/FAST-LIVO)、[sync_gps_lidar_imu_cam](https://github.com/nkliuhui/sync_gps_lidar_imu_cam)、[carvig](https://github.com/Erensu/carvig)、[LVI-SAM](https://link.zhihu.com/?target=https%3A//github.com/TixiaoShan/LVI-SAM)、[NaveGo](https://github.com/rodralez/NaveGo)、[YabLoc](https://github.com/tier4/YabLoc) |
|   **规划控制**    | [**navigation**](https://github.com/ros-planning/navigation)、**[navigation2 ](https://github.com/ros-planning/navigation2)**、[Apollo](https://github.com/ApolloAuto/apollo)、[Autoware](https://github.com/autowarefoundation/autoware)、[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)、[ardupilot](https://github.com/ArduPilot/ardupilot)、[mars](https://github.com/OPEN-AIR-SUN/mars)、[MissionPlanner](https://github.com/ArduPilot/MissionPlanner)、[navigator](https://github.com/Nova-UTD/navigator)、[self-driving-car](https://github.com/ndrplz/self-driving-car)、[Awesome-Self-Driving 整理](https://github.com/iGNSS/Awesome-Self-Driving)、[Autopilot-Notes 整理](https://github.com/gotonote/Autopilot-Notes) |
|     **其它**      | [bayes-filters-lib](https://github.com/robotology/bayes-filters-lib)、[filterpy](https://github.com/rlabbe/filterpy)、[DynAdjust](https://github.com/icsm-au/DynAdjust)、[KalmanFilter](https://github.com/mannyray/KalmanFilter)、[ahrs](https://github.com/Mayitzin/ahrs)、  [uwb-localization](https://github.com/lijx10/uwb-localization)、[Location](https://github.com/yyccR/Location)、[Indoor-Positioning](https://github.com/GXW19961104/Indoor-Positioning)、[positioning-algorithms-for-uwb-matlab](https://github.com/cliansang/positioning-algorithms-for-uwb-matlab)、[UWB_DualAntenna_AoA](https://github.com/ETH-PBL/UWB_DualAntenna_AoA)、[Pedometer](https://github.com/mirraico/Pedometer)、[GetSensorData_Android](https://github.com/lopsi/GetSensorData_Android)、[LoRa_2G4_localization](https://github.com/alphaLeporis/LoRa_2G4_localization)、[DeadReckoning](https://github.com/nisargnp/DeadReckoning)、[Smartlight_UWB](https://github.com/Zekke-e/Smartlight_UWB)、[senslogs](https://github.com/tyrex-team/senslogs)、[vrs](https://github.com/facebookresearch/vrs)、[Pedestrian-Dead-Reckoning](https://github.com/Zidane-Han/Pedestrian-Dead-Reckoning)、[UWBPositioning](https://github.com/Meihai/UWBPositioning)、[util-uwb-dataset](https://github.com/utiasDSL/util-uwb-dataset)、[RSSI-Dataset-for-Indoor-Localization-Fingerprinting](https://github.com/pspachos/RSSI-Dataset-for-Indoor-Localization-Fingerprinting)、[Awesome-Human-Activity-Recognition 整理](https://github.com/iGNSS/Awesome-Human-Activity-Recognition) |
|    **常用库**     | [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)、[OpenBLAS](https://github.com/OpenMathLib/OpenBLAS)、[Gflags](https://github.com/gflags/gflags)、[Glog](https://github.com/google/glog)、[easyloggingpp](https://github.com/abumq/easyloggingpp)、[Ceres-Solver](https://github.com/ceres-solver/ceres-solver)、[g2o](https://github.com/RainerKuemmerle/g2o)、[gtsam](https://github.com/borglab/gtsam)、[Yaml-Cpp](https://github.com/jbeder/yaml-cpp)、[OpenCV](https://github.com/opencv/opencv)、[PCL](https://pointclouds.org/)、[Boost](https://github.com/boostorg/boost)、[better-enums](http://github.com/aantron/better-enums)、[DBoW2](https://github.com/dorian3d/DBoW2) |
| **开源程序作者**  | [池澄](https://github.com/chichengcn/gici-open)、[周峰](https://github.com/zhouforme0318)、[陈超](https://github.com/heiwa0519)、[严恭敏](https://psins.org.cn/sy)、[李政](https://github.com/lizhengnss)、[苏景岚](https://github.com/Erensu)、[陈凯](https://github.com/kaichen686)、[王晗](https://github.com/wh200720041)、[刘国庆](https://github.com/DreamWaterFound)、[高翔](https://github.com/gaoxiang12)、[杨熙](https://github.com/yandld)、[张春杰](https://github.com/kongtian-SiBu)、[魏源](https://github.com/hui-Scholarliness)、[布树辉](https://link.zhihu.com/?target=http%3A//www.adv-ci.com/blog/)、[谢立华](https://link.zhihu.com/?target=https%3A//wanghan.pro/)、[邹丹平](https://link.zhihu.com/?target=http%3A//drone.sjtu.edu.cn/dpzou/index.php)、[XikunLiu-huskit](https://github.com/XikunLiu-huskit)、[cggos](https://github.com/cggos)、[shaolinbit](https://github.com/shaolinbit)、[Welson WEN](https://www.zhihu.com/people/Welson-WEN)、[cggos](https://github.com/cggos)、[zarathustr](https://github.com/zarathustr)、[YizeZhang](https://github.com/YizeZhang)、[globsky](https://github.com/globsky)、[tomojitakasu](https://github.com/tomojitakasu)、[carlesfernandez](https://github.com/carlesfernandez)、[rtklibexplorer](https://github.com/rtklibexplorer)、[GeoscienceAustralia](https://github.com/GeoscienceAustralia)、[taroz](https://github.com/taroz)、[osqzss](https://github.com/osqzss) |
| **SLAM研究团队**  | [香港科技大学空中机器人实验室](https://uav.hkust.edu.hk/)、[香港科技大学机器人与多感知实验室 RAM-LAB](https://www.ram-lab.com/)、[香港中文大学天石机器人实验室](http://ri.cuhk.edu.hk/)、[浙江大学 CAD&CG 国家重点实验室](http://www.zjucvg.net/)、[美国密歇根大学感知机器人实验室（PeRL）](https://robots.engin.umich.edu/About/)、[美国卡耐基梅陇大学机器人研究所](https://www.ri.cmu.edu/)、 [美国加州大学圣地亚哥分校语境机器人研究所](https://existentialrobotics.org/index.html)、[美国特拉华大学人感知与导航组](https://sites.udel.edu/robot/)、[美国麻省理工学院航空航天实验室](http://acl.mit.edu/)、[美国麻省理工学院 SPARK 实验室](http://web.mit.edu/sparklab/)、[美国麻省理工学院海洋机器人组](https://marinerobotics.mit.edu/)、[美国明尼苏达大学多元自主机器人系统实验室](http://mars.cs.umn.edu/index.php)、[美国宾夕法尼亚大学 Vijay Kumar 实验室](https://www.kumarrobotics.org/)、[美国麻省理工学院 Robust Robotics Group]()、[美国佐治亚理工学院智能视觉与自动化实验室](https://link.zhihu.com/?target=http%3A//ivalab.gatech.edu/)、[加拿大蒙特利尔大学机器人与http://groups.csail.mit.edu/rrg/index.php嵌入式 AI 实验室](https://link.zhihu.com/?target=http%3A//montrealrobotics.ca/)、[加拿大舍布鲁克大学智能、交互、综合、跨学科机器人实验室](https://introlab.3it.usherbrooke.ca/)、[瑞士苏黎世大学机器人与感知课题组](http//3A//rpg.ifi.uzh.ch/index.html)、[瑞士苏黎世联邦理工 Vision for Robotics Lab](https://v4rl.ethz.ch/the-group.html)、[瑞士苏黎世联邦理工计算机视觉与几何实验室](https://link.zhihu.com/?target=http%3A//www.cvg.ethz.ch/index.php)、[瑞士苏黎世联邦理工自主系统实验室](https://cvg.ethz.ch/index)、[英国帝国理工学院戴森机器人实验室](https://www.imperial.ac.uk/dyson-robotics-lab/)、[英国牛津大学信息工程学](https://www.robots.ox.ac.uk/)、[德国慕尼黑工业大学计算机视觉组](https://github.com/tum-vision)、[德国马克斯普朗克智能系统研究所嵌入式视觉组](https://ev.is.mpg.de/)、[德国弗莱堡大学智能自主系统实验室](http://ais.informatik.uni-freiburg.de/index_en.php)、[德国波恩大学摄影测量与机器人实验室](https://www.ipb.uni-bonn.de/)、[西班牙萨拉戈萨大学机器人、感知与实时组 SLAM 实验室](http://robots.unizar.es/slamlab/)、[西班牙马拉加大学机器感知与智能机器人课题组](http://mapir.uma.es/mapirwebsite/)、[奥地利格拉茨技术大学计算机图形学与视觉研究所](https://www.tugraz.at/institutes/icg/home/)、[波兰波兹南工业大学移动机器人实验室](http://lrm.put.poznan.pl/)、[澳大利亚昆士兰科技大学机器人技术中心](https://www.qut.edu.au/research/centre-for-robotics)、[澳大利亚机器人视觉中心](https://www.roboticvision.org/)、[日本国立先进工业科学技术研究所](https://www.airc.aist.go.jp/en/intro/) |

下面这些是我编译运行过的开源程序，使用的过程记录在 `<程序编译运行记录>` 文件夹内，当然大部分都只算过测试数据，谈不上有多了解：

|       类型        |                            项目名                            |
| :---------------: | :----------------------------------------------------------: |
| **GNSS 数据处理** | **[RTKLIB](https://www.rtklib.com/)**、[RTKLIB-Demo5](https://github.com/rtklibexplorer/RTKLIB)、**[GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)**、**[Ginan](https://github.com/GeoscienceAustralia/ginan)**、**[goGPS](https://github.com/goGPS-Project/goGPS_MATLAB)** |
| **GNSS 信号处理** |                                                              |
| **INS、组合导航** |                                                              |
|   **视觉 SLAM**   |                                                              |
|   **激光 SLAM**   |                                                              |
|   **多源融合**    |                                                              |
|   **规划控制**    |                                                              |
|     **其它**      |                                                              |

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
