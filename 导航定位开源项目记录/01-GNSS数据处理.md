<div align="center">
<h1>GNSS数据处理开源项目</h1>
</div>



<div align="center">
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
    <a href="https://www.zhihu.com/people/dao-ge-92-60/"><img src="https://img.shields.io/badge/Zhihu-知乎-blue" /></a>
    <img src="https://komarev.com/ghpvc/?username=LiZhengXiao99&label=Views&color=0e75b6&style=flat" alt="访问量统计" />
</div>
<br/>

* **[RTKLIB](https://www.rtklib.com/)**：由一个**核心程序库**和多个**命令行程序**、**界面程序**组成；代码规范、功能完善、可拓展性好，许多 GNSS 导航定位程序开源程序都是基于 RTKLIB 二次开发衍生而来，适合作为 GNSS 入门学习的代码；GNSS 所需的基本功能都有，支持的数据格式很多，既可以实时解算也可以后处理，既可以接自己的 GNSS 模块也可以连 IGS 的数据流，既可以解算自己采集的数据也可以算 IGS 测站的数据，既可以 RTK 也可以 PPP。
* [RTKLIB-Demo5](https://github.com/rtklibexplorer/RTKLIB)：美国的学者 [rtklibexplorer](https://github.com/rtklibexplorer)  在 RTKLIB 基础上，针对低成本接收机进行优化，在这三年的谷歌分米级手机定位比赛中都排在前几名；因为 RTKLIB 很久不更新了，而且这个版本效果不错，所以有很多人自己用这个版本；作者写了不少博客记录他做的工作，发在他的个人网站上：
* [rtklib-py](https://github.com/rtklibexplorer/rtklib-py)：Python 版 RTKLIB，由 [rtklibexplorer](https://github.com/rtklibexplorer) 编写。
* [rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge)、[rtkrcv_ros](https://github.com/ajbfinesc/rtkrcv_ros)：提供了 ROS 框架支持的 RTKLIB， ROS 节点，支持 ROS 下实现定位解算，
* [gnssgo](https://github.com/FengXuebin/gnssgo)：Go 语言版的 RTLIB。
* **[GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)**：GAMP 全称 (**G**NSS  **A**nalysis software for **M**ulti-constellation and multi-frequency **P**recise positioning)，在 RTKLIB 的基础上，将一些些多余的函数、代码简洁化，精简出后处理双频 PPP 部分，并对算法进行改进增强。对初学者非常友好，在我接触过的导航定位开源程序中算是最简单的，是用纯 C 语言编写，由于做了简化，代码比 RTKLIB 原版还要简单；使用也非常简单，软件包里直接有 VS 工程，和组织好的配置、数据文件，简单改改路径就能算出结果。
* **[Ginan](https://github.com/GeoscienceAustralia/ginan)**：澳大利亚
* **[goGPS](https://github.com/goGPS-Project/goGPS_MATLAB)**：
* [BNC](https://igs.bkg.bund.de/ntrip/bnc)：全称 BKG Ntrip Cilent，主要是数据流处理，也支持实时 PPP 解算，最早基于 Qt4 编写，后迁移到 Qt5。
* [GFZRNX](https://dataservices.gfz-potsdam.de/panmetaworks/showshort.php?id=escidoc:1577894)：由德国地学中心 (GFZ)开发，Rinex 文件处理，功能包括：版本转换、文件合并、
* [GAMIT/GLOBK]()：美国 MIT 开发，
* [GraphGNSSLib](https://github.com/weisongwen/GraphGNSSLib)：图优化 GNSS，港理工，基于 RTKLIB，基于 ROS1，支持图优化 SPP 和 RTK，三篇参考论文，港理工同课题组出的 GLIO 程序在此基础上开发 。
* [Net_Diff](https://github.com/YizeZhang/Net_Diff)：
* [POSGO](https://github.com/lizhengnss/POSGO)：图优化 GNSS，武大李政开发，投稿到 GPS Solution，目前只支持 SPP，主要用在手机定位，
* [Pride-PPPAR](https://github.com/PrideLab/PRIDE-PPPAR)：后处理 PPP，武大耿江辉课题组开发，投稿到 GPS Solution，持续更新，模糊度固定可达到毫米级定位精度，可以用于算 RTK 基准站坐标，地震学等地学研究。
* [PPPwizard](http://www.ppp-wizard.net/)：
* [GPSTk](https://github.com/SGL-UT/GPSTk)、[gnsstk](https://github.com/SGL-UT/gnsstk)、[gnsstk-apps](https://github.com/SGL-UT/gnsstk-apps)：GPSTk 改名为 GNSSTk，并且分为 [gnsstk](https://github.com/SGL-UT/gnsstk)、[gnsstk-apps](https://github.com/SGL-UT/gnsstk-apps) 两个仓库。
* [G-NUT/Anubis](https://www.pecny.cz/Joomla25/index.php/gnss/sw/anubis)：观测数据质量分析
* [Teqc](https://www.unavco.org/software/data-processing/teqc/teqc.html)：观测数据质量分析
* [RNXQCE](https://github.com/cuizilu/RNXQCE)：RINEX 观测数据质量分析，作者给的介绍所是 TEQC 很久没更新，所以开发了 RNXQCE。
* [Bernese](https://www.bernese.unibe.ch/)：伯尔尼大学天文研究所（AIUB）开发的 GNSS 数据处理软件，主要是欧洲轨道确定中心（CODE）在用，
* [gLAB](https://github.com/valgur/gLAB?tab=readme-ov-file)：加泰罗尼亚理工大学天文学和地球数学研究小组（gAGE）根据欧洲空间局（ESA）合同开发的
* [gnssrefl](https://github.com/kristinemlarson/gnssrefl)：Python 编写的 GNSS 干涉反射测量（GNSS-IR）软件包，[文档](https://gnssrefl.readthedocs.io/en/latest/)写的挺详细，主要是用信噪比来计算。
* [gnssSNR](https://github.com/kristinemlarson/gnssSNR)：gnssrefl 作者的另一套程序，从 RINEX 文件中提取信噪比、方位角、高度角，基于 Fortran 编写。
* [RobustGNSS](https://github.com/wvu-navLab/RobustGNSS)：基于 GTSAM 的图优化 GNSS。
* [PPP-BayesTree](https://github.com/wvu-navLab/PPP-BayesTree)：基于 GTSAM、GPSTK 的图优化 GNSS。
* [ntrip](https://github.com/sevensx/ntrip)：使用 NTRIP2.0 协议的简单 ntrip caster/客户端/服务器示例程序。
* [rtcm](https://github.com/Node-NTRIP/rtcm)：TypeScript 编写的 RTCM 解码器/编码器，适用于 RTCM 3 的所有报文类型。
* [cors](https://github.com/Erensu/cors)：武大苏景岚开发，连续运行参考站 (CORS) 程序，提供网络 RTK 差分服务。支持：多频多系统网络RTK解算：基线解算、模糊度闭合检验、虚拟参考站技术、基准站观测数据时间同步技术、支持连接大规模基准站、基于Delaunay三角形基准站组网、基准站大气延迟误差计算、基准站/虚拟站完好性监测、支持虚拟参考站数据播发能力和网络RTK差分定位服务。
* [georinex](https://github.com/geospace-code/georinex)：Python 编写的文件格式转换程序，实现将 RINEX 转为气象领域常用的 NetCDF4 / HDF5。
* [FAST](https://github.com/ChangChuntao/FAST)：GNSS 数据下载，命令行程序，支持 Windows/Linux。
* [gnsspy](https://github.com/GNSSpy-Project/gnsspy)：GNSSpy 是一个免费开源库，用于处理多 GNSS 和不同版本（2.X 和 3.X）的 RINEX 文件。它通过使用精确星历和时钟文件的伪距观测数据进行最小二乘法调整，提供单点定位（SPP）解决方案。GNSSpy 可用于 RINEX 文件的编辑（切片、抽取、合并）和质量检查（多径、电离层延迟、信噪比）。对于单频 RINEX 数据，可通过 IGS 的 GNSS 大气模型计算电离层延迟；对于双频 RINEX 数据，则可去除电离层延迟。它可用于可视化 GNSS 数据，如天空图、方位角-高程图、时间-高程图、地面轨迹图和波段图。此外，该库还可用于基本的大地测量计算，如参考椭球面上的大地测量位置和投影计算。
* [raPPPid](https://github.com/TUW-VieVS/raPPPid)：实时快速 PPP，投稿到 GPS Solution，
* [FCB-FILES](https://github.com/FCB-SGG/FCB-FILES)：武大张小红团队开发的 FCB
* [MG_APP](https://github.com/XiaoGongWei/MG_APP)：
* [gps_amcl](https://github.com/midemig/gps_amcl)：AMCL 是 ROS 导航功能包里
* [deep_gnss](https://github.com/Stanford-NavLab/deep_gnss)：深度学习
* [RTKinGSS](https://github.com/shaolinbit/RTKinGSS)：
* [gnss_lib_py](https://github.com/Stanford-NavLab/gnss_lib_py)：
* [PNT-Integrity](https://github.com/cisagov/PNT-Integrity)：
* [GDDS](https://geodesy.noaa.gov/gps-toolbox/gdds.shtml)：数据下载
* [QGOPDD](https://github.com/yhw605/QGOPDD)：数据下载，南师大本科生颜瀚文编写，基于 Qt 编写，支持使用 ftp 客户端从武汉大学 IGS 数据中心下载 GNSS 观测数据。特点是提供了测站选择的界面。
* [groops](https://github.com/groops-devs/groops)：
* [OREKIT](https://link.zhihu.com/?target=https%3A//www.orekit.org/)：卫星定轨，
* [gnss-tec](https://github.com/gnss-lab/gnss-tec)：Python 编写，基于 GNSS 伪距、载波，重建电离层中的斜向总电子含量。
* [utm](https://github.com/sfegan/utm)：LLH 转 UTM。
* [PW_from_GPS](https://github.com/ZiskinZiv/PW_from_GPS)：Python 编写，GNSS 水汽反演。
* [hgpt_model](https://github.com/pjmateus/hgpt_model)：基于ERA5的每小时全球气压、温度和相对湿度模型。
* [PyGPSClient](https://github.com/semuconsulting/PyGPSClient)：
* [STM32-GNSS](https://github.com/SimpleMethod/STM32-GNSS)：
* [HASlib](https://github.com/nlsfi/HASlib)：
* [GNSSAMS](https://github.com/GanAHE/GNSSAMS)：
* [ppp-tools](https://github.com/aewallin/ppp-tools)：
* [cssrlib](https://github.com/hirokawa/cssrlib)：
* [gpstest](https://github.com/barbeau/gpstest)：
* [iSniff-GPS](https://github.com/hubert3/iSniff-GPS)：
* [hypatia](https://github.com/snkas/hypatia)：
* [gps-measurement-tools](https://github.com/google/gps-measurement-tools)：
* [FE-GUT](https://github.com/zhaoqj23/FE-GUT)：清华大学开发的图优化
* [gnss-RX](https://github.com/HeryMwenegoha/gnss-RX)：
* [Pypredict](https://github.com/spel-uchile/Pypredict)：
* [novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver)：
* [GNSS-Radar](https://github.com/taroz/GNSS-Radar)：Web 程序，输入时间和经纬度，显示指定地点的 GNSS 星座，网址：http://www.taroz.net/GNSS-Radar.html。
* [gnss-odometry](https://github.com/mgoar/gnss-odometry)：基于GTSAM的图优化TDCP。
* [rviz_satellite](https://github.com/nobleo/rviz_satellite)
* [GPSToolbox 投稿](https://geodesy.noaa.gov/gps-toolbox/exist.htm)：
* [barbeau-awesome-gnss 整理](https://github.com/barbeau/awesome-gnss)：
* [hdkarimi-awesome-gnss 整理](https://github.com/hdkarimi/awesome-gnss)：
* [mcraymer 整理](https://mcraymer.github.io/geodesy/index.html)：