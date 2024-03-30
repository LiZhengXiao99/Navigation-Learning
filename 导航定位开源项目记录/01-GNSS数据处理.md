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
* [RTKLIB-Demo5](https://github.com/rtklibexplorer/RTKLIB)：一个美国的学者在RTKLIB基础上，针对低成本接收机进行优化，在这三年的谷歌分米级手机定位比赛中都排在前几名；因为 RTKLIB 很久不更新了，而且这个版本效果不错，所以有很多人自己用这个版本；作者写了不少博客记录他做的工作，
* [rtklib-py](https://github.com/rtklibexplorer/rtklib-py)：Python 版 RTKLIB
* [rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge)：
* [rtkrcv_ros](https://github.com/ajbfinesc/rtkrcv_ros)：
* **[GAMP](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)**：后处理双频 PPP，在 RTKLIB 基础上做了不少简化和算法增强，适合作为 PPP 入门学习的代码，也有不少人
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
* [GPSTk](https://github.com/SGL-UT/GPSTk)：
* [G-NUT/Anubis](https://www.pecny.cz/Joomla25/index.php/gnss/sw/anubis)：观测数据质量分析
* [Teqc](https://www.unavco.org/software/data-processing/teqc/teqc.html)：观测数据质量分析
* [RNXQCE](https://github.com/cuizilu/RNXQCE)：
* [Bernese](https://www.bernese.unibe.ch/)：
* [gLAB](https://github.com/valgur/gLAB?tab=readme-ov-file)：
* [gnssrefl](https://github.com/kristinemlarson/gnssrefl)：
* [RobustGNSS](https://github.com/wvu-navLab/RobustGNSS)：
* [ntrip](https://github.com/sevensx/ntrip)：
* [rtcm](https://github.com/Node-NTRIP/rtcm)：
* [cors](https://github.com/Erensu/cors)：武大苏景岚开发，
* [georinex](https://github.com/geospace-code/georinex)：
* [PPP-BayesTree](https://github.com/wvu-navLab/PPP-BayesTree)：
* [FAST](https://github.com/ChangChuntao/FAST)：
* [gnsspy](https://github.com/GNSSpy-Project/gnsspy)：
* [gnssgo](https://github.com/FengXuebin/gnssgo)：
* [raPPPid](https://github.com/TUW-VieVS/raPPPid)：实时快速 PPP，投稿到 GPS Solution，
* [FCB-FILES](https://github.com/FCB-SGG/FCB-FILES)：
* [MG_APP](https://github.com/XiaoGongWei/MG_APP)：
* [gps_amcl](https://github.com/midemig/gps_amcl)：AMCL 是 ROS 导航功能包里
* [deep_gnss](https://github.com/Stanford-NavLab/deep_gnss)：深度学习
* [RTKinGSS](https://github.com/shaolinbit/RTKinGSS)：
* [gnss_lib_py](https://github.com/Stanford-NavLab/gnss_lib_py)：
* [PNT-Integrity](https://github.com/cisagov/PNT-Integrity)：
* [GDDS](https://geodesy.noaa.gov/gps-toolbox/gdds.shtml)：数据下载
* [QGOPDD](https://github.com/yhw605/QGOPDD)：数据下载，南师大本科生颜瀚文编写，基于 Qt 编写，
* [groops](https://github.com/groops-devs/groops)：
* [OREKIT](https://link.zhihu.com/?target=https%3A//www.orekit.org/)：卫星定轨，
* [gnss-tec](https://github.com/gnss-lab/gnss-tec)：
* [utm](https://github.com/sfegan/utm)：
* [PW_from_GPS](https://github.com/ZiskinZiv/PW_from_GPS)：
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
* [FE-GUT](https://github.com/zhaoqj23/FE-GUT)：
* [gnss-RX](https://github.com/HeryMwenegoha/gnss-RX)：
* [Pypredict](https://github.com/spel-uchile/Pypredict)：
* [novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver)：
* [GPSToolbox 投稿](https://geodesy.noaa.gov/gps-toolbox/exist.htm)：
* [barbeau-awesome-gnss 整理](https://github.com/barbeau/awesome-gnss)：
* [hdkarimi-awesome-gnss 整理](https://github.com/hdkarimi/awesome-gnss)：
* [mcraymer 整理](https://mcraymer.github.io/geodesy/index.html)：