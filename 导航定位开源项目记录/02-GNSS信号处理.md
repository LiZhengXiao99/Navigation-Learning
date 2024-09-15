<div align="center">
	<h1>GNSS信号处理开源项目</h1>
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
</div>

<br/>

> * 包括软件接收机程序、接收机板卡PCB、FPGA接收机、数字中频信号模拟；
> * GNSS信号处理开源项目比较少，有些只有几个Star的项目我也记录上了。

* **[GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr)**：基于GNURadio的软件接收机，在Github上搜“GNSS”排第一的项目，C++编写，使用了Boost库，二十多万行，支持十几个频点、支持CUDA并行计算、支持实时运行、支持采集数据后处理、支持HackRF、USRP等软件接收机设备，有配套商用的FPGA相关器IP核（需要购买），引入 RTKLIB 进行定位解算。
  * [meta-gnss-sdr](https://github.com/carlesfernandez/meta-gnss-sdr)：定义了在 [Zynq-7000](https://www.xilinx.com/video/soc/xilinx-arm-zynq-7000-all-programmable-soc.html) 上运行 GNSS-SDR 的 OpenEmbedded 层，通过交叉编译，将构建时间从 10 多个小时缩短到 10 分钟以内，好像是用ZYNQ的两个ARM核，没用FPGA。
  * [gnss-sdr-monitor](https://github.com/acebrianjuan/gnss-sdr-monitor)：用Qt写的GNSS-SDR监控界面，独立于GNSS-SDR运行，配置好GNSS-SDR运行设备的IP和端口号，可以实时查看捕获和跟踪的状态。
  * [gnss_sdr_gui](https://github.com/UHaider/gnss_sdr_gui)：Qt写的软件接收机配置界面程序，用于生成配置文件，在图形界面上就可以配置信号源、信号调节器、捕获、跟踪、解码器、VT 块，但是程序七年没更新了，不知道生成还的配置文件还能不能用。
  * [GNSS_SDR_HACKRF](https://github.com/jdesbonnet/GNSS_SDR_HACKRF)：HackRF 做射频前端的 GNSS-SDR 配置文件，说是没有运行成功，捕获不到 GPS L1-C/A 码信号（可能是因为没加外部时钟，官方用 HackRF 实验成功了，在 YouTube 上有视频）。
  * [gnss-sdr-1pps](https://github.com/oscimp/gnss-sdr-1pps)：在 GNSS-SDR 基础上通过分析在 L1 波段发射的每颗GPS卫星的信号到达方向和额外的1-PPS输出来提供欺骗检测能力，以及通过检测多天线检测到的强相关信号来提供干扰检测和消除能力；使用 Ettus Research B210 双输入 SDR 平台、XTRX Osmocom 信号源和文件信号源进行了测试。
* **[SoftGNSS](https://github.com/kristianpaul/SoftGNSS)**：
  * [SoftGNSS-python](https://github.com/perrysou/SoftGNSS-python)：Python 版 SoftGNSS。
* **[PocketSDR](https://github.com/tomojitakasu/PocketSDR)**：
* [GNSS-SDRLIB](https://github.com/taroz/GNSS-SDRLIB)：
* [nut2nt](https://github.com/amungo/nut2nt)：
* [Beagle_SDR_GPS](https://github.com/jks-prv/Beagle_SDR_GPS)：
* [KiwiSDR](https://forum.kiwisdr.com/)：
* [FlyDog-SDR-GPS](https://github.com/flydog-sdr/FlyDog_SDR_GPS)：
* [FlyCat-SDR-GPS](https://github.com/flydog-sdr/FlyCat_SDR_GPS)：
* [Full_Stack_GPS_Receiver](https://github.com/hamsternz/Full_Stack_GPS_Receiver)：
* [BDS-3-B1C-B2a-SDR-receiver](https://github.com/lyf8118/BDS-3-B1C-B2a-SDR-receiver)：
* [FGI-GSRx](https://github.com/nlsfi/FGI-GSRx)：
* [sydr](https://github.com/aproposorg/sydr)：
* [NavLab-DPE-SDR](https://github.com/Stanford-NavLab/NavLab-DPE-SDR)：斯坦福大学四年前开源，包括CUDARecv（直接位置估计的开源并行 GPS 接收机）、PyGNSS（顺序 GPS 接收机，采用标量跟踪（传统两步法）和 DPE（一步法）定位算法）。
* [multi-channel-gnss](https://github.com/dasdboot/multi-channel-gnss)：
* [GPUAcceleratedTracking](https://github.com/coezmaden/GPUAcceleratedTracking)：利用 CUDA 加速多天线全球导航 GNSS 软件接收机，在英伟达 1050Ti 上测试，[Tracking.jl: Accelerating multi-antenna GNSS receivers with CUDA](https://proceedings.juliacon.org/)。
* [GNSS-GPS-SDR](https://github.com/JiaoXianjun/GNSS-GPS-SDR)：基于 C语言和 MATLAB 编写的一些小程序，用于 GPS 信号的接收、回放，支持 HackRF、RTL-SDR。
* [SDR-GPS-SPOOF](https://github.com/B44D3R/SDR-GPS-SPOOF)：
* [gps](https://github.com/psas/gps)：Python 和 C 语言编写的 GPS-L1-C/A 码软件接收机，相关器算法用 C 语言实现，文档挺详细，有一些 JupyterNotebook 文件。
* [SnapperGPS](https://snappergps.info/)：
* [snapshot-gnss-algorithms](https://github.com/JonasBchrt/snapshot-gnss-algorithms)：
* [SatDump](https://github.com/SatDump/SatDump)：
* [gps-sdr-simulink](https://github.com/dmiralles2009/gps-sdr-simulink)：基于 MATLAB-Simulink 的 GPS-L1-C/A 码软件接收机，可以下载配套的论文：[Development of a Simulink Library for the Design,Testing and Simulation of Software Defined GPS Radios](https://www.researchgate.net/publication/280610442_Development_of_a_Simulink_Library_for_the_Design_Testing_and_Simulation_of_Software_Defined_GPS_Radios)。
* [gps-sdr-sim](https://github.com/osqzss/gps-sdr-sim)：
  * [gps-sdr-sim-assistant](https://github.com/frank-pian/gps-sdr-sim-assistant)：JAVA写的界面程序，设置好坐标、时间，直接生成模拟的数字中频信号文件。
* [beidou-sdr-sim](https://github.com/yangfan852219770/beidou-sdr-sim)：
* [galileo-sdr-sim](https://github.com/harshadms/galileo-sdr-sim)：
* [gps-qzss-sdr-sim](https://github.com/iGNSS/gps-qzss-sdr-sim)：
* [multi-sdr-gps-sim](https://github.com/Mictronics/multi-sdr-gps-sim)：
* [pluto-gps-sim](https://github.com/Mictronics/pluto-gps-sim)：
* [SignalSim](https://github.com/globsky/SignalSim)：
* [GPS_GAL_SSS](https://github.com/domonforyou/GPS_GAL_SSS)：
* [greta-oto](https://github.com/globsky/greta-oto)：
* [BD3_FPGA](https://github.com/whc2uestc/BD3_FPGA)：
* [GNSS-matlab](https://github.com/danipascual/GNSS-matlab)：
* [oresat-gps-software](https://github.com/oresat/oresat-gps-software)/[hardware](https://github.com/oresat/oresat-gps-hardware)：包括开源PCB和用Python编写软件接收机程序，PCB上带有 [SkyTraq Orion-B16](https://navspark.mybigcommerce.com/12mm-x-16mm-gnss-receiver-module-for-leo-applications/) GNSS 模块和 [MAX2771](https://www.analog.com/en/products/max2771.html) 射频前端。
* [MAX2769FT2232H](https://github.com/WKyleGilbertson/MAX2769FT2232H)：
* [SDR-GB-SAR](https://github.com/jmfriedt/SDR-GB-SAR)：
* [gps-rf-frontend-sim](https://github.com/iliasam/gps_rf_frontend_sim)：
* [GNSS-VHDL](https://github.com/danipascual/GNSS-VHDL)：
* [GNSS-Metadata-Standard](https://github.com/IonMetadataWorkingGroup/GNSS-Metadata-Standard)：
* [gnss-baseband](https://github.com/j-core/gnss-baseband)：
* [GPSMAXIM2769b-](https://github.com/vaidhyamookiah/GPSMAXIM2769b-)：
* [Analog-GPS-data-receiver](https://github.com/leaningktower/Analog-GPS-data-receiver)：
* [GNSS_Firehose](https://github.com/pmonta/GNSS_Firehose)：
* [GNSS-DSP-tools](https://github.com/pmonta/GNSS-DSP-tools)：
* [hard_sydr](https://github.com/aproposorg/hard_sydr)：
* [B1C-Signals-Simulation](https://github.com/pandaclover/B1C_Signals_Simulation)：
* [CU-SDR-Collection](https://github.com/gnsscusdr/CU-SDR-Collection)：
* [ESP32-SDR-GPS](https://github.com/iliasam/ESP32_SDR_GPS)：
* [STM32F4-SDR-GPS](https://github.com/iliasam/STM32F4_SDR_GPS)：
* [Fast-GNSS-ReceiverMATLAB](https://github.com/JohnBagshaw/Fast_GNSS_ReceiverMATLAB)：
* [gnss-sdr-rs](https://github.com/kewei/gnss-sdr-rs)：
* [sdr-beamforming](https://github.com/ADolbyB/sdr-beamforming)：
* [gr-gnMAX2769](https://github.com/wkazubski/gr-gnMAX2769)：
