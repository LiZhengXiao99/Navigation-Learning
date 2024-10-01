<div align="center">
<h1>GPS Toolbox 投稿项目摘要翻译</h1>
</div>


<div align="center">
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
</div>


<br/>

GPS Toolbox 是 GPS Solution 期刊

接收 GNSS 数据处理开源项目的投稿

其中每个项目包括论文、程序、测试数据

网址：https://geodesy.noaa.gov/gps-toolbox/exist.htm

有意思的是，现在大部分都是中国学者的投稿

笔者把其摘要翻译成中文，从后往前列举到本文档，方便各位快速了解其中内容

只了翻译近十年的摘要，更早的程序没啥参考价值

---

## PPPH-VA：

[PPPH-VA: an open-source software for real-time multi-GNSS variometric approach using single- and dual-frequency observations by B. Bahadur, M. Bezcioglu, & C.O. Yigit ](https://geodesy.noaa.gov/gps-toolbox/ppph-va.shtml) 

Variometric approach (VA) technique has been introduced as an alternative to real-time kinematics and real-time precise point positioning techniques. As the ability of the variometric approach to detect short-term dynamic behaviors in real-time mode in applications such as Global Navigation Satellite Systems (GNSS)-seismology and structural health monitoring is demonstrated, the demand for open-source VA software is increasing. However, open-source software that is capable of VA processing in real-time mode based on single- and dual-frequency multi-GNSS observations is scarce. In view of this fact, we have developed an open-source VA processing software called PPPH-VA that can evaluate single- and dual-frequency multi-GNSS observations in real-time mode. PPPH-VA is developed in the MATLAB environment, and it can simultaneously process GPS, GLONASS, Galileo, BeiDou-2, and BeiDou-3 data with the VA technique in real-time mode, employing both single- and dual-frequency observations. We evaluated PPPH-VA using shake table experiments based on real data, and the results demonstrate that it provides high accuracy in terms of detection of dynamic displacements. Toolbox can successfully detect the dominant frequencies of short-term dynamic behaviors and is capable of determining the amplitude values corresponding to the peak frequency at the sub-mm level. Moreover, in the time domain, it can obtain dynamic behaviors with an accuracy of millimeters.

变异测量法（VA）技术是作为实时运动学和实时精密单点定位技术的替代技术而引入的。随着变分法在全球导航卫星系统（GNSS）-地震学和结构健康监测等应用中实时检测短期动态行为的能力得到证明，对开源变分法软件的需求也在不断增加。然而，能够基于单频和双频多全球导航卫星系统观测数据进行实时虚拟现实处理的开源软件却很少。有鉴于此，我们开发了一款名为 PPPH-VA 的开源 VA 处理软件，该软件能够以实时模式评估单频和双频多重全球导航卫星系统观测数据。PPPH-VA 是在 MATLAB 环境下开发的，它可以在实时模式下使用 VA 技术同时处理 GPS、GLONASS、Galileo、BeiDou-2 和 BeiDou-3 数据，同时使用单频和双频观测数据。我们利用基于真实数据的振动台实验对 PPPH-VA 进行了评估，结果表明它在检测动态位移方面具有很高的准确性。工具箱能成功检测出短期动态行为的主要频率，并能确定与亚毫米级峰值频率相对应的振幅值。此外，在时域中，它还能以毫米级的精度获得动态行为。

* VA (Variometric approach) 技术被认为是 RTK 和 RT-PPP 的替代技术；
* 随着 VA 技术在实时检测短期形变能力的提高，在 GNSS-地震学领域的需求与日俱增；
* 然而，目前能够进行单频、双频、多频 GNSS 实时 VA 处理的软件还十分匮乏；
* 为此，我们开发了名为 PPPH-VA 的 MATLAB 软件，能够进行单频、双频、多频 GNSS 实时 VA 处理；
* 基于实测数据对处理效果进行验证，结果表明，对动态变形监测较为精确：
  * 在频域中，能够确定与亚毫米级峰值频率相对应的振幅值；
  * 在时域中，能获得精确到毫米的动态行为；

---

## GHASP：Galileo HAS 解码

[GHASP: a Galileo HAS parser by D. Borio, M. Susi, & C. Gioia](https://geodesy.noaa.gov/gps-toolbox/ghasp.shtml)

Galileo High Accuracy Service (HAS) corrections are broadcast through the E6B signal using a high-parity vertical Reed-Solomon encoding scheme, which reduces message recovery time and improves transmission reliability. To recover HAS corrections, it is thus necessary to invert the encoding process and interpret the decoded bits. In order to foster HAS adoption and facilitate experimentation with HAS corrections, a Galileo HAS Parser (GHASP) has been developed. GHASP is available open-source and supports different input data types from different receiver manufacturers. Decoded corrections are provided in Comma-Separated Values files, which can be directly loaded using common data-science languages. In this way, corrections are readily available and can be used not only for Precise Point Positioning (PPP) applications but also for scientific analysis such as clock characterization using the Allan Deviation.



Galileo高精度服务（HAS）改正通过 E6B 信号广播，采用高比特垂直里德-所罗门编码方案，缩短了信息恢复时间，提高了传输可靠性。因此，要恢复 HAS 更正，必须反转编码过程并解释解码比特。为了促进 HAS 的应用并推动 HAS 改正实验，我们开发了Galileo HAS 解析器（GHASP）。GHASP 是开源的，支持来自不同接收器制造商的不同输入数据类型。解码后的改正数据以逗号分隔值文件的形式提供，可以使用常见的数据科学语言直接加载。通过这种方式，可以随时获得改正值，不仅可用于精密单点定位 (PPP) 应用，还可用于科学分析，如使用Allan 方差进行时钟鉴定。

* Galileo高精度服务（HAS）改正通过 E6B 信号广播。
* 采用高比特率垂直里德-所罗门编码方案，减少了信息恢复时间，提高了传输可靠性。
* 因此，要得到 HAS 改正数，就必须反转编码过程并解释解码比特。
* 为了推动GalileoGalileo系统（HAS）改正实验，我们开发了GalileoGalileo系统（HAS）解码程序 GHASP
* 支持输入各种不同接收机厂商的不同数据格式；
* 解码出的 HAS 数据以逗号分隔值文件的形式提供，可使用常见的数据科学语言直接加载；
* 这样，HAS 数据就可随时获取，不仅可用于 PPP 应用，还可用于科学分析，如对钟差进行Allan 方差分析；

---

## POSGO：图优化 SPP

[POSGO: an open-source software for GNSS pseudorange positioning based on graph optimization by Zhen Li, Jing Guo, & Qile Zhao](https://geodesy.noaa.gov/gps-toolbox/posgo.shtml)

Graph optimization (GO) can correlate more historical information to increase the resistance against the GNSS outliers. Therefore, GO has the potential to obtain a higher accuracy and robust position in urban canyon. Here, we develop POSGO (POSition based on Graph Optimization), an open-source software designed for single-point positioning and relative positioning with multi-GNSS pseudorange in a GO framework. It is coded in C/C+ + language and recommended to run in the Linux environment. It can be easily extended to process the carrier phase and fuse the data from multiple sensors by adding corresponding graph factors. To assess the performance of the current version, data from a kinematic vehicle experiment in urban area are processed. The results indicate GO has better accuracy and robustness than classic least squares or Kalman filters, particularly in areas with severe occlusions.

GO (Graph optimization 图优化) 可以关联更多的历史数据，以减小离群异常值对定位解算的干扰。因此，GO 有可能在城市峡谷中获得更高精度和稳健的定位。在此，我们开发了 POSGO（基于图优化的定位），它是一款开源软件，设计用于在 GO 框架内进行单点定位和多 GNSS 伪距相对定位。该软件采用 C/C+ + 语言编码，建议在 Linux 环境中运行。通过添加相应的图因子，该软件可轻松扩展到处理载波相位和融合多个传感器的数据。为了评估当前版本的性能，我们处理了来自城市地区运动车辆实验的数据。结果表明，与传统的最小二乘法或卡尔曼滤波器相比，GO 具有更好的准确性和鲁棒性，尤其是在遮挡严重的区域。

* GO (Graph optimization 图优化) 可以关联更多的历史数据，以减小离群异常值对定位解算的干扰；
* 因此在城市复杂环境下，图优化有潜力得到更高精度的结果和更好的鲁棒性，
* 所以，我们开发了 POSGO (**POS**ition based on **G**raph **O**ptimization) ，能进行伪距单点定位和伪距差分定位解算；
* 用 C++ 编写，运行在 Linux 环境；
* 通过添加相应的因子，未来可以拓展到载波相位差分、多源融合解算；
* 为了评估当前版本的性能，我们处理了来自城市地区运动车辆实验的数据。结果表明，与传统的最小二乘法或卡尔曼滤波器相比，图优化具有更高的精度和鲁棒性，尤其是在遮挡严重的区域。

---

## raPPPid：快速 PPP

[An open-source software package for Precise Point Positioning: raPPPid by Marcus Franz Glaner & Robert Weber](https://geodesy.noaa.gov/gps-toolbox/rapppid.shtml)

Precise Point Positioning (PPP) has proven to be a powerful GNSS positioning method used for various scientific and commercial applications nowadays. We present a flexible and user-friendly software package named raPPPid suitable for processing single to triple-frequency GNSS observations in various PPP approaches (e.g., ionospheric-free linear combination, uncombined model), available under https://github.com/TUW-VieVS/raPPPid. To tune the PPP procedure, the user can select from many satellite products, models, options, and parameters. This way, the software raPPPid can handle high-to-low quality observation data ranging from geodetic equipment to smartphones. Despite significant improvements, the convergence time of PPP is still a major topic in scientific research. raPPPid is specially designed to reduce the convergence period with diverse implemented approaches, such as PPP-AR or ionospheric pseudo-observations, and to offer the user multiple plots and statistics to analyze this critical period. Typically, raPPPid achieves coordinate convergence times of around 1 min or below with high-quality observations and ambiguity fixing. With smartphone data and a simplified PPP approach, a 2D position accuracy at the one-meter level or below is accomplished after two to three minutes.

精确点定位（PPP）已被证明是一种功能强大的 GNSS 定位方法，目前用于各种科学和商业应用。我们提出了一个名为 raPPPid 的灵活且用户友好的软件包，适用于以各种 PPP 方法（如无电离层线性组合、非组合模型）处理单频至三频 GNSS 观测数据，可在 https://github.com/TUW-VieVS/raPPPid 下使用。为了调整 PPP 程序，用户可以从许多卫星产品、模型、选项和参数中进行选择。这样，raPPPid 软件就可以处理从大地测量设备到智能手机等从高质量到低质量的观测数据。raPPPid 是专门为缩短收敛期而设计的，它采用了不同的实施方法，如 PPP-AR 或电离层伪观测，并为用户提供多种图表和统计数据来分析这一关键时期。通常情况下，raPPPid 通过高质量的观测和模糊性修正，可实现约 1 分钟或更短的坐标收敛时间。利用智能手机数据和简化的 PPP 方法，两到三分钟后即可实现一米或以下级别的二维定位精度。

* PPP 已被证明是一种功能强大的 GNSS 定位解算方法，可用于各种科学和商业应用；
* 我们开发了灵活且用户友好型的软件包，名为 raPPPid：支持单频到三频解算，支持消电离层组合和非差非组合；
* 可以选择各种 PPP 产品、选项、模型、参数；因此可以处理各种精度的观测值，包括测量型接收机和手机；
* 尽管 PPP 的收敛时间有了明显改善，但它仍然是科学研究中的一个重要课题；
* raPPPid 专门为缩短收敛时间而设计，使用了 PPP-AR、电离层伪观测值等各种技术，并将关键参数绘图，方便进行可视化分析；
* 通常情况下，raPPPid 在高质量观测和模糊度改正的情况下，坐标收敛时间可降到一分钟以内；
* 用智能手机数据和简化的 PPP，两到三分钟后就能实现一米或以下级别的二维定位精度。

---

## GNSS 频间偏差估计

[Open-source software for multi-GNSS inter-frequency clock bias estimation by Xingxing Li, Hongjie Zheng, Xin Li, Yongqiang Yuan, Jiaqi Wu & Xinjuan Han](https://geodesy.noaa.gov/gps-toolbox/great-ifcb.shtml)

As one of the key issues in multi-frequency Global Navigation Satellite System (GNSS) applications, the inter-frequency clock bias (IFCB) has been well studied in recent years. However, the lack of publicly available IFCB products prevents users from taking full advantages of multi-frequency GNSS observations. An open-source software called GREAT-IFCB, which is derived from the GNSS + REsearch, Application and Teaching (GREAT) software platform at Wuhan University, is designed and developed to provide multi-GNSS IFCB products for multi-frequency users. Based on the geometry-free and ionospheric-free combinations of the multi-frequency observations, GREAT-IFCB can generate IFCB products for GPS, Galileo and BDS satellites. Multi-frequency observations from day of year 001 to 007 of 2021 were selected to estimate the multi-GNSS IFCB products for demonstrating the performance of GREAT-IFCB. After applying the IFCB corrections, the averaged standard deviation (STD) of estimated extra-wide-lane (EWL) uncalibrated phase delay (UPD) for GPS satellites is improved by 68.4% from 0.076 cycles to 0.024 cycles, and the positioning accuracy of triple-frequency PPP is improved by 30.3%, 32.9% and 31.6% in the east, north and up components, respectively.

作为多频率全球导航卫星系统（GNSS）应用中的关键问题之一，频率间时钟偏差（IFCB）近年来得到了深入研究。然而，由于缺乏公开可用的 IFCB 产品，用户无法充分利用多频 GNSS 观测数据。GREAT-IFCB源于武汉大学的 "GNSS+科研、应用和教学（GREAT）"软件平台，是一款开源软件，旨在为多频用户提供多GNSS IFCB产品。基于多频观测的无几何和无电离层组合，GREAT-IFCB 可生成 GPS、伽利略和 BDS 卫星的 IFCB 产品。为展示 GREAT-IFCB 的性能，选取了从 2021 年 001 日到 007 日的多频观测数据来估算多重全球导航卫星系统的 IFCB 产品。在应用 IFCB 修正之后，GPS 卫星的估计宽带外（EWL）未校准相位延迟（UPD）的平均标准偏差（STD）从 0.076 个周期提高到 0.024 个周期，提高了 68.4%，三频 PPP 的东、北和上分量的定位精度分别提高了 30.3%、32.9% 和 31.6%。

* 频间偏差 (IFCB,inter-frequency clock bias) 是目前多频 GNSS 数据处理的关键问题；
* 然而，目前还缺乏可公开获得的 IFCB 产品，用户无法充分利用 GNSS 多频观测；
* 为此，我们基于 GREAT 开发了 GREAT-IFCB，以提供 IFCB 产品；
* 基于多频观测值的 GF 组合和消电离层组合，GREAT-IFCB 可提供 GPS、Galileo、BDS 产品；
* 在应用了 IFCB 改正后，基于超宽巷估计 UPD 精度提高了 68.4%，从 0.076 降到 0.024 周；
* 东北天方向的三频 PPP 定位相对精度分别提高了 30.3%、32.9%、31.6%；

---

## GDDS：GNSS 数据下载程序

[GDDS: Python software for GNSS data download by Liguo Lu, Qiao Liang, Weijian Hu, and Tangting Wu](https://geodesy.noaa.gov/gps-toolbox/gdds.shtml)

With the rapid development of global navigation satellite system (GNSS), GNSS data products have been widely used for high-precision positioning and navigation applications. They are typically downloaded from the international GNSS service (IGS) analysis centers and continuously operating reference stations (CORS). However, the conventional GNSS data download method is cumbersome, repetitive, and time-consuming, and it is challenging to meet the demands for rapid acquisition of multi-source data products. Therefore, we have developed a GNSS data download software with Python, which provides an interactive interface for the Windows or Linux operating system to realize the efficient and stable download for a large amount of GNSS data. The software includes five main function modules: Global IGS Data, Post-Processing Product, Regional CORS Data, Custom Download, and Data Decompression. It has the characteristics of diverse data products, map interaction support, and station information retrieval, which can meet the needs of different users.

随着全球导航卫星系统（GNSS）的迅速发展，GNSS 数据产品已广泛用于高精度定位和导航应用。这些数据产品通常从国际全球导航卫星系统服务（IGS）分析中心和连续运行基准站（CORS）下载。然而，传统的 GNSS 数据下载方法繁琐、重复、耗时，难以满足快速获取多源数据产品的需求。因此，我们利用 Python 开发了一款 GNSS 数据下载软件，为 Windows 或 Linux 操作系统提供交互式界面，实现大量 GNSS 数据的高效稳定下载。该软件包括五个主要功能模块： 全球 IGS 数据、后处理产品、区域 CORS 数据、自定义下载和数据解压缩。它具有数据产品多样化、支持地图交互、支持台站信息检索等特点，可满足不同用户的需求。

* 下载 IGS、CORS 数据是重复且耗时的工作；
* 我们用 Python 开发了 GDDS 帮助下载数据，
* 





---

## PPP-ARISEN：

[PPP-ARISEN: an open-source precise point positioning software with ambiguity resolution for interdisciplinary research of seismology, geodesy and geodynamics by Chengfeng Zhang, Aizhi Guo, Sidao Ni, Gongwei Xiao, & Hao Xu](https://geodesy.noaa.gov/gps-toolbox/ppp-arisen.shtml)

For the geoscience community, a cross-platform open-source PPP toolbox named PPP-ARISEN is developed, which can realize ambiguity resolution (AR) based on integer phase clock (IPC) method with satellite-to-satellite single difference (SSD) strategy and now is compatible with both CODE (Center for Orbit Determination in Europe) and CNES (Centre National d'Etudes Spatiales) AR products. The toolbox can achieve millimeter-level precision for static positioning and centimeter-level precision for kinematic mode, while the uniquely designed "Seismological" mode successfully captures clear dynamic signals induced by the earthquake with different products of IGS analysis centers. Moreover, an effective index called CSI-ZWDV (convergence status indicator based on Zenith wet delay variances) with clear physical meaning has been proposed and validated. It could not only determine when to search for the integer ambiguities with LAMBDA method, but also indicate the convergence status of PPP, as a distinctive alternative to experimental parameters used independently or jointly with existing indices or thresholds.

该工具箱基于整数相位时钟（IPC）方法，采用卫星间单点差分（SSD）策略，可实现模糊解析（AR），目前已与欧洲轨道确定中心（CODE）和法国国家空间研究中心（CNES）的AR产品兼容。该工具箱可实现毫米级的静态定位精度和厘米级的运动模式精度，而独特设计的 "地震 "模式可通过 IGS 分析中心的不同产品成功捕捉地震引起的清晰动态信号。此外，还提出并验证了一个具有明确物理意义的有效指标 CSI-ZWDV（基于天顶湿延迟方差的会聚状态指标）。它不仅可以确定何时使用 LAMBDA 方法搜索整数模糊度，还可以指示 PPP 的收敛状态，是独立使用或与现有指数或阈值联合使用的实验参数的独特替代品。

---

## M_GIM

[M_GIM: a MATLAB-based software for multi-system global and regional ionospheric modeling by Chunyuan Zhou, Ling Yang, Bofeng Li & Timo Balz](https://geodesy.noaa.gov/gps-toolbox/m-gim.shtml)

The ionospheric delay must be modeled and analyzed as one of the main error sources affecting the Global Navigation Satellite Systems (GNSS). To make up for the absence of open-source software in ionospheric modeling, a multi-system global and regional ionospheric modeling software named M_GIM is introduced, produced based on the M_DCB, a GNSS satellite and receiver differential code biases estimating software, and redeveloped in MATLAB 2019b. The conventional dual-frequency carrier-to-code leveling (DFCCL) method is used in M_GIM, and up to quad-system (GPS/GLONASS/Galileo/BDS) observations can be processed. After a series of experiments, the reliability of the software is evaluated, which verifies that the established global and regional ionospheric models have similar accuracy to most of the individual ionosphere associate analysis centers (IAACs) final and rapid global ionosphere map (GIM) products. Using this software, one can easily establish a reliable temporal and spatial variation model of the ionosphere, aiding ionosphere-related space weather research and precise navigation and positioning.

电离层延迟是影响全球导航卫星系统（GNSS）的主要误差源之一，必须对其进行建模和分析。为了弥补电离层建模方面开源软件的缺失，本文介绍了一款名为 M_GIM 的多系统全球和区域电离层建模软件，该软件基于 M_DCB（一款 GNSS 卫星和接收机差分码偏差估计软件），并在 MATLAB 2019b 中进行了重新开发。M_GIM 采用传统的双频载波码平差（DFCCL）方法，最多可处理四系统（GPS/GLONASS/Galileo/BDS）观测数据。经过一系列实验，对该软件的可靠性进行了评估，验证了所建立的全球和区域电离层模型与大多数单个电离层关联分析中心的最终和快速全球电离层地图（GIM）产品具有相似的精度。使用该软件，可以轻松建立可靠的电离层时空变化模型，有助于与电离层有关的空间气象研究以及精确导航和定位。

---

## GNSS2TWS

[GNSS2TWS: an open-source MATLAB-based tool for inferring daily terrestrial water storage changes using GNSS vertical data by Zhongshan Jiang, Ya-Ju Hsu, Linguo Yuan, Wei Feng, Xinghai Yang & Miao Tang](https://geodesy.noaa.gov/gps-toolbox/gnss2tws.shtml)

Technological advances in global navigation satellite system (GNSS) offer a novel environmental sensor to measure terrestrial water cycles and provide independent constraints on total terrestrial water storage (TWS) changes over various spatiotemporal scales. This study aims to develop an open-source MATLAB-based tool for inferring daily TWS changes based on the relation between GNSS annual vertical displacement and hydrological cycles. The widely used spatial-domain Green's function approach is used to estimate regional equivalent water height changes. To recover daily water storage fluctuations, we integrate the principal component analysis into our time-varying inversion strategy. To demonstrate the implementation of the inversion tool, we invert the daily TWS changes in the Pacific Northwest River Basin, the United States of America, using GNSS-measured vertical surface motions. The primary goal is to share this inversion software for hydrogeodetic scientific communities to fully use the GNSS technique for hydrological applications.

全球导航卫星系统（GNSS）的技术进步为测量陆地水循环提供了一种新型环境传感器，并为不同时空尺度的陆地总蓄水量（TWS）变化提供了独立的约束条件。本研究旨在开发一种基于 MATLAB 的开源工具，用于根据全球导航卫星系统年垂直位移与水文周期之间的关系推断陆地总蓄水量的日变化。广泛使用的空间域格林函数方法用于估算区域等效水高变化。为了恢复每日的蓄水量波动，我们将主成分分析纳入时变反演策略。为了演示反演工具的实施，我们利用全球导航卫星系统测量到的垂直表面运动，反演了美国西北太平洋流域的每日等效水高变化。主要目的是与水文地质科学界分享这一反演软件，以便在水文应用中充分利用全球导航卫星系统技术。

---

## 基于图优化的安卓手机 SPP

[Open-source optimization method for android smartphone single point positioning by Changhui Jiang, Yuwei Chen, Chen Chen, Jianxin Jia, Haibin Sun, Tinghuai Wang & Juha Hyyppa](https://geodesy.noaa.gov/gps-toolbox/phonePP.shtml)

Nowadays, a chip-scale Global Navigation Satellite System (GNSS) receiver is ubiquitous in smartphones. In a smartphone GNSS receiver, the least square (LS) or Kalman Filter (KF) is implemented to estimate the position. With the aim to improve the smartphone GNSS position accuracy, we propose a position-smoothing method considering more historical information than the traditional methods, i.e., LS and KF. More past states are regarded as unknowns, and a cost function is constructed to optimize these states. An open-source smartphone dataset from Google was used for testing the proposed method. The experimental results indicate that the proposed method outperforms the other conventional methods in position errors. In addition, we open the source code. We expect that the optimization method implemented in the smartphone GNSS position smoothing application can be an illustrative example to clearly introduce such an optimization method and a reference for its implementation, which might inspire some other meaningful and exciting applications in GNSS.

如今，芯片级全球导航卫星系统（GNSS）接收器在智能手机中无处不在。在智能手机的全球导航卫星系统接收器中，最小平方（LS）或卡尔曼滤波器（KF）被用来估计位置。为了提高智能手机全球导航卫星系统的位置精度，我们提出了一种位置平滑方法，与传统方法（即 LS 和 KF）相比，该方法考虑了更多的历史信息。我们将更多的过去状态视为未知数，并构建了一个代价函数来优化这些状态。谷歌的一个开源智能手机数据集被用来测试所提出的方法。实验结果表明，所提出的方法在位置误差方面优于其他传统方法。此外，我们还开放了源代码。我们希望在智能手机 GNSS 位置平滑应用中实现的优化方法能成为一个示例，清楚地介绍这种优化方法，并为其实现提供参考，这可能会激发 GNSS 中其他一些有意义和令人兴奋的应用。

---

## PDR/GNSS 手机定位

[Implementation and performance analysis of the PDR/GNSS integration on a smartphone by Changhui Jiang, Yuwei Chen, Chen Chen, Jianxin Jia, Haibin Sun, Tinghuai Wang & Juha Hyyppa](https://geodesy.noaa.gov/gps-toolbox/PDR_GNSS.shtml)

Pedestrian dead reckoning (PDR) is an effective technology for pedestrian navigation. In PDR, the steps are detected with the measurements of self-contained sensors, such as accelerometers, and the position is updated with additional heading angles. A smartphone is usually equipped with a low-cost microelectromechanical system accelerometer, which can be utilized to implement PDR for pedestrian navigation. Since the PDR position errors diverge with the walking distance, the global navigation satellite system (GNSS) is usually integrated with PDR for more reliable position results. This paper implemented a smartphone PDR/GNSS via a Kalman filter and factor graph optimization (FGO). In the FGO, the PDR factor is modeled, and the states are correlated with a dead reckoning algorithm. The GNSS position is modeled as the GNSS factor to constrain the states at each step. With a graphic model representing the states and measurements, the state estimation is converted to a nonlinear least square problem, and we utilize the Georgia Tech Smoothing and Mapping graph optimization library to implement the optimization. We tested the proposed method on a Huawei Mate 40 Pro handset with a standard playground field test, and the field test results showed that the FGO effectively improved the smartphone position accuracy. We have released the source codes and hope that they will inspire other works on pedestrian navigation, i.e., constructing an adaptive multi-sensor integration system using FGO on a smartphone.

行人惯性导航（PDR）是一种有效的行人导航技术。在 PDR 中，通过加速度计等自带传感器的测量来检测步数，并通过附加的航向角来更新位置。智能手机通常配有低成本的微机电系统加速度计，可用于实现行人导航的 PDR。由于 PDR 的位置误差会随着步行距离的增加而偏离，因此通常会将全球导航卫星系统（GNSS）与 PDR 组合在一起，以获得更可靠的位置结果。本文通过卡尔曼滤波器和因子图优化（FGO）实现了智能手机 PDR/GNSS 系统。在 FGO 中，PDR 因子被建模，状态与死算算法相关联。GNSS 定位被建模为 GNSS 因子，以约束每一步的状态。有了代表状态和测量值的图形模型，状态估计就转换成了非线性最小平方问题，我们利用佐治亚理工学院的平滑和映射图形优化库来实现优化。我们在华为 Mate 40 Pro 手机上对所提出的方法进行了标准操场实地测试，实地测试结果表明，FGO 有效提高了智能手机的定位精度。我们已经发布了源代码，希望能对其他行人导航工作有所启发，即利用 FGO 技术构建自适应多传感器组合系统。

---

## GIRAS

[GIRAS: an open-source MATLAB-based software for GNSS-IR analysis by Cemali Altuntas and Nursu Tunalioglu](https://geodesy.noaa.gov/gps-toolbox/GIRAS.shtml)

Global Navigation Satellite System Interferometric Reflectometry (GNSS-IR) has become a robust method to extract the characteristic environmental features of reflected surfaces, where the signal transmitted from satellites reflects before receiving at GNSS antenna. When the signal arrives at the GNSS antenna from more than one path, a multipath error occurs, which causes interference of the direct and reflected signals. The interference of direct and reflected signals shows a pattern for sensing environmental features, where the signal reflects, and multipath directly affects the signal strength. Analyzing the signal strength represented by the signal-to-noise ratio (SNR) enables the retrieval of environment-related features. The software developed, named GIRAS (GNSS-IR Analysis Software), can process multi-constellation GNSS signal data and estimate the SNR metrics, namely phase, amplitude, and frequency, for further computations with several optional statistical analyses for controlling the quality of the estimations, as required, such as snow depth retrieval, effective reflector height estimation, and soil moisture monitoring. The software developed in the MATLAB environment has a graphical user interface. To represent the processes of the working procedures of the software, we conducted a case study with 7-day site data from the multi-GNSS experiment (MGEX) Project network displaying how to process GNSS data with input and output file properties.

全球导航卫星系统干涉反射测量法（GNSS-IR）已成为一种提取反射表面环境特征的可靠方法，从卫星发射的信号在接收到全球导航卫星系统天线之前会在反射表面发生反射。当信号从多条路径到达全球导航卫星系统天线时，就会产生多径误差，导致直接信号和反射信号相互干扰。直达信号和反射信号的干扰显示了一种感知环境特征的模式，即信号反射和多径直接影响信号强度。通过分析信噪比（SNR）表示的信号强度，可以检索与环境有关的特征。开发的软件名为 GIRAS（GNSS-IR 分析软件），可处理多星座 GNSS 信号数据并估算 SNR 指标，即相位、振幅和频率，以便根据需要进行进一步计算，还可根据需要进行若干可选统计分析，以控制估算质量，例如雪深检索、有效反射体高度估算和土壤湿度监测。在 MATLAB 环境下开发的软件具有图形用户界面。为了体现该软件的工作程序流程，我们利用多重全球导航卫星系统实验（MGEX）项目网络的 7 天站点数据进行了案例研究，展示了如何处理具有输入和输出文件属性的全球导航卫星系统数据。

---

## 

[Python software to transform GPS SNR wave phases to volumetric water content by Angel Martin, Ana Belen Anquela, Sara Ibanez, Carlos Baixauli, and Sara Blanc](https://geodesy.noaa.gov/gps-toolbox/SNRtoVWC.shtml)

The global navigation satellite system interferometric reflectometry is often used to extract information about the environment surrounding the antenna. One of the most important applications is soil moisture monitoring. This manuscript presents the main ideas and implementation decisions needed to write the Python code to transform the derived phase of the interferometric GPS waves, obtained from signal-to-noise ratio data continuously observed during a period of several weeks (or months), to volumetric water content. The main goal of the manuscript is to share the software with the scientific community to help users in the GPS-IR computation.

全球导航卫星系统干涉反射测量通常用于提取天线周围环境的信息。其中最重要的应用之一是土壤湿度监测。本手稿介绍了编写 Python 代码所需的主要思路和实施决策，这些代码用于将从数周（或数月）期间连续观测的信噪比数据中获得的干涉 GPS 波的推导相位转换为体积含水量。手稿的主要目的是与科学界分享该软件，帮助用户进行 GPS-IR 计算。

---

## GINAV：GNSS/INS 组合导航 MATLAB 软件包

[GINav: a MATLAB-based software for data processing and analysis of a GNSS/INS integrated navigation system by Kai Chen, Guobin Chang, and Chao Chen](https://geodesy.noaa.gov/gps-toolbox/GINav.shtml)

With the development of GNSS, many open-source software packages have become available for GNSS data processing. However, there are only a handful of open-source software that can handle GNSS/INS integrated data, even though GNSS/INS integration schemes have been widely used in vehicle navigation systems due to their high accuracy, stability, and continuity in harsh environments. Taking the above into account, we developed an open-source software, GINav, which focuses on the data processing and analysis of a GNSS/INS integrated navigation system. GINav is suitable for in-vehicle situations and is aimed at providing a useful tool for carrying out GNSS/INS-related research. It is a convenient platform for testing new algorithms and experimental functionalities. GINav is developed in the MATLAB environment. It provides a user-friendly graphical user interface (GUI) to facilitate users to learn how to use it quickly. A visualization tool, GINavPlot, is provided for solution presentation and error analysis. We have conducted experimental tests to validate and assess the performance of GINav. The results indicate that GINav can provide navigation solutions comparable to general GNSS/INS integration standards, and it can handle both suburban and urban GNSS/INS integrated datasets.

随着 GNSS 的发展，许多用于处理 GNSS 数据的开源软件包已经面世。然而，尽管 GNSS/INS 组合导航方案因其高精度、稳定性和在恶劣环境中的连续性而被广泛应用于车辆导航系统，但能处理 GNSS/INS 组合导航数据的开源软件却屈指可数。有鉴于此，我们开发了一款开源软件 GINav，主要用于 GNSS/INS 组合导航系统的数据处理和分析。GINav 适用于车载情况，旨在为开展 GNSS/INS 相关研究提供有用的工具。它是测试新算法和实验功能的便捷平台。GINav 是在 MATLAB 环境中开发的。它提供了友好的图形界面（GUI），便于用户快速掌握使用方法。GINavPlot 是一个可视化工具，用于展示解决方案和分析误差。我们进行了实验测试，以验证和评估 GINav 的性能。结果表明，GINav 可以提供与一般 GNSS/INS 组合导航相当的定位解，并且可以处理郊区和城市的 GNSS/INS 组合导航数据集。

---

## JUST：用于变形监测和时间序列分析的 MATLAB 和 Python 软件

[JUST: MATLAB and Python Software for Change Detection and Time Series Analysis by Ebrahim Ghaderpour](https://geodesy.noaa.gov/gps-toolbox/JUST.shtml)

Change detection within unequally spaced and non-stationary time series is crucial in various applications, such as environmental monitoring and satellite navigation. The jumps upon spectrum and trend (JUST) is developed to detect potential jumps within the trend component of time series segments. JUST can simultaneously estimate the trend and seasonal components of any equally or unequally spaced time series by considering the observational uncertainties or measurement errors. JUST and its modules can also be applied to monitor vegetation time series in near-real-time. Herein, the details of the open-source software package for JUST, developed in both MATLAB and Python, are presented.

在变形监测和卫星导航等各种应用中，不等距和非稳态时间序列的变形监测至关重要。**JUST** (**j**umps **u**pon **s**pectrum and **t**rend 频谱和趋势跃变) 就是为了检测时间序列片段中趋势成分的潜在跃变而开发的。考虑到观测的不确定性或测量误差，JUST 可以同时估计任何等距或不等距时间序列的趋势和季节成分。JUST 及其模块还可用于近实时植被监测时间序列。

---

## SUPREME：C++  单频 PPP

[SUPREME: an open-source single-frequency precise point positioning software by Chuanbao Zhao, Baocheng Zhang, and Xiao Zhang](https://geodesy.noaa.gov/gps-toolbox/supreme.shtml)

With the rapid development of Global Navigation Satellite Systems (GNSS), Precise Point Positioning (PPP) has been widely used for positioning, navigation, timing (PNT), and atmosphere sensing. Currently, the demand for low-cost single-frequency hardware is increasing. However, open-source single-frequency PPP (SFPPP) softwares, in particular those that employ uncombined GNSS observations, is scarce. In view of this fact, we developed an open-source software, called Single-frequency Uncombined PREcise point positioning for Multi-parameter Estimation (SUPREME), which can process multi-GNSS observations collected from a low-cost single-frequency receiver. It can simultaneously estimate receiver coordinates, receiver clock offsets, zenith troposphere delays (ZTD), and ionosphere delays utilizing a least-squares filter (LSF). SUPREME is developed in the C/C++ computer language; thus, it has good portability and cross-platform capability. The formatted output is also beneficial for the post analysis of results. We evaluated SUPREME using several experiments based on real single-frequency data, and the results indicate that it provides high accuracy and robust performance.

随着 GNSS 的快速发展，精密单点定位（PPP）已广泛应用于定位、导航、授时（PNT）和大气探测等方面。目前，低成本单频硬件的需求日益增长。然而，开源的单频PPP（SFPPP）软件，特别是那些采用非组合 GNSS 观测值的软件，却相当稀少。鉴于这一事实，我们开发了一款名为 **SUPREME** (**S**ingle-**f**requency **U**ncombined **P**REcise **p**oint positioning for **M**ulti-parameter **E**stimation 多参数估计单频非组合精密单点定位) 的开源软件，该软件能够处理来自低成本单频接收器的多 GNSS 观测数据。它可以利用最小二乘（LSF）同时估计接收器坐标、接收器时钟偏移、天顶对流层延迟（ZTD）和电离层延迟。SUPREME 采用 C/C++ 编程语言开发，因此具有良好的可移植性和跨平台能力。其格式化的输出也便于后续结果分析。我们基于真实的单频数据进行了多次实验，以评估 SUPREME 的性能，结果表明它提供了高精度和稳健的表现。

---

## GiRsnow：

[GiRsnow: An open-source software for snow depth retrievals using GNSS Interferometric Reflectometry by Shuangcheng Zhang, Jilun Peng, Chenglong Zhang, Jingjiang Zhang, Lixia Wang, Tao Wang, Qi Liu](https://geodesy.noaa.gov/gps-toolbox/GiRsnow.htm)

Abstract: Snow is an important water resource that plays a critical role in the global climate and hydrological cycle. Thus, Global Navigation Satellite System Interferometric Reflectometry (GNSS-IR) has emerged as a new remote sensing technology for monitoring snow depth. To obtain robust and effective data retrieval, we developed a snow parameter processing software based on GNSS-IR in a MATLAB environment, called GiRsnow, that allows users to check data quality, draw reflection point trajectory and the Fresnel zone, retrieve snow depth using signal-to-noise ratio (SNR) observations or geometry-free linear carrier phase combination (termed L4) observations, and display the results based on the time and space domain. We conducted two experiments at the Plate Boundary Observatory site RN86 and at the GPS Earth Observation Network (GEONET) site 020877 to validate the performance of the software. Our results demonstrate that GiRsnow can process multi-constellation and multi-frequency GNSS data and obtain robust and effective results through quality control and a grid model to account for the effects of topography.  

雪是一种重要的水资源，在全球气候和水文循环中发挥着关键作用。目前，全球导航卫星系统干涉反射测量（GNSS-IR）已经成为一种用于监测雪深的新型遥感技术。为了获得稳健和有效的数据检索，我们在 MATLAB 环境中开发了一款基于 GNSS-IR 的雪参数反演软件，名为 GiRsnow。该软件允许用户检查数据质量，绘制反射点轨迹和菲涅耳区，利用信噪比（SNR）观测值或无几何误差线性载波相位组合（称为L4）观测值来检索雪深，并基于时间和空间域显示结果。我们在板块边界观测站RN86和GPS地球观测网络（GEONET）站点020877进行了两次实验，以验证该软件的性能。我们的结果表明，GiRsnow能够处理多星座和多频GNSS数据，并通过质量控制和网格模型来考虑地形的影响，从而获得稳健和有效的结果。

---

## GREAT-UPD

[GREAT-UPD: An open-source software for uncalibrated phase delay estimation based on multi-GNSS and multi-frequency observations by Xingxing Li, Xinjuan Han, Xin Li, Gege Liu, Guolong Feng, Bo Wang, Hongjie Zheng](https://geodesy.noaa.gov/gps-toolbox/GREAT-UPD.htm)

To meet the demands of precise orbit and clock determination, high-precision positioning, and navigation applications, a software called GREAT (GNSS+ REsearch, Application and Teaching) was designed and developed at Wuhan University. As one important module in GREAT software, GREAT-UPD was developed for multi-GNSS and multi-frequency uncalibrated phase delay (UPD) estimation. It can provide extra-wide-lane (EWL), wide-lane (WL) and narrow-lane (NL) UPDs of for GPS, GLONASS, Galileo and BDS (GREC) satellites for users to achieve precise point positioning (PPP) ambiguity resolution (AR) in the a multi-GNSS and multi-frequency environment. The open-source GREAT-UPD software is written in C++ 11 language following the Object-Oriented principles and can be compiled, and run on several popular operating systems, such as Windows, Linux, and Macintosh. Observations from 222 stations spanning days from DOY 091 to 120 were used to conduct multi-GNSS and multi-frequency UPD estimation and PPP AR. Results indicate that stable and reliable UPD products can be generated by GREAT-UPD with multi-GNSS and multi-frequency observations. After applying the UPD corrections, the multi-frequency GREC PPP AR was achieved with the averaged time to first fix (TTFF) of 9.0 min. The software package can be obtained at https://geodesy.noaa.gov/gps-toolbox, including the source code, user manual, batch processing scripts, examples data, and some auxiliary tools.  

摘要：为满足精确轨道和时钟确定、高精度定位和导航应用的需求，武汉大学设计并开发了名为 GREAT（GNSS+ 研究、应用和教学）的软件。作为 GREAT 软件的一个重要模块，GREAT-UPD 是为多 GNSS 和多频率未校准相位延迟（UPD）估计而开发的。它可以提供 GPS、GLONASS、Galileo 和 BDS（GREC）卫星的超宽线（EWL）、宽线（WL）和窄线（NL）UPD，供用户在多全球导航卫星系统和多频率环境中实现精密单点定位（PPP）的模糊分辨率（AR）。开源的 GREAT-UPD 软件使用 C++ 11 语言编写，遵循面向对象的原则，可在 Windows、Linux 和 Macintosh 等多个流行的操作系统上编译和运行。利用从 DOY 091 到 120 天的 222 个观测站的观测数据，进行了多全球导航卫星系统和多频率 UPD 估计以及 PPP AR。结果表明，GREAT-UPD 可以利用多全球导航卫星系统和多频率观测生成稳定可靠的 UPD 产品。应用 UPD 改正后，实现了多频率 GREC PPP AR，平均首次定点时间（TTFF）为 9.0 分钟。该软件包可在 https://geodesy.noaa.gov/gps-toolbox 上获取，包括源代码、用户手册、批处理脚本、示例数据和一些辅助工具。 


---

## PPPLib

[PPPLib: An open-source software for precise point positioning using GPS, BeiDou, Galileo, GLONASS, and QZSS with multi-frequency observations by Chao Chen, Guobin Chang](https://geodesy.noaa.gov/gps-toolbox/PPPLib.htm)

Precise Point Positioning Library (PPPLib) is a multi-GNSS data processing software designed to process multi-frequency data from GPS, BeiDou, Galileo, GLONASS, and QZSS. PPPLib is written in the C/C++ programming language. It can compile and run on both Linux and Windows operating systems. PPPLib mainly performs precise point positioning from single- to triple-frequency based on either ionosphere-free or uncombined observations. Moreover, it solves for abundant parameters including position, tropospheric delay, ionospheric delay, and ambiguity information. Useful scripts and visualization tools are also provided for data download, batch processing, or solution presentation. We give a preliminary review, including positioning accuracy and convergence time of PPP using dual-frequency, ionosphere-free from single-system to multi-GNSS, to show the working status of current version of the software. In addition, the software also supports post-processing kinematic mode and INS/GNSS loosely coupled mode for realtime kinematic positioning.  

摘要：精密单点定位库（PPPLib）是一个多全球导航卫星系统数据处理软件，旨在处理 GPS、北斗、Galileo、格罗纳斯和 QZSS 的多频率数据。PPPLib 用 C/C++ 编程语言编写。它可以在 Linux 和 Windows 操作系统上编译和运行。PPPLib 主要基于无电离层或非综合观测数据，执行单频到三频的精密单点定位。此外，它还能求解包括位置、对流层延迟、电离层延迟和模糊性信息在内的大量参数。还提供了有用的脚本和可视化工具，用于数据下载、批量处理或解决方案演示。我们对使用双频、无电离层、从单系统到多全球导航卫星系统的 PPP 进行了初步评述，包括定位精度和收敛时间，以展示当前版本软件的工作状态。此外，该软件还支持后处理运动模式和 INS/GNSS 松散耦合模式，以实现实时运动定位。 

---

## 

[An open-source low-cost sensor for SNR-based GNSS reflectometry: Design and long-term validation towards sea level altimetry by M.A.R. Fagundes, I. Mendonsa-Tinti, A.L. Iescheck, D.M. Akos, and F. Geremia-Nievinski](https://geodesy.noaa.gov/gps-toolbox/GNSS-R_sensor.htm)

Monitoring sea level is critical due to climate change observed over the years. Global Navigation Satellite System Reflectometry (GNSS-R) has been widely demonstrated for coastal sea level monitoring. The use of signal-to-noise ratio (SNR) observations from ground-based stations has been especially productive for altimetry applications. For a non-geostationary GNSS satellite, SNR records an interference pattern whose oscillation frequency allows retrieving the unknown reflector height. Here we report the development and validation of a complete hardware and software system for SNR-based GNSS-R. We make it available as open-source based on the Arduino platform. It costs about USD200 (including solar power supply) and requires minimal assembly of commercial off-the-shelf components. As an initial validation towards applications in coastal regions, we have evaluated the system over approximately one year by the Guaiba Lake in Brazil. We have compared water level altimetry retrievals with independent measurements from a co-located radar tide gauge (within 10 meters). The GNSS-R device ran practically uninterrupted while the reference radar gauge suffered two malfunctioning periods, resulting in gaps lasting for 44 and 38 days. The stability of GNSS-R altimetry results enabled the detection of miscalibration steps (10 cm and 15 cm) inadvertently introduced in the radar gauge after it underwent maintenance. Excluding the radar gaps and its malfunctioning periods (reducing the time series duration from 317 to 147 days), we have found a correlation of 0.989 and RMSE of 2.9 cm in daily means. To foster open science and lower the barriers for entry in SNR-based GNSS-R research and applications, we make a complete bill of materials and build tutorial freely available on the Internet, so that interested researchers can reproduce the system.  

 由于多年来观察到的气候变化，海平面监测至关重要。全球导航卫星系统反射测量法（GNSS-R）已广泛应用于沿海海平面监测。在测高应用中，利用地面站的信噪比（SNR）观测尤其富有成效。对于非地球静止 GNSS 卫星，信噪比记录的是一种干扰模式，其振荡频率可以检索未知的反射器高度。我们在此报告基于 SNR 的 GNSS-R 的完整硬件和软件系统的开发和验证情况。我们将其作为基于 Arduino 平台的开放源代码提供。它的成本约为 200 美元（包括太阳能电源），只需将现成的商用组件组装到最低限度即可。作为对沿海地区应用的初步验证，我们在巴西瓜伊巴湖对该系统进行了大约一年的评估。我们将水位测高检索与同地雷达验潮仪（10 米以内）的独立测量结果进行了比较。全球导航卫星系统-R 设备几乎不间断地运行，而参考雷达测高仪却出现了两次故障，造成 44 天和 38 天的间隙。全球导航卫星系统-R 测高结果的稳定性使得能够检测到雷达测高仪在维修后无意中引入的误校准步骤（10 厘米和 15 厘米）。排除雷达间隙及其故障期（将时间序列从 317 天缩短至 147 天），我们发现日均值的相关性为 0.989，均方根误差为 2.9 厘米。为了促进开放科学，降低基于信噪比的 GNSS-R 研究和应用的准入门槛，我们在互联网上免费提供了完整的材料清单和构建教程，以便有兴趣的研究人员可以复制该系统。 

---

## 

[Python software tools for GNSS interferometric reflectometry (GNSS-IR) by Angel Martin, Raquel Lujan, and Ana Belen Anquela](https://geodesy.noaa.gov/gps-toolbox/pyGNSS-IR.htm)

Global Navigation Satellite System (GNSS) interferometric reflectometry, also known as the GNSS-IR technique, uses data from geodetic-quality GNSS antennas to extract information about the environment surrounding the antenna. Soil moisture monitoring is one of the most important applications of the GNSS-IR technique. This manuscript presents the main ideas and implementation decisions needed to write the Python code for software tools that transform RINEX format observation and navigation files into an appropriate format for GNSS-IR (which includes the SNR observations and the azimuth and elevation of the satellites), and to determine the reflection height and the adjusted phase and amplitude values of the interferometric wave for each individual satellite track. The main goal of the manuscript is to share the software with the scientific community to introduce new users to the GNSS-IR technique.  

全球导航卫星系统（GNSS）干涉反射测量法，又称 GNSS-IR 技术，利用来自大地测量质量的 GNSS 天线的数据来提取天线周围环境的信息。土壤水分监测是 GNSS-IR 技术最重要的应用之一。本手稿介绍了编写软件工具 Python 代码所需的主要思路和实施决策，这些软件工具可将 RINEX 格式的观测和导航文件转换为 GNSS-IR 的适当格式（其中包括信噪比观测值以及卫星的方位角和仰角），并确定每个单独卫星轨迹的反射高度以及干涉波的调整相位和振幅值。手稿的主要目的是与科学界分享该软件，向新用户介绍全球导航卫星系统红外技术。 

---

## 

[Precise vehicle dynamic heading and pitch angle using time-differenced measurements from a single GNSS antenna by Rui Sun, Qi Cheng, Junhui Wang](https://geodesy.noaa.gov/gps-toolbox/iTAG.htm)

The attitude information of ground vehicles, i.e., heading and pitch information, can be used for many transport applications. This information can be determined through various solutions of onboard sensors, including magnetic sensors, inertial sensors, GNSS equipment, and their combinations. In order to find a cost-effective solution, we propose here a new toolbox named Vehicle Attitude Determination toolbox from the Intelligent Transportation And Geomatics group (iTAG_VAD) that determines vehicle dynamic heading and pitch using only one single-frequency GPS antenna, reducing costs while retaining an acceptable heading and pitch accuracy. In this toolbox, by using the constraints of the vehicle motion and road geometry, two variations of a time differenced measurement model, i.e., Time Difference Carrier Phase (TDCP) and Time Differenced Pseudorange (TDPR), are developed for heading and pitch determination. The results show that both proposed TDCP- and TDPR-based models are able to provide an accurate heading and pitch estimation with a Root Mean Square Error (RMSE) below the one-degree level, which is sufficient for many land-based applications. We describe the toolbox and its implementation in MATLAB in detail.  

地面车辆的姿态信息，即航向和俯仰信息，可用于许多运输应用。这些信息可以通过各种车载传感器来确定，包括磁传感器、惯性传感器、GNSS 设备及其组合。为了找到一种经济有效的解决方案，我们在此提出了一个新的工具箱，名为智能交通和地理信息小组的车辆姿态确定工具箱（iTAG_VAD），该工具箱仅使用一个单频 GPS 天线就能确定车辆的动态航向和俯仰角，在降低成本的同时保持了可接受的航向和俯仰角精度。在该工具箱中，利用车辆运动和道路几何的约束条件，开发了两种不同的时间差测量模型，即时间差载波相位（TDCP）和时间差伪距（TDPR），用于确定航向和俯仰。结果表明，所提出的基于 TDCP 和 TDPR 的模型都能提供精确的航向和俯仰估计，均方根误差 (RMSE) 低于 1 度，足以满足许多陆基应用的需要。我们将详细介绍工具箱及其在 MATLAB 中的实现。 

---

## GDP

[GDP: An Open Source GNSS Data Preprocessing Toolkit by Zhengsheng Chen, Yang Cui, Linyang Li, Qinghua Zhang, Zhiping Lu, Xuerui Li, Yingcai Kuang, Kaichun Yang, Fengjuan Rong](https://geodesy.noaa.gov/gps-toolbox/GDP.htm)

Abstract: GDP (GNSS Data Preprocessor) is a multi-GNSS data preprocessing software designed to process raw GNSS observation data in the RINEX (Receiver Independent Exchange Format) 2.x to 3.x standard. Published under a free and open-source license, LGPL(GNU Lesser General Public License), written in object-oriented programming language C#, it mainly includes multi-GNSS file and IGS product automatic acquisition, format conversion, file selection, data visualization, and analysis of GNSS observation files. It provides both a Windows form interface and a command shell interface for the Windows, Linux, or macOS operating system. Volunteers can also participate in and improve the software through GitHub. This software is continuing to evolve, improving its functionalities according to the updates introduced by the collaborators. We give a brief introduction to the software, including the software architecture, functions, and modules. Lastly, we give several examples, including parallel computing, visual satellite graphical display, and cycle slip detection, to show the working status of the current version of the software.  

GDP（GNSS Data Preprocessor）是一个多 GNSS 数据预处理软件，旨在处理 RINEX（接收机独立交换格式）2.x 至 3.x 标准的原始 GNSS 观测数据。它以免费开源许可证 LGPL（GNU Lesser General Public License）发布，用面向对象的 C# 编程语言编写，主要包括多 GNSS 文件和 IGS 产品的自动获取、格式转换、文件选择、数据可视化以及 GNSS 观测文件的分析。它提供 Windows 表单界面和命令 shell 界面，适用于 Windows、Linux 或 macOS 操作系统。志愿者还可以通过 GitHub 参与和改进该软件。该软件还在不断发展，根据合作者的更新完善其功能。我们将简要介绍该软件，包括软件架构、功能和模块。最后，我们举几个例子，包括并行计算、可视化卫星图形显示和周期滑移检测，以展示当前版本软件的工作状态。 

---

## 

[Design and implementation of an open-source BDS-3 B1C/B2a SDR receiver by Yafeng Li, Nagaraj C. Shivaramaiah, and Dennis M. Akos](https://geodesy.noaa.gov/gps-toolbox/BDS-SDR.htm)

Abstract: GNSS software-defined radio (SDR) receiver has been and will continue to be a tremendous research enabler given its flexibility and GNSS modernization as well as improvements to complimentary technologies. An open-source suite of GNSS SDRs capable of post-processing all open-service GNSS signals has been developed by the GNSS Lab at the University of Colorado, Boulder. As the latest expansion, processing capabilities for the B1C/B2a signals of the third-generation BeiDou navigation satellite system (BDS-3) is incorporated into this SDR package. To provide a basic implementation framework for GNSS community, separate or joint processing of the data and pilot channels are realized in the B1C/B2a SDR; and both narrowband and wideband tracking modes are implemented specifically for B1C pilot channel. Soon after the launch of the first two BDS-3 satellites, the B1C/B2a signals have been captured and the initial tracking results have been obtained. We describe the design strategy and implementation of the BDS-3 B1C/B2a SDR and report the processing results. The emphasis is placed on the B1C processing due to the novelty and complexity of the quadrature multiplexed binary offset carrier (QMBOC) modulation employed by B1C.  

GNSS 软件定义无线电（SDR）接收器因其灵活性、GNSS 现代化以及对补充技术的改进，已经并将继续成为巨大的研究推动力。科罗拉多大学博尔德分校的全球导航卫星系统实验室开发了一套开源的全球导航卫星系统 SDR，能够对所有开放服务的全球导航卫星系统信号进行后处理。作为最新的扩展，第三代北斗导航卫星系统（BDS-3）的 B1C/B2a 信号处理能力已纳入该 SDR 包。为了给全球导航卫星系统界提供一个基本的实施框架，B1C/B2a SDR 实现了数据信道和先导信道的单独或联合处理；并专门为 B1C 先导信道实现了窄带和宽带跟踪模式。首批两颗 BDS-3 卫星发射后不久，B1C/B2a 信号已被捕获，并取得了初步跟踪结果。我们介绍了 BDS-3 B1C/B2a SDR 的设计策略和实施情况，并报告了处理结果。由于 B1C 采用的正交多路复用二进制偏移载波（QMBOC）调制方式的新颖性和复杂性，我们将重点放在 B1C 处理上。 


---

## MG-APP

[MG-APP: An open-source software for multi-GNSS precise point positioning and application analysis by Gongwei Xiao, Genyou Liu, Jikun Ou, Guolin Liu, Shengli Wang, Aizhi Guo](https://geodesy.noaa.gov/gps-toolbox/MG-APP.htm)

Abstract: With the optimization of GNSS constellations, multi-GNSS positioning theory and applications have shown further development. To meet the demands of precise point positioning (PPP) theoretical research and applications, we developed a GNSS data processing software named MG-APP (Multi-GNSS Automatic Precise Positioning software). MG-APP is an open-source software that can be run on Windows/Linux/UNIX and other operating systems. It can simultaneously process GPS/GLONASS/BDS/Galileo observations using a Kalman filter or a Square Root Information Filter (SRIF). Compared with the Kalman filter, the SRIF has better numerical stability and maintains stable convergence even with a significant round-off error. MG-APP has a comprehensive and friendly graphical user interface that conveniently allows the user to select models and set parameters. It also contains several types of tropospheric and estimation models that make it easy to analyze the impact of different models and parameters on PPP data processing. After the data processing finishes, zenith tropospheric delays (ZTDs), receiver clock offsets, satellite ambiguity parameters, observation value residuals, and other results will be saved into files. Users can further analyze the solution results and construct graphs easily.  

随着全球导航卫星系统（GNSS）星座的优化，多GNSS定位理论和应用有了进一步的发展。为了满足精密单点定位（PPP）理论研究和应用的需求，我们开发了一款名为 MG-APP （Multi-GNSS Automatic Precise Positioning software）的 GNSS 数据处理软件。MG-APP 是一款开源软件，可在 Windows/Linux/UNIX 和其他操作系统上运行。它可以使用卡尔曼滤波器或平方根信息滤波器（SRIF）同时处理 GPS/GLONASS/BDS/Galileo 观测数据。与卡尔曼滤波器相比，SRIF 具有更好的数值稳定性，即使存在较大的舍入误差，也能保持稳定的收敛性。MG-APP 有一个全面而友好的图形用户界面，方便用户选择模型和设置参数。它还包含多种类型的对流层模型和估算模型，便于分析不同模型和参数对 PPP 数据处理的影响。数据处理完成后，天顶对流层延迟（ZTD）、接收机时钟偏移、卫星模糊参数、观测值残差等结果将保存到文件中。用户可以进一步分析求解结果，并轻松构建图表。 

---

## 

[Framework for GREIS-formatted GNSS data manipulation (C++ code for manipulating Javad GNSS receiver data) by Igor M. Aleshin, Kirill I. Kholodkov, Vladimir N. Koryagin](https://geodesy.noaa.gov/gps-toolbox/GREIS.htm)

We introduce an application framework that enables easy implementation of applications that process GNSS Receiver External Interface Specification (GREIS)-formatted GNSS measurement data. The framework utilizes anti-aging techniques such as automatic code generation, documentation renewal, building and component testing, which makes this framework effectively an always-up-to-date (evergreen) software. This paper also includes an example case of the framework: a simple data inspection software for measurement data produced by Javad receivers (and compatible receivers) and data acquisition software with realtime telemetry. The framework is written in C++ and is available online.  

我们介绍了一个应用框架，该框架能够轻松实施处理 GNSS 接收机外部接口规范（GREIS）格式 GNSS 测量数据的应用。该框架采用了自动代码生成、文档更新、构建和组件测试等抗老化技术，从而使该框架成为一个始终保持最新（常青）的软件。本文还包括该框架的一个示例：一个简单的数据检查软件，用于检查 Javad 接收机（及兼容接收机）产生的测量数据，以及具有实时遥测功能的数据采集软件。该框架由 C++ 编写，可在线获取。 

---

## PANG-NAV

[PANG-NAV: a tool for processing GNSS measurements in SPP, including RAIM functionality by Antonio Angrisano, Salvatore Gaglione, Nicola Crocetto, and Mario Vultaggio](https://geodesy.noaa.gov/gps-toolbox/PANG-NAV.htm)

Global Navigation Satellite Systems are theoretically able to provide accurate, three-dimensional, and continuous positioning to an unlimited number of users. An important shortcoming of GNSS is the lack of integrity, defined as the ability of a system to provide timely warnings in case of malfunction; this problem is especially felt in safety-of-life applications such as aviation. A common way to fill this gap is the use of Receiver Autonomous Integrity Monitoring (RAIM) techniques, which are able to provide integrity information by analyzing redundant measurements. A possible RAIM functionality is the ability to identify, and so discard, anomalous measurements; this functionality has made RAIM very useful also in case of severe signal degradation, such as in urban or dense vegetation areas, where blunders are common. PANG-NAV is a tool, developed by the Parthenope Navigation Group, able to process GNSS measurements (from RINEX files) in order to obtain position solution. The core of PANG-NAV is the Single Point Positioning (SPP) technique, including a RAIM functionality. Multi-constellation solution, with GPS and Galileo, can be provided. Both static and kinematic processing are possible and, in case a ground truth is available, an error analysis can be carried out.  

全球导航卫星系统理论上能够为无限数量的用户提供精确、三维和连续的定位。全球导航卫星系统的一个重要缺陷是缺乏完整性，即系统在发生故障时及时发出警告的能力；这一问题在航空等生命安全应用中尤为突出。填补这一空白的常用方法是使用接收器自主完整性监测（RAIM）技术，该技术能够通过分析冗余测量提供完整性信息。RAIM 的一个可能功能是识别并丢弃异常测量值；这一功能使得 RAIM 在信号严重衰减的情况下也非常有用，例如在城市或植被茂密的地区，失误很常见。PANG-NAV 是 Parthenope 导航小组开发的一种工具，能够处理 GNSS 测量数据（来自 RINEX 文件），以获得位置解决方案。PANG-NAV 的核心是单点定位 (SPP) 技术，包括 RAIM 功能。可提供 GPS 和Galileo的多星座解决方案。可进行静态和运动学处理，如果有地面实况，还可进行误差分析。 

---

## GLONASS 模糊度固定

[GLONASS ambiguity resolution by P. J. G. Teunissen and A. Khodabandeh](https://geodesy.noaa.gov/gps-toolbox/GLONASS-L.htm)

Abstract: A new integer-estimable GLONASS FDMA model will be studied and analyzed. The model is generally applicable and it shows a close resemblance with the well-known CDMA models. The analyses provide insights into the performance characteristics of the model and concern a variety of different ambiguity resolution critical applications. This will be done for geometry-free, geometry-fixed and several geometry-based formulations. Next to the analyses of the model's instantaneous ambiguity-resolved positioning and attitude determination capabilities, we show the ease with which the model can be combined with existing CDMA models. We thereby present the instantaneous ambiguity resolution performances of integrated L1 GPS+GLONASS, both for high-grade geodetic and mass-market receivers. We also consider the potential of the single-frequency combined model for mixed-receiver processing, particularly for the case the between-receiver GLONASS pseudorange data are biased. In all cases, the speed of successful ambiguity resolution is studied as well as the precision with which positioning is determined. Software routines for constructing the model are also provided.  

将研究和分析一种新的可整数估算的 GLONASS FDMA 模型。该模型普遍适用，与著名的 CDMA 模型非常相似。分析深入揭示了该模型的性能特征，并涉及各种不同的模糊分辨率关键应用。分析将针对无几何模型、固定几何模型和若干基于几何模型的公式。除了分析模型的瞬时模糊解决定位和姿态确定能力外，我们还展示了该模型与现有 CDMA 模型结合的易用性。因此，我们介绍了组合 L1 GPS+GLONASS 的瞬时模糊分辨率性能，既适用于高级大地测量接收机，也适用于大众市场接收机。我们还考虑了单频组合模型在混合接收机处理方面的潜力，特别是在接收机之间的格罗纳斯伪距数据有偏差的情况下。在所有情况下，我们都会研究成功解决模糊问题的速度以及确定定位的精度。还提供了构建模型的软件例程。 

---

## PRIDE PPP-AR

[PRIDE PPP-AR: an Open-source Software for GPS PPP Ambiguity Resolution by Jianghui Geng, Xingyu Chen,
Yuanxin Pan, Shuyin Mao, Chenghong Li, Jinning Zhou, and Kunlun Zhang](https://geodesy.noaa.gov/gps-toolbox/PRIDE.htm)

The PRIDE Lab at GNSS Research Center of Wuhan University has developed an open-source software for GPS precise point positioning ambiguity resolution (PPP-AR) (i.e., PRIDE PPP-AR). Released under the terms of the GNU General Public License version 3 (GPLv3, http://www.gnu.org/licenses/gpl.html), PRIDE PPP-AR supports relevant research, application and development with GPS post-processing PPP-AR. PRIDE PPP-AR is mainly composed of two modules, undifferenced GPS processing and single-station ambiguity resolution. Undifferenced GPS processing provides float solutions with wide-lane and narrow-lane ambiguity estimates. Later, single-station ambiguity resolution makes use of the phase clock/bias products, which are released also by the PRIDE Lab at ftp://pridelab.whu.edu.cn/pub/whu/phasebias/, to recover the integer nature of single-station ambiguities and then carry out integer ambiguity resolution. PRIDE PPP-AR is based on a least-squares estimator to produce daily, sub-daily or kinematic solutions for various geophysical applications. In order to facilitate the usage of this software, a few user-friendly shell scripts for batch processing have also been provided along with PRIDE PPP-AR. In this article, we use one month of GPS data (days 001-031 in 2018) to demonstrate the performance of PRIDE PPP-AR software. The PRIDE Lab is committed to persistently improve the software package and keep users updated through our website.  

武汉大学GNSS研究中心PRIDE实验室开发了GPS精密单点定位模糊度解决（PPP-AR）开源软件（即PRIDE PPP-AR）。PRIDE PPP-AR 根据 GNU General Public License version 3 (GPLv3, http://www.gnu.org/licenses/gpl.html) 条款发布，支持 GPS 后处理 PPP-AR 的相关研究、应用和开发。PRIDE PPP-AR 主要由两个模块组成：无差别 GPS 处理和单站模糊性解决。无差别 GPS 处理可提供宽线和窄线模糊估计的浮动解。之后，单站模糊性解析利用相位时钟/偏置产品（也由 PRIDE 实验室在 ftp://pridelab.whu.edu.cn/pub/whu/phasebias/ 上发布）恢复单站模糊性的整数性质，然后进行整数模糊性解析。PRIDE PPP-AR 以最小二乘估计器为基础，可为各种地球物理应用生成日解、亚日解或运动解。为了方便使用该软件，PRIDE PPP-AR 还提供了一些用户友好的 shell 脚本用于批处理。在本文中，我们使用一个月的 GPS 数据（2018 年第 001-031 天）来演示 PRIDE PPP-AR 软件的性能。PRIDE 实验室致力于不断改进软件包，并通过我们的网站向用户提供最新信息。 

---

## 

[A SIMD Intrinsic Correlator Library for GNSS Software Receivers by Damian Miralles and Dennis M. Akos](https://geodesy.noaa.gov/gps-toolbox/SIMD-AVX.htm)

An open source implementation of a Code Division Multiple Access (CDMA) software correlator library that leverages Single Instruction Multiple Data (SIMD) is presented. We initially discuss the key aspects involved in the correlation operation for software radio applications. Afterward, we present the state of the art Application Programming Interface (API) that provides SIMD capable methods for each of the components in a correlation operation, including the first of its kind parallelized code and carrier generation using lookup tables and SIMD instructions. The library is was developed using SIMD Intrinsic Instructions, which are a C type nomenclature offering access to the assembly instructions originally designed for the SIMD extensions in the processor. This design paradigm presents an advantage in terms of readability and simplified code development to accommodate future modifications. Recorded data was were used with a standalone Global Navigation Satellite System (GNSS) software receiver where the methods hereby presented were tested and profiled to validate theoretical assumptions.  

本文介绍了利用单指令多数据（SIMD）的码分多址（CDMA）软件相关器库的开源实现。我们首先讨论了软件无线电应用中相关操作所涉及的关键方面。随后，我们介绍了最先进的应用编程接口 (API)，该接口为相关操作中的每个组件提供 SIMD 功能方法，包括首次使用查找表和 SIMD 指令并行生成代码和载波。该库使用 SIMD 本征指令开发，这是一种 C 类型命名法，可访问最初为处理器中的 SIMD 扩展而设计的汇编指令。这种设计范式在可读性和简化代码开发以适应未来修改方面具有优势。记录的数据被用于一个独立的全球导航卫星系统（GNSS）软件接收器，在该接收器上对本文介绍的方法进行了测试和剖析，以验证理论假设。 

---

## LSWAVE：最小二乘小波分析

[LSWAVE: a MATLAB software for the least-squares wavelet and cross-wavelet analyses by Ebrahim Ghaderpour and Spiros D. Pagiatakis](https://geodesy.noaa.gov/gps-toolbox/LSWAVE.htm)

The least-squares wavelet analysis (LSWA) is a robust method of analyzing any type of time/data series without the need for editing and preprocessing of the original series. The LSWA can rigorously analyze any non-stationary and equally/unequally spaced series with an associated covariance matrix that may have trends and/or datum shifts. The least-squares cross-wavelet analysis complements the LSWA in the study of the coherency and phase differences of two series of any type. A MATLAB software package including a graphical user interface is developed for these methods to aid researchers in analyzing pairs of series. The package also includes the least-squares spectral analysis, the antileakage least-squares spectral analysis, and the least-squares cross-spectral analysis to further help researchers study the components of interest in a series. We demonstrate the steps that users need to take for a successful analysis using three examples: two synthetic time series, and a Global Positioning System time series.  

最小二乘小波分析（LSWA）是一种分析任何类型时间/数据序列的稳健方法，无需对原始序列进行编辑和预处理。LSWA 可以严格分析任何非平稳和等距/非等距序列，其相关协方差矩阵可能具有趋势和/或基准点偏移。最小二乘交叉小波分析是 LSWA 的补充，可用于研究任何类型两个序列的一致性和相位差。为这些方法开发了一个 MATLAB 软件包，包括一个图形用户界面，以帮助研究人员分析成对的序列。该软件包还包括最小二乘光谱分析、反渗漏最小二乘光谱分析和最小二乘交叉光谱分析，以进一步帮助研究人员研究序列中感兴趣的成分。我们用三个例子来演示用户成功分析所需的步骤：两个合成时间序列和一个全球定位系统时间序列。 

---

## 

[Open Source MATLAB Code for GPS Vector Tracking on a Software-Defined Receiver by Bing Xu and Li-Ta Hsu](https://geodesy.noaa.gov/gps-toolbox/GPS_VT_SDR.htm)

The research regarding Global Positioning System (GPS) vector tracking (VT), based on a software-defined receiver (SDR), has been increasing in recent years. The strengths of VT include its immunity to signal interference, its capability to mitigate multipath effects in urban areas, and its excellent performance in tracking signals under high-dynamic applications. We developed open source MATLAB code for GPS VT SDR to enable researchers and scientists to investigate its pros and cons in various applications and under various environments. To achieve this goal, we developed an equivalent conventional tracking (CT) SDR as a baseline to compare with VT. The GPS positioning estimator of this equivalent CT is based on an extended Kalman filter (EKF), which has exactly the same state, system and carrier measurement models and noise tuning method as VT. This baseline provides users with a tool to compare the performance of VT and CT on common ground. In addition, this MATLAB code is well-organized and easy to use. Users can quickly implement and evaluate their own newly developed baseband signal processing algorithms related to VT. The implementation of this VT code is described in detail. Finally, static and kinematic experiments were conducted in an urban and open-sky area, respectively, to show the usage and performance of the developed open source GPS VT SDR.  

近年来，有关基于软件定义接收器（SDR）的全球定位系统（GPS）矢量跟踪（VT）的研究日益增多。矢量跟踪的优点包括抗信号干扰能力强、可减轻城市地区的多径效应以及在高动态应用下跟踪信号的优异性能。我们开发了 GPS VT SDR 的开源 MATLAB 代码，使研究人员和科学家能够研究其在各种应用和环境下的优缺点。为了实现这一目标，我们开发了一个等效的传统跟踪 (CT) SDR，作为与 VT 进行比较的基线。该等效 CT 的 GPS 定位估计器基于扩展卡尔曼滤波器 (EKF)，其状态、系统和载波测量模型以及噪声调整方法与 VT 完全相同。该基线为用户提供了一种工具，用于在共同基础上比较 VT 和 CT 的性能。此外，该 MATLAB 代码条理清晰，易于使用。用户可以快速实现和评估自己新开发的与 VT 相关的基带信号处理算法。本文详细介绍了 VT 代码的实现过程。最后，分别在城市和开阔天空区域进行了静态和运动学实验，以显示所开发的开源 GPS VT SDR 的用途和性能。 

---

## SARI

[SARI: interactive GNSS position time series analysis software by Alvaro Santamaria-Gomez](https://geodesy.noaa.gov/gps-toolbox/SARI.htm)

Abstract: GNSS position time series contain signals induced by earth deformation, but also by systematic errors, at different time scales, from sub-daily tidal deformation to inter-annual surface loading deformation and secular tectonic plate rotation. This software allows users to visualize GNSS position time series, but also any other series, and interactively remove outliers and discontinuities, fit models and save the results. A comprehensive list of features is included to help the user extracting relevant information from the series, including spectral analysis with the Lomb-Scargle periodogram and wavelet transform; signal filtering with the Kalman filter and the Vondrak smoother; and estimation of the time-correlated stochastic noise of the residuals. The software can be run on a local machine if all the package dependencies are satisfied or remotely via a public web server with no other requirement than an Internet connection.  

摘要：全球导航卫星系统位置时间序列包含由地球形变引起的信号，也包含由系统误差引起的信号，时间尺度不同，从亚日潮汐形变到年际地表载荷形变和世俗构造板块旋转。该软件允许用户对全球导航卫星系统位置时间序列以及任何其他序列进行可视化，并以交互方式移除异常值和不连续性，拟合模型并保存结果。该软件包含一系列全面的功能，帮助用户从序列中提取相关信息，包括使用 Lomb-Scargle 周期图和小波变换进行频谱分析；使用卡尔曼滤波器和 Vondrak 平滑器进行信号滤波；以及估计残差的时间相关随机噪声。该软件可在满足所有软件包依赖性要求的情况下在本地机器上运行，也可通过公共网络服务器远程运行，除互联网连接外没有其他要求。 

Introduction: The Senales y Analisis de Ruido Interactivo (Interactive Signal and Noise Analysis or SARI) software uses a browser-based interactive user interface to visualize series of data, fit multi-parameter models and analyze the residuals. It is developed in the R programming language under the interactive framework of the Shiny R package. This allows the user to run the software on a local machine or remotely by connecting and uploading the series to a public web server accessible at https://alvarosg.shinyapps.io/sari. The latter option would be of interest for those users not familiar with developing R code or with limited computing resources. In order to run the code in a local machine, the user needs to install R and the required dependencies, all being non-commercial open-source packages supported on Windows and Linux/Unix-like platforms, including macOS. The app was originally developed in early 2017 to estimate tectonic velocities from GPS position time series from the RESIF/RENAG network (RESIF 2017) and other network partners in France. Despite being oriented towards daily/weekly GPS position time series in north-east-up (NEU) format, any other data series can be analyzed as long as simple and consistent tabulated format is used. For instance, monthly and hourly tide gauge series, 1-Hz GNSS-IR SNR series and distance-dependent pseudo-variogram series have also been analyzed.  

简介：交互式信号与噪声分析（SARI）软件使用基于浏览器的交互式用户界面来可视化系列数据、拟合多参数模型并分析残差。该软件是在 Shiny R 软件包的交互式框架下使用 R 编程语言开发的。用户可以在本地机器上运行该软件，也可以通过连接并将序列数据上传到 https://alvarosg.shinyapps.io/sari 的公共网络服务器上远程运行该软件。对于那些不熟悉 R 代码开发或计算资源有限的用户来说，后一种选择会更有吸引力。为了在本地机器上运行代码，用户需要安装 R 和所需的依赖项，这些都是非商业性的开源软件包，支持 Windows 和 Linux/Unix-like 平台，包括 macOS。该应用程序最初开发于 2017 年初，用于根据 RESIF/RENAG 网络（RESIF 2017）和法国其他网络合作伙伴的 GPS 定位时间序列估算构造速度。尽管该应用面向的是北东向上（NEU）格式的每日/每周 GPS 定位时间序列，但只要使用简单一致的表格格式，任何其他数据序列都可以进行分析。例如，月度和小时验潮仪系列、1 赫兹全球导航卫星系统-红外信噪比系列和与距离有关的伪变异图系列也已分析过。


---

## PPPH

[PPPH: A MATLAB-based software for multi-GNSS precise point positioning analysis by Berkay Bahadur and Metin Nohutcu](https://geodesy.noaa.gov/gps-toolbox/PPPH.htm)

The integration of different GNSS constellations offers considerable opportunities to improve Precise Point Positioning (PPP) performance. On the other hand, integrating multi-GNSS observations entails more complex models and algorithms compared with the traditional PPP approach, which takes GPS observations into account only. Being aware of the limited number of the alternatives that utilize the potential advantages of the multi-constellation and multi-frequency GNSS, we developed a MATLAB-based GNSS analysis software, named PPPH. PPPH is capable of processing GPS, GLONASS, Galileo and BeiDou data, and of forming their different combinations depending on user's preference. Thanks to its user-friendly graphical interface, PPPH allows users to determine a variety of processing options and parameters. In addition to an output file including the estimated parameters for every single epoch, PPPH also presents several analyzing and plotting tools for evaluating the results, such as positioning error, tropospheric zenith total delay, receiver clock estimation, satellite number, dilution of precisions, etc. On the other hand, we conducted experimental tests to both validate the performance of PPPH and assess the potential benefits of multi-GNSS on PPP. The results indicate that PPPH provides comparable PPP solution with the general standards and also contributes to the improvement of PPP performance with the integration of multi-GNSS. Consequently, we introduce a GNSS analysis software that is easy to use, has a robust performance and is open to progress with its modular structure.  

整合不同的全球导航卫星系统星座为提高精密单点定位（PPP）性能提供了大量机会。另一方面，与只考虑全球定位系统观测数据的传统精密单点定位方法相比，整合多全球导航卫星系统观测数据需要更复杂的模型和算法。考虑到利用多星座和多频率全球导航卫星系统潜在优势的替代方案数量有限，我们开发了一个基于 MATLAB 的全球导航卫星系统分析软件，名为 PPPH。PPPH 能够处理 GPS、GLONASS、Galileo 和 BeiDou 数据，并根据用户的偏好形成不同的组合。PPPH 具有用户友好的图形界面，允许用户确定各种处理选项和参数。除了包含每个单一纪元估计参数的输出文件外，PPPH 还提供了若干用于评估结果的分析和绘图工具，如定位误差、对流层天顶总延迟、接收机时钟估计、卫星数量、精度稀释等。另一方面，我们进行了实验测试，以验证 PPPH 的性能，并评估多全球导航卫星系统对 PPP 的潜在好处。结果表明，PPPH 可提供与一般标准相当的 PPP 解决方案，而且还有助于通过整合多重全球导航卫星系统提高 PPP 性能。因此，我们推出了一个易于使用、性能强大并可通过模块化结构不断进步的全球导航卫星系统分析软件。 

---

## 

[Software Tools for GNSS Interferometric Reflectometry (GNSS-IR) by Carolyn Roesler and Kristine M. Larson](https://geodesy.noaa.gov/gps-toolbox/GNSS-IR.htm)

GNSS-R interferometric reflectometry (also known as GNSS-IR, or GPS-IR for GPS signals) is a technique that uses data from geodetic-quality GNSS instruments for sensing the near-field environment. In contrast to positioning, atmospheric, and timing applications of GNSS, GNSS-IR uses signal to noise ratio (SNR) data. Software is provided to translate GNSS files, map GNSS-IR reflection zones, calculate GNSS-IR Nyquist frequencies, and estimate changes in the height of a reflecting surface from GNSS SNR data.  

GNSS-R 干涉反射测量法（又称 GNSS-IR，或 GPS 信号的 GPS-IR）是一种利用大地测量质量的 GNSS 仪器提供的数据来感测近场环境的技术。与 GNSS 的定位、大气和定时应用不同，GNSS-IR 使用信噪比（SNR）数据。提供的软件可翻译全球导航卫星系统文件、绘制全球导航卫星系统-红外反射区、计算全球导航卫星系统-红外奈奎斯特频率，以及根据全球导航卫星系统信噪比数据估算反射面高度的变化。 

---

## GAMP

[GAMP: An open-source software of multi-GNSS precise point positioning using undifferenced and uncombined observations by Feng Zhou, Danan Dong, Weiwei Li, Xinyuan Jiang, Jens Wickert, and Harald Schuh](https://geodesy.noaa.gov/gps-toolbox/GAMP.htm)

As the number of GNSS satellites and stations increases, GNSS data processing software should be developed that is easy to operate, efficient to run, and has a robust performance. To meet these requirements, we developed a new GNSS analysis software called GAMP (GNSS Analysis software for Multi-constellation and multi-frequency Precise positioning), which can perform multi-GNSS precise point positioning (PPP) based on undifferenced and uncombined observations. GAMP is a secondary development based on RTKLIB but with many improvements, such as cycle slip detection, receiver clock jump repair, and handling of GLONASS pseudorange inter-frequency biases. A simple, but unified format of output files, including positioning results, number of satellites, satellite elevation angles, pseudorange and carrier phase residuals, and slant Total Electron Content (sTEC), is defined for results analysis and plotting. Moreover, a new receiver-independent data exchange format called RCVEX is designed to improve computational efficiency for post-processing.  

随着全球导航卫星系统卫星和台站数量的增加，应开发操作简便、运行高效、性能稳定的全球导航卫星系统数据处理软件。为了满足这些要求，我们开发了一种新的 GNSS 分析软件，称为 GAMP（多星座和多频率精确定位 GNSS 分析软件），它可以根据未差分和未合并的观测数据执行多 GNSS 精密单点定位（PPP）。GAMP 是在 RTKLIB 的基础上进行的二次开发，但做了许多改进，如周期滑移检测、接收机时钟跳变修复和处理格洛纳斯伪距频间偏差。为结果分析和绘图定义了一种简单而统一的输出文件格式，包括定位结果、卫星数量、卫星仰角、伪距和载波相位残差以及斜面总电子含量（sTEC）。此外，还设计了一种称为 RCVEX 的新的独立于接收器的数据交换格式，以提高后处理的计算效率。 



---

## GMIS

[GMIS: A MATLAB-based Kriged Kalman Filter Software for Interpolating Missing Data in GNSS Coordinate Time Series by Ning Liu, Wujiao Dai, Rock Santerre, and Cuilin Kuang](https://geodesy.noaa.gov/gps-toolbox/GMIS.htm)

GNSS coordinate time series data for (permanent) reference stations often suffer from random, or even continuous, missing data. Missing data interpolation is necessary due to the fact that some data processing methods require evenly spaced data. Traditional missing data interpolation methods usually use single point time series, without considering spatial correlations between points. We present a MATLAB software for dynamic spatio-temporal interpolation of GNSS missing data based on the Kriged Kalman Filter model. With the graphical user interface, users can load source GNSS data, set parameters, view the interpolated series and save the final results. The SCIGN GPS data indicates that the software is an effective tool for GNSS coordinate time series missing data interpolation.  

（永久）基准台站的全球导航卫星系统坐标时间序列数据往往存在随机、甚至连续的缺失数据。由于某些数据处理方法要求数据间距均匀，因此有必要对缺失数据进行插值处理。传统的缺失数据插值方法通常使用单点时间序列，而不考虑点与点之间的空间相关性。我们提出了一种基于 Kriged 卡尔曼滤波模型的 MATLAB 软件，用于对 GNSS 缺失数据进行动态时空插值。通过图形用户界面，用户可以加载源 GNSS 数据、设置参数、查看插值序列并保存最终结果。SCIGN GPS 数据表明，该软件是进行 GNSS 坐标时间序列缺失数据插值的有效工具。 

---

## TSAnalyzer

[TSAnalyzer, a GNSS Time Series Analysis Software by WU Dingcheng, YAN Haoming, and SHEN Yingchun](https://geodesy.noaa.gov/gps-toolbox/TSAnalyzer.htm)

In geodesy and geophysics, continuous GNSS observations have been used globally. As the number of GNSS observing stations increase, GNSS time series analysis software should be developed with more flexible format support, better man-machine interaction, and robust analysis characteristics. To meet this requirement, a new software package called TSAnalyzer was written in Python and was developed for preprocessing and analyzing continuous GNSS position time series individually, as well as with batch processing. This software can read GNSS position time series with different formats, pick epochs of offsets or seismic events interactively, remove outliers and estimate linear, polynomial, and harmonic signals. It also provides Lomb-Scargle spectrum analysis. Since it is based on Python, it is cross-platform.  

在大地测量学和地球物理学中，全球导航卫星系统的连续观测已在全球范围内得到应用。随着 GNSS 观测站数量的增加，GNSS 时间序列分析软件的开发应具有更灵活的格式支持、更好的人机交互和强大的分析功能。为满足这一要求，用 Python 编写了一个名为 TSAnalyzer 的新软件包，用于对连续的 GNSS 位置时间序列进行单独预处理和分析，以及批量处理。该软件可读取不同格式的全球导航卫星系统位置时间序列，以交互方式选取偏移或地震事件的纪元，去除异常值，并估计线性、多项式和谐波信号。它还提供 Lomb-Scargle 频谱分析。由于它基于 Python，因此可以跨平台使用。 

---

## goGPS

[goGPS: open-source MATLAB software by Antonio M. Herrera, Hendy F. Suhandri, Eugenio Realini, Mirko Reguzzoni, and M. Clara de Lacy](https://geodesy.noaa.gov/gps-toolbox/goGPS.htm)

goGPS is a positioning software application designed to process single-frequency code and phase observations for absolute or relative positioning. Published under a free and open-source license, goGPS can process data collected by any receiver, but focuses on the treatment of observations by low-cost receivers. goGPS algorithms can produce epoch-by-epoch solutions by least squares adjustment, or multi-epoch solutions by Kalman filtering, which can be applied to either positions or observations. It is possible to aid the positioning by introducing additional constraints, either on the 3D trajectory such as a railway, or on a surface, e.g., a digital terrain model. goGPS is being developed by a collaboration of different research groups, and it can be downloaded from http://www.gogps-project. org. The version used in this manuscript can be also downloaded from the GPS Toolbox Web site http://www. ngs.noaa.gov/gps-toolbox. This software is continues to evolve, improving its functionalities according to the updates introduced by the collaborators. We describe the main modules of goGPS along with some examples to show the user how the software works.  

goGPS 是一款定位软件应用程序，用于处理单频代码和相位观测数据，以进行绝对或相对定位。goGPS 算法可通过最小二乘法调整生成逐个时序的解决方案，或通过卡尔曼滤波生成多时序的解决方案，这些算法既可应用于位置，也可应用于观测数据。goGPS 由不同的研究小组合作开发，可从 http://www.gogps-project. org 下载。本手稿中使用的版本也可从 GPS 工具箱网站 http://www. ngs.noaa.gov/gps-toolbox 下载。该软件仍在不断发展，根据合作者的更新改进其功能。我们将介绍 goGPS 的主要模块，并通过一些示例向用户展示软件的工作原理。 

---

## 

[An open source GPS multipath simulator in Matlab/Octave by Felipe G. Nievinski and Kristine M. Larson](https://geodesy.noaa.gov/gps-toolbox/MPsimul.htm)

Multipath is detrimental for both GPS positioning and timing applications. However, the benefits of GPS multipath for reflectometry have become increasingly clear for monitoring soil moisture, snow depth, and vegetation growth. In positioning applications, a simulator can support multipath mitigation efforts in terms of, e.g., site selection, antenna design, receiver performance assessment, and in relating different observations to a common parameterization. For reflectometry, in order to convert observed multipath parameters into useable environmental products, it is important to be able to explicitly link the GPS observables to known characteristics of the GPS receiver/antenna and the reflecting environment. Existing GPS multipath software simulators are generally not readily available for the general scientific community to use and/or modify. Here, a simulator has been implemented in Matlab/Octave and is made available as open source code. It can produce signal-to-noise ratio, carrier phase, and code pseudorange observables, based on L1 and L2 carrier frequencies and C/A, P(Y), and L2C modulations. It couples different surface and antenna types with due consideration for polarization and coherence. In addition to offering predefined material types (water, concrete, soil, etc.), it allows certain dimensional properties to be varied, such as soil moisture and snow density.  

多径对 GPS 定位和授时应用都不利。然而，在监测土壤湿度、积雪深度和植被生长方面，GPS 多径对反射测量的好处越来越明显。在定位应用中，模拟器可以在选址、天线设计、接收机性能评估以及将不同的观测结果与共同的参数化联系起来等方面为减少多径影响提供支持。对于反射测量，为了将观测到的多径参数转换为可用的环境产品，必须能够将 GPS 观测数据与 GPS 接收机/天线和反射环境的已知特性明确联系起来。现有的全球定位系统多径软件模拟器一般不方便广大科学界使用和/或修改。在这里，我们用 Matlab/Octave 实现了一个模拟器，并以开放源代码的形式提供。它可以根据 L1 和 L2 载波频率以及 C/A、P(Y) 和 L2C 调制，生成信噪比、载波相位和代码伪距观测值。它可将不同的表面和天线类型耦合在一起，并适当考虑极化和相干性。除了提供预定义的材料类型（水、混凝土、土壤等）外，它还允许改变某些尺寸属性，如土壤湿度和雪密度。 

---

## EPC

[EPC: Matlab software to estimate Euler pole parameters by Mohammad Ali Goudarzi, Marc Cocard, Rock Santerre](https://geodesy.noaa.gov/gps-toolbox/Goudarzi.htm)

The estimation of Euler pole parameters has always been an important issue in global tectonics and geodynamics studies. In addition, the increasing number of permanent GPS stations and the ease of access to their data, along with advances in computers, promise new methods and tools for the estimation and the quantitative analysis of Euler pole parameters. Therefore, we developed the Euler pole calculator software using a set of mathematical algorithms based on the model of tectonic plate motion on a spherical surface. The software is able to calculate the expected velocities for any points located on the earth's surface given the relevant Euler pole parameters and to estimate the Euler pole parameters given the observed velocities of a set of sites located on the same tectonic plate. Mathematical algorithms and functions of the software are explained in detail.  

欧拉极参数的估算一直是全球构造和地球动力学研究中的一个重要问题。此外，随着全球定位系统永久站点数量的增加和数据获取的便捷，以及计算机技术的进步，有望为欧拉极参数的估算和定量分析提供新的方法和工具。因此，我们利用一套基于球面上构造板块运动模型的数学算法开发了欧拉极计算软件。该软件能够根据相关的欧拉极参数计算出地球表面任意点的预期速度，并根据位于同一构造板块上的一组点的观测速度估算出欧拉极参数。详细解释了软件的数学算法和功能。 

---

## M_DCB

[M_DCB: Matlab code for estimating GNSS satellite and receiver differential code biases by Rui Jin, Shuanggen Jin, Guiping Feng](https://geodesy.noaa.gov/gps-toolbox/m_dcb.htm)

The Global Navigation Satellite Systems (GNSS) have been widely used to monitor variations in the earth's ionosphere by estimating total electron content (TEC) using dual-frequency observations. Differential code biases (DCBs) are one of the important error sources in estimating precise TEC from GNSS data. The International GNSS Service (IGS) Analysis Centers have routinely provided DCB estimates for GNSS satellites and IGS ground receivers, but the DCBs for regional- and local-network receivers are not provided. Furthermore, the DCB values of GNSS satellites or receivers are assumed to be constant over one day or one month, which is not always the case. We describe Matlab code to estimate GNSS satellite and receiver DCBs for time intervals from hours to days; the software is called M_DCB. The DCBs of GNSS satellites and ground receivers are tested and evaluated using data from the IGS GNSS network. The estimates from M_DCB show good agreement with the IGS Analysis Centers with a mean difference of less than 0.7 nanoseconds and an RMS of less than 0.4 nanoseconds, even for a single station DCB estimate.  

全球导航卫星系统（GNSS）被广泛用于监测地球电离层的变化，利用双频观测估算电子总含量（TEC）。差分编码偏差（DCB）是利用全球导航卫星系统数据估算精确 TEC 的重要误差源之一。国际全球导航卫星系统服务（IGS）分析中心定期提供全球导航卫星系统卫星和 IGS 地面接收器的 DCB 估计值，但没有提供区域和本地网络接收器的 DCB。此外，GNSS 卫星或接收器的 DCB 值被假定为一天或一个月内恒定不变，但事实并非总是如此。我们介绍了估算从数小时到数天时间间隔内 GNSS 卫星和接收器 DCB 的 Matlab 代码；该软件称为 M_DCB。使用来自 IGS GNSS 网络的数据对 GNSS 卫星和地面接收器的 DCB 进行了测试和评估。M_DCB的估算结果与IGS分析中心的结果一致，平均差小于0.7纳秒，有效值小于0.4纳秒，即使对单个台站的DCB估算结果也是如此。 

---

## Sigseg

[Sigseg: a tool for the detection of position and velocity discontinuities in geodetic time-series by Alfonso Vitti](https://geodesy.noaa.gov/gps-toolbox/sigseg.htm)

The detection of discontinuities in geodetic data is an ever more important topic due to both the influence of discontinuities on the results of models and analyses, and to the very meaning of discontinuities in physical phenomena. In this work, a mathematical model originally formulated for the approximation of images by smooth functions is considered and described in 1D. The model was designed to smooth the data while preserving and detecting its discontinuities following a variational approach. A second and more complex model for the approximation of images by functions with smooth first derivatives is also available. This second model was designed to smooth the data while preserving and detecting the discontinuities of the data, but also those of the first derivative. Such interesting features suggest the chance to apply this second model to 1D geodetic data, in particular for the detection of discontinuities and velocity changes in position time-series rather than for signal smoothing. The sigseg (signal segmentation) program implements the two variational models in one dimension and it is presented herein with applications to some geodetic data. Only essential mathematical elements are sketched in the paper and some details on the numerical implementation are given.  

由于不连续性对模型和分析结果的影响，以及不连续性在物理现象中的意义，大地测量数据中不连续性的检测成为一个日益重要的课题。在这项工作中，考虑并描述了一个最初为用平滑函数逼近图像而制定的一维数学模型。该模型旨在平滑数据，同时按照变分法保留和检测数据的不连续性。此外，还有第二个更复杂的模型，用于用具有平滑一阶导数的函数逼近图像。第二个模型的设计目的是在平滑数据的同时，保留和检测数据的不连续性，以及一阶导数的不连续性。这些有趣的特征表明，有机会将第二个模型应用于一维大地测量数据，特别是用于检测位置时间序列中的不连续性和速度变化，而不是用于信号平滑。sigseg（信号分割）程序在一维范围内实现了这两个变分模型，本文将介绍它在一些大地测量数据中的应用。本文只简要介绍了基本的数学元素，并给出了数值实现的一些细节。 

---

## 

[MATLAB software for GPS cycle-slip processing by Zhen Dai](https://geodesy.noaa.gov/gps-toolbox/ZDcycleslip.htm)

A MATLAB software package for GPS cycle-slip processing is presented in this paper. It realizes cycle-slip detection and repair in the measurement domain for GPS L1 and L2 signals. The software implements several classic approaches oriented to real-time processing. With the graphic user interface, the user can configure the raw data, set algorithm-related parameters, add synthetic cycle-slips, and view the detection results in both text and illustrated forms. In this paper, the theoretical background of the cycle-slip processing is introduced first. After that, the advantages and limitations of each implemented approach are identified. Finally, the functionalities of the software are briefly explained.  

本文介绍了一个用于 GPS 周期滑移处理的 MATLAB 软件包。它实现了 GPS L1 和 L2 信号测量域的周期滑动检测和修复。该软件实现了几种面向实时处理的经典方法。通过图形用户界面，用户可以配置原始数据、设置算法相关参数、添加合成周期滑移，并以文本和图解两种形式查看检测结果。本文首先介绍了周期滑动处理的理论背景。然后，确定了每种实现方法的优势和局限性。最后，简要说明软件的功能。 

---

## 

[An ActiveX control for embedding GPS capability in custom applications by Khalid Amin Khan, Gulraiz Akhter, Zulfiqar Ahmad](https://geodesy.noaa.gov/gps-toolbox/Khan.htm)

An ActiveX GPS control is presented which can be used to develop software applications with GPS functionality. It translates the NMEA 0183 interface GPS instructions and triggers event procedures which are used by applications to access the GPS data. It provides position data in the form of geographic coordinates as well as Universal Transverse Mercator (UTM) projected coordinates. This control is recommended for the development of general purpose GPS-enabled applications which do not require a high level of accuracy. A Visual Basic project is also included to demonstrate the use of various features of this control. Finally, some real-time software applications are discussed which have been developed using this control. These applications include static point averaging; path tracking; and imagery-based position mapping.  

介绍的 ActiveX GPS 控件可用于开发具有 GPS 功能的软件应用程序。它能翻译 NMEA 0183 接口的 GPS 指令，并触发应用程序用来访问 GPS 数据的事件程序。它以地理坐标和通用横轴默卡托（UTM）投影坐标的形式提供位置数据。建议在开发不要求高精度的通用 GPS 应用程序时使用该控件。此外还包括一个 Visual Basic 项目，以演示如何使用该控件的各种功能。最后，还讨论了使用该控件开发的一些实时软件应用程序。这些应用包括静态点平均、路径跟踪和基于图像的位置制图。 



---

## SATLSim：用于快速 GNSS 跟踪环路模拟的半分析框架

[SATLSim: a Semi-Analytic framework for fast GNSS tracking loop simulations by Daniele Borio, Pratibha B. Anantharamu and Gerard Lachapelle](https://geodesy.noaa.gov/gps-toolbox/SATLSim.htm)

The analysis of tracking loops for global navigation satellite systems (GNSS) receivers is often confined to Monte Carlo approaches that can result in long simulation times and a limited number of simulation runs. A different approach based on Semi-Analytic principles is considered here. Matlab� code implementing a Semi-Analytic framework for the fast simulation of GNSS digital tracking loops is presented. The code structure is detailed and two specific examples implementing a standard PLL and the Double Estimator for unambiguous binary offset carrier (BOC) tracking are provided. The code has been organized in a modular way, and can be easily modified for the simulation of different tracking loops.  

对全球导航卫星系统（GNSS）接收机跟踪环路的分析通常局限于蒙特卡罗方法，这可能导致模拟时间过长和模拟运行次数有限。本文考虑了一种基于半解析原理的不同方法。本文介绍了用于快速模拟 GNSS 数字跟踪环路的半分析框架的 Matlab 代码。详细介绍了代码结构，并提供了实现标准 PLL 和用于无差别二进制偏移载波（BOC）跟踪的双重估计器的两个具体示例。代码以模块化方式组织，可轻松修改以模拟不同的跟踪环路。 

---

## RINEX_HO：RINEX 的二阶和三阶电离层改正

[RINEX_HO: second- and third-order ionospheric corrections for RINEX observation files by H. A. Marques, J. F. G. Monico and M. Aquino](https://geodesy.noaa.gov/gps-toolbox/RINEX_HO.htm)

When GNSS receivers capable of collecting dual-frequency data are available, it is possible to eliminate the first-order ionospheric effect in the data processing through the ionosphere-free linear combination. However, the second- and third-order ionospheric effects still remain. The first-, second- and third-order ionospheric effects are directly proportional to the total electron content (TEC), although the second- and third-order effects are influenced, respectively, by the geomagnetic field and the maximum electron density. In recent years, the international scientific community has given more attention to these kinds of effects and some works have shown that for high precision GNSS positioning these effects have to be taken into consideration. We present a software tool called RINEX_HO that was developed to correct GPS observables for second- and third-order ionosphere effects. RINEX_HO requires as input a RINEX observation file, then computes the second- and third-order ionospheric effects, and applies the corrections to the original GPS observables, creating a corrected RINEX file. The mathematical models implemented to compute these effects are presented, as well as the transformations involving the earth's magnetic field. The use of TEC from global ionospheric maps and TEC calculated from raw pseudorange measurements or pseudoranges smoothed by phase is also investigated.  

如果有能够收集双频数据的全球导航卫星系统接收器，就有可能通过无电离层线性组合消除数据处理中的一阶电离层效应。但是，二阶和三阶电离层效应仍然存在。一阶、二阶和三阶电离层效应与电子总含量（TEC）成正比，但二阶和三阶效应分别受到地磁场和最大电子密度的影响。近年来，国际科学界对这类效应给予了更多关注，一些研究表明，高精度 GNSS 定位必须考虑这些效应。我们介绍一个名为 RINEX_HO 的软件工具，它是为改正 GPS 观测数据的二阶和三阶电离层效应而开发的。RINEX_HO 要求输入 RINEX 观测文件，然后计算二阶和三阶电离层效应，并将改正结果应用于原始 GPS 观测数据，生成改正后的 RINEX 文件。文中介绍了计算这些效应的数学模型，以及涉及地球磁场的变换。还研究了如何使用全球电离层图中的 TEC 和根据原始伪距测量或按相位平滑的伪距计算的 TEC。 

---

## iGPS：GPS 定位时间序列分析的 IDL 工具包

[iGPS: IDL tool package for GPS position time series analysis by Yunfeng Tian](https://geodesy.noaa.gov/gps-toolbox/Tian.htm)

A new tool package written in Interactive Data Language (IDL) was developed for processing and analyzing daily continuous GPS position time series. This software package can read continuous GPS position time series in various formats, detect outliers, remove abnormal observation spans, locate epochs of offsets or post-seismic relaxation events, and estimate their amplitudes. It also provides functionalities for epoch statistics, site selection, periodic noise analysis, spatial filtering (by regional stacking), etc. This tool is referred to as iGPS.  

开发了一个用交互式数据语言（IDL）编写的新工具包，用于处理和分析每日连续的全球定位系统位置时间序列。该软件包可以读取各种格式的连续 GPS 定位时间序列，检测异常值，去除异常观测跨度，定位偏移或震后松弛事件的历元，并估算其振幅。它还提供了历元统计、站点选择、周期性噪声分析、空间过滤（通过区域堆叠）等功能。该工具被称为 iGPS。 

---

## 利用 GPSTk 管理和处理 GNSS 数据

[GNSS data management and processing with the GPSTk by Dagoberto Salazar, Manuel Hernandez-Pajares, Jose M. Juan, and Jaume Sanz](https://geodesy.noaa.gov/gps-toolbox/gds.htm)

We organize complex problems in simple ways using a GNSS data management strategy based on "GNSS Data Structures" (GDS), coupled with the open source "GPS Toolkit" (GPSTk) suite. The code resulting from using the GDS and their associated "processing paradigm" is remarkably compact and easy to follow, yielding better code maintainability. Furthermore, the data abstraction allows flexible handling of concepts beyond mere data encapsulation, including programmable general solvers. An existing GPSTk class can be modified to achieve the goal. We briefly describe the "GDS paradigm" and show how the different GNSS data processing "objects" may be combined in a flexible way to develop data processing strategies such as Precise Point Positioning (PPP) and network-based PPP that computes satellite clock offsets on-the-fly.  

我们使用基于 "全球导航卫星系统数据结构"（GDS）的全球导航卫星系统数据管理策略，结合开源的 "全球定位系统工具包"（GPSTk）套件，以简单的方式组织复杂的问题。使用 GDS 及其相关 "处理范例 "产生的代码非常紧凑，易于遵循，从而提高了代码的可维护性。此外，数据抽象化还允许灵活处理数据封装以外的概念，包括可编程的通用求解器。现有的 GPSTk 类可以通过修改来实现这一目标。我们简要介绍了 "GDS 范式"，并展示了如何以灵活的方式将不同的 GNSS 数据处理 "对象 "结合起来，以开发数据处理策略，如精密单点定位（PPP）和基于网络的 PPP（实时计算卫星时钟偏移）。 

---

## MAAST：MATLAB 算法可用性模拟工具

[MATLAB Algorithm Availability Simulation Tool (MAAST) by Shau-Shiun Jan, Wyant Chan, and Todd Walter](https://geodesy.noaa.gov/gps-toolbox/maast.htm)

This paper describes a set of MATLAB functions currently being developed for Space Based Augmentation System (SBAS) availability analysis. This toolset includes simulation algorithms that are constantly being developed and updated by various working groups. This set of functions is intended for use as a fast, accurate, and highly customizable experimental test bed for algorithm development. A user-friendly interface has also been developed for the tool. It is open source and can be downloaded from the Stanford GPS Research Laboratory web site (http://waas.stanford.edu/~wwu/maast/maast.html). Therefore, it provides a common ground for different working groups to compare their results. This paper demonstrates the utility of this toolset by analyzing the SBAS service volume models for the Conterminous United States (CONUS). The paper describes the functionality provided within the software, and shows an example set of contour plots which are the means for investigating how different algorithms and parameters impact availability.  

本文介绍了目前正在开发的一套用于天基增强系统（SBAS）可用性分析的 MATLAB 函数。该工具集包括各工作组不断开发和更新的模拟算法。这套功能旨在用作算法开发的快速、准确和高度可定制的实验测试平台。此外，还为该工具开发了用户友好界面。它是开源的，可从斯坦福全球定位系统研究实验室网站（http://waas.stanford.edu/~wwu/maast/maast.html）下载。因此，它为不同的工作小组提供了一个比较其结果的共同基础。本文通过分析美国大陆（CONUS）的 SBAS 服务量模型，展示了该工具集的实用性。本文介绍了该软件提供的功能，并展示了一组等值线图示例，这些图是研究不同算法和参数如何影响可用性的手段。 



---

## GPS 多天线定姿

[A MATLAB toolbox for attitude determination with GPS multi-antenna systems by Zhen Dai, S. Knedlik, and O. Loffeld](https://geodesy.noaa.gov/gps-toolbox/zdai.htm)

In this paper a MATLAB toolbox for determining the attitude of a rigid platform by means of multiple non-dedicated antennas using global positioning system is presented. The programs embedded in this toolbox cover the RINEX data analysis, single point positioning, differential positioning, coordinate conversion, attitude determination, and other auxiliary functions. After forming the baselines through double-differenced (carrier phase smoothed) code observables, the attitude parameters are obtained by applying the direct attitude computation and the least squares attitude estimation. The theoretical background is summarized, and some hints regarding the software implementation are given in the paper. Moreover, improvements yielding an expanded functionality are proposed.  

本文介绍了一个 MATLAB 工具箱，用于利用全球定位系统通过多个非专用天线确定刚性平台的姿态。该工具箱内嵌的程序涵盖了 RINEX 数据分析、单点定位、差分定位、坐标转换、姿态确定和其他辅助功能。通过双差分（载波相位平滑化）代码观测值形成基线后，通过直接姿态计算和最小二乘法姿态估计获得姿态参数。本文对理论背景进行了总结，并给出了一些有关软件实现的提示。此外，还提出了一些改进措施，以扩大功能。 

---

## CATS：GPS 坐标时间序列分析

[CATS: GPS coordinate time series analysis software by Simon D. P. Williams](https://geodesy.noaa.gov/gps-toolbox/cats.htm)

Over the last 10 years, several papers have established that daily estimates of GPS coordinates are temporally correlated and it is therefore incorrect to assume that the observations are independent when estimating parameters from them. A direct consequence of this assumption is the over-optimistic estimation of the parameter uncertainties. Perhaps the perceived computational burden or the lack of suitable software for time series analysis has resulted in many heuristic methods being proposed in the scientific literature for estimating these uncertainties. We present a standalone C program, CATS, developed to study and compare stochastic noise processes in continuous GPS coordinate time series and, as a consequence, assign realistic uncertainties to parameters derived from them. The name originally stood for Create and Analyse Time Series. Although the name has survived, the creation aspect of the software has, after several versions, been abandoned. The implementation of the method is briefly described to aid understanding and an example of typical input, usage, output and the available stochastic noise models are given.  

在过去的 10 年中，有几篇论文已经证实，全球定位系统坐标的每日估算值在时间上是相关的，因此，在估算参数时假设观测值是独立的是不正确的。这种假设的直接后果是对参数不确定性的估计过于乐观。也许是考虑到计算负担或缺乏合适的时间序列分析软件，科学文献中提出了许多启发式方法来估计这些不确定性。我们介绍一个独立的 C 语言程序 CATS，它是为研究和比较连续 GPS 坐标时间序列中的随机噪声过程而开发的，因此可以为从中得出的参数分配现实的不确定性。CATS 最初的名称是 Create and Analyse Time Series（创建和分析时间序列）。虽然这个名称一直沿用至今，但该软件的创建功能在经过几个版本后已被放弃。本文简要介绍了该方法的实现过程，以帮助理解，并给出了典型输入、使用、输出和可用随机噪声模型的示例。 



---

## UNB3m_pack：空间辐射测量技术的中性大气延迟

[UNB3m_pack: a neutral atmosphere delay package for radiometric space techniques by Rodrigo F. Leandro, Richard B. Langley, and Marcelo C. Santos](https://geodesy.noaa.gov/gps-toolbox/unb3m.htm)

Several hybrid neutral atmosphere delay models have been developed at the University of New Brunswick. In this paper we are presenting UNB3m_pack, a package with subroutines in FORTRAN and corresponding functions in MatLab which provides neutral atmosphere information estimated using the UNB3m model. The main goal of UNB3m is to provide reliable predicted neutral atmosphere delays for users of global navigation satellite systems (GNSS) and other transatmospheric radiometric techniques. Slant neutral atmosphere delays are the main output of the package, however, it can be used to estimate zenith delays, Niell mapping functions values, delay rates, mapping function rates, station pressure, temperature and relative humidity and the mean temperature of water vapour in the atmospheric column. The subroutines work using day of year, latitude, height and elevation angle as input values. The files of the package have a commented section at the beginning, explaining how the subroutines work and what the input and output parameters are. The subroutines are self contained; i.e., they do not need any auxiliary files. The user has simply to add to his/her software one or more of the available files and call them in the appropriate way.  

新不伦瑞克大学开发了几种混合中性大气延迟模型。本文将介绍 UNB3m_pack，这是一个包含 FORTRAN 子程序和 MatLab 中相应函数的软件包，提供使用 UNB3m 模型估算的中性大气信息。UNB3m 的主要目标是为全球导航卫星系统 (GNSS) 和其他跨大气层辐射测量技术的用户提供可靠的中性大气层延迟预测。斜中性大气延迟是该软件包的主要输出，但也可用于估算天顶延迟、尼尔测绘函数值、延迟率、测绘函数率、站点压力、温度和相对湿度以及大气柱中水蒸气的平均温度。子程序使用年月日、纬度、高度和仰角作为输入值。软件包的文件开头有注释部分，解释子程序如何工作以及输入和输出参数是什么。子程序是自包含的，即不需要任何辅助文件。用户只需在自己的软件中添加一个或多个可用文件，并以适当的方式调用即可。 

---

## GPS N 点问题的计算机代数解决方案

[Computer algebra solution of the GPS N-points problem by Bela Palancz, Joseph L. Awange, and Erik W. Grafarend](https://geodesy.noaa.gov/gps-toolbox/Palancz.htm)

A computer algebra solution is applied here to develop and evaluate algorithms for solving the basic GPS navigation problem: finding a point position using four or more pseudoranges at one epoch (the GPS N-points problem). Using Mathematica 5.2 software, the GPS N-points problem is solved numerically, symbolically, semi-symbolically, and with Gauss-Jacobi, on a work station. For the case of N > 4, two minimization approaches based on residuals and distance norms are evaluated for the direct numerical solution and their computational duration is compared. For N = 4, it is demonstrated that the symbolic computation is twice as fast as the iterative direct numerical method. For N = 6, the direct numerical solution is twice as fast as the semi-symbolic, with the residual minimization requiring less computation time compared to the minimization of the distance norm. Gauss-Jacobi requires eight times more computation time than the direct numerical solution. It does, however, have the advantage of diagnosing poor satellite geometry and outliers. Besides offering a complete evaluation of these algorithms, we have developed Mathematica 5.2 code (a notebook file) for these algorithms (i.e., Sturmfel's resultant, Dixon's resultants, Groebner basis, reduced Groebner basis and Gauss-Jacobi). These are accessible to any geodesist, geophysicist, or geoinformation scientist via the GPS Toolbox website or the Wolfram Information Center (http://library.wolfram.com/infocenter/MathSource/6629/).  

这里应用计算机代数解决方案来开发和评估解决基本 GPS 导航问题的算法：在一个历元上使用四个或更多伪距寻找一个点的位置（GPS N 点问题）。使用 Mathematica 5.2 软件，在工作站上对 GPS N 点问题进行了数值、符号、半符号和高斯-雅可比求解。对于 N > 4 的情况，评估了直接数值求解的两种基于残差和距离规范的最小化方法，并比较了它们的计算时间。对于 N = 4 的情况，符号计算的速度是直接数值迭代法的两倍。对于 N = 6，直接数值解的速度是半符号解的两倍，残差最小化所需的计算时间少于距离规范最小化。高斯-雅可比的计算时间是直接数值解法的八倍。不过，它在诊断不良卫星几何形状和异常值方面具有优势。除了提供这些算法的完整评估外，我们还为这些算法（即 Sturmfel 结果、Dixon 结果、Groebner 基础、缩小的 Groebner 基础和高斯-雅各比）开发了 Mathematica 5.2 代码（笔记本文件）。任何大地测量学家、地球物理学家或地理信息科学家都可通过 GPS 工具箱网站或 Wolfram 信息中心 (http://library.wolfram.com/infocenter/MathSource/6629/) 查阅这些资料。 

---

## MILES：用于解决混合整数最小二乘法问题的 MATLAB 软件包

[MILES: MATLAB package for solving Mixed Integer LEast Squares problems by Xiao-Wen Chang and Tianyang Zhou](https://geodesy.noaa.gov/gps-toolbox/Miles.htm)

In GNSS, for fixing integer ambiguities and estimating positions, a mixed integer least squares problem has to be solved. The Matlab package MILES provides fast and numerically reliable routines to solve this problem. In the process of solving a mixed integer least squares problem, an ordinary integer least squares problem is solved. Thus this package can also be used to solve an ordinary integer least squares problem alone. An option to compute multiple solutions is provided. This paper gives a description of this package and provides a guide for using it.  

在全球导航卫星系统中，为了改正整数模糊性和估计位置，必须解决混合整数最小二乘法问题。Matlab 软件包 MILES 为解决这一问题提供了快速、数值可靠的例程。在求解混合整数最小二乘问题的过程中，普通整数最小二乘问题也会被求解。因此，该软件包也可用于单独求解普通整数最小二乘法问题。本软件包还提供了计算多解的选项。本文介绍了这个软件包，并提供了使用指南。 

---

## MATLAB 中的 TEQC 多路径指标

[TEQC multipath metrics in MATLAB by Clement Ogaja and Jim Hedfors](https://geodesy.noaa.gov/gps-toolbox/Ogaja.htm)

Knowledge of the local direction of multipath at a particular site is important for a number of reasons. For example, such information can be used to study site selections or during monument design for GPS installations. We present a MATLAB program for creating colorized maps of high frequency multipath using TEQC report files of single-epoch data. The maps, although not necessarily indicating the actual local direction of multipath on the ground, give the orientation with respect to the geometry of the satellites in the sky. This information can aid the interpretation of ground multipath geometry at the site. We give an example of short-span data with ~0.05 Hz multipath (i.e. repeat period ~ 20 s) although the program can be modified for long-term measurements as well.  

出于多种原因，了解特定地点的多径局部方向非常重要。例如，此类信息可用于研究站点选择或 GPS 安装的纪念碑设计。我们介绍了一种 MATLAB 程序，用于利用 TEQC 报告文件中的单波段数据绘制彩色高频多径图。这些地图虽然不一定能显示地面多径的实际局部方向，但能给出卫星在天空中的几何方向。这一信息有助于解释站点的地面多径几何。我们以 ~0.05 Hz 多径（即重复周期 ~20 秒）的短跨度数据为例进行说明，尽管该程序也可修改用于长期测量。 

---

## GPSTK：开源 GPS 工具箱

[The GPSTk: an open source GPS toolkit by R. Benjamin Harris and Richard G. Mach](https://geodesy.noaa.gov/gps-toolbox/Mach.htm)

Abstract: The Applied Research Laboratories, The University of Texas at Austin (ARL:UT) has established a cross platform open source software project called the GPSTk or the GPS Toolkit. The GPSTk consists of a library and collection of applications that support GPS research, analysis, and development. The code is released under the terms of the Lesser GNU Public License. The GPSTk supports a broad range of functionality. This includes reading and writing observations in standard formats, such as RINEX, BINEX, and SP3, ephemeris evaluation, position determination, receiver autonomous integrity monitoring (RAIM), atmospheric delay modeling, cycle slip detection and correction, and P-code generation. The GPSTk provides the core set of functionality that is used for GPS research and development at ARL:UT. ARL:UT has been involved with satellite navigation since Transit (the precursor to GPS) in the 1960s and is currently conducting research in a wide variety of GPS-related fields, including precise surveys, monitor station networks, and ionospheric studies. The GPSTk is a community wide resource for all users of GPS and GNSS technology. Participation is welcomed in all areas including: bug reports, new algorithms, suggestions for improvement, and contributions of additional functionality or applications. ARL:UT continually improves the library, shepards community participation, and is comitted to the project's development and maintenance.  

摘要：德克萨斯大学奥斯汀分校应用研究实验室（ARL:UT）建立了一个跨平台开源软件项目，称为 GPSTk 或 GPS 工具包。GPSTk 由支持 GPS 研究、分析和开发的程序库和应用程序集合组成。其代码根据小 GNU 公共许可证的条款发布。GPSTk 支持广泛的功能。其中包括读写 RINEX、BINEX 和 SP3 等标准格式的观测数据、星历评估、位置确定、接收机自主完整性监测（RAIM）、大气延迟建模、周期滑移检测和改正以及 P 代码生成。GPSTk 为 ARL:UT 的 GPS 研发提供了一套核心功能。自20世纪60年代Transit（GPS的前身）问世以来，ARL:UT一直从事卫星导航工作，目前正在开展与GPS相关的各种领域的研究，包括精确测量、监测站网络和电离层研究。GPSTk 是面向全球定位系统和全球导航卫星系统技术所有用户的社区资源。欢迎各方面的参与，包括：错误报告、新算法、改进建议以及附加功能或应用程序的贡献。ARL:UT 不断改进该库，鼓励社区参与，并致力于项目的开发和维护。 

---

## 查找 GPS 星座的重复时间

[Finding the repeat times of the GPS constellation by Duncan Carr Agnew and Kristine M. Larson](https://geodesy.noaa.gov/gps-toolbox/Larson.htm)

Abstract: Single-epoch estimates of position using GPS are improved by removing multipath signals, which repeat when the satellite constellation does. We present two programs for finding this repeat time, one using the orbital period and the other the topocentric positions of the satellites. Both methods show that the repeat time is variable across the constellation, at the few-second level for most satellites, but with a few showing much different values. The repeat time for topocentric positions, which we term the aspect repeat time, averages 247 s less than a day, with fluctuations through the day that may be as much as 2.5 s at high latitudes.  

摘要：通过消除多径信号，利用全球定位系统对位置的单次估计会得到改善，因为多径信号会在卫星星座出现重复时出现。我们提出了两个计算重复时间的程序，一个使用轨道周期，另一个使用卫星的地心位置。这两种方法都表明，整个卫星群的重复时间是可变的，大多数卫星的重复时间在几秒的水平，但也有少数卫星的重复时间数值相差很大。地心位置的重复时间，也就是我们所说的方面重复时间，平均为 247 秒，不足一天，在高纬度地区，一天中的波动可能高达 2.5 秒。 

Note: There are two Fortran programs available here: orbrep.f and asprep.f. The program orbrep.f (orbit repeat times) reads in a broadcast ephemeris file in RINEX format and writes out the repeat times relative to 86400 seconds for all satellites; if multiple ephemerides are given in the file then the values for the square root of the semi-major axis, and the correction to the mean motion, are averaged for each satellite. The computed orbit repeat times are twice the orbital period for each satellite.  

The asprep.f program (aspect repeat times) reads in a set of SP3 files, an initial time (ymdhms), an integer "N" number of days, and a location (latitude, longitude, ellipsoid height) and writes out the aspect repeat times of all satellites at "N" days from the initial time. The asprep.f program also outputs: the angular separation between the satellite position at the initial time and at the closest repeat position; the elevation and azimuth of the satellite; the angular velocity of the satellite; and the difference in the angle between the initial position and repeat position given in terms of time -- which is useful for deciding how close the actual repetition is. The files README.ORB and README.ASP show example command-line arguments for both programs, and the resultant screen output.  

注：此处有两个 Fortran 程序：orbrep.f 和 asprep.f。orbrep.f（轨道重复时间）程序读入 RINEX 格式的广播星历表文件，并写出所有卫星相对于 86400 秒的重复时间；如果文件中给出了多个星历表，则每颗卫星的半长轴平方根值和平均运动改正值取平均值。计算出的轨道重复时间是每颗卫星轨道周期的两倍。

asprep.f程序（方位重复次数）读入一组SP3文件、初始时间（ymdhms）、整数 "N "天数和位置（纬度、经度、椭球体高度），并写出从初始时间起 "N "天内所有卫星的方位重复次数。asprep.f 程序还会输出：初始时间卫星位置与最近重复位置之间的角间距；卫星的仰角和方位角；卫星的角速度；以及以时间表示的初始位置与重复位置之间的角度差--这对判断实际重复的接近程度非常有用。README.ORB 和 README.ASP 文件显示了这两个程序的命令行参数示例以及屏幕输出结果。 

---

## 用于 GNSS 软件接收器的 SIMD 相关器库

[SIMD correlator library for GNSS software receivers by Gregory W. Heckler and James L. Garrison](https://geodesy.noaa.gov/gps-toolbox/Heckler.htm)

 Software receivers have had a discernable impact on the GNSS research community. Often such receivers are implemented in a compiled programming language, such as C or C++. A software receiver must emulate the digital signal processing (DSP) algorithms executed on dedicated hardware in a traditional receiver. The DSP algorithms, most notably correlation, have a high computational cost; this burden precludes many software receivers from running in real time. However the computational cost can be lessened by utilizing single instruction multiple data (SIMD) operations found on modern x86 processors. The following demonstrates how C/C++ compatible code can be written to directly utilize the SIMD instructions. First, an analysis is carried out to demonstrate why real time operation is not possible when using traditional C/C++ code. Second, a tutorial outlines how to write and insert x86 assembly, with SIMD operations, into C/C++ code. Performance gains achieved via SIMD operations are then demonstrated, and pseudo code outlines how SIMD operations can be employed to perform correlation. Finally, a C/C++ compatible SIMD enabled arithmetic library is added to the GPS Toolbox for use in software receivers.  

 软件接收器对全球导航卫星系统研究界产生了明显的影响。这类接收器通常用 C 或 C++ 等编译编程语言实现。软件接收器必须模拟传统接收器中在专用硬件上执行的数字信号处理（DSP）算法。数字信号处理算法，尤其是相关算法，计算成本很高；这种负担使许多软件接收器无法实时运行。不过，利用现代 x86 处理器上的单指令多数据（SIMD）运算可以降低计算成本。下面将演示如何编写 C/C++ 兼容代码来直接利用 SIMD 指令。首先，将进行分析，说明为什么使用传统的 C/C++ 代码无法实现实时操作。其次，教程概述了如何编写 x86 汇编并将 SIMD 操作插入 C/C++ 代码。然后，演示了通过 SIMD 操作实现的性能提升，并用伪代码概述了如何使用 SIMD 操作来执行相关操作。最后，一个与 C/C++ 兼容的 SIMD 运算库被添加到 GPS 工具箱中，供软件接收器使用。 

---

## GPS 卫星坐标的多项式内插法

[Polynomial interpolation of GPS satellite coordinates by Milan Horemuz and Johan Vium Andersson](https://geodesy.noaa.gov/gps-toolbox/Vium.htm)

Abstract: This article describes an algorithm for polynomial interpolation of GPS satellite coordinates and its implementation in MATLAB. The algorithm is intended for realtime processing software and computes the position and velocity of GPS satellites from both broadcast and precise ephemerides. Tests with different orders of polynomials, and with different time spans used for polynomial fitting, show suitable settings with respect to the required interploation precision.  

摘要：本文介绍了 GPS 卫星坐标多项式插值算法及其在 MATLAB 中的实现。该算法用于实时处理软件，可根据广播星历表和精确星历表计算 GPS 卫星的位置和速度。使用不同阶数的多项式和用于多项式拟合的不同时间跨度进行的测试表明，对所要求的插值精度进行了适当的设置。 

Note: There are numerous MATLAB m-files included in this software package. Thus, the the authors have bundled all files and sample data in a *.zip file (KTHorb.zip). The Readme.txt file describes the directories created when the .zip file is unpacked. The file Matlab_implementation.doc (or Matlab_implementation.txt) describes some of the classes and methods included in the package for GPS satellite orbit interpolation.  

注：本软件包包含大量 MATLAB m 文件。因此，作者将所有文件和样本数据打包成一个 *.zip 文件 (KTHorb.zip)。Readme.txt 文件描述了解压 .zip 文件时创建的目录。文件 Matlab_implementation.doc（或 Matlab_implementation.txt）介绍了软件包中包含的用于 GPS 卫星轨道内插的一些类和方法。 

---

## SiGOG：GPS 观测值模拟

[SiGOG: Simulated GPS Observation Generator by Elsa Mohino, Mauricio Gende, Claudio Brunini, Miguel Heraiz](https://geodesy.noaa.gov/gps-toolbox/Mohino.htm)

Abstract: For many applications, access to unbiased or error-controlled Global Positioning System (GPS) observations can be very useful. This paper is devoted to the description of the simulated GPS observation generator (SiGOG), a software that simulates GPS observations. It presents the results of tests of SiGOG accuracy using GPS processing software, and demonstrates its successful performance as a differential GPS (DGPS) correction provider.  

摘要：对于许多应用来说，获得无偏或误差可控的全球定位系统（GPS）观测数据非常有用。本文主要介绍模拟 GPS 观测数据生成器 (SiGOG)，这是一种模拟 GPS 观测数据的软件。它介绍了使用 GPS 处理软件对 SiGOG 精确度进行测试的结果，并展示了 SiGOG 作为差分 GPS（DGPS）改正提供商的成功性能。 

Note: There are two Fortran 77 programs available here: SIGOGbcst.for, which creates simulated observations output to a RINEX observation file using a GPS broadcast ephemeris and user-defined site coordinates and antenna offsets; and SIGOGprcs.for, which does the same but uses three sequential precise ephemerides in SP3-a format for the input orbit data.  

注：这里有两个 Fortran 77 程序： SIGOGbcst.for，使用全球定位系统广播星历和用户定义的站点坐标和天线偏移，创建输出到 RINEX 观测文件的模拟观测数据；以及 SIGOGprcs.for，使用 SP3-a 格式的三个连续精确星历作为输入轨道数据。 

---

## 大地测量中递推公式的 C++ 和 Java 代码

[C++ and Java Code for recursion formulas in mathematical geodesy by Klaus Hehl](https://geodesy.noaa.gov/gps-toolbox/Hehl1.htm)

The central part of this paper is the publication of C++ and Java code for the solution of a number of basic tasks related to the geodesic/meridian arc in mathematical geodesy. The author provides an introduction to a recursive formulation of the series expansions of the underlying integrals. Recursive formulation has the advantage that simple relationships are identical for all orders and thus can easily be programmed. The presented C++ code examples, together with the discussions of the algorithms used, can be easily transferred to other programming languages.  

本文的核心部分是发布 C++ 和 Java 代码，用于解决数学大地测量学中与大地/经线弧相关的一些基本任务。作者介绍了底层积分数列展开的递推公式。递推公式的优势在于所有阶次的简单关系都是相同的，因此很容易编程。所介绍的 C++ 代码示例，以及对所用算法的讨论，可以很容易地移植到其他编程语言中。 

---

## 利用广播星历计算卫星速度

[Computing Satellite Velocity using the Broadcast Ephemeris by Benjamin W. Remondi](https://geodesy.noaa.gov/gps-toolbox/bc_velo.htm)

---

## 根据方位角和仰角绘制伪距多路径图

[Plotting Pseudorange Multipath with Respect to Azimuth and Elevation by Stephen Hilla](https://geodesy.noaa.gov/gps-toolbox/cf2sky.htm)

---

## 访问 GPS 无缝档案

[Accessing the GPS Seamless Archive by Michael Scharber, Yehuda Bock, and Brent Gilmore](https://geodesy.noaa.gov/gps-toolbox/Scharber1.htm)

---

## 绘制速度的 Matlab 工具

[Matlab tools for plotting Velocities by Thomas Herring](https://geodesy.noaa.gov/gps-toolbox/Herring1.htm)

---

## Sharc and Schedg

[Sharc and Schedg by Keith Stark](https://geodesy.noaa.gov/gps-toolbox/Stark.htm)

---

## Easy Suite：自动获取 GPS 数据的 Linux 实用程序

[The Easy Suite (MATLAB for GPS) by Kai Borre](https://geodesy.noaa.gov/gps-toolbox/Borre2.htm)

---

## GPS 轨道插值算法评价

[A Brief Review of Basic GPS Orbit Interpolation Strategies by Mark S. Schenewerk](https://geodesy.noaa.gov/gps-toolbox/sp3intrp.htm)

---

## 基于 Windows 的 TEQC 新绘图程序

[A New Plotting Program for Windows-based TEQC Users by Stephen Hilla](https://geodesy.noaa.gov/gps-toolbox/cf2ps.htm)

---

## 绘制天空视图

[Creating and Viewing Skyplots by John Marshall](https://geodesy.noaa.gov/gps-toolbox/skyplot.htm)

---

## GPS 点位计算

[GPS Point Position Calculation by Sam Storm van Leeuwen](https://geodesy.noaa.gov/gps-toolbox/Leeuwen.htm)

---

## GPS 伪距代数解

[Algebraic solution of GPS pseudo-ranging equations by Joseph L. Awange & Erik W. Grafarend](https://geodesy.noaa.gov/gps-toolbox/awange.htm)

---

## ITRF 框架转换

[ITRF Transformations by Jan Kouba](https://geodesy.noaa.gov/gps-toolbox/trnfsp3.htm)

---

## 读写 RINEX 文件的 C++ 类

[C++ Classes for Reading and Writing RINEX files by Stephen Hilla & Gordon Adams](https://geodesy.noaa.gov/gps-toolbox/rinex.htm)

---

## 克罗布歇电离层模型

[Klobuchar Ionospheric Model subroutines by Ola Ovstedal](https://geodesy.noaa.gov/gps-toolbox/ovstedal.htm)

---

## GPS Matlab 工具

[GPS MATLAB Tools at Aalborg University by Kai Borre](https://geodesy.noaa.gov/gps-toolbox/Borre.htm)

---

## 模糊度降相关

[Ambiguity Decorrelation algorithm by Shaowei Han](https://geodesy.noaa.gov/gps-toolbox/han-04.htm)

---

## 读取 SP3 星历文件的 Perl 脚本

[Perl Script for reading SP3 precise ephemerides by Doug Hunt (a *.tar.gz file)](https://geodesy.noaa.gov/gps-toolbox/SP3-0.04.tar.gz)

---

## 时间和日期转换

[Date/Time conversion algorithms by Benjamin W. Remondi](https://geodesy.noaa.gov/gps-toolbox/bwr-02.htm)

---

## 最小生成树

[Minimal Spanning Tree algorithm by V. Kevin .M. Whitney](https://geodesy.noaa.gov/gps-toolbox/span-01.htm)
