
GNSS-SDR嵌入式IP核

在处理GNSS-SDR实时信号的领域中，需要多核CPU和通过USB或以太网连接的高带宽SDR前端。通常涉及使用商用现成（COTS）计算机，电功耗在30-60瓦范围内（例如，Intel的NUC运行GNSS-SDR单频，16通道，8 MSps）。嵌入式微型计算机平台，如流行的Raspberry PI4，在满负荷下消耗约9瓦，可用于实时运行GNSS-SDR，但可用的计算能力严重限制了通道数（即卫星数）和采样频率，使软件在实时模式下几乎无法使用。

将GNSS-SDR嵌入轻量级、低功耗设备的合适实现替代方案是使用System-on-Chip（SoC）模块，如AMD/Xilinx Zynq SoC家族的模块，将GNSS信号处理链分为两个不同的部分，通常称为FPGA卸载：

1. 高速、低复杂度但要求极高的操作，如FFT和相关性，是在SoC FPGA结构中实现的。
2. 低速但复杂度高的操作，如跟踪控制环、导航消息的解码、可测量量的形成和PVT求解器，是在SoC ARM处理器中实现的。

嵌入式接收机保留了纯SDR版本GNSS-SDR的几乎所有灵活性，如检查任何内部信号、定制获取或跟踪环参数，甚至完全定制实施的算法，但整个过程通过我们的FPGA IP核加速。

下面的图片展示了使用G-ACQ-ST和G-TRK-ST IP核的完整GNSS SDR嵌入接收机FPGA实例：

![Rx1](https://navposproducts.cttc.es/assets/images/GNSS_Receiver.jpeg)

![Rx2](https://navposproducts.cttc.es/assets/images/GNSS_Receiver2.jpeg)

将GNSS-SDR嵌入ADRV936x System on Module等SoM设备，将功耗降低到5-6瓦，并支持实时、双频多星座GNSS处理，包括：

- 6个GPS L1 CA通道，每个12.5 MSps
- 6个伽利略E1通道，每个12.5 MSps
- 6个GPS L5通道，每个12.5 MSps
- 6个伽利略E5a通道，每个12.5 MSps

用户可以实例化其他自定义实现或配置。在SoC处理系统（PS）中执行的软件是完全开源的，并已在GNSS-SDR上游存储库中提供。

为了在AMD/Xilinx Zynq、Zynq Ultrascale和Zynq Ultrascale+系列中执行经过FPGA加速的嵌入式GNSS-SDR，CTTC开发了一组商用IP核，以启动客户创建GNSS-SDR驱动的嵌入式产品。

GNSS-SDR FPGA IP组合目前提供两个主要IP核：

1. G-ACQ-ST实现了一种并行码相搜索/串行频率搜索的GNSS采集加速器算法，具有标准灵敏度（35 dB-Hz）。
2. G-TRK-ST实现了用于跟踪任何当前GNSS信号的多相关器和载波抹除加速器算法。

有关每个IP核的更多信息，请参见下文。

G-ACQ-ST: GNSS-SDR标准采集IP核

![G-ACQ-ST](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/G-ACQ-ST.png)

G-ACQ-ST FPGA IP核实现了一种并行码相搜索（PCPS）算法，用于检测来自给定GNSS卫星的信号的存在/不存在。在检测到正面信号的情况下，它提供检测到的信号的码相和多普勒频移的估计。

该PCPS算法使用FFT和零填充实现。FPGA在没有任何CPU干预的情况下运行完整的采集算法，包括多普勒搜索。

G-TRK-ST: GNSS-SDR跟踪IP核

![G-TRK-ST](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/G-TRK-ST.png)

G-TRK-ST FPGA IP核实现了实时的多相关、载波抹除和导航消息解码算法。

此IP核将输入信号与导航消息的本地副本进行相关，以推导出接收信号的多普勒频率和码相的估计，并保持“Prompt”相关器与输入信号对齐。该核实现了可配置数量的相关器，配置如下：

- 用于信号数据组件的E、P、L（提前、及时、迟到）相关器。
- 用于信号导航消息组件的E、P、L相关器，以及用于数据组件的额外P相关器。
- 用于导航消息组件的VE、E、P、L、VL（非常提前、提前、及时、迟到、非常迟到）相关器，以及用于数据组件的额外P相关器。

PRN码和辅助码的本地副本会自动重新采样到接收信号的采样频率。相干积分时间是可配置的。