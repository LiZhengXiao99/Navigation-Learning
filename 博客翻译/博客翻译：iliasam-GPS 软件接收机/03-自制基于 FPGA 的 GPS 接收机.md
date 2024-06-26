> * 翻译自英语文章：http://www.aholme.co.uk/GPS/Main.htm

# Homemade GPS Receiver

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310210734536.png" alt="image-20240310210734536" style="zoom:67%;" />

上图是 GPS 接收机实验的前端、第一混频器和中频放大器。最左侧的 SMA 连接到带有集成低噪声放大器和声表面波滤波器的商用天线。合成的第一本地振荡器驱动底部的 SMA。右边的针座是电源输入和中频输出。后者连接到一个赛灵思 FPGA，该 FPGA 不仅能执行 DSP，还能承载一个分数 N 频率合成器。稍后再详述。

20 多年前，S53MV 的 Matjaž Vidmar 从零开始，主要使用分立元件开发了 GPS 接收器。我对他在硬限制中频和 1 位 ADC 之后使用 DSP 的做法很感兴趣。这里描述的接收器也是基于同样的原理。它的 1 位 ADC 是引脚接头附近的 6 引脚集成电路，是一个 LVDS 输出比较器。在出现的双电平量化蘑菇云中，来自视野中每颗卫星的信号被隐藏在噪声之下，但并未被抹去。

所有 GPS 卫星都使用相同的频率（1575.42 MHz），通过直接序列扩频（DSSS）进行发射。L1 载波在 2 MHz 的带宽上传播，在地球表面的强度为 -130 dBm。同一带宽内的热噪声功率为-111 dBm，因此接收天线上的 GPS 信号比噪声本底低约 20 dB。在双电平量化之后，任何一个叠加在一起并被噪声掩盖的信号都是可以恢复的，这似乎有悖于直觉！我写了一个仿真来说服自己。

全球定位系统依靠被称为 "黄金码 "的伪随机序列的相关特性，将信号与噪声和相互之间的信号区分开来。每颗卫星都发射一个独特的序列。所有不相关的信号都是噪音，包括其他卫星的信号和硬限制器量化误差。以正确的相位与相同的编码混合，可以消除所需的信号，并进一步分散其他信号。然后通过窄带滤波消除宽带噪声，而不影响（同样狭窄的）所需信号。硬限幅（1 位 ADC）使信噪比降低不到 3 dB，这是为避免硬件 AGC 而付出的代价。

## 2013 年 5 月更新

现在，这是一个真正便携的、电池供电的 12 信道 GPS 接收机，配有全套软件，可获取和跟踪卫星，并不断重新计算其位置，无需用户干预。整套系统（下图左）包括 16x2 液晶显示屏、Raspberry Pi "A "型电脑、两块定制印制电路板、商用贴片天线和锂离子电池。系统总电流消耗为 0.4A，电池寿命为 5 小时。Raspberry Pi 通过连接其 GPIO 接口和 "Frac7 "FPGA 板的带状电缆供电，无需其他连接。

目前，Pi 运行的是 Raspbian Linux。较小的发行版可以缩短首次修复的时间。从 SD 卡启动后，GPS 应用软件会自动启动。退出时，它提供了一种在关机前正确关闭 Pi 的方法。Pi 软件开发是通过 USB Wi-Fi 加密狗以 SSH 和 FTP 方式 "无头 "完成的。源代码和文档可在本页底部找到。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310210922950.png" alt="image-20240310210922950" style="zoom:80%;" />

两块定制印刷电路板都是简单的双层 PTH 板，底部有连续的接地平面。围绕 "Frac7 "FPGA 板上的 Xilinx Spartan 3 顺时针方向：从 12 点钟方向到 3 点钟方向是微波频率合成器的环路滤波器、VCO、功率分配器和预分频器；右下方是操纵杆和 JTAG 连接器；6 点钟方向是 Raspberry Pi 带状电缆的针座。最左侧是 LCD 连接器。左近是温度补偿压控晶体振荡器 (TCVCXO)，提供稳定的参考频率，对 GPS 接收至关重要。

TCVCXO 性能不错，但在多风的地方开箱运行时还达不到 GPS 标准。对其吹气会使 10.000000 MHz 晶体振荡器产生约 1 千万分之一或 1 Hz 的位移，而合成器 PLL 会将位移放大 150 倍。如果突然这样做，足以瞬间解锁卫星跟踪环路。该装置对卤素灯泡和电视遥控器等产生的红外线也略微敏感！

2011 年首次发布时，这是一款四通道接收器，也就是说它只能同时跟踪四颗卫星。要解决用户位置和接收机时钟偏差问题，至少需要四个通道；但如果通道数更多，精度也会更高。在最初的版本中，"跟踪器 "模块的四个相同实例充满了 FPGA。但大多数触发器每毫秒只能时钟一次。现在，FPGA 内部的定制 "软核 "CPU 将处理序列化，8 通道接收器只需 50% 的 FPGA 结构，12 通道则只需 67%。通道数是信号源中的一个参数，可以更高。

当天线能看到 360° 的天空并接收到来自各个方向的信号时，定位精度最高。一般来说，能看到的卫星越多越好。同一方位上有两颗或更多卫星会导致所谓的 "几何形状不良"。迄今为止，在一个非常开阔的地点，使用 12 颗卫星的最佳定位精度为 ±1米；但在卫星数量较少的较差地点，精度通常为 ±5米。

## 2014 年 9 月更新

该项目的源代码已根据 GNU 通用公共许可证 (GPL) 重新发布。

### 架构

FPGA 和 Pi 根据复杂性和紧迫性进行处理。Pi 按自己的节奏处理数学密集型的繁重工作。FPGA 合成第一个本地振荡器，实时处理高优先级事件，并自主跟踪卫星。Pi 通过 SPI 接口控制 FPGA。方便的是，同一 SPI 还可用于加载 FPGA 配置比特流和嵌入式 CPU 的二进制可执行代码。FPGA 也可以通过 JTAG 电缆从 Windows PC 控制，并自动检测使用的接口。

![image-20240310211726666](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310211726666.png)

L1 频率通过与 "GPS3 "前端板上的 1552.82 MHz 本地振荡器混合，下变频为 22.6 MHz 的第一中频。所有后续中频和基带信号处理均在 FPGA 中以数字方式完成。每颗卫星有两个比例积分（PI）控制器，跟踪载波和代码相位。卫星传输的导航数据收集在 FPGA 存储器中。这些数据被上传到 Pi，由 Pi 检查奇偶校验并从比特流中提取星历。当收集到所有必要的轨道参数后，FPGA 内部的某些计数器就会被快照，由此计算出传输时间，精度为 ± 15ns。

1552.82 MHz 合成器的大部分功能都是在 FPGA 中实现的。将相位检测器与其他逻辑共同托管，可能会出现抖动问题，但这是可行的。合成器的输出频谱纯净度非常高，即使 FPGA 内核在频繁切换，而且并非全部在谐波相关频率上切换。之所以采用这种方法，是因为早先的一个合成器项目已经有了一块类似于 "Frac7 "的电路板。添加一个前端是获得接收器原型的最短路径。但第一个版本并不便于携带：它有不便的电源要求，而且没有板载频率标准。

### 前端

包括硬限幅器在内的信号处理：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310211916313.png" alt="image-20240310211916313" style="zoom:50%;" />

LMH7220 比较器的最大输入失调电压为 9.5mV。放大后的热噪声必须大大超过这个值，才能保持比较器的切换。微弱的 GPS 信号只会在零交叉点附近影响比较器！它们被噪声 "采样"！为了估算比较器输入端的噪声电平，我们将增益、插入损耗和噪声数据制成表格：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310211935672.png" alt="image-20240310211935672" style="zoom: 67%;" />

混频器输出的带内噪声为 -174+0.8+28-1.5-3.9+20-6+10*log10(2.5e6) = -73 dBm 或 52µV RMS。混频器以 50 欧姆电阻端接，其后各级以更高阻抗工作。分立中频带的总体电压增益为 1000，因此比较器输入电平为 52mV RMS。

LMH7220 增加了 59 dB 的增益，使整个中频的总增益达到 119 dB。在一个频率上部署如此大的增益是有风险的。为了将风险降至最低，我们在坚实的接地平面上使用了平衡电路，并通过屏蔽双绞线将输出传送到 FPGA。这样做的目的是为了简化，避免二次转换。在实践中，电路非常稳定，因此赌注得到了回报。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212017874.png" alt="image-20240310212017874" style="zoom:80%;" />

有源去耦器 Q1 为远程 LNA 提供 5V 电压。MMIC 放大器 U2 提供 20 dB 增益（中频时没有！），确保系统整体噪声系数较低，即使使用较长的天线电缆也是如此。L1 和 L2 是手工绕制的微波扼流圈，具有非常高的自谐振频率，安装时相互垂直并远离地平面。用 7 厘米长的 32swg 漆包铜线绕制 14 圈，空气包芯，内径 1 毫米。用马可尼 2383 SA 上的跟踪发生器进行了检查，这些线圈的频率为 4 千兆赫。

之所以选择 Mini-Circuits MBA-15L DBM，是因为它在 1.5 GHz 时的转换损耗低至 6 dB，而且 LO 驱动要求低至 4 dBm。R9 端接中频端口。

混频器之后是三个全差分中频放大器级。集电极之间的低 Q 并联调谐电路将 -3 dB 带宽设定在 2.5 MHz 左右，并防止直流偏移的积累。L4、L5 和 L6 是屏蔽的 Toko 7 毫米线圈。选择 BFS17 是因为它的 1 GHz fT 较高（但也不算太高）。Ie 为 2mA，以实现最低噪声和合理的 βre 值。

22.6 MHz 的第一中频在 FPGA 中通过 10 MHz 的欠采样以数字方式下变频到 2.6 MHz。2.6 MHz 接近 5 MHz 奈奎斯特带宽的中心。由于稍后将解释的原因，最好避开确切的中心点。其他几个第一中频频率也是可行的：27.5 MHz 在第二中频产生频谱反转，也曾成功尝试过。在较低频率的图像问题和较高频率的可用 BFS17 增益之间需要权衡。

### 三维搜索

信号探测需要解决三个未知问题：视野内有哪些卫星、卫星的多普勒频移和编码相位。从所谓的 "冷启动 "开始对这个三维空间进行顺序搜索可能需要好几分钟。使用年鉴数据预测位置和速度的 "热启动 "仍需要进行代码搜索。必须对所有 1023 个代码相位进行测试，以找到最大相关峰值。在时域中计算 1023 个相关积分是非常昂贵和多余的。该 GPS 接收机采用基于 FFT 的算法，并行测试所有代码相位。从冷态开始，在 1.7 GHz 奔腾处理器上测量每颗可见卫星的信号强度、多普勒频移和代码相位需要 2.5 秒。Raspberry Pi 则稍慢一些。

以过条表示共轭，复信号 s(t) 和代码 c(t) 经偏移 Τ 移位后的交叉相关函数 y(Τ) 为：
$$
y(\tau)=\int_{-\infty}^{\infty} \bar{s}(t) c(\tau+t) d t
$$
相关定理指出，相关积分的傅里叶变换等于第一个函数的傅里叶变换与第二个函数的傅里叶变换的复共轭乘积：
$$
\mathrm{FFT}(\mathrm{y})=\operatorname{CONJUGATE}(\mathrm{FFT}(\mathrm{s})) * \mathrm{FFT}(\mathrm{c})
$$
相关计算在基带上执行。1.023 Mbps C/A 编码为 1023 个芯片或 1ms 长。前向 FFT 长度必须是其倍数。以 10 MHz 的频率采样 4 毫秒，FFT bin 大小为 250 Hz。41 必须通过旋转频域数据来测试多普勒频移，每次旋转一个频段，最多可旋转 ±20 个频段 = ±5 KHz。旋转可应用于任一函数。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212201104.png" alt="image-20240310212201104" style="zoom:50%;" />

来自 1 位 ADC 的 22.6 MHz 第 1 IF 通过 FPGA 中的 10 MHz 时钟进行欠采样，以数字方式下变频为 2.6 MHz 的第 2 IF。在软件中，利用正交本地振荡器将第二中频向下转换为复数基带 (IQ)。对于双电平信号，混频器是简单的 XOR 门。虽然上面没有显示，但采样会在 FPGA 存储器中临时缓冲。Pi 无法在 10 Mbps 速率下接受采样。

1.023 Mbps 和 2.6 MHz 由数控振荡器 (NCO) 相位累加器产生。与采样率相比，这些频率相当大，而且不是精确的次谐波。因此，数控振荡器会产生小数尖峰。每个编码芯片的采样数量在 9 和 10 之间徘徊。幸运的是，DSSS 接收机能够承受外部或自身产生的窄带干扰。

复杂基带通过前向 FFT 转换到频域，只需计算一次。每个卫星的 C/A 代码的 FFT 都是预先计算好的。处理时间主要来自最内层的循环，该循环执行移位、共轭、复数乘法以及每次卫星-多普勒测试的反向 FFT。可以利用 Raspberry Pi 的 Videocore GPU 来加快速度。

在 10 MHz 的采样率下，代码相位可解析到最接近的 100ns。典型的 CCF 输出如下图所示：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212256069.png" alt="image-20240310212256069" style="zoom: 50%;" />

计算这些数据的峰值到平均功率可以很好地估算信噪比，并用于寻找最强的信号。以下是格林尼治标准时间 2011 年 3 月 4 日 20:14 在英国剑桥收到的信号，天线位于室外朝北的窗台上：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212324004.png" alt="image-20240310212324004" style="zoom: 67%;" />

从北纬开始，通常在南天（即赤道方向）会发现更多的 GPS 卫星。

采样时间越长，信噪比就越高，信号就越弱；但当捕获时间跨越净空数据转换时，就会出现抵消现象。正向 FFT 长度是毫秒的整数；不过，反向 FFT 可以缩短，只需丢弃频率较高的数据即可。信噪比得以保留，但代码相位的解析却不那么清晰。不过，通过对两个最强的相邻频段进行加权平均，可以很好地估算出峰值位置。

### 跟踪

检测到信号后，下一步是锁定、跟踪信号并解调 50 bps 导航数据。这需要两个相互依赖的锁相环（PLL）来跟踪编码和载波相位。这些锁相环必须实时运行，并在 FPGA 中作为 DSP 功能实现。Pi 软件起监督作用：决定跟踪哪些卫星、监控锁定状态并处理接收到的导航数据。
由于跟踪环路的带宽非常窄，因此在保持锁定方面表现出色；然而，同样的特性也使它们在没有帮助的情况下无法获得锁定。它们无法 "看到 "环路带宽以外的东西，也就无法捕捉到更远的东西。初始相位和频率必须根据测量到的代码相位和目标卫星的多普勒频移进行预设。这是在 Pi 控制下进行的。环路应从一开始就保持锁定。

代码相位是相对于 FFT 样本测量的。FPGA 中的代码 NCO 在采样开始时复位，并以固定的 1.023 MHz 累积相位。随后，通过短暂暂停相位累加器，将其与接收到的编码对齐。1575.42 MHz 载波的多普勒频移为±5 KHz 或 ±3 ppm。它对 1.023 Mbps 码率的影响也是每秒 ±3 个芯片。在采样后的时间内，停顿的长度会根据代码蠕变进行调整。幸运的是，编码多普勒与载波多普勒成正比，我们对此有一个很好的估计。

### 软硬件分离

在下图中，彩色编码显示了跟踪 DSP 现在是如何在硬件和软件之间分工实施的。在此之前，这一切都由硬件完成，每个通道都重复使用相同的并行实例，这使得 FPGA 资源的利用效率很低。现在，速度较慢的 1 KHz 处理由软件完成，只需一半的 FPGA 空间就能容纳两倍的通道。
六个积分转储累加器 (Σ) 在代码周期时被锁存到移位寄存器中。服务请求标志向中央处理器发出信号，中央处理器逐位读取数据。在 8 个通道激活的情况下，CPU 8%的时间用于执行 op_rdBit 指令！不过时间还是很充裕的，而且串行 I/O 使用 FPGA 结构也很经济。像 RSSI 和 IQ 日志（例如用于散点图）这样的奢侈功能现在也能负担得起了。

F(z) 环路滤波器传递函数占用每个有效通道 2% 的 CPU 带宽。这些是标准的比例-积分（PI）控制器： 采用 64 位精度，增益系数 KI 和 KP 虽然仅限于 2 的幂次，但可动态调整。由于每个通道都必须等待轮到自己，因此 NCO 速率更新可以在代码纪元之后延迟数十或数百微秒；但在确定相位裕度的频率下，这会带来可忽略不计的相位偏移。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212528643.png" alt="image-20240310212528643" style="zoom: 50%;" />

2.6 MHz 载波首先通过与早码、晚码和准时码混合进行去扩频。由第二级 XOR 门混频器产生的 I 和 Q 复合基带乘积在 10000 个采样点或 1 毫秒内求和。这种低通滤波可显著降低噪声带宽，从而提高信噪比。将采样率降低到 1 KHz 需要在软件域中拓宽数据路径。

使用传统的延迟锁定环路或 "早-晚 "门跟踪代码相位。早期和晚期通道的功率使用 P = I2 + Q2 计算，它对相位不敏感。早期和晚期编码相差一个芯片，即准点前后各相差 ½ 个芯片。此图有助于获得正确的误差感知：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212619103.png" alt="image-20240310212619103" style="zoom: 50%;" />

科斯塔斯环路用于在准点信道中进行载波跟踪和导航数据恢复。k 是接收到的信号幅度，θ 是接收到的载波（不含调制）与本地 NCO 之间的相位差。请注意，反馈给科斯塔斯环路中 F(z) 设备控制器的误差项与接收信号功率 k² 成正比。跟踪斜率以及环路增益也随编码环路中信号功率的变化而变化。下图是 k=500 时 Costas Loop 的开环增益 Bode 图：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212658252.png" alt="image-20240310212658252" style="zoom: 67%;" />

科斯塔斯环路带宽约为 20 赫兹，是载波跟踪的最佳带宽。代码环路带宽为 1 赫兹。这种带宽下的噪声功率很小，环路可以跟踪非常微弱的信号。上述 kI 和 kP 适用于大多数信号，但对于最强信号则需要降低一个档位。Scilab 预测 k≥1500 时开始出现不稳定性，散点图也证实了这一点。除非采样偏离 IQ 平面的另一半，否则不会出现奇偶校验错误。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212729633.png" alt="image-20240310212729633" style="zoom:50%;" />

多普勒频移量一直在变化。跟踪载波频率的变化需要在环路滤波器输入端有一个小的、恒定的相位误差来驱动积分器。如果 kI 积分器增益不足，相位误差就会以散点图旋转的形式出现；真实或反相的导航数据就会出现在 Q 信道的符号位上。

## 捕获

代码生成器已对齐，两个环路 NCO 频率使用 FFT 搜索数据进行初始设置。初始载波 NCO 可偏离频率达 250 Hz（FFT 二进制大小），使其超出环路捕获范围。初始编码率误差不能超过 0.16 Hz，编码环路对载波相位不敏感。如果信号足够强，编码环路总能锁定；但载波环路有时需要帮助。幸运的是，准确的载波偏移可以通过锁定的编码 NCO 计算出来，因为两者都表现出相同的多普勒频移。一旦载波环路的 NCO 得到更新，载波环路总能锁定。
上述程序似乎是 100% 可靠的，在此之前，我不得不重试采集，直到载波环路锁定为止。幸运的是，多普勒频移是不断变化的，如果一次尝试失败，下一次往往就会成功。在顽固的情况下，将载波 NCO 向上或向下推移半个 FFT bin-width 证明是有效的。

由于 NCO 上的小数尖刺，接近 2.5 MHz 原始中频中心频率的载波很难获取。将中频频率上移 100 千赫后，情况大为改观。第一本地振荡器频率改为 1552.82 MHz，将第一和第二中频频率分别移至 22.6 MHz 和 2.6 MHz。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212836932.png" alt="image-20240310212836932" style="zoom:50%;" />

这些频谱显示，载波 NCO 设置为高于中频中心频率 2.5 和 2.6 MHz 的 50 Hz。原来的中心频率是采样率的四分之一。当频率不是简单的比例时，尖峰会安全地远离。

## 导航电文

净导航数据取自科斯塔斯环路 I 臂的符号位。Q 臂看起来应该像随机噪声。以下是 512 个原始 Ip、Qp 采样，采样率为 1 KHz：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310212930418.png" alt="image-20240310212930418" style="zoom:33%;" />

以下是格林尼治标准时间 2011 年 2 月 1 日（星期二）21:46:45 收到的净资产收益率数据片段（右边还有）：

![](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310213001504.png)

```
10001011	00001001010101	00	110100	01010001110001111	01	001	11	101100	100101010101000000000000010111100010110011111111110010110001010111111101010011011010011011110001010110110000110111000001011001001101000100010110111101110010001100001001111001011101111111111111111111101101011111111111011010110101101011100000
10001011	00001001010101	00	110100	01010001110010000	01	010	01	100000	001101111111000000110111100010001011001100100010011100000100010001100011010011110000001111000011011010101011110111100001010110101011011100001101100111111101101101101001011110110000000011011000111000101010100100001111011000011001111111111000
10001011	00001001010101	00	110100	01010001110010001	01	011	11	101000	000000000110001111011010001000101010011110010110011100000001000000000011000011011000110010101001100010110001110011010110001001010110000010110010001100000010110001010010111101100011000000000101010011011110011001110010000000100111011111000100
10001011	00001001010101	00	110100	01010001110010010	01	100	01	010100	011111111010100110011001010110101010011010100110011001010000100110101001100110101001110101010101100110011001100110100110100110011011100110011001001111010101100101011011111111001001111111111111111111111111010110000000000000000000000010001100
10001011	00001001010101	00	110100	01010001110010011	01	101	11	011100	011100110110001101010101000001000000111111111111111111110111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111010101010101010101010110100
10001011	00001001010101	00	110100	01010001110010100	01	001	10	010100	100101010101000000000000010111100010110011111111110010110001010111111101010011011010011011110001010110110000110111000001011001001101000100010110111101110010001100001001111001011101111111111111111111101101011111111111011010110101101011100000
10001011	00001001010101	00	110100	01010001110010101	01	010	11	111100	001101111111000000110111100010001011001100100010011100000100010001100011010011110000001111000011011010101011110111100001010110101011011100001101100111111101101101101001011110110000000011011000111000101010100100001111011000011001111111111000
10001011	00001001010101	00	110100	01010001110010110	01	011	10	001100	000000000110001111011010001000101010011110010110011100000001000000000011000011011000110010101001100010110001110011010110001001010110000010110010001100000010110001010010111101100011000000000101010011011110011001110010000000100111011111000100
10001011	00001001010101	00	110100	01010001110010111	01	100	11	001000	011110010101111000011000001100011110011101011100011000111000010001000000011010010111010101110111101111100011100010101101111001111000101010010111000010101010110111100000000010110010000000101010000000101011111100101010101010101010101010111100
10001011	00001001010101	00	110100	01010001110011000	01	101	00	111100	010000001010101010101010101100101010101010101010101010111100101010101010101010101010111100101010101010101010101010111100101010101010101010101010111100101010101010101010101010111100101010101010101010101010111100101010101010101010101010111100
```

以上是 2 个连续的帧，每个帧有 5 个子帧。子帧长 300 位，传输时间为 6 秒。第 1 列是前导码 10001011。它出现在每个子帧的开头，但也可以出现在数据的任何位置。第 5 列的 17 位计数器为周时间 (TOW)，在周日午夜重置为零。第 7 列中的 3 位计数器是子帧 ID 1 至 5。子帧 4 和子帧 5 分别细分为 25 个页面，由 25 个完整帧组成的完整数据报文传输时间为 12.5 分钟。目前我只使用子帧 1、2 和 3 中的数据。

## 定位解算

每颗 GPS 卫星都会发送自己的位置和时间。从接收到的时间中减去发送时间，再乘以光速，这就是接收器测量自身与卫星之间距离的方法。如果有精确的时间，那么用三颗卫星同时计算三个未知数（用户位置：x、y、z）就会得出三个方程。实际上，接收机时钟不够精确，精确时间是第四个未知数，因此需要四颗卫星，必须求解四个同步方程：
$$
\begin{array}{l}P R_{1}=c t_{b}+\sqrt{\left(x-x_{1}\right)^{2}+\left(y-y_{1}\right)^{2}+\left(z-z_{1}\right)^{2}} \\ P R_{2}=c t_{b}+\sqrt{\left(x-x_{2}\right)^{2}+\left(y-y_{2}\right)^{2}+\left(z-z_{2}\right)^{2}} \\ P R_{3}=c t_{b}+\sqrt{\left(x-x_{3}\right)^{2}+\left(y-y_{3}\right)^{2}+\left(z-z_{3}\right)^{2}} \\ P R_{4}=c t_{b}+\sqrt{\left(x-x_{4}\right)^{2}+\left(y-y_{4}\right)^{2}+\left(z-z_{4}\right)^{2}}\end{array}
$$
由于方程是非线性的，因此采用了迭代法。以地球中心（0, 0, 0）和近似时间为起点，算法只需五六次迭代就能收敛。即使用户时钟误差很大，也能找到解决方案。卫星携带原子钟，但原子钟也有误差，因此必须对传输时间应用子帧 1 中的校正系数。典型的调整可达数百微秒。

未校正的传输时间是由若干计数器按比例相加而成的。每个子帧都会发送周时间（TOW），单位为自周日午夜起的秒数。数据边沿在 300 位子帧内标出 20 毫秒的间隔。每个数据位重复发送 20 次代码。编码长度为 1023 比特，芯片速率为 1.023 Mbps。最后，附加代码 NCO 阶段的 6 个最有效位，将传输时间固定为 ± 15ns。

在第 2 和第 3 子帧中使用星历计算校正传输时间的卫星位置。提供参考时间 t_oe（星历时间）的轨道位置和参数，以便计算前后几小时内的（x,y,z）位置。星历表定期更新，卫星只发送自己的星历表。使用子帧 4 和 5 中的 Almanac 数据可以较不精确地预测整个星座的长期轨道；不过，如果使用基于 FFT 的快速搜索，这并不是必要的。

解法以地球中心、地球固定（ECEF）坐标计算。用户位置转换为纬度、经度和高度，并对赤道处凸起的地球偏心率进行修正。下面的散点图说明了重复性、平均化的好处以及卫星选择不当的影响。网格方格每边为 0.001°。蓝色圆点表示 1000 个固定点。黄色三角形标记重心：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310213252178.png" alt="image-20240310213252178" style="zoom:50%;" />

紧簇（ii）是利用天空四个不同区域的卫星获得的。只有屋顶天线在所有方向上都有清晰的视野。但即使半边天空被遮挡，通过平均也能获得良好的定点。如果选择了错误的卫星，屋顶定点也会出现类似(i)和(iii)的传播。
上述解决方案是在没有使用子帧 4 第 18 页的参数对电离层传播延迟进行补偿的情况下生成的，因为这是一个单频接收机，所以应该使用这些参数。电离层折射会增加用户和卫星之间的路径长度。

2012 年 4 月，我修复了一个导致用户位置解决方案出现重大错误的错误。最初，由于没有将卫星位置从地心-地球固定坐标（ECEF）转换为地心-惯性坐标（ECI），我实际上忽略了信号飞行过程中 60 至 80 毫秒的地球自转。现在，即使在卫星能见度有限的情况下，我也能在平均后看到 ± 5 米的定位解算精度。

我创建了一个附录，展示了迭代求解的发展过程，从几何范围方程开始，使用泰勒级数展开进行线性化，并通过矩阵方法求解，适用于四颗卫星的特殊情况或更多卫星的一般情况，并可选择使用加权最小二乘法来控制特定卫星的影响。您可以在页面底部的链接中找到此源代码和解决方案的 "C "源代码。

我非常感谢 Dan Doberstein 给我寄来了他的 GPS 一书[2]的初稿，它帮助我理解了求解算法。美国政府官方的 GPS 接口规范 [3] 是一个重要的参考资料。

## 信号监控器

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220004090.png" alt="image-20240310220004090" style="zoom:50%;" />

上述电路安排主要是在 FPGA 中实现的，通过提取 1 位中频和准时码的乘积来进行去扩频，留下 50 bps 的数据调制。可以看到由于 BPSK 载波抑制而产生的一个小缺口：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220027692.png" alt="image-20240310220027692" style="zoom:50%;" />

这些频谱显示了不同跨度和分辨率带宽（RBW）下相同的去展宽传输。多普勒频移为-1.2 千赫。本底噪声是经过中频带放大和滤波的天线热噪声。-3 dB 带宽约为 3 MHz，比计划的略宽。去展载波在 30 KHz RBW 时比噪声高 5 dB，在 300 Hz RBW 时比噪声高 25 dB。天线上的接收信号强度可估算为 -174+1+10*log10(30e3)+5 = -123 dBm。

通过硬限幅，频域信息得以很好地保留，这仍然令我感到惊讶！LVDS 发射器的恒定输出电流为 ~3mA，在 100 欧姆时为 ~1mW。在 SA 上看到的峰值功率不能超过 0 dBm。在这里，我们可以看到这些可用功率分布在不同的频率范围内。宽带综合功率谱密度必须为 ~1mW。

## 第一个本地振荡器

几年来，我一直在使用通用可编程逻辑制作实验性的分数 N 合成器：

| Project                                                 | Date | Technology            | Frequency (MHz) |
| ------------------------------------------------------- | ---- | --------------------- | --------------- |
| [FracN](http://www.aholme.co.uk/FracN/Synth.htm)        | 2004 | Altera MAX 7000 CPLD  | 4.3             |
| [Frac2](http://www.aholme.co.uk/Frac2/Main.htm)         | 2005 | Altera MAX 7000 CPLD  | 15 - 25         |
| [Frac3](http://www.aholme.co.uk/Frac3/Main.htm)         | 2009 | Xilinx Spartan 3 FPGA | 38 - 76         |
| [Frac4](http://www.aholme.co.uk/Frac3/Main.htm#May2009) | 2009 | Xilinx Spartan 3 FPGA | 38 - 76         |
| [Frac5](http://www.aholme.co.uk/Frac3/Main.htm#Frac5)   | 2010 | Xilinx Spartan 3 FPGA | 800 - 1600      |
| Frac7                                                   | 2013 | Xilinx Spartan 3 FPGA | 1500 - 1600     |

Frac7 就是为此目的而制造的；但在最初设计 Frac5 时，我并没有想到它会用于 GPS 接收机。下图显示了 Frac5 上的 ROS-1455 VCO 输出如何在输出 SMA 和 Hittite HMC363 除 8 预分频器之间进行电阻分频。200 MHz 的分频器输出被路由（差分）到 FPGA，FPGA 使用我早期项目中记录的方法将其相位锁定到主基准。Frac7 上的微波电路与此类似，但使用了 Mini-Circuits 3dB 分频器。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220305785.png" alt="image-20240310220305785" style="zoom:67%;" />

如下图所示的 VCO 输出频谱所示，它实现了高稳定性和低相位噪声。Frac5 最初作为专用频率合成器开发时，为了尽量减少互调尖峰，避免同时切换不相关的频率。当切换相位检测器输出的时钟脉冲穿过结构时，FPGA 是静态的。但幸运的是，本地振荡器输出已经足够好了：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220330769.png" alt="image-20240310220330769" style="zoom:50%;" />

马可尼 2383 频谱分析仪的 50 MHz STD 输出被用作 Frac5 和所有 GPS 接收机内部时钟的主参考源。GPS 接收机测量 1575.42 MHz L1 载波上 ±5 KHz 多普勒频移的精度需要高于 1 ppm（百万分之一）。任何频率的不确定性都需要更宽的搜索范围。

## 嵌入式 CPU

我最初的 GPS 接收器只能跟踪 4 颗卫星。可用的结构没有得到有效利用，FPGA 已满。每个通道都复制了相同的逻辑，而且只在 1 KHz 代码纪元时启用时钟。GPS 更新率要求很低，大多数 "并行 "处理都可以很容易地按顺序完成。为这一任务嵌入 CPU 既增加了通道数量，又释放了 FPGA 空间。
该中央处理器可直接执行作为本地指令的 FORTH 基元。访问我的 Mark 1 FORTH 计算机页面的人应该已经知道我对这种语言的兴趣。FORTH 并不是主流语言，在这里使用它可能会成为一个深奥的障碍；然而，在看到优秀的 J1 项目之后，我忍不住又做了一个 FORTH CPU，这次是 FPGA CPU，它给了我灵感。

FORTH 是一种基于堆栈的语言，基本上意味着 CPU 有堆栈而不是通用寄存器。维基百科上有很好的概述。

* FPGA 资源： 360 个切片 + 2 个 BRAM
* 单周期指令执行
* 类似 FORTH 的双栈架构
* 32 位堆栈和 ALU 数据路径
* 64 位双精度运算
* 硬件乘法器
* 2 千字节（可扩展至 4 千字节）代码和数据 RAM
* 宏汇编代码开发

使用两个 BRAM：一个用于主存储器，另一个用于堆栈。Xilinx 块 RAM 采用双端口设计，允许一个实例同时容纳数据堆栈和返回堆栈。每个堆栈指针的范围为阵列的一半。主存储器的双端口允许数据访问与指令获取同时进行。一个内存端口由程序计数器寻址，另一个由堆栈顶点 T 寻址。向 PC 寻址端口的写入也用于代码下载，程序计数器提供递增地址。
代码和数据共用主内存，主内存为 1024（可扩展至 2048）个 16 位字。内存访问可以是 16 位、32 位或 64 位，字对齐。所有指令均为 16 位。尽管所有循环都是展开的，但 GPS 应用程序的代码和数据总大小不到 750 字。

输入/输出没有内存映射，占用自己的 36 位选择空间（12 进 + 12 出 + 12 事件）。单热编码用于简化选择解码。I/O 操作分为 1 位串行、16 位或 32 位并行。串行数据每个时钟周期移动 1 位。事件主要用作硬件选通，与写不同的是不弹出堆栈。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220503797.png" alt="image-20240310220503797" style="zoom:50%;" />

在可能的 32 条指令中，目前有 24 条指令被分配到操作码空间 h80XX - h9FXX。这些指令主要是零操作数的堆栈/ALU 运算。从子程序返回的 "ret "选项在同一周期内并行执行。Add-immediate 是唯一的单操作数指令。进位选项扩展了（堆栈、隐含）加法精度。

堆栈和 ALU 数据路径为 32 位，但也支持 16、32 和 64 位运算。64 位数值在堆栈中占据两个位置，最小有效位在最上面。为了提高效率，堆栈顶位 T 和堆栈下一位 N 都在 BRAM 之外注册。除了 64 位左移（op_shl64）是硬连线单周期执行外，所有其他双精度功能都是软件子程序。

GPS 嵌入式二进制文件是使用微软的宏汇编器 MASM 创建的。它只支持 x86 助记符；但操作码使用 equ 声明，代码使用 "dw "指令组装。MASM 不仅提供标签解析、宏扩展和表达式评估，甚至还提供数据结构！MASM 的 dup() 操作符被广泛用于展开循环，例如 dw N dup(op_call + dest) 调用子程序 N 次。

这个片段介绍了源代码的一些风格。每一行都有堆栈效果注释：

```
op_store64      equ op_call + $             ; [63:32] [31:0] a         17 cycles
                dw op_store32               ; [63:32] a
                dw op_addi + 4              ; [63:32] a+4
                ; drop through
op_store32      equ op_call + $             ; [31:0] a                  8 cycles
                dw op_over                  ; [31:0] a [31:0]
                dw op_swap16                ; [15:0] a [31:16]
                dw op_over                  ; [15:0] a [31:16] a
                dw op_addi + 2              ; [15:0] a [31:16] a+2
                dw op_store16, op_drop      ; [15:0] a
                dw op_store16 + opt_ret     ; a
```

* op_fetch16 和 op_store16 是基元指令。
* op_store32 和 op_store64 是子程序或 "复合指令"，可以像基元指令一样使用。
* 在 op_swap16 之后，T 实际上是 [15:0,31:16]，但这里我们不关心高 16 位。
* op_store16 留下地址；堆栈深度每个周期只能变化 ±1。
* 纯粹主义者可能更喜欢：dw N + addi

FPGA 可通过 SPI 由 Raspberry Pi 控制，或通过 JTAG 电缆由 Windows PC 控制。请求优先级分为两级：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220626801.png" alt="image-20240310220626801" style="zoom: 67%;" />

新代码映像通过第三个 BRAM 复制到主存储器，该 BRAM 是 CPU 和串行时钟域之间的桥梁。这样，下载的二进制图像就会自动执行。桥接 BRAM 会捕获主机命令，并发出信号让 CPU 执行这些命令。下一次扫描时，主机将从桥接器中收集其响应。

顶层主循环轮询主机服务请求。任何主机信息的第一个字都是命令代码。请求通过命令跳转表发送：

```
dw op_rdReg + JTAG_RX       ; cmd
dw op_shl                   ; offset
dw Commands, op_add         ; &Commands[cmd]
dw op_fetch16               ; vector
dw op_to_r, op_ret  
```

* op_too_r 将向量移到返回堆栈。

某些主机请求（如 CmdGetSamples）会引起冗长的响应。桥接器 CPU 端的数据端口为 16 位。CPU 可以通过数据堆栈读写这些数据；不过，上传主内存和 GPS IF 样本有更直接的途径。指令 op_wrEvt + GET_MEMORY 使用 T 作为自动递增指针，将内存字直接传输到网桥。GET_MEMORY 是唯一具有堆栈效应的事件。指令 op_wrEvt + GET_SAMPLES 从中频采样器传输 16 个比特：

```
UploadSamples:  dw 16 dup (op_wrEvt + GET_SAMPLES)     ;  16*16 =  256 samples copied
                dw op_ret
              
CmdGetSamples:  dw op_wrEvt + JTAG_RST                 ; addr = 0
                dw 16 dup (op_call + UploadSamples)    ; 256*16 = 4096 samples copied
                dw op_ret
```

使用 dup() 在汇编时解滚循环，可以用代码大小换取性能，避免了递减测试分支的冲击；而且整个应用程序二进制文件仍然很小；不过，长循环必须嵌套，如上图所示。

结构数组用于保存状态变量和通道的缓冲 NAV 数据。MASM 对数据结构的支持非常出色。字段偏移会自动定义为常量，而 sizeof 运算符也非常有用。

```
MAX_BITS        equ 64

CHANNEL         struct
ch_NAV_MS       dw ?                        ; Milliseconds 0 ... 19
ch_NAV_BITS     dw ?                        ; Bit count
ch_NAV_PREV     dw ?                        ; Last data bit = ip[15]
ch_NAV_BUF      dw MAX_BITS/16 dup (?)      ; NAV data buffer
ch_CA_FREQ      dq ?                        ; Loop integrator
ch_LO_FREQ      dq ?                        ; Loop integrator
ch_IQ           dw 2 dup (?)                ; Last IP, QP
ch_CA_GAIN      dw 2 dup (?)                ; KI, KP
ch_LO_GAIN      dw 2 dup (?)                ; KI, KP
CHANNEL         ends

Chans:          CHANNEL NUM_CHANS dup (<>)
```

划时代服务例程（标记为 Method:）在调用时会指向堆栈上的 CHANNEL 结构指针。受 OO 风格影响，堆栈效应注释在整个例程中将其称为 "this"。在返回堆栈中保留了一个副本，以便于访问结构成员：

```
dw op_r                     ; ... this
dw op_addi + ch_NAV_MS      ; ... &ms
dw op_fetch16               ; ... ms
```

Chans 阵列会定期上传到主机。

## 树莓派应用程序

Raspberry Pi 软件的多任务处理使用了各种不同的线程（coroutines）、连续线程（continuation）、用户模式线程（user-mode）或轻量级线程（light-weight threads）。这些线程使用 "C "库 setjmp/longjmp 非本地 goto，以轮循方式协同控制，避免了内核上下文切换的代价：

````
void NextTask() {
    static int id;
    if (setjmp(Tasks[id].jb)) return;
    if (++id==NumTasks) id=0;
    longjmp(Tasks[id].jb, 1);
}
````

最多可激活 16 个线程：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310220932730.png" alt="image-20240310220932730" style="zoom: 67%;" />

以线程方式编码，每个线程负责一项任务，这样产生的代码可读性更高。其他源文件包括：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240310221005409.png" alt="image-20240310221005409" style="zoom:67%;" />

本项目中没有 Arduino，但使用了其 LCD 驱动程序文件 [LiquidCrystal.cpp](http://code.google.com/p/arduino/source/browse/trunk/libraries/LiquidCrystal/LiquidCrystal.cpp) 和 [LiquidCrystal.h](http://code.google.com/p/arduino/source/browse/trunk/libraries/LiquidCrystal/LiquidCrystal.h)。

## 源代码

* 2019 年更新

  * Pointer mangling, a security feature, broke coroutines.cpp
  * Function `Sample()` in search.cpp wrote beyond FFT input buffer end

  Both issues are fixed in [2019 patch](http://www.aholme.co.uk/GPS/SRC/2019/)

* 2013 年（最新版本，根据 GNU 通用公共许可证重新发布）

  * [ASM](http://www.aholme.co.uk/GPS/SRC/2013/ASM/) Embedded CPU FORTH
  * [Verilog](http://www.aholme.co.uk/GPS/SRC/2013/Verilog/) Spartan 3 FPGA
  * [C++](http://www.aholme.co.uk/GPS/SRC/2013/C++/) Raspberry Pi

* 旧版本（Win32 C++）

  * [2012](http://www.aholme.co.uk/GPS/SRC/2012/)
  * [2011](http://www.aholme.co.uk/GPS/SRC/2011/)

* 示意图

  * [GPS3](http://www.aholme.co.uk/GPS/SCH/GPS3_schematic.pdf) front-end
  * [Frac7](http://www.aholme.co.uk/GPS/SCH/Frac7_schematic.pdf) FPGA

* 链接和资源

  * [User position solution](http://www.aholme.co.uk/GPS/user_position_solution.pdf) derivation
  * [N2YO ](http://www.n2yo.com/whats-up/?c=20)live tracking of GPS satellites above your horizon
  * [Celestrak](http://celestrak.com/GPS/) daily GPS ephemeris TLE (two line element) updates
  * [SPACETRACK Report No. 3](http://celestrak.com/NORAD/documentation/spacetrk.pdf) mathematical models for processing orbital elements
  * [STSPlus](http://celestrak.com/software/dransom/stsplus.html) orbital tracking software
  * [How to locate the preamble in NAV data](http://kom.aau.dk/~borre/masters/receiver/z-count.pdf)
  * [PyEphem](http://rhodesmill.org/pyephem/) Python library for astronomical computations
  * [Doppler.py](http://www.aholme.co.uk/GPS/SRC/2011/Python/Doppler.py) Python script for predicting GPS satellite Doppler shifts
  * [www.gps.gov/technical/icwg](http://www.gps.gov/technical/icwg/) GPS documentation

## 参考资料

1. [GPS/GLONASS receiver](http://lea.hamradio.si/~s53mv/navsats/theory.html) Matjaž Vidmar, S53MV
2. [PRINCIPLES OF GPS RECEIVERS - A HARDWARE APPROACH](http://www.dkdinst.com/gpstxt.html) by Dan Doberstein
3. [IS-GPS-200E](http://www.gps.gov/technical/icwg/IS-GPS-200E.pdf) GPS Interface Specification