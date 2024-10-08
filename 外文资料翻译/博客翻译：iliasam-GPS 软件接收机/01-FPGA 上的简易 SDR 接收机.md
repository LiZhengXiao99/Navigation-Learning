> * 翻译自俄语文章：https://habr.com/ru/articles/204310/
> * Quartus 项目 + 源代码和 ExtIO dll 本身：https://github.com/iliasam/SDR_projects

# FPGA 上的简易 SDR 接收机

在本文中，我将告诉你如何基于 DE0-nano 调试板制作一个相当简单的 SW SDR 接收器。接收信号示例：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311091827266.png" alt="image-20240311091827266" style="zoom: 67%;" />

您可以在[这里](http://habrahabr.ru/post/158401/)了解有关 SDR 技术的信息。简而言之，这是一种以数字方式进行大量信息处理的无线电接收技术。利用 FPGA 和高速 ADC，可以制造出一种接收器，甚至可以用数字方式进行频率下变频。这种方法称为 DDC（数字下变频），您可以在[这里](http://www.radioexpert.ru/articles/sdr-i-ddc-priemniki-i-transivery/167/)和[这里](http://r4n.su/forum/viewtopic.php?f=28&p=2199)（更多理论）了解更多相关信息。使用这种技术可以大大简化接收机，其中唯一的模拟部分就是 ADC。

现在来了解一下我的接收器。它基于 Altera 制造的 FPGA，安装在 DE0-Nano 调试板上。这块调试板相对便宜（学生用 60 美元），但运费相当昂贵（50 美元）。现在，它在开始熟悉 FPGA 的无线电爱好者中越来越受欢迎。FPGA 的主要任务是从 ADC 中 "捕获 "数字信号，将其传输到低频区域，进行滤波并将结果发送到计算机。我实现的接收器结构图如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092000429.png" alt="image-20240311092000429" style="zoom: 67%;" />

让我们依次考虑无线电信号和数字信息所经过的各个组成部分。

[TOC]

## 天线

无线电爱好者有一句谚语："好的天线是最好的放大器"。的确，这在很大程度上取决于天线。大多数最有趣的短波信号都无法通过简单的天线（例如一根电线）接收到。在城市外没有特别的问题--一根足够长的电线就可以作为良好的天线（用于接收）。在城市里，尤其是在大型钢筋混凝土房屋内，情况就糟糕得多--长天线无法拉伸，而且干扰噪音很大（家用电器会在空气中产生很高的噪音），因此天线的选择就成了一个难题。为了接收无线电信号，我使用了有源框架天线，其构造在这里有[介绍](http://www.techlib.com/electronics/antennas.html)。我的天线是这样的：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092125290.png" alt="image-20240311092125290" style="zoom:50%;" />

实际上，天线是一个大型振荡电路（电容器就在桌子上的盒子里）。它安装在阳台上，效果很好。框架天线的主要优点是可以利用共振现象抑制未使用频率上的噪音，但也有缺点--当从一个频率范围切换到另一个频率范围时，必须对天线进行调谐。

## ADC

选择 ADC 也不容易。ADC 必须具有较高的比特率，以增加动态范围，而且对于 DDC 接收器来说，它还必须具有较高的速度。通常，好的 DDC 接收器都配备速度大于 50 MSPS 的 16 位 ADC。然而，这种 ADC 的价格超过 50 美元，因此我想在实验设计中采用更简单的 ADC。我选择了 AD9200 - 一种 10 位 20 MSPS ADC，价格为 200 卢布。对于 DDC 接收器来说，这些特性非常一般，但实践证明，ADC 非常适合接收信号。ADC 安装在一块单独的电路板上，并插入调试电路板：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092211288.png" alt="image-20240311092211288" style="zoom:50%;" />

电路板底部经过金属化处理，金属层与 ADC 接地相连，这也是为了防止干扰。ADC 接线图如下：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092259819.png" alt="image-20240311092259819" style="zoom: 67%;" />

我没有射频设计布线方面的经验，因此电路和布线有可能需要改进。

由于 ADC 只对正电平信号进行数字化处理，而来自天线的信号是双极性的，因此必须对信号进行基准电压一半的移位（电阻 R1 和 R2 用于此目的）。然后从 FPGA 中的数字信号中减去人为产生的常数分量。

ADC 之后的所有进一步信号处理均由 FPGA 完成。来自 ADC 的数据流为 200 Mbit（10 位 x 20 MSPS）。要将这样的数据流直接传输到计算机并进行处理非常困难，因此必须有意降低信号频率。向较低频率传输信号会产生 "镜像信道 "现象，而使用正交频率转换则可以抵消这种现象--信号被转换成复数形式（信号被分成两个 I/Q 信道）。通过将原始信号乘以信号发生器信号，将信号转换为较低频率。所使用的 FPGA 有足够的硬件乘法器，因此这不成问题。

## NCO

为了将输入信号传输到所需频率，必须创建一个振荡器。为此，我们使用了一个现成的 Quartus 组件 - NCO（数控振荡器）。振荡器的时钟频率与 ADC 相同（20 MHz），一个确定频率的值被输入到振荡器的控制输入端，然后在其输出端产生一个所需频率的数字正弦信号，采样频率为 20 MHz。NCO 还能并行产生余弦信号，从而产生正交信号。

## CIC 滤波器

在与乘法器输出的振荡器信号混合后，输出的信号已经移频到较低的频率，但采样率仍然很高（20 MSPS）。需要对信号进行抽取，即丢弃部分采样。不可能简单地舍弃多余的采样，因为这会导致输出信号失真。因此，信号必须经过一个特殊的滤波器（CIC-滤波器）。在这种情况下，我希望在接收器输出端获得 50 kHz 的采样频率。因此，频率必须降低一个系数（20e6 / 50e3 = 400）。因此，必须分两步进行去微处理--首先是 200 倍，然后是 2 倍。执行第一阶段的是 CIC 滤波器。我使用了一个 5 级滤波器。

CIC 滤波器工作的结果是，通过降低信号带宽，提高了输出信号的数字化程度。在我的接收机中，我人为地将其限制为 16 比特。由于接收机有两个通道，因此也需要两个滤波器。遗憾的是，CIC 滤波器的 AFC 相当陡峭，当接近输出采样频率（100 kHz）时，AFC 趋于 0。下面的滤波器可用于补偿其曲率。

## 补偿 FIR 滤波器

需要使用该滤波器来补偿 CIC 滤波器 AFC 的下降，并执行另一个抽取步骤（系数为 2）。Altera 已经解决了这一滤波器的计算方法--在创建 CIC 滤波器时，会自动生成一个 Matlab 程序，运行该程序即可生成补偿滤波器的系数。

查看 CIC、FIR 和结果（由同一 Matlab 程序绘制）：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092520537.png" alt="image-20240311092520537" style="zoom:50%;" />

可以看出，在 25 kHz 频率时，CIC 滤波器将使信号衰减 20 dB，这是一个很大的衰减量，而 FIR 滤波器的衰减量仅为 10 dB，在较低频率时几乎没有衰减。在 FIR 滤波器的输出端，信号的采样频率将为 50 kHz。

为什么不能立即对信号进行 400 倍的抽取呢？这是因为 FIR 滤波器的截止频率应为其输出频率的 1/4。在这种情况下，滤波器输出端的采样频率和输入端的采样频率都是 100 kHz。如上图所示，这将导致截止频率仅为 25 kHz。

这两个滤波器都是 Quartus 现成的元件。

## 将数据传输到 PC

接收到的数据流（(16+16)比特 x 50 KSPS = 1.6 Mbit）需要传输到计算机。我决定通过以太网传输数据。调试板上没有这样的接口。最正确的方法是制作一个带有 PHY 控制器的独立电路板，运行 Nios 软处理器，并通过它们传输数据。然而，这样做会使设计变得相当复杂。我采用了一种更简单的方法--以太网数据包可以在 FPGA 本身生成，因此可以 10 Mbit 的速度传输数据。在这种情况下，以太网电缆通过隔离变压器连接到 FPGA 引脚。您可以在[这里](http://www.fpga4fun.com/10BASE-T0.html)和[这里](http://www.marsohod.org/index.php/projects/94-ether-send)看到采用这种原理的项目。

我选择了第一个项目作为基础，并对其进行了部分修改。在最初的项目中，FPGA 将一个 UDP 数据包发送给一台具有给定 IP 和 MAC 地址的计算机。修改后，以太网发送器模块可以从 RAM 中读取 1024 个字节进行发送。因此，从滤波器输出端提取的 256 对 16 位信号值将在一个数据包中发送给计算机。由于从 ADC 不断接收数据并以数据包的形式发送到计算机，我们必须对存储器进行双重缓冲--当一个 RAM 满载时，另一个 RAM 中的数据通过以太网传输。第一个 RAM 填满后，两个 RAM 会 "交换位置"，这由一个非常简单的控制模块负责。

由于滤波器的输出是一对 16 位的数据流，而单个字节通过以太网传输，因此设计中引入了一个模块，将 32 位 50 KSPS 数据流转换为 8 位 200 KSPS 数据流，以转换数据流。

事实证明，如果以 1.6 Mbit 的速度传输数据流，接收器所连接的设备甚至无法检测到数据流（无链接）。这是因为数据包的传输周期约为 5 毫秒，而为了通知另一个网络设备连接速度（10 Mbit），必须每隔 8-24 毫秒传输一个特殊的短脉冲（NLP）。由于数据包速率较高，以太网模块没有时间传输这些脉冲，自动协商也就无法进行。因此，为了使对面的设备仍能确定连接速度，在打开接收器时，只需暂时降低数据包速率（在我的例子中降低了 4 倍），这样以太网模块就有时间传输 NLP 脉冲。

## PC 端接收数据

为了控制接收机（设置调谐频率），必须向接收机发送一定的数值，用于设置 NCO 频率。为了接收这个数值，还使用了上述网站的一个组件，该组件经过修改，可以接收数据并以 24 位数字的形式输出。由于接收器和发射器模块之间没有连接，因此不可能实现 ARP，事实上这意味着接收器没有 IP 和 MAC 地址。可以通过向网络发送广播数据包的方式向其发送信息。在物理上，与发射器一样，网络线通过变压器连接到调试板。但是，由于信号很小，因此无法连接到任意的 FPGA 引脚。必须使用支持 LVDS 接口的引脚--它是差分接口。FPGA 计划使用的资源：

- 5006 LE
- 68 个 9 位乘法器（其中 64 个用于 FIR 滤波器）。
- 16 826 位内存（8 个 M9K 块）。

在 Quartus 中查看项目设计：

![image-20240311092837093](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311092837093.png)

## PC 端数据处理

计算机接受数据后，需要对其进行处理。最好使用现成的程序。通常，SDR 程序会执行必要的数字滤波器、旨在形成声音及其滤波的算法、接收信号的 FFT、频谱构建和 "瀑布图"。我使用 HDSDR 和 SDRSharp 程序，它们都支持使用相同的 ExtIO 库（Winrad 程序格式）输入数据。程序对库的要求都有[详细记录](http://www.winrad.org/bin/Winrad_Extio.pdf)。

下面是创建这样一个库的示例。我重新制作了这个示例，增加了从网络接收数据、粘合两个数据包（程序每次至少接受 512 对 I/Q 采样）、将它们发送到程序，以及在程序中改变频率时发送带有 NCO 计算值的广播数据包的功能。我以前从未创建过程序库，而且我也不太擅长 C++，所以程序库可能写得不太理想。由于接收器滤波器输出端 I/Q 信号的采样频率为 50 kHz，因此在接收时，程序将对 50 kHz 频段进行审查。(与 NCO 产生的频率相差 ± 25 kHz）。

组装好的接收器看起来是这样的：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093025847.png" alt="image-20240311093025847" style="zoom:50%;" />

电阻器将变压器的中点连接到电路板的 3.3V 电压上，从而改善网络接收和传输。

在接收机组装完毕并编写了所有节目之后，发现灵敏度不够。即使在有源天线上，也只能接收到广播电台和业余无线电操作员大功率工作的信号。据我所知，这与 ADC 位数容量低有关。为了提高灵敏度，我不得不在晶体管 BF988（位于一个小金属盒内）上加装了一个放大器。放大器能够明显提高接收器的灵敏度。整个结构的外观：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093102483.png" alt="image-20240311093102483" style="zoom:50%;" />

电源提供 12 V 电压，为天线放大器供电，金属圆盒包含几个带通滤波器，可减少带外信号，从而改善接收效果。我注意到，在许多情况下，没有 DPF 也能接收信号。

现在来谈谈 KV 可以接收到的信号。尽管噪音水平相当高，但还是可以接收到相当多的信号，广播电台的信号可以很好地接收，业余电台的信号也可以很好地接收。HDSDR 程序中的信号接收示例（在 CQ WW DX Contest 期间接收）：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093131012.png" alt="image-20240311093131012" style="zoom:50%;" />

可以接收 WSPRnet 信号。[WSPRnet](http://wsprnet.org/drupal/) 是一个业余无线电信标网络，可自动相互交换短信息。信标的数据会自动发布到互联网上。在这种情况下，只要安装一个特殊程序，就可以解码接收到的信号并将其发送到网络上。您可以在网站上查看地图，了解信标在一定时间间隔内的连接情况。这就是我半天预约的结果：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093232411.png" alt="image-20240311093232411" style="zoom:50%;" />

WSPR 的一个重要特点是发射器的功率很小（小于 5 W），传输信号的带宽很窄，一条信息的持续时间很长（2 分钟）。由于解码器程序进行了数字处理，可以接收到非常微弱的信号。我能够接收到距离约 2000 千米的 100 毫瓦信标的信号。

无线电爱好者使用 JT65 工作。JT65 是无线电爱好者之间进行数字通信的协议之一。与 WSPR 一样，它使用小功率和长传输（1 分钟）。信息是自动接收的，因此您可以长时间打开接收机，然后看看您能接收到谁的信息。接收示例：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093307398.png" alt="image-20240311093307398" style="zoom:67%;" />

数字无线电广播（DRM）。一些广播电台以数字形式传输声音。在城市环境中接收此类信号并不容易--信号强度不够。收到一个电台：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240311093333653.png" alt="image-20240311093333653" style="zoom:67%;" />

还有许多其他无线电信号也值得接收。还有气象传真、RBU 精确时间站（在 66.6 千赫的奇异频率上）等。