# Raspberry Pi based PPK and RTK solutions with RTKLIB

[TOC]

距离我上一篇关于如何在树莓派（Raspberry PI）上运行 RTKLIB 的文章已经过去六年多了，现在是时候更新了。在上一篇文章中，我介绍了如何使用 Pi Zero 作为 u-blox M8N 用于 PPK 解决方案的数据记录器。在这篇文章中，我将使用 Pi Model 4 和 u-blox M8T 来演示 PPK 解决方案的日志记录和实时 RTK 解决方案。好消息是这次不需要焊接，因为我们将使用 Pi 上的 USB 端口来连接接收器。这些说明适用于任何支持原始观测数据的 u-blox 接收机和任何带有外设 USB 端口的 Pi 型号。只需稍作修改，即可用于任何具有 USB 或 UART 端口并支持原始观测数据的接收器。

这是组装好的设备的图片。中间是 Pi，顶部是 u-blox M8T 接收器。我们将使用无线连接从外部电脑与 Pi 通信，因此不需要键盘或显示器。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-6.png)

## 步骤一：配置树莓派

第一步是将 Pi 配置为 "headless "模式，这样我们就可以通过外部计算机与它通信。这在这篇[文章](https://picockpit.com/raspberry-pi/headless-setup-fro-raspberry-pi/)中解释得很清楚，也很直接，所以我就不在这里描述如何操作了。本练习只需进行帖子中的第 1 和第 2 步。如果您打算将其用于 RTK 解决方案，请注意，Pi 将依靠与互联网的无线连接来进行基站观测。这意味着，如果您不想仅限于在家里的无线路由器范围内使用它，那么您可能需要用手机连接到热点。如果您只想为 PPK 解决方案收集数据，那么这并不重要。

完成上述第 1 和第 2 步后，您应该已经打开 Putty 窗口并登录了 Pi。下一步是编译并安装 RTKLIB 代码。下面的命令将克隆 Github 代码库中的 RTKLIB 代码，编译流服务器应用程序 (str2str) 和 RTK 解决方案应用程序 (rtkrcv)，并将可执行文件复制到可以从任何目录访问的位置。

```bash
> sudo apt update
> sudo apt install git
> mkdir rtklib
> cd rtklib
> git clone https://github.com/rtklibexplorer/RTKLIB.git
> cd RTKLIB/app/consapp/str2str/gcc
> make
> sudo cp str2str /usr/local/bin/str2str
> cd ../../rtkrcv/gcc
> make
> sudo cp rtkrcv /usr/local/bin/rtkrcv
> cd ../../../../..
```

## 步骤二：配置接收机

在将 u-blox 接收机连接到 Pi 之前，我们需要对其进行配置，以输出原始的观测和导航信息。最简单的方法是在电脑上使用 u-center 应用程序，该程序可从 u-blox 网站下载。用 USB 电缆将接收器连接到电脑，启动 u-center，然后使用 "接收器 "选项卡中的 "连接 "选项连接接收器，如下图所示。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-1.png)

接下来，使用 "Messages View" 菜单中的 "View"窗口启用 RAWX 和 SFRBX 信息，如下图所示。在信息视图中，您还可以禁用任何不必要的 NMEA 信息，以节省通信带宽。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-3.png)

接下来，我们将切换到 "配置视图 "窗口，配置其他所需的设置，然后将其保存到闪存中。我建议使用 "GNSS "命令确认所有星座都已启用，并使用 "RATE "命令将采样率设置为所需值。我通常将其设置为 5 Hz。如果不使用 UART 端口，我还建议使用 "PORT "命令禁用这两个端口。如果波特率设置过低，将限制包括 USB 端口在内的所有端口的带宽，即使这些端口未连接任何设备。最后，使用 "CFG "命令将设置保存到闪存中，如下图所示。

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-5.png)

## 步骤三：验证数据流

接下来，我们将确认从流动站接收器接收到数据，如果运行实时解决方案，还将从基地接收器接收到数据。这一步并不是绝对必要的，但它可以验证我们在将所有部件组装在一起之前，各个部件是否都在工作，还可以让我们练习使用 RTKLIB str2str 命令。

断开流动站接收机与电脑的连接，使用 USB 电缆将其连接到 Pi，如本帖顶部图片所示。在 Putty 控制台输入以下命令，创建新文件夹并运行流媒体服务器。这将连接到 Pi 的 USB 端口。如果使用的是 UART 端口，则需要使用相应的端口名称。

```bash
> mkdir data
> cd data
> str2str -in serial://ttyACM0
```

现在接收器的输出应该会在 Putty 控制台屏幕上滚动显示。如果启用了 NMEA 信息，您应该可以看到这些信息与二进制信息中的随机字符混合在一起。确认数据流后，点击 Control C 停止数据流。

如果我们想记录 PPK 解决方案的接收机输出，只需在前一条命令中添加一个文件名，即可将屏幕上的数据流重定向到文件中。下面的命令就可以做到这一点，使用文件名中的关键字创建一个包含当前月、日、时、分的文件名。

```bash
> str2str -in serial://ttyACM0 -out rover_%m%d_%h%M.ubx
```

下图显示了两条命令的预期输出结果。

![image-20240107145142835](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240107145142835.png)

如果您只是使用 Pi 来记录接收器数据，那么此时您的工作就完成了，除非您想配置 Pi，使其在开启时自动开始收集数据。有几种方法可以做到这一点，本帖将一一介绍。修改 rc.local 文件是最简单的方法。

对于那些希望运行 RTK 解决方案而非仅为 PPK 解决方案提供日志数据的用户，下一步是确认基础数据流。我们将再次使用 "str2str "命令，但这次我们将使用以下格式指定输入为 NTRIP 数据流：

```bash
“ntrip://username:password@ipaddress:port/mountpoint”
```

我的命令如下（去掉了用户名和密码）：

```bash
> str2str -in ntrip://username:password@rtgpsout.unavco.org:2101/P041_RTCM3: -out temp.log
```

如果一切运行正常，你应该会看到非零的传输数据，并且不会出现错误，如上例所示，在这种情况下，你可以再次使用 Control C 停止。

请注意，如果您的 NTRIP 提供商使用的是 VRS（虚拟参考站），情况就会稍微复杂一些。我们需要在 GGA 报文中发送本地位置。要做到这一点，您必须在配置接收器时启用 NEMA GGA 报文。要将这些 GGA 信息路由回 NTRIP 服务器，我们需要将流服务器输出连接到接收器，并使用"-b "选项启用回传功能。下面是我使用 VRS NTRIP 服务器连接测试的示例。

```bash
str2str -in ntrip://username:password@na.l1l2.skylark.swiftnav.com:2101/CRS -b 1 -out serial://ttyACM0
```

## 步骤四：进行 RTK 解算

好了，现在我们已经确认可以从基准站和移动站获取数据，是时候进行 RTK 解算。我们将使用 RTKLIB 中的 "rtkrcv"控制台应用程序来完成这项工作，我们在步骤 1 中安装了该应用程序。

我们需要一个 rtkrcv 配置文件。您可以使用 demo5 版本中的 "rtknavi_example.conf "文件作为起点，但需要编辑流配置设置。以下是我更改的设置，以及一些值得验证的重要设置，以确保您的配置正确无误。我将其配置为以 LLH 格式将输出写入文件。如果您希望以 NMEA 信息输出，可以将输出流 1 更改为 "nmea "格式，或者启用输出流 2 以同时获得文件和 NMEA 信息流。

```bash
pos1-posmode =kinematic  # (0:single,1:dgps,2:kin,3:static)
pos1-frequency =l1  # (1:l1,2:l1+l2,3:l1+l2+l5)
pos1-navsys =13  # (1:gps+2:sbas+4:glo+8:gal+16:qzs+32:comp)
pos2-armode =fix-and-hold # (0:off,1:cont,2:inst,3:fix-and-hold)
pos2-gloarmode =fix-and-hold # (0:off,1:on,2:autocal,3:fix-and-hold)
out-solformat =llh #    (0:llh,1:xyz,2:enu,3:nmea)
ant2-postype =rtcm # (0:llh,1:xyz,2:sing,3:file,4:rinex,5:rtcm)
inpstr1-type =serial (0:off,1:ser,2:file,3:,...,7:ntrip)
inpstr2-type =ntripcli # (0:off,1:ser,2:file,3:,...,7:ntrip)
inpstr1-path =ttyACM0
inpstr2-path =usrname:pwd@rtgpsout.unavco.org:2101/P041_RTCM3
inpstr1-format =ubx # (0:rtcm2,1:rtcm3, ...)
inpstr2-format =rtcm3 # (0:rtcm2,1:rtcm3,...)
inpstr2-nmeareq =single # (0:off,1:latlon,2:single)
outstr1-type =file # (0:off,1:serial,2:file, ...)
outstr2-type =off # (0:off,1:serial,2:file, ...)
outstr1-path =rtkrcv_%m%d_%h%M.pos
outstr2-path =
outstr1-format =llh # (0:llh,1:xyz,2:enu,3:nmea)
outstr2-format =nmea # (0:llh,1:xyz,2:enu,3:nmea)
logstr1-type =file # (0:off,1:serial,2:file, ...)
logstr2-type =file # (0:off,1:serial,2:file, ...)
logstr1-path =rover_%m%d_%h%M.ubx
logstr2-path =base_%m%d_%h%M.rtcm3
```

我喜欢使用 WinSCP 在 Pi 和外部电脑之间编辑和传输文件，但也有很多其他方法。编辑完成后，配置文件需要放在运行 rtkrcv 的当前文件夹中。在我的例子中，我将其重命名为 "rtkrcv_pi.conf"。

要使用名为 "rtkrcv_pi.conf "的配置文件运行 rtkrcv，请使用以下命令：

```bash
> rtkrcv -s -o rtkrcv_pi.conf
  >> status  1
```

If all is well, you should see a status screen updated every second that looks something like this：

![image-20240107150121921](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240107150121921.png)

我更改了 Putty 显示默认值，使其更容易阅读。我还用黄色标出了一些数字，以便检查它们是否看起来正常。确保您看到的是 RTCM 基本位置信息（通常为 1005）。如果想更详细地检查输入流，可以使用 control c 退出状态菜单，然后输入"? "查看其他一些 rtkrcv 命令。要退出 rtkrcv，请使用 "shutdown "命令。

如果所有输入看起来都很好，但解决方案却不起作用，而且原因不明，则可以在命令行中加入"-t 3"，重新运行 rtkrcv。这将启用跟踪模式，从而创建一个跟踪调试文件，该文件可能会为找出问题所在提供线索。

这样就可以开始使用了。要了解更多配置选项，请参阅 RTKLIB 用户手册附录 A 中的 str2str 和 rtkrcv 部分。