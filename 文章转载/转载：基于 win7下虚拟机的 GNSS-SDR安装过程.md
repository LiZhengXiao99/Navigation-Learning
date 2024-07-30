> 原文链接：https://www.cnblogs.com/taqikema/p/9040387.html

最近在安装 GNSS-SDR软件时，遇到了很多问题，这里回顾了我的安装过程，罗列了所遇到的问题和解决办法。希望后来者不要再踩这些坑了！

1. 首先，在官方文档中看到，GNSS-SDR目前并不支持直接在 Windows系统下直接运行，通常需要借助虚拟机构建客户操作系统（Linux），在虚拟系统内安装并运行该软件。
2. 官网上给出的建议虚拟工具是（VirtualBox），于是我就去官网下载了最新版本的 VirtualBox5.2.12。具体安装和新建虚拟机的过程都算简单，在网上也能找到很多相关介绍，所以这里不再赘言。
3. Linux系统，这里我选用了 Ubuntu，一来是之前就经常听到这个名称，很好奇到底是个啥东西；二来是搜到的[教程](https://blog.csdn.net/u012732259/article/details/70172704)所安装的就是 Ubuntu。所以去官网上下载了最新的 18.04版本，这里要吐槽一下，也许是因为墙的原因，很多次都是卡在下载之前的那个网页上，就发生了“404”现象，很气！
4. 按照教程安装和配置好 Ubuntu之后，需要安装增强功能，在教程中去找那个光盘标志并执行弹出操作时，也许是因为教程中是16.04版本的问题，反正我就是找不到光盘标志，于是作罢。
5. 命令行执行“sudo apt-get install gnss-sdr”，真正准备安装 GNSS-SDR了，结果报错“can not locate passage”。于是去百度解决方法，才知道默认 Ubuntu的下载源都在国外，速度很慢，需要切换国内的镜像源。教程也很多，这里给出一个看起来简单一点的，[切换阿里云](https://www.cnblogs.com/BlueMountain-HaggenDazs/p/6600967.html)。切换完软件源之后，再执行“sudo apt-get install gnss-sdr”时，就不会再报这个错误了。
6. 接下来要说的这个问题是本次安装过程中最恶心的问题了，但可能只是此次我遇到了，也许并不具有普遍性。就是所安装的 VirtualBox 5.2.12 + Ubuntu 18.04在虚拟系统开机、鼠标点击或执行操作时，经常会发生电脑蓝屏、卡死的现象。卡死了，只能强制关机，可笑的是强制关机或蓝屏后虚拟机内所进行的配置（源设置、下载的 GNSS-SDR）都没了，前前后后发生了十多次，我终于是不能忍，于是决定使用 VMWare，并且安装较低版本的 Ubuntu 16.04。
7. 按照[安装教程](https://jingyan.baidu.com/article/c275f6ba07e269e33d756714.html)，在[该网站](https://www.cr173.com/soft/177242.html)下载了 VMware Workstation Pro 12，又在官网上下载了 Ubuntu 16.04，执行安装操作。总体安装过程其实与 VirtualBox 类似，大多都是直接执行“下一步”。
8. 安装完虚拟 Ubuntu系统后，第一件事就是安装 VMware Tools，可以很方便的使虚拟系统全屏和进行宿主机与客户机的文件交换。安装 VMware Tools，我主要参考了[这篇文章](https://www.linuxidc.com/Linux/2016-04/130807.htm)。安装完 VMware Tools后，令虚拟系统全屏，只需在 VMware的 “查看”菜单栏里点击“立即适应客户机”就可以了。然后就是设置共享文件夹，这个还是很有必要的，通过虚拟机进行操作所得到的文件通过这种方式也能够被宿主机使用。具体过程参照[这里](https://blog.csdn.net/klq6743/article/details/78838080)。进行完以上设置后，别忘记还要切换软件源！
9. 接着在命令行执行“sudo apt-get install gnss-sdr”，这一次安装过程很顺利，没有像 VirtualBox那样会报“dpkg错误”或卡死的事情。测试是否成功安装 GNSS-SDR，命令行输入“GNSS-SDR --version”，结果显示的版本居然是“0.0.6”。官网上特别强调过，如果早于“0.0.9”的版本，对于所给出的测试用例，可能会有问题。没办法，只能卸载刚刚安装好的 GNSS-SDR，使用从源代码编译的方法来安装最新版本(0.0.9)的 GNSS-SDR了。
10. 具体过程就是参考[官网](https://gnss-sdr.org/build-and-install/)，一步步跟着操作就可以了。但在执行 “git clone https://github.com/gnss-sdr/gnss-sdr”时，下载速度特别慢，只有 5Kb/s，无法忍受。遂百度一下，发现这个问题很普遍，大家都会遇到，解决办法我用的是[这一个](http://www.bubuko.com/infodetail-1860413.html)，亲测可用。之后继续官网上给出的操作，就可以成功安装 GNSS-SDR了。使用 “GNSS-SDR --version”检测时，版本号也是最新的 “0.0.9”了，至此 GNSS-SDR的安装过程算是完成了。
11. 下载测试数据，文件有 1.6G，还是挺大的，不知道是不是国外资源的问题，下载起来速度并不快。
12. 之后就是制作配置文件，这里有几点需要格外注意。一、“GNSS-SDR.internal_fs_hz”应换成 “GNSS-SDR.internal_fs_sps”  二、SignalSource.filename那一项需要填写自己系统内数据文件的实际存放位置  三、`Observables.implementation那一项中的 “GPS_L1_CA_Obserables”需要替换成“Hybrid_Observables”  四、PVT.implementation那一项的 “GPS_LA_CA_PVT”需要替换成“RTKLIB_PVT”。这样的话，配置文件也制作完了。`
13. 转到数据文件所在目录下，执行“`gnss-sdr --config_file=./my-first-GNSS-SDR-receiver.conf"操作，即可顺利运行，并生成相应的文件。`
14. 官网上 “Configuration”部分主要是介绍数据源是硬件芯片时的配置，手头并没有相应芯片，所以这一节就跳过没看了。出于好奇，查了一下 “HackRF”芯片的价格，居然要 300美元，看来不是能随便玩的东西了。。。另外，还查了下制作团队的信息，发现他们都属于 "CTTC"。百度后，得知全称为“Center Tecnologic Telecommunications Catalunya”，翻译成中文就是“加泰罗尼亚电信科技中心”，怪不得下载到的数据文件是在 Spain测到的。不过很羡慕他们，平时做研究，周末可以去诺坎普看球呀。。。

好了，至此，本次按照 GNSS-SDR官网教程，安装和测试基本用例的过程就记录完毕了。