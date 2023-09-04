> https://gogps-project.github.io/wiki/

[TOC]

**goGPS**是一个处理GNSS原始数据的软件，最初支持单频低成本GPS接收机数据，但现在也可以用来处理多频多系统GNSS数据。它实现了多种算法来解算，目前包括两个主要的最小二乘法（LS）引擎：一个基于于组合观测数据（例如无电离层观测）；另一个能够使用所有的频率和记录的信号数据，而不进行任何组合（电离层延迟是正常方程的参数）。组合和不组合的引擎都支持PPP、NET解算。

**注意**：目前只支持静止测站的解算，还不能动态解算。

## 一、软件历史

那是2007年，在米兰理工大学--科莫校区，最初有一套用来教学生GPS数据处理和 Kalman 滤波的程序。当时Mirko Reguzzoni 和 Eugenio Realini 觉得，基于MATLAB语言的新的GPS处理软件可能有发展空间。在一些学生（创始者）的帮助下，他们开始写一套利用 Kalman 滤波的完整软件。goGPS 的第一个版本可以追溯到2009年，当时代码在 GPL 下发布并上传到 SourceForge 平台。之后该软件开始在大学里被使用，并开始收到来自世界不同地区的贡献。同时， Eugenio 在通过 goGPS 的博士论文答辩后，前往日本从事对流层研究的博士后工作，在那里他编写了 PPP 解算的大部分代码。2014年，Eugenio 回到意大利，与他在 Politecnico 的一些前同事一起成立了一个名为 GReD 的子公司，并发布了第一个版本，用 Kalman 滤波实现了 PPP。

![goGPS_history](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_History.png)

当我在 2016 年底开始为 goGPS 工作时，该软件使用了一个固定的数据结构，限制了最多两个频率的使用，而且还没有为现代星座做好准备。要扩大它或修改处理顺序是非常复杂的；开始时是为了提高性能，但很快就变成了对软件的全面重写。 我决定使用**面向对象**的编程方法，以获得更大的灵活性和模块化，但 **Kalman 滤波仍然是一个限制性的性能因素**（MATLAB在进行迭代处理方面是出了名的慢）。Eugenio 测试了基于**批处理最小二乘**引擎的差分组合数据处理，结果相当好，所以当 Giulio Tagliaferro 加入 GReD 团队，我们必须开发一个新的引擎时，他提出并编写了第一个非差的批处理程序，这仍然是 goGP S预处理的核心。MATLAB 擅长处理矩阵，新的引擎比旧的 KF 引擎快得多，但我们一直在做准静态站的后处理，我们对它们的处理进行了优化，并且现在我们**不再支持移动接收机**。2018年，我离开 Politecnico，加入了 GReD ；从那时起，目前 99% 的发展是由我们的子公司支持的。在过去的一年里，Giulio 和我负责了大部分的开发工作，改进了软件，现在与原始版本相同的代码行数有所减少。带 KF 的最后一个版本是0.5.x，现在可以在遗留分支找到。新的 1.0 版本是一个全新的软件，有两个引擎（组合和非组合），能够进行 PPP 和 NET，可以很容易地扩展到其它算法，成为测试新算法的基础。

## 二、环境要求

goGPS 在 MATLAB 环境下开发，为避免编译和额外的要求，没有使用任何付费工具箱和mex函数。

### 1、最低版本软件要求

* **MATLAB 2016a**：由于使用了一些新函数如：`movmedian`、`movstd`
* **GUI Layout Toolbox**：由于有图形用户界面

### 2、额外要求

* **LAMBDA v3**：用于整周模糊度固定，可以去[网站](http://saegnss2.curtin.edu.au/gnssweb/index.php?request=getlambda)上下载
* **aria2**：为了加快自动下载速度，goGPS软件包中包含了一个适用于Windows的编译版本。
* **Google Maps API**：、谷歌地图用户 API 需要编译成一个 `.mat`文件`<install_dir>/goGPS/utility/thirdParty/plotGoogleMap/api_key.mat`，这个 MATLAB 文件里放入你的 API KEY，如果你没有 API KEY，可以在[此](https://developers.google.com/maps/documentation/javascript/get-api-key)请求。

### 3、goGPS中包含的第三方功能

在goGPS/utility/thirdParty文件夹中可以找到一整套额外的第三方功能，它们与本软件一起被重新分发，并附有许可证。

非常感谢所有的开发者为整个社区分享他们的工作!

## 三、软件安装

获取 goGPS 的最佳方式就是去 [GitHub](https://github.com/goGPS-Project) 上下载，上面有最新的版本，当用户发现并在GitHub上报告任何错误时，我们会尽可能快地修复它。

1. 在你的文件夹中克隆 repo，可以通过 `git clone` 命令来完成，或使用你喜欢的Git客户端（如Github Desktop、GitKraken、SourceTree，...）

   ```shell
   git clone https://github.com/goGPS-Project/goGPS_MATLAB.git goGPS_MATLAB
   ```

2. 安装 [GUI Layout Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/47982-gui-layout-toolbox) 。它将在第一次执行 goGPS GUI 时被自动提示。

3. 请求一份 LAMBDA v3 用于模糊度固定，放在文件夹：

   ```\
   <install_dir>/goGPS/positioning/lambda/lambda_v3
   ```

4. 下载并安装 [aria2](https://aria2.github.io/)  以加快下载速度

   * **Windows**: goGPS 中已经包含了aria2 的版本，无需再下载
   * **Linux** (Debian based)：`sudo apt-get install aria2`
   * **Mac OS** (Using [brew](https://brew.sh/))： `brew install aria2`

## 四、软件目录

当从GitHub下载仓库的克隆时，一组文件夹会出现在安装目录下。

* `docs` - 包含一些对 goGPS 开发有用的文件集。
* `icons` - 含记录窗口所使用的图标集。
* `goGPS` - 包含 goGPS 代码的主文件夹
* `data` - 存放项目和资源的文件夹。

关于goGPS的dir结构的更详细描述，可以在这个 nfo 文件中阅读：[dir_structure.nfo](https://github.com/goGPS-Project/goGPS_MATLAB/blob/goGPS_1.0_beta/docs/DIR_STRUCTURE.nfo)

```
goGPS_MATLAB
    │
    ├─ DOCS                            // Collection of some files useful for the development of goGPS
    │
    ├─ goGPS                           // Main folder containing the goGPS code
    │   ├─ last_settings.ini           // Store the latest setting in use
    │   ├  ...
    │   └  ...
    │
    ├─ ICONS                           // Icon set for the log window
    │
    └─ DATA
        │
        ├─ Antenna
        │   ├─ ATX                     // ANTEX file
        │   └─ MP                      // Multipath maps
        │
        ├─ Satellite
        │   ├─ CLK                     // clk files
        │   ├─ EPH                     // Ephemeris files (e.g. igs SP3 files)
        │   ├─ CRX                     // CRX files storing Satellite problems
        │   ├─ ATM                     // Atmospheric error grids / models
        │   ├─ SBAS                    // Downloaded SBAS data
        │   │   └─ EMS                 // EMS files from the EGNOS Message Server, via Internet
        │   └─ DCB                     // Differential code biases
        │
        ├─ Reference
        │   ├─ Geoid                   // Geoids for orthometric correction
        │   └─ DTM                     // DTM for reference altitude
        │
        ├─ Station
        │   ├─ station_info.ini        // Info of the station
        │   ├─ network_info.ini        // Info of the network (FTP info)
        │   ├─ < MET >                 // Meteorological data files
        │   ├─ < CRD >                 // Permanent station positions
        │   ├─ < SNX >                 // Sinex files, global solutions
        │   └─ < Ocean >               // Computed ocean loading
        │
        └─ Project
            └─ TestName                // Storing a project
                ├─ Config
                │   └─ config.ini      // config file of the project
                │                      // (storing ALL the goGPS parameters)
                │
                ├─ RINEX               // All the RINEX files of the project
                │   └─ < Year >        // Year corresponding to GPS time
                │       └─ DOY         // DOY corresponding to GPS time
                │
                ├─ < Ref_path >        // <opt> reference paths
                │
                ├─ < Antenna >         // <opt> use the global dir if not present
                │   ├─ < ATX >         // <opt> use the global dir if not present
                │   │   └─ < custom >  // <opt> put custom .ATX here
                │   └─ <MP>           // Multipath maps
                │
                ├─ < Stations >        // <opt> use the global dir if not present
                │   ├─ < CRD >         // <opt> use the global dir if not present
                │   ├─ < MET >         // <opt> Meteorological data files
                │   └─ < Ocean >       // <opt> use the global dir if not present
                │
                └ Out
```

## 五、软件执行

要使用 goGPS，你只需要移动到 goGPS 文件夹并执行命令 `goGPS`

1. 移动到 goGPS 文件夹：`cd <install_dir>/goGPS`
2. 执行命令：`goGPS`

如果一切正常，你的屏幕上应该出现类似这样的图形用户界面：

![goGPS Main Window](https://gogps-project.github.io/wiki/images/goGPS_MainWindow.png?raw=true) 

## 六、软件配置

goGPS 项目的设置存储在 `.ini` 文件中。它被完全映射到一个 `Main_Settings` 类的对象实例中，并会自动导出所有必要的注释，以了解参数的含义。`.ini` 文件可以用普通的文本编辑器进行编辑，并传递给 goGPS 自动运行。你也可以使用 goGPS 编辑 GUI 来修改 99% 的设置，该 GUI 将在 goGPS 启动时自动加载，其余 1% 的设置可以在界面的高级选项卡中与纯文本 `ini` 文件一起找到。

以下各节，将对当前 goGPS 编辑设置界面中存在的所有元素以及设置文件中存在的相应参数进行了说明。

goGPS 编辑界面的结构包括一个菜单、一个主侧边栏、一系列标签和底部的栏：

![goGPS Advanced Tab](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Advanced.png)

### 1、菜单

goGPS菜单允许执行简单的操作和打开工具或其他窗口。它仍在开发中，并将在未来进行扩展。目前，它可以进行以下操作：

* **goGPS** 
  * **About**：打开 "关于 "窗口
* **Options**：包含一组所有设置的组合修改器，以执行基本处理。
  * 设置 PPP 解算 对流层估计
  * 设置 NET 解算（短基线 - 忽略电离层 - 忽略对流层）
  * 设置 NET 解算（中等基线<20km - 忽略电离层）
  * 设置 NET 解算（长基线-消电离层组合）
* **Project** 
  * **New**：打开一个窗口，创建一个新的项目。自动程序将指导用户生成一个具有主要结构的项目文件夹和一个新的通用设置文件。
  * **Load**：加载 `.ini` 配置文件。
  * **Save**：保存当前 `.ini` 文件上的当前配置，当前文件显示在界面底部的栏中。
  * **Save As**：在指定定的位置保存当前配置。

### 2、侧边栏

在 goGPS 编辑界面的左侧，有一个侧边栏，包含了关于将可用于处理的数据的基本信息。

在顶部有一个专门用于显示处理会话的部分，它包含：

* 检查（**check**）按钮，以便在会话限制没有自动更新的情况下更新它们。
* 要处理的第一个（First）和最后一个（Last）纪元。
* 每个单一会话的持续时间（Duration)。
* 缓冲区（Buffer）的尺寸，以秒为单位，除了其中一个会话之外，还要使用数据间隔。(例如，一个会话为86400秒--一天，缓冲区为10800秒--3小时，每个会话将总共处理30小时的数据，每个会话将与下一个会话重叠，从而使对流层参数的估计更加平滑，弧线模糊度的估计也更加准确)。

在侧边栏的底部有一个专门用于接收机列表的部分。在那里你可以找到3个按钮：

* **Check**：刷新接收者列表。

  **Plot**：生成一个显示每个RINEX文件所覆盖的时期的图，这些文件将被用于项目。

  **Trackings**： 生成一个显示所有被信号的代码观察类型的表格（在RINEX 3文件上效果最好，这个工具仍在试验阶段）。

下面是该项目所有站点的列表。对于每个站点，有两栏报告了所要求的文件的存在或缺失情况（例如，在处理1个月的每日文件时，一些RINEX可能缺失）。

### 3、选项卡

在侧边栏的右边，一些标签包含了可以定义的各种全局设置。

#### 1.Advanced 选项卡

这个标签用于改变所有的设置，包括隐藏的设置。

![goGPS Advanced Tab](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Advanced.png) 

在编辑器中，有一个当前 `ini` 配置文件的副本，可以直接修改它。当 `ini` 文件被修改时，两个按钮被用来刷新用户界面。

通信目录（Communication dir）是 goGPS 并行主义用来与从属处理器通信的目录。这个文件夹必须能被所有的从属进程和主进程（也就是运行 goGPS 主程序的 MATLAB 进程）访问。

#### 2.Resources 选项卡

资源选项卡包含所有定义需要使用的轨道种类的设置，以及所有可下载资源的位置。注意：Antenna、Geoid 和 Geomagnetic reference 等文件与 goGPS 的代码一起提供，并在 GitHub 资源库中不断更新，而所有其他资源都由 goGPS 自动下载并保存在指定的文件夹中（如果下载标志被选中）。

![goGPS Resources](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Resources.png) 

在这个选项卡中，有许多命令和字段，在这里可以看到的第一个信息是远程_资源 `.ini` 文件的位置，可以在选项卡的顶部找到。这个资源文件包含了寻找和下载 goGPS 资源（如轨道、时钟误差、Vienna  函数）所需的所有信息，以及它们的名称。一般来说，这个文件决不能被修改，关于这个文件的更详细的指南将被添加到这个 wiki 中。要使用 goGPS centers 列表中没有的中心的轨道或其他资源，必须手动修改 `remote_resource.ini` 文件（如果你有一个 goGPS centers 的建议，并希望它被添加到 `remote_resource` 文件中，请在 GitHub 上 ticket，我们会尽快添加它）。

**命令/字段的清单**：

* **Allow automatic download [...]**：这个命令是一个标志，如果启用，它允许goGPS在网上搜索它所需要的资源（总是推荐）。
* **Center**：弹出列表 - 这个控件允许选择要使用的资源/供应商的种类，主要的中心已经 goGPS 中可用：IGS、EMR、ESOC、JAXA、CODE...。请注意，BNC 实时轨道不能由软件自动下载，它们是由 BNC 软件从 IGS 流中保存的，而且需要已经存在于磁盘上。在命令的左边，支持的星座列表被报告给主动选择。例如，选择 JAXA，支持的星座是 "GRJ"，每个字母代表一个星座：
  * **G** GPS (US)
  * **R** GLONASS (Russia)
  * **E** Galileo (EU)
  * **J** QZSS (Japan)
  * **C** Beidou (China)
  * **I** IRNSS (India)
* **Center orbit type preference**：包含一组标志，用于指定要使用的轨道的种类。如果有更多的选择，最左边的资源是主要的选择，其它为候选；当一种类型的资源（如最终轨道）不存在时（即使在远程服务器上），将使用候选产品资源（如快速轨道）。
* **Center iono type preference**：与精密轨道产品相似，不同类型的电离层模型从左到右优先级依次降低。
* **Resource tree inspector**：这个字段不能被修改，它只是显示所有需要的资源的远程位置。它所显示的所有信息都是在运行时从 `remote_resource.ini` 文件中建立的。
* **Reset all resources path**：这个按钮在从另一台电脑导入项目时非常有用，在设置文件中，资源的位置可能被表达为一个完整的路径，它可能在导入的电脑中不再存在。按下这个按钮，所有的资源路径将被重置为它们的默认值。在这个选项卡中显示的绿色/红色标志对于理解是否缺少一个文件/文件夹很有用。

在该按钮的下面是所有资源的本地路径列表。

* 在路径中可以使用特殊的时间关键词，它们将自动替换为对应的值：
  * `${WWWW}`：4 char GPS week
  * `${WWWWD}`：4+1 char GPS week + day of the week
  * `${D}`：1 char day of the week
  * `${3H}`：2 char GPS hour (00, 03, 06, 09, 12, 15, 18, 21)
  * `${6H}`：2 char GPS hour (00, 06, 12, 18)
  * `${HH}`：2 char GPS hour
  * `${QQ}`：2 char GPS quarter of hour (00, 15, 30, 45)
  * `${5M}`：2 char GPS five minutes (05, 10, ... , 55)
  * `${YYDOY}`：2+3 char GPS year + day of year
  * `${YYYY}`：4 char GPS year
  * `${YY}`：2 char GPS year
  * `${MM}`：2 char GPS month
  * `${DD}`：2 char GPS day
  * `${DOY}`：3 char GPS day of the year
  * `${S}`：1 char session
* **Antenna (ATX) filename**：在这里，用户必须设置指向天线文件的路径（通常是最后一个可用的 IGS 文件）。该文件包含卫星和接收机天线的相位中心偏移（PCO）和相位中心变化（PCV）的信息。这是一个必须的文件，没有它，计算解决方案的误差会非常大。如果没有找到关于频率的信息，将使用与最接近的频率对应的数据。如果没有找到关于星座的信息，将使用与频率最近的GPS对应的数据。注意：带有自定义值的额外天线文件（ANTEX格式）可以传递给 goGPS，为了加载它们，必须将它们保存在你正在运行的项目主页中的一个文件夹里，这个文件夹是：`<prj_home>/station/ATX/custom/`，，该文件可以有任何名称，但其扩展名必须是：`.ATX`
* **Geoid local path**：在这一行中，用户必须设置用于计算正射高度的大地水准面文件（MATLAB `.mat`格式）的位置，注意，在这里传递准大地水准面的网格将导致使用正常高度。对于99.9%的处理，EGM2008 的 goGPS 网格是绰绰有余的；当一个点不在网格的确切结点上时，会进行双线性（或 cubic）插值。
* **CRX path**：这是包含 CRX 文件（卫星问题文件）的文件夹的路径，这些文件是自动从 CODE FTP 服务器下载的，它们包含卫星状态的有用信息（机动、中断和其他异常行为）。
* **Eph local dir**：轨道星历表是从它们的标准交换格式 SP3 加载的，这是它们的存储位置，轨道的名称是根据所选择的 "中心轨道类型 "自动生成的。轨道会被 goGPS 自动下载到正确的位置，但也可以手动把它们放到本地位置。
* **Clk local dir**：与星历路径类似，这是相应的轨道时钟文件的路径。有些轨道有不同的时钟文件（如@30秒，@5秒），它们是按照 `remote_resource` 文件中列出的优先顺序选择的。一般来说，这个路径也不能改变，goGPS 会尝试自己下载它所需要的资源。
* **ERP local dir**：这是包含地球自转参数文件的文件夹，要和它们相应的精密轨道文件一起使用。
* **IONO local path**：指向 goGPS 将下载电离层模型的本地文件夹。它们可以用来减少电离层的延迟，以便更好地进行单频定位，或者只是帮助收敛和改进插值技术。
* **IGRF local path**：这是指向国际地磁参考场 Schmidt 半归一化球面谐波系数的文件夹，通常用户不需要改变这个文件或文件夹，因为它们与goGPS 代码一起提供。
* **DCB local path**：指向goGPS将下载差分码偏差的本地文件夹。goGPS 会自动使用差分码偏差，如果有的话。
* **VMF local path**：指向 goGPS 自动下载的 VMF 投影映射本地文件夹。
* **ATM local path**：指向 goGPS 自动下载 Atmospheric Loading 文件的本地文件夹。

#### 4.Data Sources 选项卡

在这个选项卡中，用户定义了观测文件的位置、处理周期以及会话的大小。

![goGPS Data Sources](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_DataSource.png) 

##### Project

**Project home directory** 这个字段指向项目根目录`.`。包含项目的所有文件夹的文件夹。需要注意的是，在配置中可以使用的所有相对路径都是从这个文件夹开始的，这使得项目具有可移植性，只要在 `.ini` 文件中把所有其他资源路径写成相对路径，所有东西都被移到一起。GUI 会自动将全路径转换为相对路径。

##### Sessions

在会话面板中，可以定义要处理的历时限制，用于定义路径的时间关键词（`${WWWW}`、`${DOY}`、`${YYYY}`...）将根据当前会话进行相应的替换。goGPS 以相对复杂但灵活的方式管理一组 RINEX 文件的处理。用户通过选择开始和停止日期来定义 goGPS 能够解算的时期，这将是项目的最大尺寸，无论它是否将被完全处理。处理的历时数将取决于其他参数：会议的 for 循环和会议本身的持续时间。如果选择了两周的时间（开始停止限制），用户可以运行 14 个会话，每个会话的维度为 86400s（每天）。(注意：即使观测值被存储到每小时的 RINEX 文件中，也可以处理每日的会话，反之亦然。）

* **Start**：设置解算的开始时间。

* **Stop**：设置解算的结束时间。

* **Session duration**：这对于限制批处理的历时数，或者以更高的速率获得解决方案是很有用的。请注意，在这里插入一个过大的数字将导致产生一个巨大的矩阵，调整可能会使用所有的空闲RAM，并需要永远完成。最终LS调整的尺寸将取决于所使用的可观测的数量（星座、信号......）和数据的速率。经常性的会话持续时间为：：

  * 86400 seconds：将 24 小时的数据全部处理在一个的 LS 解算中。
  * 3600 seconds：每个小时的数据分开算

* **Buffers**：缓冲区是 goGPS 的一个几乎独特的功能，它们对于限制边界效应非常有用。缓冲区中的历时将与联合 LS 解算方案一致，但它们不会影响会话的尺寸。只有在会话中估计的参数才会在输出接收器对象中输出（缓冲区从未被考虑）。

  例如，[14400, 14400]秒--意味着每个缓冲区为3小时，如果会话为86400秒，这将使goGPS在一个独特的解决方案中总共处理30小时的数据，每个会话将与下一个会话重叠，总共6小时。

* **Smooth troposphere at boundaries**：这是一个标志，只在缓冲区设置不同于0时使用。当设置时，在与另一个会话重叠时估计的对流层参数将被合并，从而使变换会话时有一个更平滑的解。

* **Separate coordinates at boundaries**：设置后，缓冲区将用于对流层参数和相位模糊的估计，但不用于位置估计。将为两个缓冲区中的每一个估计一组单独的坐标，这将改善模糊性估计并限制边界效应，但不应该影响（只是最小的）所定义的会议的估计位置。

* **RINEX based session**：这个标志强制要求会话的维度与RINEX观测文件的维度完全一致（这个维度在所有文件中应该是同样的）。

* **Session character list**：RINEX2标准命名惯例使用一个字符来定义会话，在这些字段中，可以定义要使用的字符。通常，这些只是 `["0", "0", "0"]`，用于每日文件，`["abcdefghijklmnopqrstuvwx", "a", "x"]`用于每小时文件。

##### Stations

* **Observation directory**：这是包含所有观测文件的根文件夹，下一栏中的相对路径从这里开始。当选择一个包含观测文件的文件夹时，goGPS 会尝试将其中的所有站点加入到处理中。
* **Observation files**：包含要处理的站点（接收机）的文件名称和相对路径。注意，goGPS 只接受 RINEX2 和 RINEX3 观测文件。
* **Recursive get marker names**：使用这个按钮可以从一个目录开始以递归的方式添加站点。

#### 5.Receiver Info 选项卡

在这里，将来有可能为每个台站设置单独的参数，以覆盖通用参数（如使用的信号、截止等），但目前只能提供坐标。在标签的底部，用户可以设置包含多路径地图文件的目录路径和海洋加载文件的位置。

![goGPS Receiver Info](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_RecInfo.png) 

* **CRD filename**：指向所有站点的坐标文件。这是这个选项卡中唯一一个在 `.ini` 设置文件中有相应匹配的字段，goGPS 在开始计算前会导入坐标文件，为此，任何未保存的对坐标的修改在处理过程中会被忽略。
* **Coordinate table**：这是一个包含从坐标文件加载的信息的表格，要修改这些信息，用户可以直接用文本编辑器或通过这个界面操作文件。目前，存储在坐标文件中的数据是：
  * 地心地固（ECEF）XYZ坐标
  * 标识着如何考虑这些坐标的 type：
    - **0**：将坐标作为一个粗略的先验位置（误差大于100米）。
    - **1**：使用坐标作为一个精确的先验位置。
    - **2**：将坐标视为固定的（它们将不会被goGPS估计）。
    - **3**：将坐标视为仅由预处理固定的坐标（除非检测到坐标不够好，否则不会由 goGPS 在预处理步骤中进行估计），这使得预处理和离群点检测更加快速和均匀。
  * 坐标的时间有效性限制
  * 坐标的速度是一个线性趋势。用于计算不同时间的坐标的参考纪元是 "起始 "纪元。
* **Clear All**：清除坐标表。
* **Add a line**：在坐标表中添加一个空行。
* **Remove selected**：从坐标表中删除带有选定单元格的一行。
* **Import from RINEX**：将尝试从 RINEX 文件头中读取 XYZ 值来填充坐标表，这些文件将在goGPS执行期间被读取。
* **Save**：将把表格中可视化的坐标的当前状态保存到当前坐标文件中。
* **Save as**：将保存坐标的当前状态，就像表格中要求的文件路径一样。
* **Save (Default)**：将把表格中显示的坐标的当前状态保存到 `<prj_home>/station/CRD/>` 文件夹中一个名为 `station.crd` 的文件。
* **Inspect Trackings**：生成一个显示所有被信号的代码观察类型的表格（在RINEX 3文件上效果最好，这个工具仍在试验阶段）。
* **ShowMap**：将显示在表格中的坐标处的站点的地图，并叠加谷歌卫星图像。这个工具对于快速浏览站点非常有用。
* **Multipath mitigation dir**：这是包含接收机特定多路径地图的目录。
* **Ocean loading filename**：这是一个不能自动下载的资源，与测站严格相关。资源选项卡几乎不需要修改，而这个字段通常在每次添加测站时都要检查，为此，它的位置正好在观测文件的定义之后。Ocean loading 被存储在一个扩展名为 `.blq` 的文件中，它们必须 [Chalmers website](http://holt.oso.chalmers.se/loading/)  使用潮汐模型 FES2004 生成。为了帮助用户索取这个缺失的潮汐文件，可以按下 "获取缺失的BLQ "按钮，它将打开一个包含代码的窗口，将其复制并粘贴到 [Chalmers website](http://holt.oso.chalmers.se/loading/) 。当他们通过电子邮件发送海洋潮汐系数时，用户应手动创建或将其附加到该字段所指的文件中。

![goGPS Ocean Loading Helper](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_OceanLoading.png)  

#### 6.Processing 选项卡

处理选项卡是 goGPS 最复杂的选项卡，因此，它需要有自己的描述页面。在这里，用户可以操作从数据选择到观测数据的过滤，再到最小二乘法调整的全部参数的定义等一切。分为 6 个子选项卡：

![goGPS Processing](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Processing.png) 

##### ①Data Selection 子选项卡

###### Constellation Selection 卫星系统和频率的选择

 这个选项卡用于启用和禁用一些卫星系统/频率。注意：

* 用户需要为启用的星座选择适当的轨道（例如，在选择各种系统时使用MGEX轨道）。
* 当用组合观测值模式处理时，只有最好的频率被用来消电离层组合。
* 请注意，由于 goGPS 没有自动处理 L5 的频间钟差，选择 GPS L5 可能会产生不良结果。
* 要同时使用两个以上的频率，必须使用非组合模式进行处理。
* 如果接收器中不存在某个频率/星座，goGPS 应该可以使用其余的观测数据。

###### Data exclusion 数据剔除

* **Min satellite per epoch**：最小卫星数，在此阈值下，该纪元将从处理中被丢弃（默认值为2）。

* **Data cut-off angle**：最低卫星高度角，

* **SNR absolute threshold**：最低信噪比，信噪比可以很好地了解接收器所登记的观测数据的质量，特别是它受到多径的严重影响。当这个指数低于某个值时，表明观测值（尤其是伪距）可能会下降，设置一个阈值可以让 goGPS 丢弃这些不良数据。更多信息见 `Receiver_Work_Space.m` 中的函数 `remUnderSnrThr`。

* **SNR scaled threshold**：不同的信号（如 GPS C1C、C1W 或 GLONASS C1P 和 C1C）和不同的频率有不同的信噪比比例。将阈值设置为 28dB 可能对一个信号来说是好的，但对另一个信号来说同样的值可能会删除太多的值。goGPS 试图将来自不同系统/频率/信号的所有信噪比重新调整为 C1C 的信噪比（或类似的信噪比），以便能够为所有观测点设置一个独特的信噪比限制，而不是为每个观测点设置一个不同的限制。一般来说，25-32dB 之间的值是好的。为了扩大信噪比，要对伪距噪声进行粗略的估计，用它们的合成版本减少数据，去除偏差，并在移动窗口上逐弧计算残差的标准偏差。在预处理中，低于阈值的伪距在最后一次估计仅有编码的解决方案之前从接收器中删除。更多信息见 `Receiver_Work_Space.m` 中的函数 `remUnderSnrThr`。在下图中，比较了 GLONASS C1 的 WTZZ（Wetzel）站的噪声水平，C1P 的噪声较低，但信噪比也较低，其信噪比按比例计算，在 C1C 的相同噪声水平下有相似的值。

  ![goGPS clock re-alignment](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/SNR_Scaling.png) 

* **Min arc length**：goGPS 将删除任何小于这个值的弧，这个数字是以 `epochs` 而不是秒来表示的。请记住，小圆弧的模糊性一般是不好估计的，在 30 秒的处理中，这个参数的合理值是 10-12 个历元。

##### ②Atmosphere 选项卡

 这个选项卡用于设置大气延迟改正。无论是否估计对流层，都需要这一信息。

![goGPS Atmosphere](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Atmosphere.png) 

###### Ionosphere options 电离层选项

* **Ionosphere a-priori Model**：在这个弹出式菜单中，用户可以选择要从观测值中删除的先验电离层模型的类型。一个好的模型有助于离群点的检测和预处理阶段。可用的选项有：
  * **1 - none**
  * **2 - Klobuchar**：标准的 Klobuchar 模型，它需要广播星历提供电离层参数。
  * **3 - IONEX**：这是可以应用的最精确的模型，它从 IONEX 文件中读取，取决于在资源标签中选择的首选类型，它可以是一个来自观测值的模型，也可以只是一个预测。

###### Tropospheric options 对流层选项

* **Mapping function gradients**：在这个弹出式菜单中，可以选择对流层投影映射函数，目前只有 3 个模型可用，其他模型可以在 `Receiver_Work_Space.getSlantMF` 中实现。
  * **GMF**：Global Mapping Function 
  * **VMF gridded**：Vienna Mapping Function  (在需要的时候会自动下载)
  * **Neil**：Niel Mapping Function 
* **Mapping function gradients**：除了用于估计天顶延迟的对流层映射函数外，还可以指定梯度的映射函数。
  * **1 - Chen and Herring**
  * **2 - MacMillan**
* **A-priori zenith delay**： 这是用于计算天顶对流层延迟的先验模型，有两个选项可供选择：
  * **1 - Saastamoinen from meteo data**： 这个选项使 goGPS 使用 Saastamoinen 模型，所使用的数据来源取决于 GUI 的下一个字段。
  * **2 - VMF gridded zenith delays**：这些是来 Vienna 投影映射网格化地图的天顶对流层延迟。
* **Meteo Data**：气象数据的来源
  * **1 - Standard Atmosphere**：标准大气模型，压力（1013.25mbar）、温度。（18℃elsius）、湿度（50%）
  * **2 - GPT**：全球气压温度模型。
  * **3 - MET files**：使用下面字段中指定的气象RINEX
* **MET files**：goGPS 可以使用气象学 RINEX 文件来导入气象站的温度、压力和湿度。它自动在正确的海拔高度和站点位置上插值数据（更多细节见 `Meteo_Data.getVMS`）。

#####③Tab Generic-Options

在这个选项卡中，用户可以选择修改预处理工作方式的选项。

![goGPS Pre-Processing](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_PrePro.png) 

* **Max code positioning error**：如果一个站的预处理返回的 $\sigma_0$（来自 LS 解算）大于该值，则认为该站有问题，预处理失败，PPP 或 NET 无法计算。
* **Max code observation error**：所有具有大于该值的上一步预处理的残差的伪距观测值被剔除除，并在没有它们的情况下进行新的 LS 解算（RAIM-FDA）。

##### ④Tab Generic-Options

该选项卡包含所有可能影响 PPP 或 NET 的参数，除了 LS 的参数。

![goGPS Generic Options](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_GenericOpt.png) 

###### Input manipulation 输入操作

在这一节中，可以修改接收机的观测数据和卫星钟差数据。

* **Trackings combination**：当选择这个标志时，来自同一频率的多个跟踪的载波相位观测值被合并，使用它们的近似噪声水平作为权重，跟踪偏差和不存在于所有跟踪中的周期滑移被估计和纠正（默认为 "on"）。更多细节见 `Receiver_Work_Space.m` 中的函数 `combinePhTrackings`。为了估计每个载波相位的噪声水平，然后估计权重，采用了以下算法：

  * 利用预处理最后计算出的位置，计算出接收机视线卫星的合成伪距`_spr_`。
  * 计算残差 `_ph_ - _spr_`。
  * 去除相对跟踪偏差。
  * 计算同一观测指标（如 L1C L1W）的每次跟踪的移动方差（滑动窗口 5 个历元）。
  * 计算高度角的方差的三次立方插值。
  * 将高度角插值的倒数作为权值。

* **Clock re-alignment**：许多卫星钟差产品在一天结束时有严重的跳动。这些跳动从几厘米到几米不等；选择这个标志，goGPS 将尝试补偿和纠正这些跳动（默认为 "off"）。这种影响可能是由于不同日期的时钟误差的独立估计造成的。更多细节见 `Core_Sky.m` 中的函数 `addClk` 。下面是这个跳跃的例子，被视为卫星时钟漂移中的一个尖峰（以米为单位）：

  ![goGPS clock re-alignment](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_clock_re_alignment.png)

  在格林尼治标准时间的午夜附近，当两组新旧星历相连接时，这个标志的效果会影响解决方案。去掉这个跳变可以改善轨道的平滑性，从而改善离群点检测阶段，否则可能会发现假阳性离群点。在下面的例子中，使用不同的轨道供应商，并使用 PPP 非组合 goGPS 算法处理ZIMM（Zimmerwald）站，我们可以看到 GFZ 和 CNES 的轨道与其他解算方法相比，在估计的 ZTD 方面产生较大的偏差：

  ![goGPS clock re-alignment off](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_u2_clock_re_alignment_off.png)

  当选择重新对齐 re-alignment 标志时，解算结果产生如下变化： 

  ![goGPS clock re-alignment on](https://gogps-project.github.io/wiki/images/goGPS_u2_clock_re_alignment_on.png?raw=true)

  用 GFZ 和 CNES 的轨道计算出的解决方案得到了改善，而其他一些解决方案现在则变得更糟。仔细使用这个标志，逐一检查这个调整是否改善了解决方案。 目前的校正算法非常简单，工作原理如下： 

  * 在文件更改的纪元左右侧各取 200 个纪元。
  * 计算两个数据子集的漂移值。
  * 如果跳跃大于标准差，则计算出修正值。
  * (修正）对两个子集进行线性插值，预测两边时钟的一个纪元，并对右边进行偏移，以尽量减少跳跃在网络解决方案中，卫星时钟的偏移不会导致任何问题，而在 PPP 中可能会影响弧线的模糊性估计，这种修正仍然是实验性的，需要额外的调查才能安全应用。

###### Range corrections 距离修正

这个选项卡用于设置精密定位所需的修正。所有的修正都在预处理阶段结束时应用。尽管所有这些修正对 NET 来说不是必须的，但应用它们可以更好地检测出离群点。在所有可用的修正中，"Atmospheric Loading  "和 "High Order Ionosphere "是影响较小的修正，可以安全地禁用。可以应用于 GNSS 观测的修正有：

* **Receiver PCO/PCV**：接收机相位中心偏差和变化，这种影响的大小取决于天线类型。
* **Solid Earth Tide**：由于地壳运动引起的潮汐（几十厘米）。
* **Pole Earth Tide**：由于地球自转轴的摆动而产生的潮汐（垂直方向可达2.5厘米-水平方向可达0.7厘米）。
* **Phase Wind Up**：由于圆极化波的电磁性质，只影响相位，它取决于卫星和接收机天线的相对方向（厘米级）。
* **Shapiro Delay**：相对论效应（小于2厘米）。
* **Ocean Loading**：由于海洋潮汐造成的海底变形和邻近陆地的表面位移（几厘米）。
* **Atmospheric Loading**：这种影响是由于大气压力的变化造成的。
* **High Order Ionosphere**：二三阶电离层延迟。
* **Use a-priori Iono Model**：模型计算的电离层的改正，这在电离层插值的情况下非常有用。
* **Multipath mitigation**：通过这个弹出式菜单，用户可以选择使用先前生成的多径图来缓解多径。可以选择基于 Zernike 多项式扩展的地图（更平滑）或恢复了更高频率的残差的版本，更多细节见命令部分多路径管理。



![MP final maps](https://gogps-project.github.io/wiki/images/MP_Final.png?raw=true) 

左边是比较 Zernike 多项式平滑的结果，右边是保留了最高细节的结果。

###### MPEST Options

本节专门介绍创建多路径地图的设置，更多信息请参见多路径管理命令部分。goGPS 创建了 6 个网格，每个网格都有自己优缺点。下面是创建这些网格的主要设置，将它们设置为零以避免它们的计算：

* **Regular grid [n x m]**：网格的尺寸单位是度。这是最简单的网格，在方位角和仰角有成本尺寸的单元。
* **Regular smoothed [n x m]**：网格的尺寸为度。对于第一个网格，空单元格中使用了额外的伪观测，最终结果被放大为 0.5 x 0.5 度的网格。
* **Congruent grid [n x m]**：网格的尺寸单位是度。这是一个全等的网格，意味着增加海拔高度，单元格的数量就会减少（它们在方位角上会变大）。然后，这个网格被缩放为一个常规网格。
* **Congruent smoothed grid [n x m]**：网格的尺寸单位是度。与常规的平滑化类似，这个网格被放大为常规的 0.5 x 0.5 度网格
* **Zernike max degrees**：设置扩展程度，以创建一个 0.5 x 0.5 的平滑规则网格。这三个条目是三个扩展，可以增强靠近地平线、中间高点和天顶的频率。
* **Additional congruent smoothed grid [n x m]**：网格的尺寸单位是度。这个网格被放大为常规的0.5 x 0.5度，并与Zernike网格相加。
* **Min obs per cell**：对于非 Zernike 网格，这代表了每个单元获得有效值的最小观测数。如果没有达到这个阈值，该单元将被设置为零。

###### Common Processing Options

* **Observation weighting**：选择观察加权函数的类型。这些是可用的选项：
  * uniform：等权
  * satellite elevation：$sin(el)$ 
  * square of satellite elevation：$sinn(el)^2$ 
* **Max code observation err**：残差超过这个阈值的伪距观测值被排除出 LS 解算（这个阈值只对非组合模式有效）。
* **Max phase observation err**：残差超过这个阈值的载波相位观测值被排除出 LS 解算。

###### PPP Options

这些选项专门用于 PPP 解算。

* **PPP Snooping / Reweight**：这个弹出式菜单允许用户选择不同类型的离群点剔除/定权方法。目前可用的方法列表如下：
  * **1 - none**：既不探测粗差，也不定权。
  * **2 - re-weight Huber**：
  * **3 - re-weight Huber (no threshold)**：
  * **4 - re-weight Danish**：
  * **5 - re-weight DanishWM**：
  * **6 - re-weight Tukey**：
  * **7 - simple snooping**：
  * **8 - smart snooping**：
  * **9 - smart snooping + arc trim**：
* **PPP Try to fix Ambiguity (Experimental)**：
* **Enable PPP for receivers containing only a single frequency**：





###### Network Options

* **NET Snooping / Reweight**：
  * **1 - none**：
  * **2 - simple 4 loops**：
  * **3 - 4 loops + remove bad satellites**：
* **NET fixing approach**：
  * **1 - none**
  * **2 - lambda3 search and shrink**
  * **3 - lambda3 integer bootstrapping**
  * **4 - lambda partial**
  * **5 - bayesian**
  * **6 - bayesian best integer equivariant**
  * **7 - sequential best integer equivariant**

##### ⑤PPP Parameters 选项卡

在这个选项卡中，用户可以设置 PPP 处理的参数。

注：正则化一词用于 Tykhonov 正则化，也被称为脊回归或 L2 正则化，它是通过伪观测实现的。

![goGPS Parametrization PPP](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Parametrization_PPP.png) 

###### Coordinates

* **Estimate**：
* **Time parametrization**：
  * **1 - Costant**：
  * **2 - Epoch wise**：
  * **4 - Regular spaced constant**：
  * **5 - Linear spline**：
  * **6 - Cubic spline**：
* **Rate**：
* **Frequency parametrization**：
  * **1 - Per tracking**：
  * **2 - Per frequency**：
  * **3 - One for all**：
  * **4 - Per binned frequency**：
  * **5 - Per frequency and constellation**：
  * **6 - Per rinex band**：



###### Ionosphere



* **Estimate**：
* **Ionosphere Management**：
  * **1 - Iono-free**：
  * **2 - Smooth GF**：
  * **3 - External-model**：



###### Troposphere

**ZTD** 

* **Time parametrization**：
  * **1 - Epoch wise**：
  * **2 - Linear spline**：
  * **3 - Cubic spline**：

- **Rate**：
- **Absolute Regularization**：
- **Differential Regularization**：

**ZTD Gradients**

* **Time parametrization**：
  * **1 - Epoch wise**：
  * **2 - Linear spline**：
  * **3 - Cubic spline**：

- **Rate**：
- **Absolute Regularization**：
- **Differential Regularization**：

###### Receiver Bias











##### ⑥Network parameters 选项卡

在这个标签中，用户可以设置 NET 解算的参数。

注：正则化一词用于 Tykhonov 正则化，也被称为脊回归或 L2 正则化，它是通过伪观测实现的。

![goGPS Regularization](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Parametrization_NET_SAT.png) 

###### Coordinates

- **Estimate**：
- **Time parametrization**：
  - **1 - Costant**：
  - **2 - Epoch wise**：
  - **4 - Regular spaced constant**：
  - **5 - Linear spline**：
  - **6 - Cubic spline**：
- **Rate**：
- **Frequency parametrization**：
  - **1 - Per tracking**：
  - **2 - Per frequency**：
  - **3 - One for all**：
  - **4 - Per binned frequency**：
  - **5 - Per frequency and constellation**：
  - **6 - Per rinex band**：







###### Ionosphere



- **Estimate**：
- **Ionosphere Management**：
  - **1 - Iono-free**：
  - **2 - Smooth GF**：
  - **3 - External-model**：



###### Troposphere



**ZTD** 

- **Time parametrization**：
  - **1 - Epoch wise**：
  - **2 - Linear spline**：
  - **3 - Cubic spline**：

- **Rate**：
- **Absolute Regularization**：
- **Differential Regularization**：

**ZTD Gradients**

- **Time parametrization**：
  - **1 - Epoch wise**：
  - **2 - Linear spline**：
  - **3 - Cubic spline**：

- **Rate**：
- **Absolute Regularization**：
- **Differential Regularization**：





###### Receiver Bias









###### Satellite Bias









#### 7.Commands 选项卡

这是主选项卡，定义了goGPS在执行过程中要执行的命令序列。

![goGPS Commands](https://gogps-project.github.io/wiki/images/goGPS_MainWindow.png?raw=true)

 这里可以看到的组件只有3个：

* 右边有一个编辑器，包含一些命令列表 各种处理的执行例子。用户不能与之互动，它只能复制这里的部分代码。
* 左边是另一个编辑器，用户可以在这里输入要由 goGPS 执行的命令。这些命令必须按照 goGPS 的命令语言来编写。
* 在编辑器的底部，有一个按钮可以打开一个窗口，显示一个简单的命令语言帮助，其中有所有可用的命令及其相应的参数列表。

![goGPS Language Help](https://gogps-project.github.io/wiki/images/goGPS_LanguageHelp.png?raw=true) 

#### 8.Output 选项卡

这是修改输出选项的选项卡。

![goGPS Output](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goGPS_Output.png) 

* **Output folder**：在这个领域，可以改变结果的最终存储文件夹。注意，有些结果将被保存在自动创建的子文件夹中。
* **Output rate**：改变这里的速率可以改变 `PUSHOUT` 命令期间数据从工作区复制到输出区的速率。
* **Results to store**： 以下标志选择了在执行过程中要保留的输出。注意：存储在出的数据会占用大量的RAM空间，这就是选择必须保留的内容的可能性的原因。
  * **Dt (clock errors)**：钟差
  * **PWV**：Precipitable Water Vapour 
  * **ZWD**：天顶对流层湿延迟
  * **ZTD**：天顶对流层总延迟
  * **Tropo Gradients**：对流层梯度（北，东）
  * **A-priori tropo**：从模型中计算出的对流层延迟
  * **P / T / H**：压力/温度/湿度
  * **Outliers / CS**：异常值和周跳（每颗卫星）
  * **Quality (SNR)**：这是一个可能在未来被改进的指数，目前它只存储了 SNR
  * **Number of Sat. per Epoch**：每个历元元所使用的卫星数量（以及按星座划分）。
  * **Azimuth / Elevation**：视线中所有卫星的方位角和俯仰角 
  * **Combined Residuals**：组合残差（每个观测值一个）
  * **Uncombined Code Res**：非组合伪距残差（每个观测值一个）
  * **Uncombined Phase Res**：非组合相位残差（每个观测值一个）
  * **Mapping functions**：对流层投影映射函数（每个卫星一个）

## 七、命令语句

goGPS 被设计成具有灵活性和可扩展性。为了允许不同类型的处理和复杂的操作，我们创建了一种伪语言。它允许顺序和平行操作。在这一部分，可以找到所有命令的含义。在 MATLAB 命令窗口中输入 `goHelpCommands`，就可以获得所有命令的简短帮助。

![goGPS Language Help](https://gogps-project.github.io/wiki/images/goGPS_LanguageHelp.png?raw=true) 

### 1、接收机数据组织结构

为了能够理解 goGPS 是如何工作的，有必要对数据的内部组织做一个简单的回顾。goGPS 是面向对象的，每个接收机都被认为是一个单独的 `GNSS_Station`，软件内部将数据存储在两个对象中：`work` 和 `out` 。`work` 是一个 `Receiver_Work_Space` 类的对象，包含了观测数据和计算单个会话结果所需的一切，`out` 是一个 `Receiver_Output` 类的对象，收集了一个接收机在所有会话中的输出。存储在工作对象中的结果只有在发出 `PUSHOUT` 命令时才会推送到 `out`。下面的标志选择要保留的输出。注意：存储在 `out` 中的数据会占用大量的RAM空间，这就是选择必须保留的内容的可能性的原因。

### 2、基本示例

goGPS 语言使用一个基本的固定结构来执行命令：一个运行在会话上的外部 `FOR` 循环来处理，也可以选择在接收机上的内部循环来处理。如果没有会话循环，goGPS 就不能做任何事情，这个循环初始化接收器的工作空间，并准备好所需的资源（例如，加载轨道）。一个最基本的示例如下：

```
FOR S*
   <some commands>
   PUSHOUT T*
END
<some commands>
```

每个 `FOR` 循环都必须以 `END` 结束。必须根据下面的方法选择会话和接收机。

为了将解算结果保存，需调用 `PUSHOUT`，且必须至少指定一个目标接收机。要从接收机工作空间（单个会话）复制到接收机输出（多个会话）的结果在处理面板上指定

**PUSHOUT命令参数**：

* **@**：以秒为单位的输出频率，如 `@30s`，`-r=30s`

可以用不同的频率存储数据，使用处理频率的倍数的值。这样修改可以节省内存，接收机输出对象上的所有输出都会受到影响。如果不使用该修改器，结果将以处理频率导出。如：5 分钟

**PUSHOUT命令示例**：

```
PUSHOUT T* @300s
```

### 3、选择会话

在关键词 `FOR` 之后是字母 `S`（Session），表示要迭代处理的会话标号，向用户提供各种选项：

* **S\***：选择全部会话。
* **S1,2,5,6**：选择 1、2、5、6 会话。
* **S3:6**：选择 3~6 会话。
* **S1:7:30**：1~30 每隔 7 位被选择。
* **S10:END**：选择 10 到最后的会话。

 可以结合上面的选择方式，如：**S1:3,6,24:2:30**，选择了 1,2,3,6,24,26,28,30 

### 4、选择接收机

接收机标号的选择与会话类似，在 goGPS 语言中，用户需要定义必须处理或用于特定目标的接收机，一些命令需要不同 "类型 "的接收机：

* **T**：这表示目标接收机（**T**arget），这些是命令将直接作用于的接收机。
* **R**：这表示参考接收机（**R**eference），这些是命令用于特殊目的的接收机，如 NET 中的参考接收机，或插值程序中的参考接收机。
* **P**：这表示接收机将被原封不动地传递（**P**assed）。

### 5、主要命令

处理GNSS数据所需的最小命令是三个： `LOAD`、`PREPRO`、`PPP` 或可选的 `NET`，这些加上会话循环可以处理一些数据。如 **PPP 解算**：

```
FOR S*
   LOAD T*
   PREPRO T*
   PPP T*
   PUSHOUT T*
END
```

**NET 解算**：

```
FOR S*
   LOAD T*
   PREPRO T*
   NET T* R1
   PUSHOUT T*
END
```

命令的主要选项可以在设置面板中设置，也可用 goGPS 命令的参数设置。下面是这几个命令的介绍：

#### 1.LOAD

`LOAD` 命令允许软件从 RINEX 文件中导入数据。要加载的文件是在 Data Sources 选项卡中选择的，且是相对于当前会话而言的。goGPS 以存储在 RINEX 文件中的速率导入所有数据，处理 Pre-Processing 选项卡中启用的所有星座，除非使用了一些参数修改。

**LOAD命令参数**：

* **-s=**：启用的系统，如 `-s=GRE`
* **@**：处理频率，以秒为单位，如 `@30s`，`-r=30s`

可以以不同的频率率（只要它是文件中存储的数据频率的倍数）和子集的系统来加载数据。软件以块为单位读取文件，但当这些选项被启用时，goGPS将只导入所要求的数据，即以较低的速率（如30s）导入一个包含 1Hz 数据的文件，比导入其原始频率要快。如：以 30s 的频率导入 GPS 和 Galileo 观测数据：

**LOAD命令示例**：

```
LOAD T* @30s -s=GE
```

#### 2.PREPRO

预处理命令是一个强制性的命令，用于为进一步的操作准备加载的观测数据。在这个功能中，goGPS执行了不同的任务：

* code **positioning** - needed for computing a good a-priori position of the receiver, this is done unless a-priori coordinates are provided with a coordinate file in [Receiver Info tab](https://gogps-project.github.io/wiki/Command-language/Settings#tab-receiver-info);
* computation of **clock delays** (from code observations)
* computation of **satellite positions** at the time of signals transmission
* **synchronization** of the receiver to the nominal epoch of observation - low-cost receivers, and some geodetic ones may read the observations with an incorrect inconstant rate. To avoid problems in network solutions, and to restore a constant observation rate goGPS apply a correction term to all the observations to "move" them to the proper epoch. This term is computed by taking the difference of the synthesized pseudo-ranges at the nominal (transmission) time and at the observed (transmission) time (e.g. shifted by a few milliseconds).
* 计算先验大气层延迟。
* 应用在设置中启用的所有 PPP 修正。
* 粗差探测。

**PREPRO命令参数**：

* **-s=**：启用的系统，如 `-s=GRE`

通过使用 `s=` 参数，加快计算速度以启用单一星座的数据可能是有用的。goGPS 将进行更快的先验定位，但不同系统的所有其他观测将被预处理，就像它们被启用一样。这意味着PPP校正、大气延迟和离群点检测将在所有数据上执行。即使在想要一个多星座的最终结果时，也可以采用下面使用例子中的命令。

**PREPRO命令示例**：

```
PREPRO T* -s=G
```

#### 3.PPP

对所有目标接收机 进行PPP 解算。大气延迟参数和应用的修正策略必须在设置中预先选择，并可在执行处理之前在 GUI 中改变。

**PPP命令参数**：

* **-s=**：启用的系统，如 `-s=GRE`
* **-u**：使用非组合解算

非组合模式解算慢于组合模式，大多数情况下还是用组合模式。

**PPP命令示例**：

```
PPP T*
```

#### 4.NET

对所有目标接收机进行 NET 解算，使用选定的参考接收机。该命令允许计算单个基线（如NET T1:2 R1）或许多接收机的 NET 解算，如 `NET T1:10 R*`

**NET命令参数**：

* **-s=**：启用的系统，如 `-s=GRE`
* **@**：处理频率，以秒为单位，如 `@30s`，`-r=30s`
* **--free**：单频解算时使用的频率，如 `L1`
* **COO_CRD**：将坐标导出为 .CRD 文件，这种导出是为调试测试而创建的，不推荐使用，请使用 `EXPORT` 命令代替。
* **--clk**：导出网络中的通用参数，这个导出是为了测试组合引擎而创建的，不推荐使用。
* **-u**：使用非组合解算

非组合模式解算慢于组合模式，但在长基线情况下有更好的效果。

### 6、接收机的循环

使用循环，必须在关键字 `FOR` 后面加上目标接收机，在循环中执行的命令子集必须用 `END` 关键字结束。

**FOR 循环示例**：

这个例子使用 FOR 循环对每个接收机进行行 PPP 解算，然后再用所有接收机进行 NET 解算：

```
FOR S*
   FOR T*
      LOAD T$
      PREPRO T$ -s=G
      PPP T$
   END
   NET T* R* -free -iono -U
   PUSHOUT T*
END
```

请注意，`$` 符号是在目标循环中使用的，这个符号表示当前的目标。在每个迭代中，它将承担当前目标的值，在这种情况为：1,2,...直到接收者的数量。

### 7、Parallel execution

goGPS 实现了一种基于主从方式（master-slaves）的并行。这种方法可能有一些缺点，但不需要 MATLAB 并行工具箱。goGPS 在后台自动执行 MATLAB 的新实例（有时在 Windows 平台上有些实例不能正常启动，它们是可见的），然后对于这些新实例中的每一个，软件将启动一个能够等待或执行有限任务的函数。想要手动启动一个从机，用户可以执行：

```
gos = Go_Slave.getInstance(); gos.live();
```

执行 goGPS 的 MATLAB 是调度工作和收集结果的主站。所有的通信都是通过在磁盘上写和读文件来管理的，为了速度起见，必须在一个快速驱动器上创建这个文件夹。通信 "目录路径在项目ini文件中设置，可以在 GUI 的高级标签中修改。

`Parallel_Manager` 类用来管理并行的从机：

* `Parallel_Manager.killAll();`：杀死后台的从机。
* `Parallel_Manager.testSlaves();` ：获取后台从机的状态。
* `Parallel_Manager.requestSlaves(<num_of_slaves>);`： 初始化 `num_of_slaves` 从机。

为了简化从机的生成和维护，我们创建了几个 goGPS 命令：

#### 1.PINIT

该命令用于初始化或检查在后台运行的并行从机的存在。

**PINIT命令参数**：

* **-n=**：这是唯一且是强制性的参数，指定要并行执行的后台任务的数量。如：`PINIT n=7`、`PINIT N7`

**PINIT命令示例**：

```
PINIT -n=7
```

#### 2.PKILL

该命令没有参数，用于杀死所有后台从机。

* 不幸的是，由于错误或意外的操作，一些后台实例甚至在发出 PKILL 命令后仍然活着。在这种情况下，建议使用任务管理器手动杀死后台运行的从机。
* 在未使用此命令的情况下关闭 goGPS，将使后台运行的从机依然活着。

#### 3.PAR

一旦创建了从机，就可以使用并行执行。在goGPS中，有两种类型的支持并行执行的方式：

* 并行处理会话（session）
* 并行处理目标（targets）

可以通过 `PAR` 使用其中一种的并行方式。语法和 `FOR` 很像。

**PAR命令示例1**：

这个例子使用 `PAR` 循环在所有目标上并行进行 PPP 解算。

```
PINIT N5
PAR S*
   FOR T*
      LOAD T$
      PREPRO T$ -s=G
      PPP T$
   END
   NET T* R* -free -iono -U
END
PKILL
```

注意：会话上的 `PAR` 循环在加载每个从属设备处理的数据后自动执行 `PUSHOUT` 。从机只保存每个处理过的接收机的工作空间，不保存输出部分。

**PAR命令示例2**：

本示例使用 `PAR` 循环对所有会话的所有目标并行执行 PPP 解算。

```
PINIT N5
FOR S*
   PAR T*
      LOAD T$
      PREPRO T$ -s=G
      PPP T$
      PUSHOUT T$
   END
END
PKILL
```

注意：并行从机不会自动接收存储在主进程中的所有接收机；当它们开始执行时，它们的接收机是空的，除非使用了参数 `P`。相反，对象 `GNSS_Station`（包含 `work` 和 `out`）是由主进程导入的，在其计算结果时在从机中使用。

**PAR命令参数**：

* **P***：将工作空间传递到从机

这意味着这段用于计算相对于第一个接收机的所有基线的代码不成立：

```
PINIT N5
FOR S*
   LOAD T1
   PREPRO T1 -s=G
   PPP T1
   PAR T2:END
      NET T1,$ R1   <= the receiver 1 for the slaves is empty
   END
   PUSHOUT T$
END
PKILL
```

下面是成立的：

```
PINIT N5
FOR S*
   LOAD T1
   PREPRO T1 -s=G
   PPP T1
   PAR T2:END P1    <= the work-space of receiver 1 is passed to all the slaves
      NET T1,$ R1
   END
   PUSHOUT T$
END
PKILL
```

### 8、Plots and export

goGPS 实现了许多用于可视化和输出结果的功能，它们可以作为一些类（如 `GNSS_Station`, `Receiver_Work_Space`, ...）的方法被访问，可以从 `MATLAB` 命令窗口调用，作为 goGPS 语言的命令，也可以从 GUI 中调用。

显示绘图或地图的类的方法有前缀 `show`，而导出函数有前缀 `export`。

有许多选项可用于创建绘图，为此，我们创建了一个简化的图形用户界面。它可以通过在 MATLAB 命令窗口中输入 `goInspector` 来打开，但如果 GUI 被启用，它将由 goGPS 在其执行结束时自动显示。这个界面仍在开发中，未来将增加更多的内容。

![goInspector](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goInspector.png) 

#### 1.SHOW

命令 `SHOW` 允许创建绘图和地图，它要求结果已经被计算出来并存储在接收器中。有许多可以使用的 SHOW 参数，每一个都是一个类方法的快捷方式。`goInspector ` 中的两个标签 "plot 1 "和 "plot 2 "提供了一个快速的方法来手动调用它们。

![goInspector_plot1_2](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goInspector_plot1_2.png) 

**SHOW命令参数**：

* **-s=**：启用的系统，如 `-s=GRE`。并不是每一种显示都可用此选项。
* **-e=<"name">**：用 `name_postfix` 导出 - 如果此标志被传递给命令 `SHOW`，生成的数字将被导出。`name`是可选的，它将被附加到一个自动生成的名称上（存储在 MATLAB 数字的 `UserData` 属性中）。goGPS 会自动将 `out` 文件夹中的图像保存为 "PNG"，以要求不同的格式，"名称 "可以设置为符合特定的文件类型（例如，`SHOW T* ZTD -e=".pdf "`将图像保存为 PDF 文件，`SHOW T* ZTD -e`将所有接收机的 ZTD 保存为 PNG。）
* **-c**：导出后关闭图表（仅在先前的导出修改器存在的情况下有效）
* **MAP**：测站坐标图（谷歌地图背景）--（所有接收机一张图）
* **L_MAP**：遗留 （Legacy）测站坐标图（谷歌地图背景）
* **G_MAP**：站点坐标图（谷歌地图背景）--（所有接收机一张图）
* **DTM_MAP**：站点坐标图（DTM地面高程模型 背景）--（所有接收机一张图）
* **G_MAP_R**：站点坐标图（谷歌地图背景）+  RAOB  --（所有接收机一张图）
* **DTM_MAP_R**：站点坐标图（DTM 背景）+  RAOB  --（所有接收机一张图）
* **DA**：数据可用性 Data Availability--（每个接收机一张图表）
* **ENU**：东北天位置（每个接收机一张图）
  * 该命令还接受修饰语 `--ctype=<coo_type>`，该修饰语指出了作为绘图来源的坐标类型（0个会议坐标，1个第一附加坐标，2个第二附加坐标，3个第三附加坐标）。
* **PUP**：平面向上的位置--（每个接收机一张图）。
  * 该命令还接受参数 `--ctype=<coo_type>`，指出了坐标类型（0：会话坐标，1：第一附加坐标，2：第二附加坐标，3：第三附加坐标）。
* **ENUBSL**：东北偏东向上基线，
* **PUPBSL**：平面上基线，这个需要同时指定参考接收机和目标接收机。
* **XYZ**：
* **CKW**：
* **CK**：
* **MP1**：
* **MP2**：
* **SNR**：
* **SNRI**：
* **OSTAT**：
* **PSTAT**：
* **OCS**：
* **OCSP**：
* **RES**：
* **RES_O_PR**：
* **RES_W_PR**：
* **RES_O_PH**：
* **RES_W_PH**：
* **RES_O_PR_STAT**：
* **RES_W_PR_STAT**：
* **RES_O_PH_STAT**：
* **RES_W_PH_STAT**：
* **RES_O_PR_SKY**：
* **RES_W_PR_SKY**：
* **RES_O_PH_SKY**：
* **RES_W_PH_SKY**：
* **RES_O_PR_SKYP**：
* **RES_W_PR_SKYP**：
* **RES_O_PH_SKYP**：
* **RES_W_PH_SKYP**：
* **PTH**：
* **NSAT**：
* **NSATSS**：
* **NSATSSS**：
* **ZTD**：
* **ZTD_VSH**：
* **ZHD**：
* **ZWD**：
* **ZWD_VSH**：
* **PWV**：
* **STD**：
* **RES_STD**：
* **TGRAD**：

#### 2.EXPORT

该命令用于在磁盘上保存用 goGPS 计算的结果。`goInspector` 中的 "导出 "标签提供了手动调用导出方法的快捷方式。

![goInspector_export](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/goInspector_export.png) 

**EXPORT命令参数**：

* **CORE_MAT**：
* **PLAIN_MAT**：
* **REC_MAT**：
* **REC_RIN**：
* **MP**：
* **COO_CRD**：
* **XYZ_TXT**：
* **ENU_TXT**：
* **GEO_TXT**：
* **TRP_SNX**：
* **TRP_MAT**：
* **TRP_CSV**：
* **TRP_HN**：











### 9、Validation



### 10、Ionospheric interpolation





### 11、Multipath management

goGPS 集成了创建和使用多路径地图的可能性，以减轻 GNSS 载波相位信号的反射影响。我们通过使用 goGPS 引擎处理长时间的数据（建议至少 10 天）来创建我们的地图。从获得的载波相位残差中，我们可以生成极坐标系统中常见的 "地理 "误差地图（例如，来自结构的多径效应总是以同样的方式影响信号，所以从某个海拔高度和角度发射的每颗卫星总是受到同样的多径干扰）。然后，多路径地图可以用来减轻射程误差，从而获得更好的定位。我们的初步测试表明，根据多径的程度，短时（如每 15 分钟/每小时）的坐标估计可以提高 15-30% 左右。

goGPS 可以在 PREPRO 的步骤中应用 RAW 观测的多路径地图，如果存在消电离层组合的地图，它们将被应用于静态 PPP 系统的生成，组合的地图将不会直接修改观测结果。

在内插残差之前，通过过滤较差的观测值来选择这些残差，特别是，除了在 PPP/NET 解算中应用的窥探方法外，还使用了 $3\sigma$ 阈值来抛弃异常值。落在纬度 1 度和经度 360 度范围内的残差被用来逐个评估残差的纬度，所有高于 $3\sigma$ 水平的观测值在内插过程中被忽略（通常约 1-1.5% 的数据被忽略）。

![Residuals and outliers](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/ResOutliers.png) 



为了使每个小于2个观测值的1x1度的单元的解决方案正规化，将1个空的伪观测值添加到要内插的数据集中。

对于接收机中存在的每个频率（或组合），使用一个多步骤程序来生成 goGPS 多径图。未组合的载波相位残差全部用于 LS 解，以计算 Zernike 扩展系数，上限为 43 度（可在代码中改变）。然后，这些系数被用来合成一个 0.5 x 0.5 度的地图，其坐标系为仰角 x 方位角（180x720）。

Zernike 多项式很适合绘制磁盘上的低度变化（如PCV），但对于多路径地图来说，更高的度数是最好的。Zernike多项式能够描述信号的变化，这取决于与圆盘中心的距离，半径接近1时，可以表示更高的频率。在goGPS中，我们使用3个映射函数在同一度数水平上进行3个分析，将海拔高度映射到单位半径上，这样我们就可以更好地描述多径引起的起伏情况。这两个映射函数是：
$$
\begin{array}{c}M F 1=\cos (e l)^{2} \\ M F 2=\sin \left(p i / 2 * \cos (e l)^{2}\right) \\ M F 3=\sin (p i / 2 * \cos (e l))\end{array}
$$
![MP Mapping functions](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/MP_MF.png) 

在第一次分析之后，对第一次分析的残差进行第二次分析，然后再进行第三次分析。通过添加三个矩阵计算出一个合成的最终地图。

![MP Step 1, 2](https://gogps-project.github.io/wiki/images/MP_Steps_1_2.png?raw=true) 



图中显示了从基于 Zernike 插值的 WTZZ 站的前两次迭代中得到的图。

叠加图可以从上一步的残差中产生，提供低频的内插。网格化程序使用全等的单元，对每个单元取平均值，每个值在极地坐标中转换，通过极地坐标系统的双线性插值，创建一个分辨率为0.5度的赤道坐标（海拔x方位角）的最终网格。

![MP Step 3](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/MP_Steps_3_4.png) 

这是对 WTZZ 站残差的第三次迭代和最后的网格化图。

最后，这些地图被加在一起，生成两个可能的适用模型，第一个是比较平滑的，第二个是含有较高频率的定点地图。用户可以通过在设置中选择，决定在他的项目中最有效的地图。

![MP final maps](https://gogps-project.github.io/wiki/images/MP_Final.png?raw=true) 

左边是比较 Zernike 多项式平滑的结果，右边是保留了最高细节的结果。

除了这两个网格之外，goGPS 还将从剩余部分开始创建其他四个格网。这对于方法的比较和测试是很有用的。

* 两个规则格网**（Regular grids）**。一个是只使用现有的观测数据，另一个是在1x1的单元中加入零的观测数据，并将其线性放大到0.5x0.5度的网格。

* 两个共轭格网**（Congruent cells grids）**。一个是只使用现有的观测数据，另一个是在1x1的单元中加入零的观测数据，并将其线性放大到0.5x0.5度的网格。全等网格的单元数随着海拔高度的增加而减少。参考论文：[Fuhrman et al 2015](https://link.springer.com/article/10.1007/s10291-014-0367-7) 

  ![MP final maps](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/CongruentCellGrid.png) 

#### MPEST

 `MPEST` 命令可用于先前在 PPP/NET 中处理过的仍含有相位残差的目标接收机。它为每个观测组合（或 RAW ）自动生成六张地图：一张由 Zernike 插值估计（Zernike interpolation ）的图，第二张由简单堆叠（simple stacking）而成，加上两个规则格网（regular grids）和两个共轭格网。如果启用了多径缓解，并且存在未合并的残差，在计算结束时，该命令将修正结果应用于包含在接收机中的相位观测。该命令的结果存储在 `GNSS_Staion` 类的 `obj` 中的 `ant_mp` 字段中。 

用命令 `EXPORT T* MP` 输出一个多路径图：

```
PAR S*
   FOR T*
      LOAD T$ @30s
      PREPRO T$ -s=G
      PPP T$ -U
   END
   PUSHOUT T*
END
MPEST T*
EXPORT T* MP
```

### 12、Other Commands

这些命令可以用来做一些小的操作，如清理部分变量以释放空间，重新命名接收机，计算简化的位置，或过滤观测数据等。

#### 1.RENAME

重命名是一个简单的命令，可以设置接收机名称，这在比较用不同方法处理的同一接收机时可能很有用。

**RENAME命令示例**：

假设要比较只用 GPS、只用 GLONASS 和只用 Galileo 处理的台站 ZIMM，可以将从同一 RINEX 文件加载的接收机数据重命名为 ZIMG、ZIMR、ZIME，以便在图中显示不同的标签

```
FOR S*   
   LOAD T1 -s=G
   LOAD T2 -s=R
   LOAD T3 -s=E
   PREPRO T*
   PPP T*
   RENAME T1 ZIMG
   RENAME T2 ZIMR
   RENAME T3 ZIME
END
SHOW T* ZTD
```

#### 2.EMPTY

**RENAME命令示例**：

这个函数重置了目标 `GNSS_Station` 的内容到它还没有被加载和处理的状态，释放了所有的内存

计算 100 个测站的 ZTD，每次计算完导出数据后释放内存。

```
FOR S1
   FOR T1:100
      LOAD T$
      PREPRO T$
      PPP T$
      EXPORT T$ TRP_SNX
      SHOW T$ ZTD -e -c
      EMPTY T$
   END
END
```

#### 3.EMPTYWORK

重置目标 `GNSS_Station` 的工作空间（当前会话）的内容，释放了接收机占用的部分内存。

**EMPTYWORK命令示例**：

计算 100 个站的 PWV，导出后释放内存（多个会话）。

```
FOR S*
   FOR T1:100
      LOAD T$
      PREPRO T$
      PPP T$
      PUSHOUT T$
      EMPTYWORK T$
   END
END
SHOW T* PW
```

#### 4.EMPTYOUT

重置目标 `GNSS_Station` 的输出对象的内容，释放接收机所占用的部分内存。

**EMPTYOUT命令示例**：

输出两张 PWV 比较图，一张是前 10 个会话，一张是后 10 个会话

```
FOR S1:10
   FOR T*
      LOAD T$
      PREPRO T$
      PPP T$
      PUSHOUT T$
      EMPTYWORK T$
   END
END
SHOW T* PWV -e

FOR S11:20
   FOR T*
      LOAD T$
      PREPRO T$
      PPP T$
      PUSHOUT T$
      EMPTYWORK T$
   END
END
SHOW T* PWV -e
```

#### 5.REMTMP

与 `EMPTYWOR` 命令类似，从工作空间中删除所有不需要的数据，在并行计算中可能很有用，可以限制保存在磁盘上的数据数量并释放一些内存。

**REMTMP命令示例**：

计算多个并行会话的 `ZWD`。请记住，会话上的 `PAR` 循环在加载每个从机解算的数据后自动执行`PUSHOUT`，因此它要求对象工作是可导出的（这里不能使用 `EMPTYWORK`）。

```
PINIT N5
PAR S*
   FOR T*
      LOAD T$
      PREPRO T$
      PPP T$
      REMTMP T$
   END
END
SHOW T* ZWD
```

#### 6.AZEL

计算接收机的方位角和俯仰角，如果已经计算了一个位置或在坐标文件中存在。

**AZEL命令示例**：

计算方位角和俯仰角以显示接收机的信噪比极坐标图

```
FOR S1
   LOAD T1
   AZEL T1
   SHOW T1 SNR
END
```

#### 7.BASICPP

在目标接收机上简单而快速的计算一个位置，而不应用任何校正。

**BASICPP命令示例**：

为所有接收机创建一个先验的坐标文件

```
FOR S1
   LOAD T*
   BASICPP T*
   EXPORT COO_CRD 
END
```

#### 8.CODEPP

运行这个命令，goGPS 将计算出一个 Code 定位，这个命令不执行任何离群点剔除，也不对观测值进行任何解同步，在没有对目标接收进行预处理的情况下，不应该使用这个命令。

**CODEPP命令示例**：

```
CODEPP T1

```

#### 9.OUTDET

对各阶段进行离群点检测，通常这是在预处理中进行，但在有些情况下在别处也需要，比如， PPP 和 NET 处理会修改离群点的状态。

**OUTDET命令示例**：

在这个示例下，`OUTDET` 被用来在每次迭代时重置标记的异常值

```
FOR S*
   LOAD T1
   PREPRO T1 -s=G
   PPP T1
   FOR T2:END P1
      OUTDET T1
      NET T1,$ R1
   END
   PUSHOUT T$
END
```

#### 10.FIXPOS

该命令允许改变参考坐标类型（默认情况下，它将位置设置为固定）。

**FIXPOS命令参数**：

* **FROM_WORK** - (flag)  使用当前会话工作空间的数据。
* **FROM_OUT** - (flag) 使用来自接收者输出对象的数据。
* **AS_APR** - (flag) 将计算出的位置作为一个新的先验位置（非固定）。

**FIXPOS命令示例**：

在这个例子中，一个 PPP 解在第一个会话中被计算出来，随后其他 29 个会话将第一个解作为先验位置。

```
FOR S1 
   LOAD T1
   PREPRO T1 -s=G
   PPP T1
   FIXPOS T1
END
FOR S2:30
   LOAD T1
   PREPRO T1 -s=G
   PPP T1
END
```

#### 11.KEEP

这条命令可以用来从接收器的工作空间中删除一些数据，告诉 goGPS 应该保留什么。

**KEEP命令参数**：

* **@**：以秒为单位的频率，如 `@30s`，`-r=30s`
* **-s=**：Active constellations (e.g. -s=GRE)
* **-e=**：以度为单位的截止高度角，如：`-e=7`
* **-q=**：以dbHZ为单位的截止信噪比，如：`-q=7`

**KEEP命令示例**：

只保留仰角大于15度的GPS卫星的观测数据

```
KEEP T1 -s=G -e=15
```

#### 12.SYNC

以相同的频率对所有接收器进行同步，只保留所有接收机中第一个和最后一个之间的纪元。

SYNC命令参数：

* **@**：以秒为单位的频率，如 `@30s`，`-r=30s`

**SYNC命令示例**：

```
SYNC T* @60s 
```

#### 13.REMSAT

从目标接收机中删除一个卫星的所有观测数据。该函数接收一个要删除的卫星列表，其中包括星座的第一个字母（GREJCI）和两个数字的PRN号码。格式： `<1ch sat. sys. (GREJCI)><2ch sat. prn>` 

**REMSAT命令示例**：

删除PRN4或PRN29的GPS卫星，以及PRN25的伽利略卫星。

```
REMSAT T* G04,G29,E25
```

#### 14.REMOBS

从一个目标接收机中删除一些观察类型。格式：`<1ch obs. type (CPDS)><1ch freq><1ch tracking>`

**REMOBS命令示例**：

从第一台接收机中删除所有的多普勒观测值，对第二频率的信噪比和第二频率的载波相位进行民用跟踪。

```
REMOBS T1 D,S2,L2C
```

## 八、创建项目

创建一个新的 goGPS 项目的最简单方法是使用所提供的 GUI 或复制另一个现有项目。

通过在 MATLAB 命令窗口中执行 `goGPS`，打开 goGPS 界面。 

![Menu Project New](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/menu_new_prj.png) 

- 在菜单中选择 `Project` -> `New`，goGPS 将打开这个窗口：

![Create New Project GUI](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/new_prj.png) 

填写所有的字段：

* **Project Time**：选择你想创建的项目类型，根据这个选择，新项目的设置将被调整为建议的配置。可能的选择有四个：
  * 设置 PPP 解算 对流层估计
  * 设置 NET 解算（短基线 - 忽略电离层 - 忽略对流层）
  * 设置 NET 解算（中等基线<20km - 忽略电离层）
  * 设置 NET 解算（长基线-消电离层组合）
* **Where to create**：选择将包含新项目文件夹的目录。
* **Project Name**：为新项目提供一个名称（这也将是项目文件夹的名称）。
* **Observations folder**：选择存储观测数据的目录，可以出现一个子文件夹结构（如`RINEX/YEAR/DOY/RINEX_FILE_NAME.YYY`），并选择要执行的操作：
  * **Copy**：将观测的文件夹复制到新项目中。
  * **Move**：将观测的文件夹移动到新项目中。
  * **Keep**：将观测的文件夹留在原位。
  * **Do not add any receiver now**：现在不要添加任何接收器。
  * **Create New Project**：如果没有发生错误，项目将被创建，用于编辑设置的 goGPS 界面将用新的参数刷新。

**提示**：一个 goGPS 项目通常存储在一个具有固定结构的文件夹中，尽管所有的目录都可以在设置文件中改变，但我们建议保持原始结构（见 goGPS 的目录部分）。星历和外部资源可以存储在项目文件夹中，但由于相同的资源可以在不同的项目中使用，我们将它们保存在所有项目的共同文件夹中。