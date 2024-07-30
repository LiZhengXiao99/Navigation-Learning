> 转载：https://blog.csdn.net/u010740725/article/details/51387810

　　最近在做与OpenCV有关的项目，需要参考OpenCV源码的实现。起初在VS中手动查看OpenCV源码，发现简单的函数能够很快整理出该函数的层次调用关系，但是遇到一些复杂的函数时，一层一层的往下查看一会儿脑袋就大了。并且，在查看源码的过程中，发现很多底层的函数都会被重复调用，因此把函数的层次调用关系以图的形式表示出来很有必要。一方面，可以更加清晰的掌握函数的实现过程；另一方面，快速的把最底层的一些调用次数较多的函数挑选出来并加以实现，缩短项目的开发周期。
　　网上关于自动生成函数调用关系图的方法有很多。LiNux环境下采用CodeViz+GraphViz+gcc可以自动生成函数调用关系，不过配置过程中比较麻烦；Windows环境下的方法有很多，其中大家最常用的VS中就有自动生成函数调用关系图的工具。另外还有开源利器SI(source insight)，然而在SI生成的调用关系图中，当两个函数调用同一个底层函数时，该底层函数会分别出现在两个函数调用图的下面，因此这种方法也不能最快的看出那些经常被调用的底层函数。还有一种方法，也就是即将要介绍的方法：Doxygen+GraphViz+HtmlHelp自动在本地生成函数调用关系图，最终结果以html的形式表示。

### 1.下载

首先，下载三个软件，地址如下：

* Doxygen：http://sourceforge.net/projects/doxygen/
* GraphViz：http://www.graphviz.org/Download..php
* HtmlHelp：http://www.softpedia.com/get/Authoring-tools/Help-e-book-creators/HTML-Help-Workshop.shtml

> 注意：Doxygen我下载的是最新的版本1.8.11。GraphViz我下载的也是最新的版本2.3.8，需要注意的是下载GraphViz时，需要选择Windows版本的安装包，如下图所示。

![GraphViz下载页面](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160512215933166)

### 2.安装
三个软件的安装都非常简单，根据提示一步步next即可。需要注意的一点是GraphViz2.3.8安装成功后，如果想打开GraphViz的界面时，需要进入bin安装目录，找到如下图所示的应用程序，打开运行即可。

![GraphViz应用程序](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513100720877)

### 3.配置

上面的下载安装搞定后，打开Doxygen应用程序，应用程序位置如下图所示。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513101445278)

第一步：打开Doxygen GUI frontend，按下图所示方法配置（运行指定路径的workspace是自己创建的）后，点击next下一步。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513110345687)

第二步：根据下图所示方法配置，点击next下一步。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513111013648)

第三步：根据下图所示方法配置，点击next下一步。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513111257303)

第四步：根据下图所示方法配置后，点击Expert标签（若继续点击next会进入Run），如下图所示。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513112131038)

第五步：点击Expert标签，进入Build小标签下，按如下图所示方法配置。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160518093509696)

第六步：点击Expert标签下的Dot标签后，很关键的一步来了，这里就要配置我们安装的GraphViz了，按照下图所示方法配置，尤其需要注意的是在配置Dot Path时需要写到GraphViz安装目录的bin目录下，我的路径是：“D:\GraphViz\bin”。配置完成后点击next下一步。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160518093341132)

第七步：最后来到Run标签下，点击 Run Doxygen即可。如下图所示（根据你源代码的大小，等待的时间不同，直到出现finished），最后点击“Show HTML output”，转到

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513144755749)

### 4.效果展示
打开HTML文档输出，得到如下图所示的函数调用函数。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160513150727838)

### 注意：如果需要中文版的在上述步骤的第五步点击Expert标签后，先进入Project标签，设置语言为Chinese即可。如下图所示。

![这里写图片描述](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/20160518094606247)