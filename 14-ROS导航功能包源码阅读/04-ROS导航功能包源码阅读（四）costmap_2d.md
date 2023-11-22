

论文：http://wustl.probablydavid.com/publications/IROS2014.pdf

>  摘要：
>
> 



![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-affda41f2203d4bc04600e7ea545b645_1440w.webp)



layer 类提供了最重要的两个接口，正因为有接口这个东西，costmap_2d 才能实现插件化地图：

* updateCosts：计算打算更新的范围
* updateCosts：更新 cost



插件化地图的概念有一篇论文讲的很清楚，在ros的注释当中也推荐了，Layered Costmaps for Context-Sensitive Navigation，比较懒的看下论文的图就行：

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-337b36bad882a8169f1eb75f16f328c6_1440w.webp)

![img](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-98df8e6c5fcea86b5dae15b3b60a9c06_1440w.webp)

不管后面的类有多少函数，主要看updateCosts跟updateBounds这两个就行。layer类中还使用到了LayeredCostmap，用来将各个层聚合在一起。LayeredCostmap类中主要的更新函数是updateMap，可以看到在udpateMap里利用到前面提到的多态，遍历调用了所有层的两个核心函数。不得不吐槽LayeredCostmap这个名字起的一点都不好，这哪里看得出来是用来聚合的。

基类中还有一个costmap2D，这个类读下来感觉没啥核心功能，就换算下坐标啊，找多边形框起来了哪些点啊。可以认为是一个打杂的类，现在暂时不用管，因为我们的目标是找到最有价值的部分。唯一值得注意的是costmap2D里面有个unsigned char* costmap_;这个应该就是之后继承了costmap2D的各种层的实体了。

然后就是继承了layer跟costmap2D的costmapLayer了。costmaplayer其实就定义了几个更新cost的方法，比如不同层同一点的cost是覆盖 相加 还是选最大。不得不再吐槽一句这个名字起的也不好，一个类的名字至少要能看出来这个类打算干啥吧。





* observation observationBuffer：就是记录点云的，一次观测得到一堆点云，存到observation类的`pcl::PointCloud<pcl::PointXYZ>* cloud`_里，多次观测得到的一系列点云存到observationBuffer的std::list<Observation> observation_list_里。

* costmap2dROS：封装整个功能，对外提供简洁的接口。这个名字起的很好，在所有的包里，带ROS后缀的都是封装功能用的，可以理解为加精。从costmap2dROS看是最有效率的。
  * private不用看，因为肯定是被调用的；含有get的函数不用看，因为就是读取下变量的；含有footprint不用看，这是处理底座的。剩下来的函数有：start stop pause resume updatemap resetlayers.
  * start stop pause resume可以看到仅仅在调用layeredCostmap激活每层；updatemap也是调用layeredCostmap更新地图，resetlayers还是在调用layeredCostmap。可见主要的工作都是layeredCostmap这个包工头在干，layeredCostmap拿到命令后叫手下各层小弟干活，然后合起来发给costmap2dROS。
  * 稍微读下layeredCostmap，可以看到它包工头的气质展现地淋漓尽致：声明一个插件集合std::vector<boost::shared_ptr<Layer> > plugins_;在每个函数里面都是遍历这个插件集合然后调用每一层的具体函数。这样插件化的设计让地图的可扩展性很强。这样一来从上到下的调用结构就打通了，我们可以开始看真正干活的几个类了。









