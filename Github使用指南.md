<div align="center">
    <a name="Top"></a>
	<h1>Github 使用指南</h1>
</div>
<div align="center">
    <img alt="Static Badge" src="https://img.shields.io/badge/QQ-1482275402-red">
    <img alt="Static Badge" src="https://img.shields.io/badge/%E5%BE%AE%E4%BF%A1-lizhengxiao99-green">
    <img alt="Static Badge" src="https://img.shields.io/badge/Email-dauger%40126.com-brown">
    <a href="https://blog.csdn.net/daoge2666/"><img src="https://img.shields.io/badge/CSDN-论坛-c32136" /></a>
    <a href="https://www.zhihu.com/people/dao-ge-92-60/"><img src="https://img.shields.io/badge/Zhihu-知乎-blue" /></a>
    <img src="https://komarev.com/ghpvc/?username=LiZhengXiao99&label=Views&color=0e75b6&style=flat" alt="访问量统计" />
</div>

<br/>

> 转自知乎文章：[【小白向】最新最详细的GitHub全站使用指南](https://zhuanlan.zhihu.com/p/664195515)，稍作修改

本文是一篇面向全体小白的文章，图文兼备。为了让小白们知道如何使用GitHub，我努力将本文写得通俗易懂，尽量让刚刚上网的小白也能明白。所以各位程序员们都可以滑走了～

[TOC]

## 啥是GitHub？

百度百科会告诉你：

> GitHub是一个面向开源及私有软件项目的托管平台，因为只支持Git作为唯一的版本库格式进行托管，故名GitHub。

啊？托管平台？Git？这都是什么玩意？其实并不复杂，大家可别被这些名词吓跑了，广大程序员特别喜欢造一些很高大上的词语，但这些词语背后往往是相当简单的概念...

要知道GitHub究竟是干什么用的，我们必须知道GitHub的使用群体都有哪些。程序员，大学生，企业……都是GitHub的用户。这些人有一个共同的特点，就是：**需要写代码**。

想象一下，你开发了一个软件，一开始很顺利。但后来你的好兄弟感觉这个软件不错，想要贡献点代码让它更屌一点。这时候问题来了，他在写代码的同时你也在写，他改完后把代码发给了你，你怎么知道他改了这个软件的哪些功能呢？怎么直观地看他改了哪些东西呢？怎么合并你的代码和他的代码呢？

如果后面又加入了几十个、几百个这样的好兄弟呢？如果又和你隔了十万八千里呢？

你将你的软件给大家用，大家都说好，但是就是有好多bug，通过什么渠道反馈呢？

这时候你的救星——GitHub，出现了，你将你的代码上传到了GitHub上，每一次改代码都标注好改动了哪些地方，添加了哪些功能，修复了哪些bug，这样就会使你的代码一目了然。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-4c33a5ab758c03de86fdf3565dbf331f_1440w.webp" alt="img" style="zoom: 67%;" />

如果别人要改你的代码，只需要拷贝（Fork）你的代码，然后修修补补，最后再合并（Merge）进去。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-2cc50d02d6c47209001b7ac51801f5a0_1440w.webp" alt="img" style="zoom: 67%;" />

如果别人要反馈问题，或者提出新的需求，只需要在问题（Issue）一栏里提问，就会有大佬帮你解答或者实现相应的功能。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-16f5f0c15a87d4a54a2556c0ac92c338_1440w.webp" alt="img" style="zoom:50%;" />

（大雾，发错了）

应该差不多是这样的：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-637ab32377d6cb929c69d5eb59ff109b_1440w.webp" alt="img" style="zoom:50%;" />

## 怎么逛GitHub？

在了解完GitHub的基本用途后，就可以看一下基本界面和功能了。

## 注册

首先点击[注册链接](https://link.zhihu.com/?target=https%3A//github.com/signup%3Fsource%3Dlogin)，填入邮箱：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-6c967d2f645af0aa2a97d5f1b8b7362a_1440w.webp" alt="img" style="zoom: 50%;" />

填入密码和用户名：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-edfa20e97e7f7a3afb09ea8ad602acb9_1440w.webp" alt="img" style="zoom: 67%;" />

接着会有一个验证你是否是人类的环节，就是通常所说的验证码。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-6cc0c03d1fe6482fc685849af85ccdfc_1440w.webp" alt="img" style="zoom:50%;" />

全部完成后即可创建账户，

验证好邮箱后GitHub会给你做个问卷，如实回答即可。当然也可以跳过哦。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-ac41ba1e69482ebce33071f33dff609b_1440w.webp" alt="img" style="zoom:50%;" />

接下来的主页就是这个界面啦。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-6a088258a555bb33f0b1a1986bd78cf7_1440w.webp" alt="img" style="zoom:50%;" />

你别看GitHub这英文很多，其实来来去去就这几个英文单词，记下意思就和中文差不多了。

## 代码界面

首先我们需要引入一个概念，叫“仓库”，英文名是`repository`，简称repo。仓库顾名思义就是用来放代码的。所谓代码托管，就是托管在仓库里面的。

下面就是一个仓库：https://github.com/microsoft/vscode

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-075ce6016aab9780c44531994fdca9b4_1440w.webp" alt="img" style="zoom:50%;" />

我们可以通过这个仓库了解到很多信息。

这个界面基本上每个地方都是可以点的……

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-154247db28157640b24bc3302a84925e_1440w.webp" alt="img" style="zoom: 67%;" />

star数量是我首先关注的点，在点star时，可以点击左边的下拉箭头选择你设定的不同栏目，所以star其实除了支持作者外，还有**收藏**的意思。

至于watch，其实要比star的人少很多，毕竟有谁希望提交个代码就要被通知一下呢？不过发布一个新版本还是有必要知道一下的，这样就可以体验最新版本。于是我们可以这样设置：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-4433319b2020466ac82b11a1d0821cd0_1440w.webp" alt="img" style="zoom: 33%;" />

这样当vscode发布一些新版本的时候，GitHub就会发邮件通知你。

那如果我需要**了解更多信息**呢？那就需要看`readme.md`了。
所谓readme，其实就是仓库里的一个文件，叫readme.md（简称readme），然后GitHub自动识别了这个文件，直接在仓库页面显示了。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-a0ecfb784d6e330142929a9390869a95_1440w.webp" alt="img" style="zoom:50%;" />

看完，我们可以可以知道（如果英文好的话）如何编译，如何下载，如何给vscode贡献代码等等。这样就可以对这个项目有更深入的了解。一个好的readme需要调理清晰，当然这不是我们小白需要考虑的事情了……

接下来讲一下什么是**分支**（branch）和**标签**（tag）。当你和队友们在写一个软件时，你的队友有不同的想法，如果他的想法不能说服你，他就可以直接做一个他的分支，这样你的代码因为他的想法就变成了两个分支。比如当年Vim和neovim，一个小伙子本来想给Vim做贡献的结果Bram没接受他的Pull request，于是小伙子直接Fork了Vim，成立了neovim，所以neovim其实是Vim的一个分支，当然这属于比较激进的做法。此外如果一个项目有一个长期维护版本（long time support简称LTS），那么也可以从仓库里分支出来，单独进行维护。

标签呢就很好理解了，就是你写着写着觉得这版代码很稳定很不错，就搞一个标签。通常来说tags就是版本号，在GitHub里，你可以找到代码的任何分支中的任何一个版本：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-aecb1076d1b08ce6e55f64af23cf3327_1440w.webp" alt="img" style="zoom:50%;" />

## Issue界面

另外一个很重要的功能就是GitHub的issue了。 Issue可以干很多事情，包括给开发者反映bug，向开发者求新功能，包括小白求助等等。
我们点击上面代码界面上方工具栏的issue，就可以看到：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-c92a3cf8f25b1234bf5d0f4ec61f466b_1440w.webp" alt="img" style="zoom:50%;" />

可以看出issue的检索功能和分类功能都十分强大，所以甚至还有在用issue写个人博客的程序员。。。

比如下面这个仓库：https://github.com/yygmind/blog

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-6e50377548cb6c97033b21d99bfc2245_1440w.webp" alt="img" style="zoom:50%;" />

## PR界面

`Pull Request`，就是你兄弟改完代码后给你提交的东西，简称PR。PR代表着新的功能或者bug修复，但要不要接受这些PR就要看仓库的拥有者你了。

PR界面长得跟issue界面非常像：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-201776fe028a299e7fd55698bd1cbc9a_1440w.webp" alt="img" style="zoom:50%;" />

但我点进去一个PR，界面就完全不同了。

conversation就是提交者对自己PR的介绍，以及和开发人员之间的交流。

commits就是这个代码被分出去以后经历的提交过程。

比如下面这个PR，只提交了一次。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-88c6f1e61e5943a91962c31564edc52b_1440w.webp" alt="img" style="zoom:50%;" />

再看到file changed，就是详细地看到底改了哪些地方：
可以看到这位仁兄给`extensionHostStarter.ts`文件增加了一段代码：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-3b7a1d239f324bf8b1c8d880a9bcbe92_1440w.webp" alt="img" style="zoom:50%;" />

至于Checks，就是GitHub的机器人干的事情了，这个不需要了解太多。

## Wiki界面

Wiki即维基，就是项目的小型文档。这可不是一般项目都有的，只有比较大型的项目才有维基，比如vscode。vscode的维基是给专业人士看的，告诉人们应该怎么贡献代码，如何写vscode插件等等，但是也有的项目的维基是给用户看的，因项目而异。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-053357ec46118deab619bccd1c18e8d3_1440w.webp" alt="img" style="zoom:50%;" />

## insight界面

另外一个很重要的功能就是insight界面。这个给你了很多关于这个项目的统计数据，可以说是非常详细，你可以看到许多贡献者在这个项目里贡献了多少代码，什么时候贡献的。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-d12128594427772cb82ea0681269f1d1_1440w.webp" alt="img" style="zoom:50%;" />

这里我愿称之为：大佬列表

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-df5756df3e8e8524b7f1c8a9a7b0d459_1440w.webp" alt="img" style="zoom:50%;" />

这是这个项目的Fork列表，看看都有谁抄袭（bushi）了这个项目：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-b176c1fc4471922d04823382f3caf670_1440w.webp" alt="img" style="zoom:50%;" />

其他栏目我就不一一列举了。

那么我能看到Fork列表，可不可以看star列表呢？答案是肯定的。入口就在项目主页中：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-2b518cf49e6960bc79bafcb3618252a7_1440w.webp" alt="img" style="zoom:50%;" />

点开后就是：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-b959a25de3250b54255f4ef6bb973b2c_1440w.webp" alt="img" style="zoom:50%;" />

## release界面

release界面是一个项目最有用的界面了，可谓伸手党的温床，白嫖党的乐园。release就是发布的意思，一个软件稳定了，bug被修得差不多了就会到了发布的时候了。这时候大部分开发者就会把源代码（zip/tar.gz后缀的压缩文件）和编译好的软件都发布在这个页面，要用的话直接下载就行了。

此外我们还可以通过compare功能来比较不同版本之间的差异：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-75b49069a4297e9e2f92f48582c397c1_1440w.webp" alt="img" style="zoom:50%;" />

点进去以后GitHub给你详细地展示了两个版本之间经历了多少次提交，多少个文件改动，甚至哪些代码的增删都给你展示出来。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-2f60c442d9fd47b98fbbdafa63a24a14_1440w.webp" alt="img" style="zoom:50%;" />

## discussion界面

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-e74797240aae2cc4ac65cc6d3462862f_1440w.webp" alt="img" style="zoom:50%;" />

有的项目是有discussion的，是一个简易的论坛，里面有很多求助和公告等等。如果对项目有问题，就可以在discussion里发出来。

## 用户/公司主页

GitHub的用户分为两种，一种是个人用户，一种是企业用户。
比如这是Linus torvalds的个人主页

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-f183281dcb24d7b0965e01380ebee235_1440w.webp" alt="img" style="zoom:50%;" />

这些绿色的方块代表什么呢？就是你一天里提交的次数，包括提交issue/PR/代码等等，总之就是你给社区贡献的次数越多，你的方块就越深。当然每个方块都是可以点击的，下面的activity就会显示作者在这一天内的各种活动。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-fb19b4c89c9eac15904077b523bb6f4e_1440w.webp" alt="img" style="zoom: 50%;" />

组织/公司的页面：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-20a881242718f9ab1e548a8909a46efc_1440w.webp" alt="img" style="zoom:50%;" />

组织/公司的主页就和个人主页很不一样了。但是我们的主要关注点在repository那一栏里，就是微软这个公司创建的仓库：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-458dcad974f76bb79194fed9e2d9ff51_1440w.webp" alt="img" style="zoom:50%;" />

可以看出巨硬不愧是巨硬，能同时维护这么多仓库，这几年对开源社区也贡献了不少。

## explore界面

explore界面就在：https://github.com/explore

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-8473648ed7202922d5e7db74fca0b37c_1440w.webp" alt="img" style="zoom:50%;" />

我这个号由于没有star任何仓库，所以它没有东西给我看。如果你经常star仓库的话，它根据你的偏好给你推送各种仓库，是不是感觉和某某社交软件操作一样（笑），但个人感觉这推送质量我不是很认可……

Topics就是一个仓库简介下面写的蓝色的类似于标签一样的东西，用于给仓库分类。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-d2de78c64e3438e7702bdc4075864149_1440w.webp" alt="img" style="zoom:50%;" />

点进一个topic以后，它默认的排序就是Most stars，是完完全全按照star的数量进行排序的，

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-a1e0a98cb7a9ba3422ab525a239d49d3_1440w.webp" alt="img" style="zoom:50%;" />

这样我们就可以进入一个topic，按star排序，就可以发现很多有用的仓库（轮子），我通过这一功能发现了许多好用的仓库，而且事实上仓库就是靠添加topic来曝光自己的（不然你怎么定位一个仓库）

接下来就是著名的**GitHub Trending**了，

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-c3f43067c020a7ad20a3072274cf7b3b_1440w.webp" alt="img" style="zoom:50%;" />

GitHub trending，译作GitHub趋势，仓库/作者一天内被star的次数越多，越有可能进这个榜单。这个榜单有很多明星项目，可以当日报每日刷刷。当然也可以筛选/排序，甚至选择语言。

collection就是GitHub的官方选集了，里面有很多正在维护的精品项目，而且还做好了分类，值得一看。

## 如何参与贡献

说到参与社区建设，就不得不提markdown了。

## Markdown

什么是markdown？它其实就是一个非常简单的语法，但可以让你打字更加方便，比如我这篇文章就是用markdown写的。markdown的流行GitHub简直功不可没，具体语法可以参考：https://zhuanlan.zhihu.com/p/270716843

强烈强烈建议各位学一下markdown，简直就是网页上的word，非常有用而且简单，5分钟就能学会。

## 创建仓库

首先需要配置一下token，进入[Token生成中心](https://link.zhihu.com/?target=https%3A//github.com/settings/tokens)

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/image-20240322230453341.png" alt="image-20240322230453341" style="zoom:50%;" />

然后会让你登陆确认一遍，接着就到了设置token的页面

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-4b7c8a1d2f32fdd146a5f823c328faf0_1440w.webp" alt="img" style="zoom:50%;" />

其实只要选择`repo`就够用了，接下来点击`generate token`,一个token就生成好了，但注意这个只显示一次，要好好保存。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-1f122b188932b674264f4da740c6537d_1440w.webp" alt="img" style="zoom:50%;" />

设置完token后就可以创建仓库了：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-615a1782eb87e8c09d8bd12cad02298d_1440w.webp" alt="img" style="zoom:50%;" />

于是我们来到这个页面。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-5a1487a7645812123e84d103c9b719b4_1440w.webp" alt="img" style="zoom:50%;" />

我新建了一个helloWord仓库，注意添加一下readme file，这样就可以创建一个仓库。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-2fd032b96d473f6943ef7bf81ec182c2_1440w.webp" alt="img" style="zoom:50%;" />

那如果我要改一下这个readme呢？注意这只是在GitHub这个网站上有这个仓库，而写代码则是在本地进行的，那么我们就需要将GitHub仓库和本地代码联系起来。所以**git**就闪亮登场了：

Windows用户可以安装git客户端，Mac用户可以通过homebrew安装：
`brew install git`

具体安装方法可以百度一下，每个系统都不太一样。

接下来在cmd或者终端里执行：

```text
$ git clone https://github.com/meIonhu/HelloWord
Username: 你的用户名
Password: 刚刚生成的token
```

这样你就在本地有一个名叫`HelloWord`的文件夹了。这个文件夹可不是一般的文件夹，这是已经初始化的git仓库。因为这个文件夹里有一个叫`.git`的隐藏文件夹，就是git的所有设置和你的提交记录。当然如果你把`.git`删掉的话这就跟普通的文件夹没什么区别了。

然后你可以在这里面一通乱改，最后将代码上传到GitHub：

```text
git add .
git commit -m "对于这次提交你想说的话"
git push -u origin main
```

这三段命令下去，你的改动记录就会被上传到GitHub上面，你的仓库也能看到你刚刚的commit了。

## 进阶技能

## Github gists

gist入口：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-73ce519049617f952d427a1f888483c8_1440w.webp" alt="img" style="zoom:50%;" />

什么是gists？其实就是GitHub推出的一个小型的代码托管服务，它针对代码片段进行托管。比如你有一个很好的代码片段，需要时常用一下但写起来又麻烦，就可以用gists来备忘，gists还提供了很多丰富的标签等功能，帮助对代码片段进行分类。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-68518649f745cb0b033a5e341a4ead87_1440w.webp" alt="img" style="zoom: 50%;" />



## Github搜索进阶

图形化的高级搜索入口：https://github.com/search/advanced

基本命令：

**In修饰词：**

`xxx in:name`：名字限定有xxx的仓库

`xxx in:description`：描述限定有xxx的仓库

`xxx in:topics`：在topics里有xxx的仓库

`xxx in:readme`：在readme里有xxx的仓库

**用户限制：**

`user:example`：获取所有来自example用户的仓库

`org:example`：获取所有来自example组织的仓库

**size修饰词**

`size:>n`：获取所有占用空间大于n kb的仓库

`size:<n`：获取所有占用空间小于n kb的仓库

`size:n1..n2`：获取所有占用空间在n1 kb到n2 kb之间的仓库

**star修饰词**`size:>n`：获取所有star大于n的仓库

`size:<n`：获取所有star小于n的仓库

`size:n1..n2`：获取所有star在n1到n2之间的仓库

**license修饰词**

`license:apache-2.0`: 获取所有license是apache-2.0的仓库

**language修饰词**

`language:rails language:javascript`：获取所有语言是javascript的匹配仓库

**NOT关键词**

`NOT cat`：排除所有含cat的搜索结果。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-c2e518c4d06590587be36809931aab2a_1440w.webp" alt="img" style="zoom:50%;" />

修饰词还有很多，但是基本语法大同小异，具体可以参考[github文档](https://link.zhihu.com/?target=https%3A//docs.github.com/en/search-github/searching-on-github/searching-for-repositories)。

学废了嘛？让我们来实操一下吧～

很多同学会很好奇，GitHub star数top10的仓库是哪些呢？其实完全不需要爬取所有仓库，只需要简单地搜索一下就可以了：

在搜索栏键入：`stars:>200000`

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-9df4cf3ab5a322a249d9c4192995c228_1440w.webp" alt="img" style="zoom:50%;" />



就得到最受欢迎的仓库名单了。

## GitHub快捷键

`ctrl+K`可直接打开命令栏：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-ecbbb879c33dd8d4824d3e839a3507ef_1440w.webp" alt="img" style="zoom:50%;" />



默认是快捷跳转，或者搜索，我们通过键入`>`可以快速让GitHub执行我们的命令：

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/v2-b8965f4cd8f18a98a2e2d14316ac22f5_1440w.webp" alt="img" style="zoom:50%;" />



此外还通过g键可以快速跳转到各种地方:

`g d`:跳转到首页

`g n`:跳转到通知

`g i`:跳转到issue

`g p`:跳转到PR

`g a`:跳转到Actions

`g b`:跳转到Projects

`g w`:跳转到Wiki

`g g`:跳转到Discussions