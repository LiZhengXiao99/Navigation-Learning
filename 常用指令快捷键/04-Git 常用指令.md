Git 基本组成框架：

* **Workspace：开发者工作区**，也就是你当前写代码的目录，它一般保持的是最新仓库代码。
* **Index / Stage：缓存区**，最早叫Stage，现在新版本已经改成index，位于.git目录中，它用来存放临时动作，比如我们做了git add或者git rm，都是把文件提交到缓存区，这是可以撤销的，然后在通过git commit将缓存区的内容提交到本地仓库
* **Repository：仓库区**，是仓库代码，你所有的提交都在这里，git会保存好每一个历史版本，存放在仓库区，它可以是服务端的也可以是本地的，因为在分布式中，任何人都可以是主仓库。
* **Remote：远程仓库**，只能是别的电脑上的仓库，即服务器仓库。

Git 命令：

* **配置 Git 环境**：初次使用git需要设置你的用户名以及邮箱，一些需要登录权限的仓库会要求登录，git默认使用配置邮箱以及用户名登入，但会要求你手动输入密码。

  ```bash
  git config --global user.name "你的用户名"
  git config --global user.email "你的邮箱"
  ```

* **创建本地仓库**：需要一个空文件夹，先 `mkdir` 创建一个，再 `cd` 进入，在空目录下 `git init` 初始化当前仓库；初始化后会生成隐藏的 `Git` 的配置文件目录，可以用 `ls -ah`查看。

* **添加文件到缓存**：`git add`

* **提交到本地仓库**：`git commit -m "commit 信息"`，不加后面信息的话会打开编辑器让你写。

* **重新上一次提交信息**：`git commit --amend`，之后会打开编辑器。

* **查看历史提交日志**：`git log`、`git reflog`

* **回滚代码仓库**：`git reset --恢复等级`，有三个等级：

  * 使用`--soft`就仅仅将头指针恢复，已经 add 的暂存区以及工作空间的所有东西都不变。
  * 如果使用`--mixed`，就将头恢复掉，已经 add 的暂存区也会丢失掉，工作空间的代码什么的是不变的。
  * 如果使用`--hard`，那么一切就全都恢复了，头变，aad 的暂存区消失，代码什么的也恢复到以前状态。

* **回滚到指定历史版本**：先使用 `git log` 查看历史版本，再使用 `git reset --hard 要回滚id` 命令回滚。

* **单个文件回滚**：`git log 文件名`，`git reset 要回滚id 文件名`。

* **查看提交之后文件是否做了改动**：`git status`。

* **将文件撤销回到最近一次修改的状态**：`git checkout -- file`。

* **创建分支**：使用 `git checkout -b 分支名` 参数来创建一个分支，创建完成分支后会自动切换过去。

* **查看分支**：`git branch`，也就是查看HEAD的指向。

* **切换分支**：`git checkout 分支名`。



安装git：

```shell
yum install git
```

终端直接输入`git`，会告诉你git的各种使用方法

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683439309212.png" alt="1683439309212" style="zoom:50%;" />



想让当前文件夹用Git管理，现将当前文件夹变成Git仓库，进入文件夹输入命令

```shell
git init
```

在目录中新建一个文件newfile.txt

```shell
vim newfile.txt
```

查看当前状态，说newfile.txt未被追踪，需要用`git add 文件名`提交

```shell
git status
```

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440034894.png" alt="1683440034894" style="zoom:50%;" />

提交之后，再查看状态，显示修改将要被提交

```shell
git add newfile.txt
git status
```

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440221282.png" alt="1683440221282" style="zoom:50%;" />

> 如果要提交所有的文件修改，可`git add .`

修改newfile.txt，再查看状态，修改没有被添加。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440492512.png" alt="1683440492512" style="zoom:50%;" />

用`git diff`命令可以查看两文件的区别,即还没有被添加的更改内容到底是什么。如下图，可以看出我加了一个“change”

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440644141.png" alt="1683440644141" style="zoom:50%;" />

`git add .`添加更改

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440492512.png" alt="1683440492512" style="zoom:50%;" />

`git reset`撤回更改，文件又没有被追踪了

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440904418.png" alt="1683440904418" style="zoom:50%;" />



加提交人，加邮箱，以后提交就会显示是我的提交

```shell
git config --global user.name "lizhengxiao"
git config --global user.email  "dauger@126.com"
```

`git commit`把修改提交，`-m`后面加修改的描述

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683441575077.png" alt="1683441575077" style="zoom:50%;" />



有些新文件只想放在文件夹里，但不想提交由git管理，可以建立一个`.gitignore`，写入这些文件的文件名列表。

如果git已经追踪一个文件，再把它加到`.gitignore`列表中，git还是会继续追踪它。必须先停止追踪它

```shell
git rm --cashed newfile1
```

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683442868403.png" alt="1683442868403" style="zoom:50%;" />



`git commit -a`可以添加所有修改，再提交，但以前没追踪过的文件不会被添加进去



 `git branch 分支名`创建分支，`git checkout 分支名`进入分支，`git branch`查看分支

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683443497967.png" alt="1683443497967" style="zoom:50%;" />

在主分支下，`git merge 分支名`合并分支。合并后子分支还在，可以`git branch -d 分支名`删除；分支没合并的时候想删除，会提示，用D才能删。

<img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683444080049.png" alt="1683444080049" style="zoom:50%;" />



把文件上传到GitHub托管，建立仓库，他会提示你怎么上传

![1683444583188](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683444583188.png)

`git remote add`告诉Git项目在网上的位置在哪

```shell
git remote add origin https://github.com/LiZhengXiao99/d.git
```

如果不保存密码，每次push都要输入git密码，先执行下面命令，即可保存密码

```shell
git config credential.helper store
```

`git push`将更改上传

```shell
git push --set-upstream origin master
```



邀请别人合作，在setting-Collaborators里，搜他的用户名，邀请会发到对方的邮箱里，对方同意之后，也能对项目提交更改。



对方先将仓库复制到自己的电脑

```shell
git clone 仓库链接
```



对方修改后，我再`git pull`就会将项目最新的文件都拉到本地