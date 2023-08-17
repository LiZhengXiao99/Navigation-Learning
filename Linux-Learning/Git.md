安装git：

```shell
yum install git
```

终端直接输入`git`，会告诉你git的各种使用方法

![1683439309212](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683439309212.png)



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

![1683440034894](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440034894.png)

提交之后，再查看状态，显示修改将要被提交

```shell
git add newfile.txt
git status
```

![1683440221282](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440221282.png)

> 如果要提交所有的文件修改，可`git add .`

修改newfile.txt，再查看状态，修改没有被添加。

![1683440492512](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440492512.png)

用`git diff`命令可以查看两文件的区别,即还没有被添加的更改内容到底是什么。如下图，可以看出我加了一个“change”

![1683440644141](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440644141.png)

`git add .`添加更改

![1683440492512](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440492512.png)

`git reset`撤回更改，文件又没有被追踪了

![1683440904418](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683440904418.png)



加提交人，加邮箱，以后提交就会显示是我的提交

```shell
git config --global user.name "lizhengxiao"
git config --global user.email  "dauger@126.com"
```

`git commit`把修改提交，`-m`后面加修改的描述

![1683441575077](C:/Users/李郑骁的spin5/Documents/Obsidian Vault/Linux/Git.assets/1683441575077.png)



有些新文件只想放在文件夹里，但不想提交由git管理，可以建立一个`.gitignore`，写入这些文件的文件名列表。

如果git已经追踪一个文件，再把它加到`.gitignore`列表中，git还是会继续追踪它。必须先停止追踪它

```shell
git rm --cashed newfile1
```

![1683442868403](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1683442868403.png)



`git commit -a`可以添加所有修改，再提交，但以前没追踪过的文件不会被添加进去



 `git branch 分支名`创建分支，`git checkout 分支名`进入分支，`git branch`查看分支

![1683443497967](C:/Users/李郑骁的spin5/Documents/Obsidian Vault/Linux/Git.assets/1683443497967.png)

在主分支下，`git merge 分支名`合并分支。合并后子分支还在，可以`git branch -d 分支名`删除；分支没合并的时候想删除，会提示，用D才能删。

![1683444080049](C:/Users/李郑骁的spin5/Documents/Obsidian Vault/Linux/Git.assets/1683444080049.png)



把文件上传到GitHub托管，建立仓库，他会提示你怎么上传

![1683444583188](C:/Users/李郑骁的spin5/Documents/Obsidian Vault/Linux/Git.assets/1683444583188.png)

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









