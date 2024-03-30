[TOC]

## 一、命令解释器

1. shell就是命令解释器

2. 命令解析器的作用：对用户输入到终端的命令进行解析，调用对应的执行程序。用户在终端输入命令, 由shell命令解释器对命令进行解析(按照`$PATH`环境变量搜索命令), 解析成内核能够识别的指令, 然后由内核执行命令, 最后由终端显示命令执行的结果给用户。shell在寻找命令的时候是按照`$PATH`环境变量去查找的，如果找到了就执行对应的命令，若找不到就报错。

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1682494290240.png" alt="1682494290240" style="zoom:50%;" />

3. 常用命令解释器：

   * shell -- Bourne Shell，目录：` /bin/sh`
   * bash -- Bourne Again Shell ，目录：`/bin/bash`

4. 查看当前系统所使用的shell ： `echo $SHELL`

5. 查看当前系统有哪些shell：`cat /etc/shells`



## 二、Linux常用快捷键

1. **Tab**：补全命令/文件（尽量多用），输入l然后按tab键, 会显示所有以l开头的命令，如果在执行ls, 然后按tab键, 会显示当前目录下所有的文件。

2. 历史命令：

   * 从当前位置向上遍历：ctrl + p（**↑**） 
   * 从当前位置向下遍历：ctrl + n（**↓**）
   * history命令显示用户输入的所有命令（输入his，然后tab补全）

3. 光标移动：

   * 光标左移： ctrl + b （**←**）
   * 坐标右移： ctrl + f （**→**）
   * 移动到头部： ctrl + a（**home**） 
   * 移动到尾部： ctlr + e（**end**） 

4. 字符删除

   * ctrl + h（Backspace）：删除光标前边的字符
   * ctrl + d（Delete）：删除光标后边的字符（光标覆盖的字符）
   * ctrl + u：删除光标前所有内容
   * ctrl + k：删除光标后所有内容 

   

## 三、Linux目录结构

1. 一个倒立的树状结构, 根目录用`/`表示

   <img src="https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1682495337229.png" alt="1682495337229" style="zoom: 67%;" />

2. 常用路径：

   * `/bin`: binary，二进制文件，可执行程序，shell命令，如: ls , rm , mv, cp等常用命令 
   * `/sbin`: s是Super User的意思，这里存放的是系统管理员使用的系统管理程序 ，如ifconfig, halt, shutdown, reboot等系统命令
   * ` /dev`: device，设备文件，linux下一切皆文件，硬盘, 显卡, 显示器，字符设备文件、块设备文件
   * `/lib`：linux运行的时候需要加载的一些动态库，如: libc.so、libpthread.so等
   * `/mnt`: 手动的挂载目录, 如U盘等
   * `/media`: 外设的自动挂载目录, 如光驱等。
   * `/root`: linux的超级用户root的家目录
   * `/usr`: unix system resource--类似于WINDOWS的programe files目录 
     - include目录里存放头文件, 如: stdio.h、stdlib.h、string.h、pthread.h 
     - games目录下的小游戏-如: sl小火车游戏
   * `/etc`: 存放配置文件
     - `/etc/passwd`：man 5 passwd可以查看passwd文件的格式信息
     - `/etc/group`： man 5 group可以查看group文件的格式信息 
     - `/etc/profile`：系统的配置文件, 修改该文件会影响这个系统下面的所有的用户
   * `/opt`: 安装第三方应用程序，比如安装mysql数据库可以在这个目录下
   * `/tmp`: 存放临时文件，新建在这个目录下的文件会在系统重启后自动清除 

3. 绝对路径，相对路径：

   * **绝对路径**：从根目录开始表示的路径，也就是从`/`开始，例如：`/home/itcast`

     > 注意：Windows的连接符：`\`，Linux的连接符：`/`

   * **相对路径**：从当前所处的目录开始表示的路径 ，`. `表示当前目录，` .. `表示当前目录的上一级目录 

4. 终端提示符



## 四、文件和目录相关命令

1. `tree`命令：以树状形式查看指定目录内容，比ls直观的多

   * 使用该命令需要安装软件tree：

     ```bash
     sudo apt-get update
     sudo apt-get install tree
     ```

   * 命令使用方法：`直接tree`、`tree + 目录`

2. `ls`命令：查看指定目录下的文件信息 ，常用参数：

   * `-a`：列出当前目录下的所有文件

     * `.`当前目录 
     * `..`当前目录的上一级目录
     * 隐藏文件, 以`.`开头的文件名, 如.bashrc "
     * 普通文件

   * ` -R`：递归方式列出所有目录中的内容

   * `-l`：列出文件的详细信息, 7部分内容，

     ![1682497582439](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1682497582439.png)![1682497007645](https://pic-bed-1316053657.cos.ap-nanjing.myqcloud.com/img/1682497007645.png)

     * 文件类型 （第1个字符）

       * `-`： 普通文件
       * `d` ： 目录
       * `l` ：  符号链接，相当于windows中的快捷方式 
       * `s` ： 套接字 
       * `p` ： 管道
       * `b` ： 块设备
       * `c` ： 字符设备

       用户的操作权限 （2 – 10个字符） 

       * `r`：可读；`w`：可写；`x`：可执行
       * 2-4：文件所有者对文件的操作权限 
       * 5-7：文件所属组用户对文件的操作权限 
       * 8-10：其他人对文件的操作权限

     * 硬链接计数：

       * 对于目录来说, 链接计数等于该目录下所有的目录总数(含. 和 ..), 但是不包含该目录的子目录下的目录文件数量, 执行ls -la命令可以进行查看. 
       * 对于文件来说, 指的是该文件所有的硬链接文件数量

     * 文件所有者：itcast 

     * 文件所属组：itcast

     * 文件大小：36

       * 如果是目录: 只表示目录大小, 不包含目录中的内容, 目录大小为4k
       * 如果是文件：表示文件大小

     * 文件的创建日期或最后修改时间：10月 13 11:41

     * 文件名：test.log

   * 参数之间可结合使用

     * `ls -la` : 列出当前目录下所有文件的相信信息, 包括隐藏文件 
     * `ls -ltr`: 列出当前目录下的文件, 按照时间逆向排序

3. `cd`命令：切换目录

   * 直接`cd`：回到家目录
   * `cd-`：回到上一次的目录

4. `pwd`命令：查看用户当前所处的工作目录 printf working directory

5. `which`命令：查看命令所在目录：`which ls`   `which cp` 

6. `touch`命令：`touch 213.txt`，如果文件不存在, 创建新文件；如果文件存在, 更新文件的最后修改时间

7. `mkdi`r命令：创建目录

   * 单个：`mkdir aa`
   * 多级目录加`-p`：`mkdir -p aa/bb/cc`

8. `rmdir`命令：只能删除空目录

9. `rm`命令：删除，使用rm命令删除的文件或目录不会放入回收站中，数据不易恢复

   * 参数：`-r`：递归删除目录 ，`-i`：提示用户是否删除文件或目录，`-f`：强制删除

   * 删除文件：`rm 文件名`
   * 删除目录：`rm -r 目录名`

10. `cp`命令：复制文件，`cp 源文件 目标文件`

    * `cp file1 file2`：file2不存在则复制一份，存在则覆盖
    * `cp file dir`：复制file到目录dir
    * `cp -r：dir1 dir2`：dir2不存在则复制一份，存在则把dir1复制到dir2中做子目录
    * `cp -a：file1 file2`：把属性信息也复制

11. `mv`命令：移动文件

    * mv file1 file2：改名，存在则覆盖
    * mv file dir：把文件file移动到目录dir
    * mv dir1 dir2（不存）：改名
    * mv dir1 dir2（存在）：目录dir1移动到dir2中做子目录

12. `cat`命令：将文件内容一次性输出到终端

    * 只能用于小文件，太长无法全部显示
    * 使用方式：`cat 文件名`
    * 可用于文件重定向：`cat file1>file2`, 相当于`cp file1 file2` 

13. `more`命令：文件内容分页显示到终端

    * 但是只能一直向下浏览，不能回退
    * 使用方式：`more + 文件名`
    * 操作：
      * 显示下一行：`回车`
      * 显示下一页：`空格`
      * 退出：`q`（ctrl + c）

14. `less`命令：文件内容分页显示到终端，可以自由上下浏览。

    * 使用方式：`less 文件名`
    * 操作：
      * 显示下一行：回车、ctrl + p、键盘向下键
      * 显示上一行：ctrl + n、键盘向上键
      * 显示下一页：空格、PageDown
      * 显示上一页：PageUp
      * 退出：q

15. `head`命令：从文件头部开始查看前n行的内容

    * 使用方式：`head -n[行数] 文件名`
    * 如果没有指定行数，默认显示前10行内容

16. `tail`命令：从文件尾部向上查看最后n行的内容

    * 使用方式：`tail -n[行数] 文件名` 

    * `tail -f text.log`：实时显示文件内容（日志）

17. 软链接：

    * 类似快捷方式，类型l。删除软连接，源文件还在。
    * 创建软连接：`ln -s 文件名 快捷方式的名字`，如：`ln -s aa aa.soft`
    * 目录也可以创建软连接，如：`ln -s tmp tmp.link`
    * 创建软连接要用绝对路径，因为如果不使用绝对路径，一旦这个连接文件发生位置变动，就不能找到那个文件了
    * 软连接文件的大小是: 路径+文件名的总字节数

18. 硬连接：

    * 几个文件指向同一个inode，指向相同的数据块。删除硬连接，源文件也删除。
      * `ls -i 文件名`：可以查看文件的inode节点 
      * `stat 文件名`：可以查看inode节点信息
    * 创建硬连接：`ln 文件名 硬链接的名字`，如：`ln test.log test.log.hard`
    * 硬连接计数：
      - 当新创建了一个文件, 硬链接计数为1
      - 给文件创建了一个硬链接后, 硬链接计数加1
      - 删除一个硬链接后, 硬链接计数减1
      - 如果删除硬链接后, 硬链接计数为0, 则该文件会删除
    * 应用场合：同步文件，保护文件，删除文件的时候, 只要硬链接计数不为0, 不会真正删除

19. wc命令：显示文件行数, 字节数, 单词数

    * `wc -l file`：显示文件的总行数
    * `wc -c file`：显示文件的总字节数
    * `wc -w file`：显示文件的总单词数
    * `wc file`：显示文件的总行数, 单词数和总字节数

20. `whoami`命令：显示当前登录的用户名

21. `chmod`命令：修改用户权限`chmod [who][+|-|=] [mode] 文件名`

    * 操作对象【who】
      * u -- 用户（user）
      * g -- 同组用户（group）
      * o -- 其他用户（other）
      * a -- 所用用户（all）【默认】
    * 操作符【+-=】
      * +：添加权限
      * -：取消权限
      * =：赋予给定权限并取消其他权限
    * 权限【mode】
      * r -- 读
      * w -- 写
      * x -- 执行
      * 给文件file.txt的所有者和所属组添加读写权限 ：`chmod ug+wr file.txt`
    * 数字设定法：
      * 0：没有权限
      * 1：执行权限
      * 2：写权限
      * 4：读权限
      * 给file.txt文件设置 rw-rw-r--：`chmod 664 file.txt`

22. `chown`命令：修改文件所有者和所属组

    * 要加sudo管理员权限
    * 修改文件所有者：`chown 文件所有者 文件名`
    * 修改文件所有者和所属组：`chown 文件所有者:文件所属组 文件名` 
    * 若系统没有其他用户, 可以使用`sudo adduser 用户名`创建一个新用户

23. `chgrp`命令：修改文件所属组

24. `find`命令

    * 按文件名查找：`find  路径  -name   "文件名" `
    * 按文件类型查询：`find 路径 -type 类型`
      * f -> 普通文件
      * d -> 目录
      * l -> 符号链接
      * b -> 块设备文件
      * c -> 字符设备文件
      * s -> socket文件
      * p -> 管道文件
    * 按文件大小查询：`find  路径  -size  范围`
      * 大于：+表示 --  +100k
      * 小于：-表示  --  -100k
      * 等于: 不需要添加符号 --  100k
    * 按文件日期查找
      * 创建日期：`-ctime -n/+n`
        * -n: n天以内 
        * +n: n天以外
      * 修改日期：`-mtime -n/+n`
      * 访问日期：`-atime -n/+n`
    * 按深度查找
      * 搜索n层以下的目录：`-maxdepth n(层数）`
      * 搜索n层以上的目录：`-mindepth n（层数)`
    * 查找后操作
      * 用`-exec`：`find ./ -type d -exec shell命令 {} \`
      * 用`-ok`，询问是否执行：`find ./ -type d -ok shell命令 {} \`
      * 用管道符`|`，文件量大时考虑：`find ./ -type d | xargs shell命令`

25. `grep`命令

* `-r`参数, 若是目录, 则可以递归搜索
* `-n`参数可以显示该查找内容所在的行号
* `-i`参数可以忽略大小写进行查找
* ` -v`参数不显示含有某字符串
* 例：搜索当前目录下包含hello world字符串的文件
  * `grep -r -n "hello world" ./ `    ------显示行号
  * `grep -r -i -n "HELLO world" ./  `-------忽略大小小查找
* find和grep命令结合使用：先使用find命令查找文件, 然后使用grep命令查找哪些文件包含某个字符串：`find . -name "*.c" | xargs grep -n "main"`

26. 软件安装卸载（以Ubuntu为例）

    > Ubuntu用apt-get，centos用yum

    * 在线安装：sudo apt-get install 软件名
    * 卸载：sudo apt-get remove 软件名
    * 更新列表：sudo apt-get update
    * 清理安装包：sudo apt-get clean 
    * 软件包安装：sudo dpkg -i xxx.deb
    * 软件包卸载：sudo dpkg -r 软件名

27. tar压缩解压缩

    * 参数：
      * z：用gzip来压缩/解压缩文件
      * j：用bzip2来压缩/解压缩文件
      * c：create, 创建新的压缩文件, 与x互斥使用
      * x：从压缩文件中释放文件, 与c互斥使用
      * v：详细报告tar处理的文件信息
      * f：指定压缩文件的名字
      * t:  查看压缩包中有哪些文件
    * 压缩：
      * tar cvf 压缩包名字.tar 原材料[要打包压缩的文件或目录]
      * tar zcvf 压缩包名字.tar.gz 原材料[要打包压缩的文件或目录]
      * tar jcvf 压缩包名字.tar.bz2 原材料[要打包压缩的文件或目录]
    * 解压缩
      * tar  xvf   已有的压缩包（test.tar.gz）
      * tar  zxvf  已有的压缩包（test.tar.gz）
      * tar  jxvf  已有的压缩包（test.tar.bz2）
      * 解压到指定目录：添加参数 -C（大写 ），`tar zxvf test.tar.gz -C 解压目录（./mytest)`
      * 查看压缩包中有哪些文件：`tar -tvf test.tar`