<center><span style="font-size:2rem;font-weight:bold;">nullmax常用的一些tricks</span></center>

<div style="page-break-after: always;"></div>

[toc]

<div style="page-break-after: always;"></div>

# h264视频转换为mp4的命令

```bash
ffmpeg -f h264 -i front_far.264 -vcodec copy front_far.mp4
```

# linux电脑切换默认内核版本

## 首先查看当前内核版本

```BASH
uname -r
```

## 查看原本内核启动顺序

```BASH
grep menuentry /boot/grub/grub.cfg
```

![image-20230803112743638](nullmax-tricks.assets/image-20230803112743638.png)

## 数一下要换第几个内核(从0开始)

## 修改内核启动顺序

```BASH
 sudo gedit /etc/default/grub
```

  ![image-20230803112907338](nullmax-tricks.assets/image-20230803112907338.png)

 红框内GRUB_DEFAULT=0 修改为 GRUB_DEFAULT="1> 7"。

**注意：7表示第7个内核**

## 最后更新grud，且重新启动电脑

```bash
sudo update-grub
sudo shutdown -r now
```

# terminator设置文本颜色等

terminator是Ubuntu自带终端terminal的改进版本。支持同时显示多个终端界面。

安装指令

```bash
sudo apt-install -y terminator
```

使用terminator可以打开终端，鼠标进入终端，点击右键，可以更改配置文件。

 ![image-20230809171018509](nullmax-tricks.assets/image-20230809171018509.png)

![image-20230809171045295](nullmax-tricks.assets/image-20230809171045295.png) 

## terminator修改默认配色

在terminator中修改字体颜色和主题是统一修改的，如果我们想让命令行中的命令，以及最前端的用户名，设备名，地址等用不同的颜色显示，并设置是否高亮，需要在/home/username/.bashrc文件中修改。

### 首先打开命令行,打.bashrc文件

```bash
sudo gedit ~/.bashrc
```

### 找到文件中的PS1命令，大概在第60行

原本的文件

![image-20230810103529036](nullmax-tricks.assets/image-20230810103529036.png)

其中`${debian_chroot:+($debian_chroot)}` 表示如果当前用户是 root 用户，则在提示符前面显示一个 `#` 符号，否则不显示。

### PS1中常用的参数和含义如下：

```text
\d ：代表日期，格式为weekday month date
\H ：完整的主机名称
\h ：仅取主机名中的第一个名字
\t ：显示时间为24小时格式，如：HH：MM：SS
\T ：显示时间为12小时格式
\A ：显示时间为24小时格式：HH：MM
\u ：当前用户的账号名称
\v ：BASH的版本信息
\w ：完整的工作目录名称
\W ：利用basename取得工作目录名称，只显示最后一个目录名
#  ：下达的第几个命令
$  ：提示字符，如果是root用户，提示符为 # ，普通用户则为 $
[ ]：方括号
```

### PS1中常用的颜色和主题参数和含义如下：

 `0`：重置所有属性，包括颜色、背景色和样式。

\- `1`：设置粗体。

\- `2`：设置模糊（不常用）。

\- `3`：设置斜体（不常用）。

\- `4`：设置下划线。

\- `5`：设置闪烁。

\- `6`：设置闪烁（不常用）。

\- `7`：设置反显。

\- `8`：设置隐藏（不常用）。

\- `30`：设置前景色为黑色。

\- `31`：设置前景色为红色。

\- `32`：设置前景色为绿色。

\- `33`：设置前景色为黄色。

\- `34`：设置前景色为蓝色。

\- `35`：设置前景色为紫色。

\- `36`：设置前景色为青色。

\- `37`：设置前景色为白色。

\- `40`：设置背景色为黑色。

\- `41`：设置背景色为红色。

\- `42`：设置背景色为绿色。

\- `43`：设置背景色为黄色。

\- `44`：设置背景色为蓝色。

\- `45`：设置背景色为紫色。

\- `46`：设置背景色为青色。

\- `47`：设置背景色为白色。

### 命令格式

```BASH
\[\e[设置的参数(加粗等);31(前景色);35m(背景色)\]
\[\e[1;31;47m\]\u    #表示以加粗红色前景色以及白色背景色的方式显示用户名称
```

### 我目前的使用代码

```BASH
PS1="\[\e[1;5;35m\]\u\[\e[1;0m\]@\[\e[1;32m\]\w\[\e[0m\]\\$ "
```



# C++根据core文件查询哪里crash

1. 首先将使用的可执行文件bin，以及使用的动态lib库，下载下来
2. 引入环境变量，因为NM使用的是QNX作为交叉编译环境，因此需要引入qnx710的环境

```bash
source ~/qnx710/qnxsdp-env.sh
```

3. 执行gdb，分析core文件

```bash
cd qnx710/host/linux/x86_64/usr/bin/aarch64-unknown-nto-qnx7.1.0-gdb bin文件 core文件
```

4.  加入动态库文件

```bash
set solib-search-path 动态lib库的路径
```

5. 执行分析core文件

```bash
target core core文件
```

6. 执行命令

```BASH
bt        #看到crash的函数
bt-full   #看到所有的调试信息
```

# 使用VSCODE调试各种代码
