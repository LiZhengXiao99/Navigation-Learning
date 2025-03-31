> * 机翻自 RTKLIB-Demo5 作者的博客：[Getting started with RTKLIB: 2023](https://www.kaggle.com/code/timeverett/getting-started-with-rtklib-2023)
> * CDDIS 相关链接
>   * 官方说明主页：https://cddis.nasa.gov/Data_and_Derived_Products/CDDIS_Archive_Access.html
>   * CDDIS用户注册地址：https://urs.earthdata.nasa.gov/
>   * IGS-CDDIS数据地址（注册后才能访问）：https://cddis.nasa.gov/archive/
>   * 创建_netrc文件方法：https://cddis.nasa.gov/Data_and_Derived_Products/CreateNetrcFile.html
>   * CURL下载数据使用方法：https://cddis.nasa.gov/About/CDDIS_File_Download_Documentation.html

[TOC]

## 介绍

RTKLIB 是一个开源软件库，用于（除其他外）从原始观测数据中计算 GNSS 解决方案。它最初是由东京海洋科技大学的 Tomoji Takasu 编写的，但现在已有多个分叉，包括我维护的 demo5 分支。与谷歌提供的基线解决方案相比，它在生成 PPK（后处理运动学）解决方案时有两个优势。首先，它使用载波相位观测数据（ADR）和 pseduorange 观测数据。载波相位观测数据更难使用，但误差也比伪距观测数据小。其次，PPK 解决方案是差分的，相对于附近的已知基地位置，而不是像谷歌解决方案那样是绝对的。差分解法允许我们对漫游车和基地之间的原始观测数据进行差分，从而有效抵消了大部分卫星轨道、时钟和大气误差，使解法更加精确。通常情况下，PPK 解决方案还使用整数模糊分辨率来进一步提高精确度，但在本例中，我只使用浮动解决方案，因为智能手机观测数据的质量使得模糊分辨率极具挑战性。

本笔记本基于我在去年比赛结束时分享的 "RTKLIB 入门 "笔记本版本，但已更新为使用今年的数据运行。它将在 public 排行榜上产生 1.803 的分数。我还做了一些改动，使代码与 Linux 和 Windows 兼容。在这两种情况下，你都需要将代码下载到电脑上运行。我已在 Windows 11 和 WSL2（Windows Subsystem for Linux 2）上成功测试过。一般来说，每个文件的顶部都有一组输入参数。除非你的文件夹名称和路径与我的完全相同，否则你通常需要在运行前更新这些参数。

我在本解决方案中使用的文件夹结构是：

```
config
data
  - test
  - train
python
  - android_rinex
rtklib
```

如果使用相同的文件夹结构，将更容易遵循这些说明。

我希望提供一个平台，让竞争者可以直接扩展现有的 GNSS 理论，而不必从头开始构建解决方案。除了我在本笔记本中描述的 C 版本 RTKLIB 外，我还创建了 rtklib-py，这是 RTKLIB 的全 Python 子集，用于 PPK 解决方案。python 代码的运行速度比 C 代码慢一些，但它确实是一个更简单的开发平台。本笔记本仅介绍如何使用 C 代码版本的 RTKLIB，但我在去年竞赛笔记本部分的 "rtklib-py 入门 "笔记本中介绍了如何使用 python 版本。

## 步骤 1：检索基准观测数据和卫星导航文件

由于这些都是差分解法，因此我们需要从附近的基站获取每个数据集的原始观测测量数据。幸运的是，这些数据可以从美国国家大地测量局（NGS）的网站上获得。我们还需要 GPS、GLONASS 和伽利略星座每个数据集的卫星轨道数据。这些数据可从多个网站获取。我选择从 CDDIS 网站获取这些数据，部分原因是这些文件包括伽利略导航数据以及 GPS 和 GLONASS 数据，而且去年这些数据似乎比其他来源的数据更完整。

要自行下载 CDDIS 文件，您需要设置一个免费账户，然后按照 https://cddis.nasa.gov/Data_and_Derived_Products/CreateNetrcFile.html 上的说明在用户主目录下创建一个 .netrc 文件。

如果您只需要测试和训练数据集所需的基本观测和导航文件，我已将它们压缩并包含在本笔记本的输入数据中。直接解压缩到测试或训练文件夹中即可。

今年有来自湾区和洛杉矶地区的数据集，因此我们需要为每个数据集选择合适的基站，并在解决方案中使用该基站的正确位置。

下面的代码只需检索与每个数据集起始时间相对应的全天基站和导航数据。这对测试数据集很有效，因为所有数据集都在同一 UTC 日开始和结束，但在训练集中，有一个数据集在一个（UTC）日开始，在第二天结束。我将在测试数据上演示这个练习，所以不用担心这个问题，但如果你想在训练数据中检索这个数据，我建议你删除这一个数据集。

观测文件经过双重压缩。首先需要用 gzip 解压缩，然后再用 crx2rnx 解压缩。第二步是将压缩的 rinex 转换为未压缩的 rinex 格式。这需要 CRX2RNX 可执行文件，Windows 和 Linux 均可使用，网址是 https://terras.gsi.go.jp/ja/crx2rnx.html。将此文件放到 "rtklib "文件夹中。

```python
""" 
get_base_data.py - retrieve base observation and navigation data for the
    2023 GSDC competition 
"""

import os
from os.path import join
from datetime import datetime
import numpy as np
import requests
import gzip
from glob import glob
import subprocess

# Input parameters
datadir = '../data/test'  # relative to python script
# List of CORS stations to use
stas = ['slac', 'vdcy', 'p222']  # Bay Area, LA, backup for Bay Area

# site to retrieve base observation data
obs_url_base = 'https://geodesy.noaa.gov/corsdata/rinex'  

# site to retrieve satellite navigation data
nav_url_base = 'https://cddis.nasa.gov/archive/gnss/data/daily' #/2021/342/21p/ 
nav_file_base = 'BRDM00DLR_S_' # 20213420000_01D_MN.rnx.gz
# Access to CDDIS navigation data requires registering for a free account and 
# setup of a .netrc file as described at 
# https://cddis.nasa.gov/Data_and_Derived_Products/CreateNetrcFile.html.  
# Make sure this file  is in the users home directory 

# Make sure you have downloaded this executable before running this code
crx2rnx_bin = '../../rtklib/CRX2RNX' # relative to data directory

# Loop through data sets in the data directory
os.chdir(datadir)
for dataset in np.sort(os.listdir()):
    if not os.path.isdir(join(dataset)):
        continue
    print(dataset)
    ymd = dataset.split('-')
    doy = datetime(int(ymd[0]), int(ymd[1]), int(ymd[2])).timetuple().tm_yday # get day of year
    doy = str(doy).zfill(3)
    
    if len(glob(join(dataset,'*.*o'))) == 0:
        # get obs data
        i = 1 if '-lax-' in dataset else 0  # use different base for LA
        fname = stas[i] + doy + '0.' + ymd[0][2:4] + 'd.gz'
        url = '/'.join([obs_url_base, ymd[0], doy, stas[i], fname])
        try:
            obs = gzip.decompress(requests.get(url).content) # get obs and decompress
            # write obs data
            open(join(dataset, fname[:-3]), "wb").write(obs)
        except:
            # try backup CORS station
            print('Try backup CORS:', dataset)
            i += 2
            fname = stas[i] + doy + '0.' + ymd[0][2:4] + 'd.gz'
            url = '/'.join([obs_url_base, ymd[0], doy, stas[i], fname])
            try:
                obs = gzip.decompress(requests.get(url).content) # get obs and decompress
                # write obs data
                open(join(dataset, fname[:-3]), "wb").write(obs)
            except:
                print('Fail obs: %s' % dataset)
            
        # convert compact rinex to rinex
        crx_files = glob(join(dataset,'*.*d'))
        if len(crx_files) > 0:
            subprocess.call([crx2rnx_bin, '-f', crx_files[0]])
    
    # get nav data
    if len(glob(join(dataset,'*.rnx'))) > 0:
           continue  # file already exists
    fname = nav_file_base + ymd[0] + doy + '0000_01D_MN' + '.rnx.gz'
    url = '/'.join([nav_url_base, ymd[0], doy, ymd[0][2:4]+'p', fname])
    try:
        obs = gzip.decompress(requests.get(url).content) # get obs and decompress    
        # write nav data
        open(join(dataset, fname[:-3]), "wb").write(obs)
    except:
        print('Fail nav: %s' % dataset)
```

## 步骤 2：下载 android_rinex 库并创建 RTKLIB 配置文件

您需要使用 android_rinex 库将原始 Android 观测文件转换为 rinex 格式。RTKLIB 后处理解决方案要求输入文件为 rinex 格式。您需要使用我分叉出来的这个库，它的地址如下代码所示。请确保您使用的是 8/1/22 更新的最新版本。

将其放入 GSDC_2023/python/android_rinex 文件夹。目前，在多进程模式下运行时存在路径问题。为避免这一问题，您需要将 android_rinex/src 文件夹中的文件复制到 GSDC_2023/python 文件夹中。

您还需要 RTKLIB PPK 解决方案的配置文件。将以下配置文件复制到 GSDC_2023/config 文件夹，文件名为 gsdc_2023_config1.conf。

```python
git clone https://github.com/rtklibexplorer/android_rinex.git
```

```python
# gsdc_2023_config1.conf - config file for RTKLIB PPK solution

pos1-posmode       =kinematic  # (0:single,1:dgps,2:kinematic,3:static,4:static-start,5:movingbase,6:fixed,7:ppp-kine,8:ppp-static,9:ppp-fixed)
pos1-frequency     =l1+l2+l5   # (1:l1,2:l1+l2,3:l1+l2+l5,4:l1+l2+l5+l6)
pos1-soltype       =combined-nophasereset # (0:forward,1:backward,2:combined,3:combined-nophasereset)
pos1-elmask        =5          # (deg)
pos1-snrmask_r     =on         # (0:off,1:on)
pos1-snrmask_b     =off        # (0:off,1:on)
pos1-snrmask_L1    =28,28,28,28,28,28,28,28,28
pos1-snrmask_L2    =34,34,34,34,34,34,34,34,34
pos1-snrmask_L5    =20,20,20,20,20,20,20,20,20
pos1-dynamics      =on         # (0:off,1:on)
pos1-tidecorr      =off        # (0:off,1:on,2:otl)
pos1-ionoopt       =brdc       # (0:off,1:brdc,2:sbas,3:dual-freq,4:est-stec,5:ionex-tec,6:qzs-brdc)
pos1-tropopt       =saas       # (0:off,1:saas,2:sbas,3:est-ztd,4:est-ztdgrad)
pos1-sateph        =brdc       # (0:brdc,1:precise,2:brdc+sbas,3:brdc+ssrapc,4:brdc+ssrcom)
pos1-exclsats      =           # (prn ...)
pos1-navsys        =13         # (1:gps+2:sbas+4:glo+8:gal+16:qzs+32:bds+64:navic)
pos2-armode        =off        # (0:off,1:continuous,2:instantaneous,3:fix-and-hold)
pos2-gloarmode     =off        # (0:off,1:on,2:autocal,3:fix-and-hold)
pos2-bdsarmode     =off         # (0:off,1:on)
pos2-arelmask      =15         # (deg)
pos2-arminfix      =10
pos2-armaxiter     =1
pos2-elmaskhold    =15         # (deg)
pos2-aroutcnt      =1
pos2-maxage        =30         # (s)
pos2-syncsol       =off        # (0:off,1:on)
pos2-slipthres     =0.1        # (m)
pos2-dopthres      =10         # (m)
pos2-rejionno      =5          # (m)
pos2-rejcode       =10         # (m)
pos2-niter         =1
pos2-baselen       =0          # (m)
pos2-basesig       =0          # (m)
out-solformat      =llh        # (0:llh,1:xyz,2:enu,3:nmea)
out-outhead        =on         # (0:off,1:on)
out-outopt         =on         # (0:off,1:on)
out-outvel         =off        # (0:off,1:on)
out-timesys        =gpst       # (0:gpst,1:utc,2:jst)
out-timeform       =tow        # (0:tow,1:hms)
out-timendec       =3
out-degform        =deg        # (0:deg,1:dms)
out-fieldsep       =
out-outsingle      =off        # (0:off,1:on)
out-maxsolstd      =0          # (m)
out-height         =ellipsoidal # (0:ellipsoidal,1:geodetic)
out-geoid          =internal   # (0:internal,1:egm96,2:egm08_2.5,3:egm08_1,4:gsi2000)
out-solstatic      =all        # (0:all,1:single)
out-outstat        =residual   # (0:off,1:state,2:residual)
stats-eratio1      =200
stats-eratio2      =300
stats-eratio5      =25
stats-errphase     =0.005      # (m)
stats-errphaseel   =0          # (m)
stats-errphasebl   =0          # (m/10km)
stats-snrmax       =45         # (dB.Hz)
stats-errsnr       =0.005      # (m)
stats-errrcv       =0          # ( )
stats-stdbias      =30         # (m)
stats-stdiono      =0.03       # (m)
stats-stdtrop      =0.3        # (m)
stats-prnaccelh    =0.5        # (m/s^2)
stats-prnaccelv    =0.1        # (m/s^2)
stats-prnbias      =0.001      # (m)
stats-prniono      =0.01       # (m)
stats-prntrop      =0.001      # (m)
stats-prnpos       =0          # (m)
stats-clkstab      =0          # (s/s)
ant1-postype       =llh        # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)
ant2-postype       =posfile    # (0:llh,1:xyz,2:single,3:posfile,4:rinexhead,5:rtcm,6:raw)
ant2-maxaveep      =1
ant2-initrst       =on         # (0:off,1:on)
misc-timeinterp    =off        # (0:off,1:on)
file-satantfile    =
file-rcvantfile    =
file-staposfile    =../../../../config/bases.sta
```

## 步骤 3：设置基站坐标

由于我们要处理多个基站，因此需要一个包含不同基站位置的单独文件。在 C:\gps\GSDC_2023\config 文件夹中创建一个名为 bases.sta 的文件，并将下面几行复制到该文件中。RTKLIB 将使用基站文件的前四个字符从该列表中选择正确的位置。请注意，如果您使用的文件名和文件夹名与我使用的不完全相同，则需要修改上述配置文件中的 "file-staposfile "参数。

由于构造板块运动，精确的基站位置会不断发生微小变化，但我忽略了数据集之间的相对运动，下面的位置是使用每个基站可用坐标文件中指定的基准速度计算得出的，时间大致为 2022 年年中。

```python
%  LATITUDE(DEG) LONGITUDE(DEG)    HEIGHT(M)   NAME
37.41651904  -122.20426828  63.778  SLAC
34.17856659  -118.22000501  318.230  VDCY
37.53924080  -122.08326860  53.605  P222
```

## 步骤 4：下载 RTKLIB 代码

您可以从 https://github.com/rtklibexplorer/RTKLIB/releases/tag/gsdc_2022_v1.0 下载 RTKLIB 的可执行文件和源代码，该版本是我为智能手机解决方案优化的 RTKLIB。

您可以直接从此处下载 Windows 可执行文件，并将其放入 GSDC_2023/rtklib 文件夹。如果在 Linux 下运行，则需要根据源代码构建自己的可执行文件。https://rtklibexplorer.wordpress.com/2020/12/18/building-rtklib-code-in-linux/ 上的博文对此进行了说明。

请注意，如果您想自己构建 Windows 可执行文件，Windows 说明中会介绍使用 Embarcadero 编译器，这是 GUI 应用程序所必需的。如果只是编译 rnx2rtkp 应用程序，则可以使用 \app\consapp\rnx2rtkp\msc 文件夹中的项目文件，使用 VisualStudio 编译器进行编译。

## 步骤 5：转换原始观测文件并进行 PPK 解算

按照下面标题中的配置，该代码将把原始 Android 文件转换为 RINEX 格式，并运行测试集的 RTKLIB PPK 解法。请注意，这些将是浮点解法，我们并不试图解决整数歧义，因为智能手机观测数据的质量非常低。

在文件底部的主代码中，可以通过注释或取消注释适当的行来设置顺序执行或多进程执行，这既是为了文件转换，也是为了进行批处理解算。顺序运行时更容易调试，但速度要慢得多。我建议先按顺序运行每一步，直到确信运行正常后再切换到多进程。

所有的解决方案文件都会被标头中定义的 "soltag_rtklib "参数标记，因此你可以用它将多次运行的结果分开。这些文件将放在每个 phone 文件夹内的 "supplemental "文件夹中。

请注意，文件头中的参数会覆盖所有 rinex 文件和解决方案文件。如果只想重新运行数据子集，可将其中一个或两个参数设置为 "假"，这样代码只会在输出文件丢失时运行。

这段代码既可以运行 C 语言版本的 RTKLIB，也可以运行 Python 版本的 RTKLIB，还可以同时运行这两个版本。在本笔记本中，我只讨论 C 版本，如果您想运行 python 代码，请参阅我的其他笔记本。

> **调试提示**：如果代码运行无误，但没有生成任何解决方案文件，那么错误很可能发生在调用 rtklib 可执行文件的过程中，因为该代码中出现的任何错误都不会反馈给 python 代码。最简单的调试方法是在顺序执行模式下，在 "run_rtklib "函数中设置一个断点，打开控制台窗口，将目录更改为 "folder "变量的内容，然后将 "rtkcmd_debug "变量的内容复制并粘贴到控制台窗口并运行。您很可能会发现其中一个输入文件丢失或位置错误。

```python
"""
run_ppk_multi.py - convert raw android files to rinex and run PPK solutions for GDSC_2023
data set with RTKLIB and/or rtklib-py.   
"""

import sys
import numpy as np
if 'rtklib-py/src' not in sys.path:
    sys.path.append('rtklib-py/src')
if 'android_rinex/src' not in sys.path:
    sys.path.append('android_rinex/src')

import os, shutil
from os.path import join, isdir, isfile, abspath
from glob import glob
from multiprocessing import Pool
import subprocess
import gnsslogger_to_rnx as rnx
from time import time

# set run parameters
maxepoch = None # max number of epochs, used for debug, None = no limit

# Set solution choices
ENABLE_PY = False        # Use RTKLIB-PY to generate solutions 
ENABLE_RTKLIB = True     # Use RTKLIB to generate solutions
OVERWRITE_RINEX = True  # overwrite existing rinex filex
OVERWRITE_SOL = True    # overwrite existing solution files

# specify location of input folder and files
datadir = '../data/test'   # relative to python script
# base and nav file locations are relative to obs files
basefiles = '../*0.2*o' # rinex2, use this for rtklib only
#basefiles = '../base.obs' # rinex3, use this for python only
navfiles = '../BRDM*MN.rnx' # navigation files with wild cards

# Setup for RTKLIB,  paths relative to python script
binpath_rtklib  = '../rtklib/rnx2rtkp'
cfgfile_rtklib = '../config/gsdc_2023_config1.conf'
soltag_rtklib = '_rtklib' # postfix for solution file names

# Setup for rtklib-py - not supported in this notebook
#cfgfile = '../config/ppk_phone_0510.py'
soltag_py = '_py0510'  # postfix for python solution file names

# convert relative paths to absolute paths
datadir = abspath(datadir)
binpath_rtklib = abspath(binpath_rtklib)
cfgfile_rtklib = abspath(cfgfile_rtklib)

# Select phones to process
# all phones
# PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel5a', 'pixel6pro', 'pixel7pro',
#           'mi8', 'xiaomimi8',
#           'sm-g988b', 'sm-g955f', 'sm-s908b', 'sm-a226b', 'sm-a600t',
#           'sm-a505g', 'sm-a325f', 'sm-a217m', 'sm-a205u', 'sm-a505u', 
#           'samsungs22ultra', 'samsunga325g', 'samsunga32', 'samsung21ultra']
# phones in test set
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 'sm-a205u',
          'samsunga325g', 'samsunga32']

# These are only for rtklib-py, see the bases.sta file described above for RTKLIB base locations
BASE_POS = {'slac' : [-2703116.3527, -4291766.8501, 3854248.1361],  # WGS84 XYZ coordinates
            'vdcy' : [-2497836.8748, -4654543.0665, 3563029.0635],
            'p222' : [-2689640.5799, -4290437.1653, 3865051.0923]}

# input structure for rinex conversion
class args:
    def __init__(self):
        # Input parameters for conversion to rinex
        self.slip_mask = 0 # overwritten below
        self.fix_bias = True
        self.timeadj = 1e-7
        self.pseudorange_bias = 0
        self.filter_mode = 'sync'
        # Optional hader values for rinex files
        self.marker_name = ''
        self.observer = ''
        self.agency = ''
        self.receiver_number = ''
        self.receiver_type = ''
        self.receiver_version = ''
        self.antenna_number = ''
        self.antenna_type = ''

# Copy and read config file
if ENABLE_PY:
    shutil.copyfile(cfgfile, '__ppk_config.py')
    import __ppk_config as cfg
    import rinex as rn
    import rtkcmn as gn
    from rtkpos import rtkinit
    from postpos import procpos, savesol

# function to convert single rinex file
def convert_rnx(folder, rawFile, rovFile, slipMask):
    os.chdir(folder)
    argsIn = args()
    argsIn.input_log = rawFile
    argsIn.output = os.path.basename(rovFile)
    argsIn.slip_mask = slipMask
    rnx.convert2rnx(argsIn)

# function to run single RTKLIB-Py solution
def run_ppk(folder, rovfile, basefile, navfile, solfile):
    # init solution
    os.chdir(folder)
    gn.tracelevel(0)
    nav = rtkinit(cfg)
    nav.maxepoch = maxepoch
    print(folder)

    # load rover obs
    rov = rn.rnx_decode(cfg)
    print('    Reading rover obs...')
    if nav.filtertype == 'backward':
        maxobs = None   # load all obs for backwards
    else:
        maxobs = maxepoch
    rov.decode_obsfile(nav, rovfile, maxobs)

    # load base obs and location
    base = rn.rnx_decode(cfg)
    print('   Reading base obs...')
    base.decode_obsfile(nav, basefile, None)
    
    # determine base location from original base obs file name
    if len(BASE_POS) > 1:
        baseName = glob('../*.2*o')[0][-12:-8]
        nav.rb[0:3]  = BASE_POS[baseName]
    elif nav.rb[0] == 0:
        nav.rb = base.pos # from obs file
        
    # load nav data from rover obs
    print('   Reading nav data...')
    rov.decode_nav(navfile, nav)

    # calculate solution
    print('    Calculating solution...')
    sol = procpos(nav, rov, base)

    # save solution to file
    savesol(sol, solfile)
    return rovfile

# function to run single RTKLIB solution
def run_rtklib(binpath_rtklib, cfgfile_rtklib, folder, rovfile, basefile, 
               navfile, solfile):
    # create command to run solution
    rtkcmd = ['%s' % binpath_rtklib, '-x', '0', '-y', '2', '-k', cfgfile_rtklib,
              '-o', solfile, rovfile, basefile, navfile]
    
    # use this command line for debug from console, run from path in folder variable
    rtkcmd_debug = '%s -x 0 -y 2 -k %s -o %s %s %s %s' % (binpath_rtklib, cfgfile_rtklib,
              solfile, rovfile, basefile, navfile)
    
    # run command
    os.chdir(folder)
    subprocess.run(rtkcmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)   

####### Start of main code ##########################

def main():

    # get list of data sets in data path
    datasets = np.sort(os.listdir(datadir))

    # loop through data set folders
    rinexIn = []
    ppkIn = []
    rtklibIn = []
    for dataset in datasets:
        for phone in PHONES:
            # skip if no folder for this phone
            folder = join(datadir, dataset, phone)
            if not isdir(folder):  
                continue
            os.chdir(folder)
            rawFile = join('supplemental', 'gnss_log.txt')
            rovFile = join('supplemental', 'gnss_log.obs')

            rinex = False
            # check if need rinex conversion
            if OVERWRITE_RINEX or not isfile(rovFile):
                # generate list of input parameters for each rinex conversion
                if phone[:7] == 'samsung': # Use cycle slip flags for Samsung phones
                    slipMask = 0 # 1 to unmask recevier cycle slips
                else:
                    slipMask = 0 
                rinexIn.append((folder, rawFile, rovFile, slipMask))
                print(rawFile, '->', rovFile) 
                rinex = True
            
            # check if need to create PPK solution
            try:
                baseFile = glob(basefiles)[0]
                navFile = glob(navfiles)[0]
                solFile = rovFile[:-4] + soltag_py + '.pos'
                solFile_rtklib = rovFile[:-4] + soltag_rtklib + '.pos'
            except:
                print(folder,'  Error: Missing file')
                continue
            if ENABLE_PY and (OVERWRITE_SOL == True or len(glob(solFile)) == 0 
                              or rinex == True):
                # generate list of input/output files for each python ppk solution
                print('PY: ', join(dataset, phone))
                ppkIn.append((folder, rovFile, baseFile, navFile, solFile))
            if ENABLE_RTKLIB and (OVERWRITE_SOL == True or 
                        len(glob(solFile_rtklib)) == 0 or rinex == True):
                # generate list of input/output files for each rtklib ppk solution
                print('RTKLIB: ', join(dataset, phone))
                rtklibIn.append((binpath_rtklib, cfgfile_rtklib,
                        folder, rovFile, baseFile, navFile, solFile_rtklib))

    if len(rinexIn) > 0:
        print('\nConvert rinex files...')
        # generate rinx obs files in parallel, does not give error messages
        #with Pool() as pool: # defaults to using cpu_count for number of procceses
        #    res = pool.starmap(convert_rnx, rinexIn)
        # run sequentially, use for debug
        for input in rinexIn:
            convert_rnx(input[0],input[1],input[2],input[3])

    if ENABLE_PY and len(ppkIn) > 0:
        print('Calculate rtklib-py solutions...')
        # run PPK solutions in parallel, does not give error messages
        #with Pool() as pool: # defaults to using cpu_count for number of procceses
        #    res = pool.starmap(run_ppk, ppkIn)
        # run sequentially, use for debug
        for input in ppkIn:
            run_ppk(input[0],input[1],input[2],input[3],input[4])

    if ENABLE_RTKLIB and len(rtklibIn) > 0:
        print('Calculate RTKLIB solutions...')
        # run PPK solutions in parallel, does not give error messages
        #with Pool() as pool: # defaults to using cpu_count for number of procceses
        #    res = pool.starmap(run_rtklib, rtklibIn)
        # run sequentially, use for debug
        for input in rtklibIn:
            run_rtklib(input[0],input[1],input[2],input[3],input[4],input[5],input[6])

if __name__ == '__main__':
    t0 = time()
    main()
    print('Runtime=%.1f' % (time() - t0))
```

## 第 6 步：将 RTKLIB 解果合并为一个 .csv 文件

下面的代码将读入所有单独的 RTKLIB 解算文件，并以正确的格式创建一个 .csv 文件，以便提交给 Kaggle。RTKLIB 解决方案中的时间戳可能与原始数据中的时间戳不完全一致，也可能缺少某些数据点或解决方案文件，因此会将 RTKLIB 解决方案点插值到样本提交文件中的时间戳上。如果缺少解算文件，则使用样本提交文件中的数据。我建议从其他笔记本中下载当前得分最高的笔记本结果，并将其重命名以创建此文件。确保该文件命名为 "best_submission.csv"，并位于数据文件夹中。我使用的文件在输入数据中，是来自 Chirag Chauhan 的基线笔记本的结果。

这对测试数据有效，但对于训练数据，你需要从地面实况数据中生成一个时间戳正确的参考文件。这将在下文的训练数据部分进行说明。

只有具有相同标记的解决方案才会被包含在内，因此请确保此处使用的标记 (SOL_TAG) 与上一步创建解决方案时使用的标记相同。

输出文件名将包括测试集和日期，并位于 datapath 文件夹中。

```python
""" create_baseline_csv_from_pos.py -  Create csv file PPK solution files using timestamps in reference file
"""

import os
from os.path import join, isfile
import numpy as np
from datetime import date

########### Input parameters ###############################

DATA_SET = 'test'
SOL_TAG = '_rtklib'
datapath = '../data' # relative to python script
rovfile = 'gnss_log'
hdrlen = 25    # 25 for RTKLIB, 1 for RTKLIB-py

outThresh = 100   # max horizontal accuracy estimate
# phones in test set
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 'sm-a205u',
          'samsunga325g', 'samsunga32']
# PHONES = []  # use all phones

# Also make sure the appropriate reference file is in the datapath
#  test: best_submission.csv - best available sample submission
# train: ground_truths_train.csv - created with create_ground_truths.py

############################################################

GPS_TO_UTC = 315964782  # second

def create_csv(datapath, DATA_SET, SOL_TAG):
    # get timestamps from existing baseline file
    datapath = os.path.abspath(datapath)
    os.chdir(datapath)
    if DATA_SET[:5] == 'train':
        baseline_file = 'ground_truths_' + DATA_SET + '.csv'
    else: # 'test'
        baseline_file = 'best_submission.csv'
    # read data from baseline file
    base_txt = np.genfromtxt(baseline_file, delimiter=',',invalid_raise=False, 
                             skip_header=1, dtype=str)
    msecs_base = base_txt[:,1].astype(np.int64)
    phones_base = base_txt[:,0]
    pos_base = base_txt[:,2:4].astype(float) # baseline positions
    
    # open output file
    fout =open('locations_' + DATA_SET + '_' + date.today().strftime("%m_%d") + '.csv','w')
    fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n')
    
    # get list of data sets in data path
    os.chdir(join(datapath, DATA_SET))
    trips = np.sort(os.listdir())
    
    # loop through data set folders
    ix_b, npts = [], 0
    for trip in trips:
        if isfile(trip):
            continue
        phones = os.listdir(trip)
        # loop through phone folders
        for phone in phones:
            if isinstance(phone, bytearray):
                phone = phone.decode('utf-8')
            # check for valid folder and file
            folder = join(trip, phone)
            if isfile(folder):
                continue
            if PHONES != [] and phone not in PHONES:
                continue
            trip_phone = trip + '/' + phone
            #print(trip_phone)
    
            ix_b = np.where(phones_base == trip_phone)[0]
            sol_path = join(folder, 'supplemental', rovfile + SOL_TAG + '.pos')
            fields = []
            if isfile(sol_path):
                # parse solution file
                fields = np.genfromtxt(sol_path, invalid_raise=False, skip_header=hdrlen)
            if len(fields) > 1:
                if int(fields[0,1]) > int(fields[-1,1]): # invert if backwards solution
                    fields = fields[::-1]
                pos = fields[:,2:5]
                qs = fields[:,5].astype(int)
                nss = fields[:,6].astype(int)
                acc = fields[:,7:10]
                msecs = (1000 * (fields[:,0] * 7 * 24 * 3600 + fields[:,1])).astype(np.int64)
                msecs += GPS_TO_UTC * 1000
            # if no data, use baseline data
            if not isfile(sol_path) or len(fields) == 0:
                print('Warning: data substitution: ', sol_path)
                msecs = msecs_base[ix_b].copy()
                pos = acc = np.zeros((len(msecs), 3))
                pos[:,:2] = pos_base[ix_b].copy()
                qs = nss = np.zeros(len(msecs))
           
            # interpolate to baseline timestamps to fill in missing samples
            llhs = []; stds = []
            for j in range(6):
                if j < 3:
                    llhs.append(np.interp(msecs_base[ix_b], msecs, pos[:,j]))
                    stds.append(np.interp(msecs_base[ix_b], msecs, acc[:,j]))
            qsi = np.interp(msecs_base[ix_b], msecs, qs)
            nssi = np.interp(msecs_base[ix_b], msecs, nss)
    
            # write results to combined file
            for i in range(len(ix_b)):
                fout.write('%s,%d,%.12f,%.12f,%.2f,%.0f,%.0f,%.3f,%.3f,%.3f\n' % 
                        (trip_phone, msecs_base[ix_b[i]], llhs[0][i], llhs[1][i],
                         llhs[2][i], qsi[i], nssi[i], stds[0][i], stds[1][i], 
                         stds[2][i]))
                try:
                    npts += len(fields)
                except:
                    pass
    
    fout.close()
    return npts

if __name__ == '__main__':
    create_csv(datapath, DATA_SET, SOL_TAG)
```

## 第 7 步：创建最终提交并过滤掉有问题的 RTKLIB 解

遗憾的是，测试数据中有几个数据集，训练数据集中还有几个数据集的 RTKLIB 解算结果很差。这些问题需要进一步研究，但目前我们将用上一节中提到的 "best_submission.csv "文件中的值替换估计误差较大的解算点。

请注意，上一步将生成一个名称中包含当前日期的文件，因此您需要重命名下面的输入文件。

```python
"""
create_submission.py - convert baseline file into submission file
"""

import numpy as np
import os

LOCATIONS_FILE = 'locations_test_09_20.csv'
OUT_FILE = 'submit_0920.csv'
# specify data locations
datapath = '../data'  # relative to python script
max_hstd = 0.5

lowQualityRides = [
    '2022-06-28-20-56-us-ca-sjc-r/samsunga32',
    '2022-10-06-20-46-us-ca-sjc-r/sm-a205u']

datapath = os.path.abspath(datapath)
os.chdir(datapath)

# load baseline data 
baseline_file = 'best_submission.csv'
base_txt = np.genfromtxt(baseline_file, delimiter=',',invalid_raise=False, 
                         skip_header=1, dtype=str)
msecs_base = base_txt[:,1].astype(np.int64)
phones_base = base_txt[:,0]
pos_base = base_txt[:,2:4].astype(float)

# load test data
d = np.genfromtxt(LOCATIONS_FILE, delimiter=',',invalid_raise=False, skip_header=1, dtype=str)
stds = d[:,7:10].astype(float)
hstds = np.sqrt(stds[:,0]**2 + stds[:,1]**2)

        
# merge low quality rides with Google baseline
for trip_phone in np.unique(d[:,0]):
    if trip_phone in lowQualityRides:
        ixt = np.where(d[:,0] == trip_phone)[0]
        #ix = ixt[np.where(d[ixt,5] != '2')[0]]
        ix = ixt[np.where(hstds[ixt] >= max_hstd)[0]]
        d[ix,2:4] = pos_base[ix,0:2]

# save results to file
fout =open( OUT_FILE,'w')
fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n')
for i in range(len(d)):
    # write results to combined file
    fout.write('%s, %s, %3.12f, %3.12f\n' % (d[i,0], d[i,1], float(d[i,2]), float(d[i,3])))
fout.close()
```

## 第 8 步：向 Kaggle 提交 CSV 文件

现在您可以将上一步创建的 csv 文件提交到 Kaggle。这样，您就可以在公共排行榜上获得 1.803 米的分数。

## 在训练数据上运行此代码

在大多数情况下，您只需将上述所有引用从 test 文件夹更改为 train 文件夹，即可使用相同的代码为训练数据生成解决方案。不过，您需要先运行下面的代码来创建参考文件，以便在第 6 步中合并解决方案文件。

```python
"""
create_groundtruth_csv.py - create csv file from all ground truth files
"""

import os
from os.path import join, isfile


datapath = '../data/train' # relative to python script
# Select phones to process
# all phones
# PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel5a', 'pixel6pro', 'pixel7pro',
#           'mi8', 'xiaomimi8',
#           'sm-g988b', 'sm-g955f', 'sm-s908b', 'sm-a226b', 'sm-a600t',
#           'sm-a505g', 'sm-a325f', 'sm-a217m', 'sm-a205u', 'sm-a505u', 
#           'samsungs22ultra', 'samsunga325g', 'samsunga32', 'samsung21ultra']

# just phones in test set
PHONES = ['pixel4', 'pixel4xl', 'pixel5', 'pixel6pro', 'pixel7pro',
          'mi8', 'xiaomimi8',
          'sm-g988b', 'sm-s908b', 'sm-a325f', 'sm-a505u', 
          'samsunga325g', 'samsunga32']
GPS_TO_UTC = 315964782  # second

# open output file
datapath = os.path.abspath(datapath)
os.chdir(datapath)
fout =open('../ground_truths_train.csv','w')
fout.write('tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees, Height, Heading\n')

# get list of data sets in data path
datasets = sorted(os.listdir(datapath))

# loop through data set folders
for dataset in datasets:
    if isfile(dataset):
        continue
    try:
        phones = sorted(PHONES)
    except:
        phones = os.listdir(join(datapath,dataset))
    for phone in phones:
        folder = join(datapath, dataset, phone)
        if isfile(folder):
            continue
        
        csv_file = join(folder, 'ground_truth.csv')
        if not isfile(csv_file):
            continue

        # parse ground truth file
        with open(csv_file) as f:
            lines = f.readlines()[1:]
        flag = 0
        for line in lines:
            if len(line) <= 1:
                continue
            d = line.split(',')
            t = float(d[8]) # get time stamp
            if flag == 0:            
                print('%20s,%16s' % (dataset, phone))
                flag = 1
            # write results to combined file
            fout.write('%s/%s,%.0f,%s,%s,%s, %s\n' % ((dataset, phone, t, d[2],
                                                      d[3], d[4][:7], d[7][:5])))
        
fout.close()
```

## 最后的思考

本手册的目的不是提供完全优化的解决方案，而只是让您开始使用 RTKLIB 并展示其部分功能。

按照这些说明操作将提供一个改进的基线解决方案文件，该文件可以通过滤波、地图匹配等进行后处理，使您在竞争中脱颖而出。不过，仅凭这些可能还不足以在比赛中获胜。为此，我认为您需要改进 RKTLIB 解决方案本身。其中一些可以通过修改配置文件来实现。而更大的改动则需要修改代码。关于配置文件和代码算法的更多信息，请参阅《demo5 RTKLIB 用户手册》，尤其是第 3.5 节和附录 F 中关于配置的信息，以及附录 E 中关于核心算法的信息。

我的博客 (https://rtklibexplorer.wordpress.com/) 上有在 Windows 或 Linux 下编译代码的说明。请注意，Windows 说明使用的是 GUI 应用程序所需的 Embarcadero 编译器，但如果只是编译 rnx2rtkp 应用程序，则可以使用 VisualStudio 编译器，使用 \app\consapp\rnx2rtkp\msc 文件夹中的项目文件进行编译。在 rtklib-py （RTKLIB 的 python 版本）中可以更轻松地对代码进行更多修改。

我很乐意回答任何有关 RTKLIB 的问题。我只是要求，为了遵守比赛规则，请在这里的讨论组中提问，以便所有参赛者都能得到答案。如果您发现本代码中有任何错误或遗漏，请告诉我，我会及时更新。有关我针对智能手机观测对 RTKLIB 所做优化的更多详情，请参阅这些链接：

* [RTKLIBexplorer blog post: Google Smartphone Decimeter Challenge](http://https//rtklibexplorer.wordpress.com/2022/01/10/google-smartphone-decimeter-challenge/)
* [Optimizing the Use of RTKLIB for Smartphone-Based GNSS Measurements](http://https//www.mdpi.com/1424-8220/22/10/3825)
* [3rd Place Winner: 2022 Smartphone Decimeter Challenge: An RTKLIB Open-Source Based Solution](http://https//www.ion.org/publications/abstract.cfm?articleID=18376)









