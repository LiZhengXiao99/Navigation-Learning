## 创建、激活、退出环境

|         操作         |                    命令                     |
| :------------------: | :-----------------------------------------: |
|   查看 conda 信息    |                `conda info`                 |
| 查看已创建的虚拟环境 |     `conda env list` or `conda info -e`     |
|     创建虚拟环境     |   `conda create -n env_name python=3.12`    |
|     激活虚拟环境     |          `conda activate env_name`          |
|     退出虚拟环境     |            `deactivate env_name`            |
|     删除虚拟环境     |      `conda remove -n env_name --all`       |
|      克隆旧环境      | `conda create -n new_name --clone old_name` |

### Anaconda的包管理，类似 pip

|           操作           |                   命令                   |
| :----------------------: | :--------------------------------------: |
|        更新 conda        |           `conda update conda`           |
|      更新 anaconda       |         `conda update anaconda`          |
|       更新 python        |          `conda update python`           |
| 查看当前环境下已安装的包 |               `conda list`               |
| 查看指定环境的已安装的包 |         `conda list -n env_name`         |
|    查找 package 信息     |       `conda search package_name`        |
|       安装 package       |       `conda install package_name`       |
|   指定环境安装 package   | `conda install -n env_name package_name` |
|  安装制定版本的 package  |     `conda install package_name=3.1`     |
|       更新 package       |              `conda update`              |
|       删除 package       |              `conda remove`              |
|      清理安装包缓存      |           `conda clean --all`            |

