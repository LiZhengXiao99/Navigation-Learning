













###安装 boost

```bash
sudo wget -c https://boostorg.jfrog.io/artifactory/main/release/1.73.0/source/boost_1_73_0.tar.gz
sudo tar -xf boost_1_73_0.tar.gz
cd boost_1_73_0/
sudo ./bootstrap.sh
sudo ./b2 install
```

### 安装 OpenBLAS

```bash
sudo apt-get install libopenblas-dev
```

### 安装 YAML

- 下载

  ```bash
  git clone https://github.com/jbeder/yaml-cpp.git
  ```

- 进入 Yaml-cpp 目录编译安装

  ```bash
  cd yaml-cpp
  mkdir build 
  cd build
  
  cmake -D BUILD_SHARED_LIBS=ON ..
  
  make -j16
  sudo make install
  ```

### 安装 MongoDB



### 安装 Mongo_C



### 安装 Mongo_cxx



### 安装 netCDF4



### 安装 Python



### 安装 gfortune















### 安装 Miniconda











