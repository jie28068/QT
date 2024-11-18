# hmi

通用hmi界面程序

- [接口](#接口)
  - [支持的平台](#支持的平台)
  - [注意](#注意)
  - [主机配置](#主机配置)
  - [配置容器](#配置容器)
  - [编译运行](#编译运行)
  - [异常记录](#异常记录)
  - [ros远程通讯](#ros远程通讯)

## 接口

关于开发当前项目需要做的一些工作

由下位机程序开发者给出所有界面通讯用到的接口数据结构和枚举类型

更新hmi程序，包括需要修改的Subscriber和Publisher，需要用到的msg文件和srv文件，service通信等

完成hmi界面和下位机的数据通信测试

## 支持的平台

| Operating System | Architectures | Versions | Status |
| -- | -- | -- | -- |
| Ubuntu 20.04 | x64 | gcc 9.4, cmake 3.16.3 | Not Test |
| Ubuntu 18.04 | x64 | gcc 7.5, cmake 3.10.2 |  Supported |
| Ubuntu 16.04 | x64 | gcc5.4, cmake 3.16.9 | Not Test |

## 注意
目前在docker中运行仍存在稳定性方面的问题，推荐在主机中运行界面，在主机中运行hmi程序可以跳过docker相关步骤
在docker中运行hmi，可能出现偶发的图形界面显示失败问题，此时ui对应位置的按钮实际可以生效 
# 可以直接联系作者获取配置好的docker镜像 hmi:18.04

## 主机配置
# 图形界面库 开放图形界面权限给docker
sudo apt-get install x11-xserver-utils
# 每次重启docker前需要调用此步骤
xhost +local:docker

# 启动容器 需要有 hmi:18.04 的镜像 然后直接进入 [](#编译运行)
docker run -itd --name hmi --add-host=host.docker.internal:host-gateway -p 554:554 -v /tmp/.X11-unix/:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY hmi:18.04 bash

## 配置容器
# 容器内的配置步骤可以在主机上使用

- Ubuntu 18.04
# 没有 hmi:18.04 的镜像 可以按照以下步骤配置
docker pull ubuntu:18.04

docker run -it --name hmi ubuntu:18.04 bash
# 可能需要生成用户 这里默认输入全填1
adduser ubuntu
# 安装ros
apt update && apt upgrade && apt-get update && apt-get upgrade && apt-get install -y lsb-release gnupg2 curl vim && apt-get clean all
# 换源
vim /etc/apt/sources.list
# 文本全部替换
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-updates main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-backports main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-security main restricted universe multiverse
deb https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse
deb-src https://mirrors.ustc.edu.cn/ubuntu/ bionic-proposed main restricted universe multiverse

sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'

apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

apt update && apt upgrade && apt-get update && apt-get upgrade && apt-get install -y ros-melodic-desktop-full && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && source ~/.bashrc
# qt库 中文字符解析 摄像头需要的gstreamer库 mysql驱动
apt-get install -y wget git iputils-ping libqt5sql5-mysql gstreamer1.0-qt5 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-tools gstreamer1.0-pulseaudio gstreamer1.0-libav gtk-doc-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgl1-mesa-dev libqt5charts5-dev libqt5multimedia5 libqt5multimedia5-plugins qtmultimedia5-dev language-pack-zh-hans fonts-noto-cjk-extra && echo "export LC_ALL=zh_CN.utf8" >>~/.bashrc

exit
# 保存镜像
docker commit hmi hmi:18.04

docker save -o hmi_image.tar.gz hmi:18.04

## 编译运行
# 若在刚配置的容器内 需要重启容器 界面的中文才可以生效 然后导入程序文件
docker cp hmi_qt_template/ hmi:/home/ubuntu/hmi && docker restart hmi && docker exec -it hmi bash
# 进入容器 编译运行
cd home/ubuntu/hmi/ && rm -rf build/ devel/ && catkin_make && source devel/setup.bash

roslaunch hmi_qt hmi.launch

## 异常记录
# 程序运行后偶发 界面未覆盖 或显示黑屏

## ros远程通讯
# 查看本机的局域网IP
ifconfig
# 在hosts文件中加入主机和从机的IP地址和对应的计算机名  主机、从机都需要加这两条
sudo gedit /etc/hosts
# 用tab，别用空格
192.168.88.253	wzf-lenovo-N50-80 
192.168.88.250	nvidia-desktop

# 修改完后检测能否ping
# 从机配置环境变量
sudo gedit ~/.bashrc

export ROS_HOSTNAME=192.168.88.253	#从机
export ROS_MASTER_URI=http://192.168.88.250:11311	#主机的IP，11311不要更改

source ~/.bashrc

# 主机配置环境变量
sudo gedit ~/.bashrc

export ROS_HOSTNAME=192.168.88.250	#主机
export ROS_MASTER_URI=http://192.168.88.250:11311	#主机的IP，11311不要更改

source ~/.bashrc

