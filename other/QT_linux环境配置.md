### vmware-tools
sudo apt install open-vm-tools
sudo apt install open-vm-tools-desktop

### opengl and qt depends
sudo apt-get install build-essential libgl1-mesa-dev libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev libglfw3-dev libglfw3
sudo apt-get install gcc gedit flex bison gperf libxcb* build-essential libgl1-mesa-dev libglu1-mesa-dev libegl1-mesa-dev freeglut3-dev libxkbcommon-x11-dev libxkbcommon-dev libxrender-dev libx11-dev libx11-xcb-dev

sudo apt install git cmake

### ninjia（非必要）
sudo apt install pipx
sudo pipx install ninja

### gstreamer
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

### qt
sudo apt install libfreetype6-dev
sudo apt install libfontconfig1-dev

./configure -opensource -confirm-license -system-freetype -fontconfig -skip qtwebengine -skip qtlocation -nomake tests -nomake examples -mp -optimize-size -strip 
make
make install
### sdk
https://wiki.t-firefly.com/zh_CN/ROC-RK3588-PC/linux_compile.html
https://wiki.t-firefly.com/zh_CN/Firefly-Linux-Guide/index.html

### 交叉编译环境
sudo apt install -y crossbuild-essential-arm64
cp -rfd firefly_qt5.15.2_arm64_20.04  /opt/
cp -rfd sysroot  /opt/
cp host/host_qtEnv.sh  /etc/profile.d/
#### gdb
apt install -y gdb-multiarch
检查目标机上是否存在 /usr/bin/gdbserver，没有的话需要安装：apt install -y gdbserver (Buildroot 自带，无需安装)

### cmake
添加环境变量,以下内容添加到~/.bashrc文件末尾，重启vs code
>export PATH=$PATH:/usr/local/Qt-5.15.2/bin/

