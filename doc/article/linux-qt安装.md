自 Qt 5.15 开始，不再提供 open source offline installers，也就是原来的 .run 的安装文件，只能通过源码编译来安装了
[安装思路](https://blog.csdn.net/weixin_44200757/article/details/134043270)

# 下载源码和源码的 md5 校验码

wget <https://download.qt.io/archive/qt/5.15/5.15.11/single/qt-everywhere-opensource-src-5.15.11.tar.xz>
wget <https://download.qt.io/archive/qt/5.15/5.15.11/single/md5sums.txt>

# 校验

md5sum qt-everywhere-opensource-src-5.15.11.tar.xz

# 解压并进入目录

tar -xvf qt-everywhere-opensource-src-5.15.11.tar.xz
cd qt-everywhere-src-5.15.11/

# 安装依赖项

- 以下命令涵盖了大部分基础编译的依赖项：

```sh
sudo apt install bison build-essential flex gperf libasound2-dev libatkmm-1.6-dev libbz2-dev libcap-dev libcups2-dev libdrm-dev libegl1-mesa-dev libfontconfig1-dev libfreetype6-dev libglu1-mesa-dev  libicu-dev libnss3-dev libpci-dev libpulse-dev libssl-dev libudev-dev libx11-dev libx11-xcb-dev libxcb-composite0 libxcb-composite0-dev libxcb-cursor-dev libxcb-cursor0 libxcb-damage0 libxcb-damage0-dev libxcb-dpms0 libxcb-dpms0-dev libxcb-dri2-0 libxcb-dri2-0-dev libxcb-dri3-0 libxcb-dri3-dev libxcb-ewmh-dev libxcb-ewmh2 libxcb-glx0 libxcb-glx0-dev libxcb-icccm4 libxcb-icccm4-dev libxcb-image0 libxcb-image0-dev libxcb-keysyms1 libxcb-keysyms1-dev libxcb-present-dev libxcb-present0 libxcb-randr0 libxcb-randr0-dev libxcb-record0 libxcb-record0-dev libxcb-render-util0 libxcb-render-util0-dev libxcb-render0 libxcb-render0-dev libxcb-res0 libxcb-res0-dev libxcb-screensaver0 libxcb-screensaver0-dev libxcb-shape0 libxcb-shape0-dev libxcb-shm0 libxcb-shm0-dev libxcb-sync-dev libxcb-sync1 libxcb-util-dev libxcb-util0-dev libxcb-util1 libxcb-xf86dri0 libxcb-xf86dri0-dev libxcb-xfixes0 libxcb-xfixes0-dev libxcb-xinerama0 libxcb-xinerama0-dev libxcb-xkb-dev libxcb-xkb1 libxcb-xtest0 libxcb-xtest0-dev libxcb-xv0 libxcb-xv0-dev libxcb-xvmc0 libxcb-xvmc0-dev libxcb1 libxcb1-dev libxcomposite-dev libxcursor-dev libxdamage-dev libxext-dev libxfixes-dev libxi-dev libxrandr-dev libxrender-dev libxslt1-dev libxss-dev libxtst-dev perl ruby  python-is-python2 llvm libclang-dev zstd dbus libpango1.0-dev libcogl-pango-dev libjpeg-dev libdbus-1-dev libdbus-glib-1-dev
sudo apt install libfontconfig1-dev libfreetype6-dev libx11-dev libx11-xcb-dev libxext-dev libxfixes-dev libxi-dev libxrender-dev libxcb1-dev libxcb-glx0-dev libxcb-keysyms1-dev libxcb-image0-dev libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev  libxkbcommon-dev libxkbcommon-x11-dev
```

# 编译

```sh
./configure
# 键入 o 选择 open source 编译
# 键入 y 选择接受 license
sudo make -j8
sudo make install
```

- 按`o`使用开源版本，`c`是商业版
![alt text](image-7.png)
- 按`y`接受协议
![alt text](image-8.png)

![alt text](image-9.png)

- 报错`Project ERROR: Library 'assimp' is not defined. make[4]: *** [Makefile:85：sub-assimp-install_subtargets] 错误 3`
[解决方案](https://blog.csdn.net/Taozi825232603/article/details/132220891)
![alt text](image-10.png)

![alt text](image-12.png)

![alt text](image-11.png)

![alt text](image-13.png)

sudo apt install qtchooser

# 其中 qt5.15.11 是我们给版本的命名, 后面的地址根据安装位置确定 (更换版本同理)

qtchooser -install qt5.15.11 /usr/local/Qt-5.15.11/bin/qmake

# 设置环境变量, 启用 qt5.15.11, 这个要与我们的命名保持一致

sudo vim /etc/profile

# 最后补充

export QT_SELECT=qt5.15.11

qmake -version

# QMake version 3.1

# Using Qt version 5.15.11 in /usr/local/Qt-5.15.11/lib

![alt text](image-14.png)

![alt text](image-15.png)
