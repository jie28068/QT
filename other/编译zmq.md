## git
git clone https://gitcode.com/zeromq/libzmq.git
git clone https://gitcode.com/zeromq/cppzmq.git
## libzmq
```bash
./autogen.sh 
./configure
make check
sudo make install
```

## 交叉编译
更新环境变量：
```bash
export PATH=$PATH:/usr/local/Qt-5.15.2/bin/
export PATH=$PATH:/usr/local/Qt-5.15.2/lib/cmake/Qt5/
export PATH=$PATH:/opt/sysroot/firefly-arm64-sysroot-20.04/usr/bin/
```
```bash
 make distclean
./autogen.sh  
./configure --host=arm-linux CC=aarch64-linux-gnu-gcc AR=aarch64-linux-gnu-ar CXX=aarch64-linux-gnu-g++ CCFLAGS=-fPIC --prefix=/home/endoscope/Documents/gits/endoscope/src/endoscope/lib/arm/
make
make install
```


