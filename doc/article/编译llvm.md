tar -xvf llvm-18.1.6.src.tar.xz  
tar -xvf clang-tools-extra-18.1.6.src.tar.xz  
tar -xvf clang-18.1.6.src.tar.xz  
tar -xvf cmake-18.1.6.src.tar.xz  
mv clang-18.1.6.src llvm-18.1.6.src/tools/clang  
mv clang-tools-extra-18.1.6.src llvm-18.1.6.src/tools/clang/tools/extra  
mv cmake-18.1.6.src cmake  
mkdir build
cmake ../ -G Ninja -DCMAKE_BUILD_TYPE="Release" -DCMAKE_INSTALL_PREFIX="../install"  
ninja -j4  
ninja install
