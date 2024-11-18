cmake_minimum_required(VERSION 3.0.0)
# 项目名称
project(hello)
# 使用C++ 11标准进行编译
set(CMAKE_CXX_STANDARD 11)
# 要求代码必须遵守C++11标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#支持自定义信号，为Qt中的QObject子类自动生成moc文件*
set(CMAKE_AUTOMOC ON)
# 为Qt资源文件(.qrc)自动生成源文件
set(CMAKE_AUTORCC ON)
#为Qt设计器中生成的UI文件(.ui)自动生成头文件和源文件
set(CMAKE_AUTOUIC ON)
#乱码问题处理 指定编译器与编码格式
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
#设置图片资源
set(puzzle_qrc_files resources.qrc)
# 获取.h文件
file(GLOB headers 
${CMAKE_CURRENT_LIST_DIR}/include/*.h
${CMAKE_CURRENT_LIST_DIR}/source/*.h
)
#获取.cpp文件
file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/*.cpp
${CMAKE_CURRENT_LIST_DIR}/source/*.cpp
)
set(source_dir
${CMAKE_CURRENT_LIST_DIR}/include
)
#找第三方库
find_package(Qt5 REQUIRED Widgets Gui Core)
#文件链接生成可执行文件
add_executable(hello  
            ${headers}
            ${soucers}
            ${puzzle_qrc_files}
)
# 不需要指定头文件路径
target_include_directories(
    hello
    PRIVATE
	${source_dir}
)

#链接第三方库
target_link_libraries(
    hello 
    Qt5::Widgets
    Qt5::Gui
    Qt5::Core
)

# 生成可执行文件.exe
install(TARGETS hello DESTINATION ${CMAKE_CURRENT_LIST_DIR})