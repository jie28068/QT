cmake_minimum_required(VERSION 3.0.0)
project(graphics VERSION 1.0.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置自动生成moc文件
SET(CMAKE_AUTOMOC ON)
# 设置自动生成ui.h文件
SET(CMAKE_AUTOUIC ON)
# 设置自动处理.qrc文件
SET(CMAKE_AUTORCC ON)

set(graphics_qrc_files resources.qrc)

#乱码问题处理
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

file(GLOB headers 
${CMAKE_CURRENT_LIST_DIR}/heard/*.h
)
file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/source/*.cpp
)

# 查找并加载Qt5模块
find_package(Qt5 REQUIRED Widgets Gui Core)

# 添加可执行文件
add_executable(graphics 
${headers} 
${soucers}
${graphics_qrc_files}
)

# 链接Qt5模块
target_link_libraries(
    graphics 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
)

install(TARGETS graphics DESTINATION ${CMAKE_CURRENT_LIST_DIR})