cmake_minimum_required(VERSION 3.0.0)
project(dock VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_AUTOMOC ON)

file(GLOB headers 
${CMAKE_CURRENT_LIST_DIR}/*.h
)
file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

find_package(Qt5 REQUIRED Widgets Gui Core)

add_executable(dock 
    ${headers} 
    ${soucers}
    resoure.qrc
)

target_link_libraries(
    dock 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
)

install(TARGETS dock DESTINATION ${CMAKE_CURRENT_LIST_DIR})
