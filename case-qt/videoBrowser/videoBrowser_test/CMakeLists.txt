cmake_minimum_required(VERSION 3.0.0)

project(hello)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC ON)

file(GLOB headers 
${CMAKE_CURRENT_LIST_DIR}/*.h
)
file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

find_package(Qt5 REQUIRED Widgets Gui Core Multimedia MultimediaWidgets)

add_executable(hello 
    ${headers} 
    ${soucers}
)

target_link_libraries(
    hello 
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
    Qt5::Multimedia 
    Qt5::MultimediaWidgets
)

install(TARGETS hello DESTINATION ${CMAKE_CURRENT_LIST_DIR})
