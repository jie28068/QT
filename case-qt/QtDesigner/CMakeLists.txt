cmake_minimum_required(VERSION 3.0.0)

project(hello)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC ON)

file(GLOB headers 
${CMAKE_CURRENT_LIST_DIR}/*.h
${CMAKE_CURRENT_LIST_DIR}/control/*.h
${CMAKE_CURRENT_LIST_DIR}/control/controlDevice/*.h
${CMAKE_CURRENT_LIST_DIR}/control/controlProperty/*.h
${CMAKE_CURRENT_LIST_DIR}/control/controlView/*.h
${CMAKE_CURRENT_LIST_DIR}/control/controlView/delegate/*.h
${CMAKE_CURRENT_LIST_DIR}/control/controlView/model/*.h
)
file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/controlDevice/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/controlProperty/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/controlView/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/controlView/delegate/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/controlView/model/*.cpp
)
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
set(FFMPEG_DIR "${CMAKE_CURRENT_LIST_DIR}/qtpropertybrowser")

include_directories(${FFMPEG_DIR})

find_package(Qt5 REQUIRED Widgets Gui Core)

add_subdirectory(qtpropertybrowser)

add_executable(hello 
    ${headers} 
    ${soucers}
)

target_include_directories(hello 
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/control
    ${CMAKE_CURRENT_LIST_DIR}/control/controlDevice
    ${CMAKE_CURRENT_LIST_DIR}/control/controlProperty
    ${CMAKE_CURRENT_LIST_DIR}/control/controlView
    ${CMAKE_CURRENT_LIST_DIR}/control/controlView/delegate
    ${CMAKE_CURRENT_LIST_DIR}/control/controlView/model
)

target_link_libraries(
    hello 
    PUBLIC
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
    qtpropertybrowser
)

install(TARGETS hello DESTINATION ${CMAKE_CURRENT_LIST_DIR})
