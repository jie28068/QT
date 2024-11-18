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
${CMAKE_CURRENT_LIST_DIR}/imageView/*.h
${CMAKE_CURRENT_LIST_DIR}/textView/*.h
)

file(GLOB soucers 
${CMAKE_CURRENT_LIST_DIR}/*.cpp
${CMAKE_CURRENT_LIST_DIR}/control/*.cpp
${CMAKE_CURRENT_LIST_DIR}/imageView/*.cpp
${CMAKE_CURRENT_LIST_DIR}/textView/*.cpp
)
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")


find_package(Qt5 REQUIRED Widgets Gui Core Multimedia)

set(qrc_files  ${CMAKE_CURRENT_LIST_DIR}/resource/resource.qrc)

add_executable(hello 
    ${headers} 
    ${soucers}
    ${qrc_files}
)

target_include_directories(hello 
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/control
    ${CMAKE_CURRENT_LIST_DIR}/imageView
    ${CMAKE_CURRENT_LIST_DIR}/textView
)

target_link_libraries(
    hello 
    PUBLIC
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
    Qt5::Multimedia
)

install(TARGETS hello DESTINATION ${CMAKE_CURRENT_LIST_DIR})
