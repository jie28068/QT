cmake_minimum_required(VERSION 3.0.0)

project(HMI)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)

set(CMAKE_AUTORCC ON)

# set(CMAKE_AUTOUIC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

file(GLOB header_s 
${CMAKE_CURRENT_LIST_DIR}/*.h

${CMAKE_CURRENT_LIST_DIR}/jsonModelTree/*.h
)

file(GLOB soucer_s 
${CMAKE_CURRENT_LIST_DIR}/*.cpp

${CMAKE_CURRENT_LIST_DIR}/jsonModelTree/*.cpp
)

file(GLOB ui_s 
${CMAKE_CURRENT_LIST_DIR}/jsonModelTree/*.ui
)

# set(src_qrc resources.qrc)

# set(FFMPEG_DIR "${CMAKE_CURRENT_LIST_DIR}/qtpropertybrowser")

# include_directories(${FFMPEG_DIR})

find_package(Qt5 REQUIRED Widgets Gui Core)

# add_subdirectory(qtpropertybrowser)

qt5_wrap_ui(QT_FORMS_HPP ${ui_s})

add_executable(HMI 
    ${header_s} 
    ${soucer_s}
    ${QT_FORMS_HPP}
)

target_include_directories(HMI 
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/jsonModelTree
)

target_link_libraries(
    HMI 
    PUBLIC
    Qt5::Widgets 
    Qt5::Gui 
    Qt5::Core 
    )
    
    # qtpropertybrowser