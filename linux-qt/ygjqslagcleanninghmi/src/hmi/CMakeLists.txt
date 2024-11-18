##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.0)
set(PROJECT_NAME hmi_qt)
project(${PROJECT_NAME})
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CXX_STANDARD 11)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS rviz roscpp sensor_msgs std_msgs cv_bridge image_transport message_generation)

find_package(Qt5 REQUIRED Core Widgets Multimedia Charts Sql Network MultimediaWidgets Svg)
set(QT_LIBRARIES Qt5::Widgets Qt5::Charts Qt5::Sql Qt5::Network Qt5::MultimediaWidgets Qt5::Multimedia Qt5::Svg)

#yaml-cpp
file(GLOB YAML_CPP_LIBRARIES /usr/local/x86_64-linux-gnu/libyaml.cpp*.so*)
find_package(yaml-cpp REQUIRED)

#set(YAML_CPP_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/../../third_party/yaml-cpp/include/)
#set(YAML_CPP_LIBRARIES ${PROJECT_SOURCE_DIR}/../../third_party/yaml-cpp/lib/libyaml-cpp.so)

add_message_files(
  FILES
  HmiStatus.msg
)

# add_service_files(
#   FILES
#   SetWeldToothIndex.srv
# )

generate_messages(
  DEPENDENCIES
  geometry_msgs sensor_msgs std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPEND roscpp sensor_msgs std_msgs geometry_msgs message_generation tf commu_msgs tcp
  #DEPENDS
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/customControl
  ${PROJECT_SOURCE_DIR}/privateHeader
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include/generated
)

link_directories(${catkin_LIBRAY_DIRS})
##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h customControl/*.h privateHeader/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp customControl/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} yaml-cpp glog)
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_dependencies(${PROJECT_NAME} commu_msgs_gencpp)
