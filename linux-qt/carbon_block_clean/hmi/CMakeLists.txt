##############################################################################
# CMake
##############################################################################
cmake_minimum_required(VERSION 2.8.0)
set(PROJECT_NAME hmi)
project(${PROJECT_NAME})
include(./grpc.cmake)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CXX_STANDARD 11)

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(Eigen3 REQUIRED)

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Qt5 REQUIRED Core Widgets Multimedia Charts Sql Network MultimediaWidgets)
set(QT_LIBRARIES Qt5::Widgets Qt5::Charts Qt5::Sql Qt5::Network Qt5::MultimediaWidgets Qt5::Multimedia)

set(JSON_PATH ${PROJECT_SOURCE_DIR}/../utils/)

#yaml-cpp
file(GLOB YAML_CPP_LIBRARIES /usr/local/x86_64-linux-gnu/libyaml.cpp*.so*)
find_package(yaml-cpp REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/include/generated
  ${JSON_PATH}
  ${JSONCPP_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRAY_DIRS})

aux_source_directory(${PROJECT_SOURCE_DIR}/../utils/ UTILS_SOURCES)

function(grpc_generate_cpp SRC_FILES HDR_FILES)
  if (NOT ARGN)
    message(SEND_ERROR "Error: grpc_generate_cpp() called without any proto files")
    return()
  endif()

  set(${SRC_FILES})
  set(${HDR_FILES})

  foreach(proto_file ${ARGN})
    get_filename_component(proto_name ${proto_file} NAME_WE)
    get_filename_component(proto_full_path ${proto_file} ABSOLUTE)
    file(RELATIVE_PATH proto_relative_path
      ${CMAKE_CURRENT_SOURCE_DIR}/${INCLUDE_ROOT} ${proto_full_path})
    get_filename_component(relative_directory ${proto_relative_path} DIRECTORY)

    if (NOT relative_directory)
      set(proto_name_full "${proto_name}")
    else()
      set(proto_name_full "${relative_directory}/${proto_name}")
    endif()

    set(generated_pb_cc "${CMAKE_CURRENT_BINARY_DIR}/${proto_name_full}.pb.cc")
    set(generated_pb_h "${CMAKE_CURRENT_BINARY_DIR}/${proto_name_full}.pb.h")
    set(generated_grpc_pb_cc "${CMAKE_CURRENT_BINARY_DIR}/${proto_name_full}.grpc.pb.cc")
    set(generated_grpc_pb_h "${CMAKE_CURRENT_BINARY_DIR}/${proto_name_full}.grpc.pb.h")

    list(APPEND ${SRC_FILES} "${generated_pb_cc}")
    list(APPEND ${HDR_FILES} "${generated_pb_h}")
    list(APPEND ${SRC_FILES} "${generated_grpc_pb_cc}")
    list(APPEND ${HDR_FILES} "${generated_grpc_pb_h}")

    add_custom_command(
      OUTPUT "${generated_pb_cc}"
      "${generated_pb_h}"
      "${generated_grpc_pb_cc}"
      "${generated_grpc_pb_h}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out=${CMAKE_CURRENT_BINARY_DIR}
      --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
      --plugin=protoc-gen-grpc=${_GRPC_CPP_PLUGIN_EXECUTABLE}
      -I ${CMAKE_CURRENT_SOURCE_DIR}
      ${proto_relative_path}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      DEPENDS)
  endforeach()

  set_source_files_properties(${${SRC_FILES}} ${${HDR_FILES}} PROPERTIES GENERATED TRUE)
  set(${SRC_FILES} ${${SRC_FILES}} PARENT_SCOPE)
  set(${HDR_FILES} ${${HDR_FILES}} PARENT_SCOPE)
endfunction()

grpc_generate_cpp(HOST_SRC HOST_HDR 
  ${PROJECT_SOURCE_DIR}/msg/hostCompute.proto
)
grpc_generate_cpp(SCHE_SRC SCHE_HDR 
  ${PROJECT_SOURCE_DIR}/msg/scheduling.proto
)

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# grpc
add_library(hmi_grpc_proto
  ${HOST_SRC}
  ${HOST_HDR}
  ${SCHE_SRC}
  ${SCHE_HDR}
  )
target_link_libraries(hmi_grpc_proto
  ${_REFLECTION}
  ${_GRPC_GRPCPP}
  ${_PROTOBUF_LIBPROTOBUF})
#qt

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui customTable/*.ui carbonPage/*.ui adjustPage/*.ui customControl/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
${CMAKE_CURRENT_LIST_DIR}/*.h include/*.hpp *.h customControl/*.h customTable/*.h carbonPage/*.h communication/*.h adjustPage/*.h)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS 
src/*.cpp customControl/*.cpp customTable/*.cpp carbonPage/*.cpp communication/*.cpp adjustPage/*.cpp)

##############################################################################
add_executable(${PROJECT_NAME} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP} ${GRPC_SRCS} ${UTILS_SOURCES})

target_include_directories(${PROJECT_NAME} 
    PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/customControl
    ${CMAKE_CURRENT_LIST_DIR}/customTable
    ${CMAKE_CURRENT_LIST_DIR}/carbonPage
    ${CMAKE_CURRENT_LIST_DIR}/communication
    ${CMAKE_CURRENT_LIST_DIR}/adjustPage

)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES} 
  ${YAML_CPP_LIBRARIES} 
  ${_GRPC_GRPCPP} 
  hmi_grpc_proto
  glog
  ${_PROTOBUF_LIBPROTOBUF}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
