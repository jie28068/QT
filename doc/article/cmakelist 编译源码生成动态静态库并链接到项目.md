- **当我们使用vscode编译c++代码时，需要加入第三方代码，而它没有库时。这时候我们就需要自己写一个Cmakelist编译成库，然后链接到自己的项目上。**

- 下面我以qt的`qtpropertybrowser`类为例，这个类并不在qt的标准库中，若是在qtcreator中使用，需要在`pro`引入该文件路径(**qt安装目录里-\Qt\5.15.2\Src\qttools\src\shared\qtpropertybrowser**)，该pri文件如下

  ```c_cpp
  include(../common.pri)
  greaterThan(QT_MAJOR_VERSION, 4): QT *= widgets
  INCLUDEPATH += $$PWD
  DEPENDPATH += $$PWD
  
  qtpropertybrowser-uselib:!qtpropertybrowser-buildlib {
      LIBS += -L$$QTPROPERTYBROWSER_LIBDIR -l$$QTPROPERTYBROWSER_LIBNAME
  } else {
      DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0
      SOURCES += $$PWD/qtpropertybrowser.cpp \
              $$PWD/qtpropertymanager.cpp \
              $$PWD/qteditorfactory.cpp \
              $$PWD/qtvariantproperty.cpp \
              $$PWD/qttreepropertybrowser.cpp \
              $$PWD/qtbuttonpropertybrowser.cpp \
              $$PWD/qtgroupboxpropertybrowser.cpp \
              $$PWD/qtpropertybrowserutils.cpp
      HEADERS += $$PWD/qtpropertybrowser.h \
              $$PWD/qtpropertymanager.h \
              $$PWD/qteditorfactory.h \
              $$PWD/qtvariantproperty.h \
              $$PWD/qttreepropertybrowser.h \
              $$PWD/qtbuttonpropertybrowser.h \
              $$PWD/qtgroupboxpropertybrowser.h \
              $$PWD/qtpropertybrowserutils_p.h
      RESOURCES += $$PWD/qtpropertybrowser.qrc
  }
  
  win32 {
      contains(TEMPLATE, lib):contains(CONFIG, shared):DEFINES += QT_QTPROPERTYBROWSER_EXPORT
      else:qtpropertybrowser-uselib:DEFINES += QT_QTPROPERTYBROWSER_IMPORT
  }
  
  ```

- 简单的转为Cmakelist,如下

  ```c_cpp
  project(qtpropertybrowser)
  
  file(GLOB SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
  file(GLOB HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/*.h)
  
  add_library(qtpropertybrowser STATIC ${SRC_FILES} ${HEADER_FILES})
  
  target_link_libraries(qtpropertybrowser
      PRIVATE
      Qt5::Widgets
  )
  
  target_include_directories(qtpropertybrowser
      PUBLIC
      ${CMAKE_CURRENT_LIST_DIR}
  )
  ```

- **上述为编译为静态库，若是动态库则需要改为如下**，`STATIC`改为`SHARED`，加入`-DQT_QTPROPERTYBROWSER_EXPORT`的导出配置，这是因为`qtpropertybrowser`是动态库它原本的qmake中也写了，需要导出信息，否则会报错。

```c_cpp
project(qtpropertybrowser)

file(GLOB SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
file(GLOB HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/*.h)

add_library(qtpropertybrowser SHARED  ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(qtpropertybrowser
    PRIVATE
    Qt5::Widgets
)

 target_compile_definitions(qtpropertybrowser
     PRIVATE
     -DQT_QTPROPERTYBROWSER_EXPORT
 )

target_include_directories(qtpropertybrowser
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)
```

- 如此，便使源码生成了库，可以链接到项目中了，需要使用的项目中的cmakelist中加入以下语句

```c_cpp

add_subdirectory(qtpropertybrowser)
target_link_libraries(
    .....
    qtpropertybrowser
)
```
