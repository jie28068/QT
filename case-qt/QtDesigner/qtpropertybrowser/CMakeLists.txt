project(qtpropertybrowser)

file(GLOB SRC_FILES ${CMAKE_CURRENT_LIST_DIR}/*.cpp)
file(GLOB HEADER_FILES ${CMAKE_CURRENT_LIST_DIR}/*.h)

add_library(qtpropertybrowser STATIC ${SRC_FILES} ${HEADER_FILES})

target_link_libraries(qtpropertybrowser
    PRIVATE
    Qt5::Widgets
)

# target_compile_definitions(qtpropertybrowser
#     PRIVATE
#     -DQT_QTPROPERTYBROWSER_EXPORT
# )

target_include_directories(qtpropertybrowser
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)