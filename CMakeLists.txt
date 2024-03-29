cmake_minimum_required(VERSION 3.5)

project(PathfinderVisualization LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check https://doc.qt.io/qt/deployment-android.html for more information.
# They need to be set before the find_package( ...) calls below.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

#set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/Pathfinder")
#set (Pathfinder_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Pathfinder")

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
#find_package(Pathfinder REQUIRED PATHS "${CMAKE_CURRENT_SOURCE_DIR}/Pathfinder")

add_subdirectory(Pathfinder)

set(PROJECT_SOURCES
  main.cpp
  main_window.cpp
  main_window.hpp
  navmesh_display.hpp
  navmesh_display_base.cpp
  navmesh_display_base.hpp
  navmesh_render_area.hpp
  navmesh_render_area_base.cpp
  navmesh_render_area_base.hpp
  zoomable_scroll_area.cpp
  zoomable_scroll_area.hpp
)

set(RESOURCE_FILES
  resources.qrc
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(PathfinderVisualization
        ${PROJECT_SOURCES}
        ${RESOURCE_FILES}
    )
else()
    if(ANDROID)
        add_library(PathfinderVisualization SHARED
            ${PROJECT_SOURCES}
            ${RESOURCE_FILES}
        )
    else()
        add_executable(PathfinderVisualization
            ${PROJECT_SOURCES}
            ${RESOURCE_FILES}
        )
    endif()
endif()

target_compile_features(PathfinderVisualization PUBLIC cxx_std_11)

# target_link_libraries(PathfinderVisualization PRIVATE )
target_link_libraries(PathfinderVisualization
  PRIVATE
    Qt${QT_VERSION_MAJOR}::Widgets
    Pathfinder
)
target_include_directories(${PROJECT_NAME} PRIVATE ../../Combined/Combined/src)
