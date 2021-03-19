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
        mainwindow.cpp
        mainwindow.h
        navmeshrenderarea.cpp
        navmeshrenderarea.hpp
        zoomablescrollarea.cpp
        zoomablescrollarea.hpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(PathfinderVisualization
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(PathfinderVisualization SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(PathfinderVisualization
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_compile_features(PathfinderVisualization PUBLIC cxx_std_11)

target_link_libraries(PathfinderVisualization PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(PathfinderVisualization PRIVATE Pathfinder)
