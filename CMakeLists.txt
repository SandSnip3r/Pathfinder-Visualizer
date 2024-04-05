cmake_minimum_required(VERSION 3.26.4)

project(PathfinderVisualization LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

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
    qt_add_executable(${PROJECT_NAME}
        ${PROJECT_SOURCES}
        ${RESOURCE_FILES}
    )
else()
    if(ANDROID)
        add_library(${PROJECT_NAME} SHARED
            ${PROJECT_SOURCES}
            ${RESOURCE_FILES}
        )
    else()
        add_executable(${PROJECT_NAME}
            ${PROJECT_SOURCES}
            ${RESOURCE_FILES}
        )
    endif()
endif()

target_link_libraries(${PROJECT_NAME}
PRIVATE
  Qt${QT_VERSION_MAJOR}::Widgets
  Pathfinder
PUBLIC
  absl::log
  absl::log_initialize
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)