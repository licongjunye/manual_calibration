cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_BUILD_TYPE "Debug")
project(manualcalibra)
add_compile_options(-std=c++17)
set(QT_VERSION_MAJOR 5)
set(QT_VERSION_MINOR 12) # 设置你的Qt版本
set(QT_VERSION_PATCH 0)
set(QT_VERSION "${QT_VERSION_MAJOR}.${QT_VERSION_MINOR}.${QT_VERSION_PATCH}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  cv_bridge
  pcl_conversions
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/qt
)
set(HEADERS
  qt/mygraphicsview.h
)


set(SRC_FILES
  main.cpp 
  qt/widget.cpp
  qt/mygraphicsview.cpp
)

qt5_wrap_cpp(MOC_FILES qt/widget.h)

add_executable(${PROJECT_NAME}_node ${SRC_FILES} ${MOC_FILES} ${UI_FILES})


target_sources(${PROJECT_NAME}_node
  PRIVATE
  ${MOC_FILES}
  ${HEADERS}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBS}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
  Qt5::Core 
  Qt5::Gui 
  Qt5::Widgets
)


install(TARGETS ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
