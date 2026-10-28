cmake_minimum_required(VERSION 3.8)
project(rviz_camera_stream)

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

## Compiler flags
# if(CMAKE_COMPILER_IS_GNUCXX)
#  set(CMAKE_CXX_FLAGS "-std=c++0x")  ## Enable C++11
# endif()
# set(CMAKE_CXX_FLAGS "-Wall -Werror ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)

## This plugin includes Qt widgets, so we must include Qt.
## rviz2 uses Qt5.
find_package(Qt5 REQUIRED Core Widgets)
## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
set(QT_LIBRARIES Qt5::Widgets)

## Avoid defining "emit", "slots",
## etc for Qt signals because they can conflict with boost signals
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/camera_display.h
)

add_library(${PROJECT_NAME} SHARED
  src/camera_display.cpp
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  image_transport::image_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  Boost::thread
  ${QT_LIBRARIES}
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# install
install (TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
