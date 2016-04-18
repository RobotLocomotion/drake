cmake_minimum_required(VERSION 2.8.3)
project(drake)

if(roscpp_FOUND)
  # The catkin and genmsg packages are required to generate ROS message code from
  # their specification files.
  find_package(catkin REQUIRED)
  find_package(genmsg REQUIRED)

  find_package(std_msgs REQUIRED)
  include_directories(${std_msgs_INCLUDE_DIRS})

  # Enumerate our custom messages files
  add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES
    rost_viewer_draw.msg
    rost_viewer_geometry_data.msg
    rost_viewer_link_data.msg
    rost_viewer_load_robot.msg)

  # Do code generation, specifying which other message packages we depend on.
  generate_messages(DEPENDENCIES std_msgs)
endif()
