if(roscpp_FOUND)
  add_library_with_exports(LIB_NAME drakeROS
      SOURCE_FILES drake_ros_tf_publisher.cc drake_ros_tf_publisher.h
                   sensor_visualizer_lidar.cc sensor_visualizer_lidar.h)

  drake_install_headers(drake_ros_tf_publisher.h
                        sensor_visualizer_lidar.h)
endif()