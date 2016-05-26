if(roscpp_FOUND)
  # add_library_with_exports(LIB_NAME drakeROS
  #     SOURCE_FILES )

  drake_install_headers(drake_ros_tf_publisher.h
                        ros_vehicle_system.h
                        sensor_visualizer_lidar.h)
endif()
