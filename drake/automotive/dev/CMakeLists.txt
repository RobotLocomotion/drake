if(BUILD_TESTING)
  add_subdirectory(test)
endif()

drake_install_headers(
  endless_road_car.h
  endless_road_oracle.h
  )
