
add_library_with_exports(LIB_NAME drakeEstimators SOURCE_FILES luenberger_observer.cc)
target_link_libraries(drakeEstimators drakeSystemFramework)

drake_install_libraries(drakeEstimators)
drake_install_headers(luenberger_observer.h)
drake_install_pkg_config_file(drake-estimators
    TARGET drakeEstimators
    LIBS -ldrakeEstimators)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()
