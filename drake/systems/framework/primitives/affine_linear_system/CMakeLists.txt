add_library_with_exports(LIB_NAME drakeAffineLinearSystemPlant SOURCE_FILES
    affine_system_plant.cc
    linear_system_plant.cc)
target_link_libraries(drakeAffineLinearSystemPlant drakeSystemFramework)

drake_install_headers(
    affine_system_plant.h
    linear_system_plant.h)
pods_install_libraries(drakeAffineLinearSystemPlant)
pods_install_pkg_config_file(drake-affine-linear-system-plant
    CFLAGS -I${CMAKE_INSTALL_PREFIX}/include
    LIBS -ldrakeAffineLinearSystemPlant -ldrakeSystemFramework
    VERSION 0.0.1)
