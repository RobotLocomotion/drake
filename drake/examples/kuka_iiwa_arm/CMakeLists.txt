if(lcm_FOUND)
  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
  add_library_with_exports(LIB_NAME drakeKukaIiwaArmCommon SOURCE_FILES
      iiwa_common.cc iiwa_lcm.cc)
  target_link_libraries(drakeKukaIiwaArmCommon
      drakeIK
      drakeLcmSystem
      drakeRBM
      drakeMultibodyParsers
      robotlocomotion-lcmtypes-cpp
      Eigen3::Eigen)
  drake_install_libraries(drakeKukaIiwaArmCommon)
  drake_install_headers(
      iiwa_common.h iiwa_lcm.h)
  drake_install_pkg_config_file(drake-kuka-iiwa-arm-common
      TARGET drakeKukaIiwaArmCommon
      LIBS -ldrakeKukaIiwaArmCommon
      REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm eigen3)
endif()

if(BUILD_TESTING)
  add_subdirectory(test)
endif()


#if(lcm_FOUND)
#  # Defines a shared library for use by KUKA iiwa demos based on System 2.0.
#  add_library_with_exports(LIB_NAME drakeKukaIiwaArmCommon SOURCE_FILES
#          iiwa_common.cc iiwa_lcm.cc robot_plan_interpolator.cc)
#  target_link_libraries(drakeKukaIiwaArmCommon
#          drakeIK
#          drakeLcmSystem
#          drakeRBM
#          drakeMultibodyParsers
#          robotlocomotion-lcmtypes-cpp
#          Eigen3::Eigen)
#  drake_install_libraries(drakeKukaIiwaArmCommon)
#  drake_install_headers(
#          iiwa_common.h iiwa_lcm.h robot_plan_interpolator.h)
#  drake_install_pkg_config_file(drake-kuka-iiwa-arm-common
#          TARGET drakeKukaIiwaArmCommon
#          LIBS -ldrakeKukaIiwaArmCommon
#          REQUIRES drake-lcm-system drake-lcmtypes-cpp drake-rbm eigen3)
#endif()
#
#if(BUILD_TESTING)
#  add_subdirectory(test)
#endif()
