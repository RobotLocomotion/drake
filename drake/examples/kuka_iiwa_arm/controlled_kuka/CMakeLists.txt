add_executable(kuka_demo kuka_demo.cc)
target_link_libraries(kuka_demo drake_rbs drakeSystemAnalysis drakeRBSystem
        drakeBotVisualizerSystem drakeLCMSystem2 drakeIK drakeTrajectories
        ${GTEST_BOTH_LIBRARIES})
#add_test(NAME kuka_demo COMMAND kuka_demo)

pods_find_pkg_config(lcmtypes_bot2-core)
add_executable(kuka_plan_listener_demo kuka_plan_listener_demo.cc)
target_link_libraries(kuka_plan_listener_demo
    drakeKukaIiwaArm drake_rbs drakeSystemAnalysis drakeRBSystem
    drakeLCMSystem2 ${GTEST_BOTH_LIBRARIES})
add_test(NAME kuka_plan_listener_demo COMMAND kuka_plan_listener_demo)

add_executable(kuka_director_plan_control_demo kuka_director_plan_control_demo.cc)
target_link_libraries(kuka_director_plan_control_demo
    drakeKukaIiwaArm drake_rbs drakeSystemAnalysis drakeRBSystem
    drakeLCMSystem2 ${GTEST_BOTH_LIBRARIES})
#add_test(NAME kuka_plan_kuka_director_plan_control_demolistener_demo COMMAND kuka_director_plan_control_demo)

pods_use_pkg_config_packages(kuka_plan_listener_demo
    lcmtypes_bot2-core)
include_directories(${CMAKE_INSTALL_PREFIX}/include/lcmtypes)