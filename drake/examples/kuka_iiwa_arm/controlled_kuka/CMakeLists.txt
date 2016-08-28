add_executable(kuka_demo kuka_demo.cc)
target_link_libraries(kuka_demo drake_rbs drakeSystemAnalysis drakeRBSystem
        ${GTEST_BOTH_LIBRARIES})
add_test(NAME kuka_demo COMMAND kuka_demo)
