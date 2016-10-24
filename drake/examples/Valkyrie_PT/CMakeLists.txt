if(lcm_FOUND)
add_executable(ValkyrieIKTest ValkyrieIKTest.cpp)
target_link_libraries(ValkyrieIKTest
        drakeCommon
        drakeIK
        drakeLCMSystem
        drakeLCMTypes
        drakeRBSystem)
endif()