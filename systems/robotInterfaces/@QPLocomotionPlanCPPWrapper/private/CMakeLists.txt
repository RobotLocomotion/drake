include_directories(../..)

add_mex(setDuration setDuration.cpp)
target_link_libraries(setDuration drakeQPLocomotionPlan drakeUtil)
