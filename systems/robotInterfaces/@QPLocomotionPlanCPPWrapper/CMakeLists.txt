include_directories(..)

add_mex(getQPControllerInput getQPControllerInput.cpp)
target_link_libraries(getQPControllerInput drakeQPLocomotionPlan drakeUtil)

add_mex(start_time start_time.cpp)
target_link_libraries(start_time drakeQPLocomotionPlan drakeUtil)

add_mex(setStartTime setStartTime.cpp)
target_link_libraries(setStartTime drakeQPLocomotionPlan drakeUtil)

add_mex(getLastQPInput getLastQPInput.cpp)
target_link_libraries(getLastQPInput drakeQPLocomotionPlan drakeUtil)

add_mex(isFinished isFinished.cpp)
target_link_libraries(isFinished drakeQPLocomotionPlan drakeUtil)

add_mex(duration duration.cpp)
target_link_libraries(duration drakeQPLocomotionPlan drakeUtil)

add_subdirectory(private)
