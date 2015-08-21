
include_directories(${PROJECT_SOURCE_DIR}/systems)
include_directories(${PROJECT_SOURCE_DIR}/systems/frames)

if (LCM_FOUND)
  include_directories(${PROJECT_SOURCE_DIR}/pod-build/lcmgen)
  add_executable(runPassivePendulum runPassive.cpp)
  target_link_libraries(runPassivePendulum drakeSystem)
  add_test(NAME runPassivePendulum COMMAND runPassivePendulum)
endif()