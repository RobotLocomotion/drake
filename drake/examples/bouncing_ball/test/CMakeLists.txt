if(lcm_FOUND)
  add_executable(ball_test
	ball_test.cc
      )
  target_link_libraries(ball_test
    ${GTEST_BOTH_LIBRARIES} drakeBouncingBall)
  drake_add_test(NAME ball_test
    COMMAND ball_test)

  add_executable(bouncing_ball_test
	bouncing_ball_test.cc
      )
  target_link_libraries(bouncing_ball_test
    ${GTEST_BOTH_LIBRARIES} drakeBouncingBall)
  drake_add_test(NAME bouncing_ball_test
    COMMAND bouncing_ball_test)

endif()
