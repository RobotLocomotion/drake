
add_mex(qpSplineMex csolve.c ldl.c matrix_support.c solver.c util.c)
set_target_properties(qpSplineMex PROPERTIES COMPILE_FLAGS "-Wall -Os")
target_link_libraries(qpSplineMex m)

install(TARGETS qpSplineMex DESTINATION matlab)

add_library(qpSpline solver.c ldl.c matrix_support.c util.c)
pods_install_libraries(qpSpline)
pods_install_headers(solver.h DESTINATION qpSpline)
pods_install_pkg_config_file(qpSpline
  LIBS -lqpSpline
  REQUIRES
  VERSION 0.0.1)