add_library(drakePolynomial Polynomial.cpp)
pods_install_libraries(drakePolynomial)
pods_install_headers(Polynomial.h DESTINATION drake)
pods_install_pkg_config_file(drakePolynomial
LIBS -ldrakePolynomial
REQUIRES
VERSION 0.0.2)

add_subdirectory(test)
