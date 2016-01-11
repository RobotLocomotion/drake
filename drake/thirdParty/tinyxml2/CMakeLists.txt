add_library(tinyxml2 SHARED tinyxml2.cpp)
pods_install_libraries(tinyxml2)
pods_install_headers(tinyxml2.h DESTINATION drake)
