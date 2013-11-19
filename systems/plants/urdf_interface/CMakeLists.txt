
find_package( Boost REQUIRED  )
include_directories ( ${Boost_INCLUDE_DIR} )
link_directories( ${Boost_LIBRARY_DIRS} )

add_library(drakeURDFinterface joint.cpp link.cpp ../tinyxml/tinyxml.cpp ../tinyxml/tinyxmlparser.cpp ../tinyxml/tinyxmlerror.cpp)
target_link_libraries(drakeURDFinterface ${Boost_LIBRARIES})

pods_install_libraries(drakeURDFinterface)
