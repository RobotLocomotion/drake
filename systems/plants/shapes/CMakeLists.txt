include_directories ( .. )

add_library(drakeShapes SHARED Geometry.cpp Element.cpp VisualElement.cpp)

#enable_testing()
#add_subdirectory(test)

pods_install_libraries(drakeShapes)
pods_install_headers(DrakeShapes.h Geometry.h Element.h VisualElement.h drakeShapesMacros.h DESTINATION drake/shapes)
  
