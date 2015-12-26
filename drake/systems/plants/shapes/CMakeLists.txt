include_directories ( .. )

add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES Geometry.cpp Element.cpp VisualElement.cpp)
target_link_libraries(drakeShapes spruce)

#enable_testing()
#add_subdirectory(test)

pods_install_libraries(drakeShapes)
pods_install_headers(DrakeShapes.h Geometry.h Element.h VisualElement.h DESTINATION drake/shapes)
  
