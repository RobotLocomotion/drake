add_library_with_exports(LIB_NAME drakeShapes SOURCE_FILES Geometry.cpp Element.cpp VisualElement.cpp HeightMapTerrain.cpp)
target_link_libraries(drakeShapes spruce)

#enable_testing()
#add_subdirectory(test)

pods_install_libraries(drakeShapes)
drake_install_headers(DrakeShapes.h Geometry.h Element.h VisualElement.h HeightMapTerrain.h)
  
