add_library(drakeAtlasUtil SHARED atlasUtil.cpp)
pods_install_libraries(drakeAtlasUtil)
pods_install_headers(atlasUtil.h DESTINATION drake)

