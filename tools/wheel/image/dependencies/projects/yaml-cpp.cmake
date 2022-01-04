ExternalProject_Add(yaml-cpp
    URL ${yaml-cpp_url}
    URL_MD5 ${yaml-cpp_md5}
    DOWNLOAD_NAME ${yaml-cpp_dlname}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
        -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON
        -DYAML_CPP_BUILD_CONTRIB:BOOL=OFF
        -DYAML_CPP_BUILD_TESTS:BOOL=OFF
        -DYAML_CPP_BUILD_TOOLS:BOOL=OFF
)

extract_license(yaml-cpp
    LICENSE
)
