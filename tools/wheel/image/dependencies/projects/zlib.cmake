ExternalProject_Add(zlib
    URL ${zlib_url}
    URL_MD5 ${zlib_md5}
    DOWNLOAD_NAME ${zlib_dlname}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
    PATCH_COMMAND ${CMAKE_COMMAND}
        -Dzlib_patch=${CMAKE_SOURCE_DIR}/patches/zlib
        -Dzlib_source=${CMAKE_BINARY_DIR}/src/zlib
        -P ${CMAKE_SOURCE_DIR}/patches/zlib/patch.cmake
)

# Note: the zlib license does not require the license notice to be included in
# non-source distributions.
