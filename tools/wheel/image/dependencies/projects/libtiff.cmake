ExternalProject_Add(libtiff
    DEPENDS zlib xz libjpeg-turbo
    URL ${libtiff_url}
    URL_MD5 ${libtiff_md5}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
        -DJPEG_INCLUDE=${CMAKE_INSTALL_PREFIX}/include
        -DJPEG_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/libjpeg.a
        -DLIBLZMA_INCLUDE=${CMAKE_INSTALL_PREFIX}/include
        -DLIBLZMA_LIBRARY=${CMAKE_INSTALL_PREFIX}/lib/liblzma.a
        -DZLIB_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
        -DZLIB_LIBRARY_DEBUG=${CMAKE_INSTALL_PREFIX}/lib/libz.a
        -DZLIB_LIBRARY_RELEASE=${CMAKE_INSTALL_PREFIX}/lib/libz.a
        )

extract_license(libtiff
    COPYRIGHT
)
