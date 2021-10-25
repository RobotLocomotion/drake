if(APPLE)
    set(_libjpeg-turbo_ARGS_APPLE --host x86_64-apple-darwin )
endif()

ExternalProject_Add(libjpeg-turbo
    URL ${libjpeg-turbo_url}
    URL_MD5 ${libjpeg-turbo_md5}
    DEPENDS ${libjpeg-turbo_DEPENDS}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        ${_libjpeg-turbo_ARGS_APPLE}
        NASM=yasm
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

extract_license(libjpeg-turbo
    README
)
