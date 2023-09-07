ExternalProject_Add(xz
    URL ${xz_url}
    URL_MD5 ${xz_md5}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    # autogen.sh generates configure
    CONFIGURE_COMMAND ./autogen.sh && ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

# Note: Although various utilities are under [L]GPL, we only use liblzma, which
# is Public Domain.
