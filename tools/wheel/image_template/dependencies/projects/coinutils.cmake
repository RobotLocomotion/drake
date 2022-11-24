ExternalProject_Add(coinutils
    URL ${coinutils_url}
    URL_MD5 ${coinutils_md5}
    DOWNLOAD_NAME ${coinutils_dlname}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

extract_license(coinutils
    CoinUtils/LICENSE
)
