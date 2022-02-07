ExternalProject_Add(clp
    URL ${clp_url}
    URL_MD5 ${clp_md5}
    DOWNLOAD_NAME ${clp_dlname}
    DEPENDS coinutils
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

extract_license(clp
    Clp/LICENSE
)
