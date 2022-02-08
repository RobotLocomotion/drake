ExternalProject_Add(ipopt
    URL ${ipopt_url}
    URL_MD5 ${ipopt_md5}
    DOWNLOAD_NAME ${ipopt_dlname}
    DEPENDS lapack
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        FFLAGS=-fPIC
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
        LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

extract_license(ipopt
    Ipopt/LICENSE
)
