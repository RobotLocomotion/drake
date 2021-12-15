ExternalProject_Add(bzip2
    URL ${bzip2_url}
    URL_MD5 ${bzip2_md5}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND make "CFLAGS=-fPIC -O2"
    INSTALL_COMMAND make PREFIX=${CMAKE_INSTALL_PREFIX} install
    )

extract_license(bzip2
    LICENSE
)
