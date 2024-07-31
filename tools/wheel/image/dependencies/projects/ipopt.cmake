if(NOT APPLE)
    message(FATAL_ERROR "mumps should only be built on macOS")
endif()

ExternalProject_Add(ipopt
    URL ${ipopt_url}
    URL_MD5 ${ipopt_md5}
    DOWNLOAD_NAME ${ipopt_dlname}
    DEPENDS mumps
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        FFLAGS=-fPIC
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
        LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
        --with-lapack-lflags=-framework\ Accelerate
        --with-mumps-lflags=-ldmumps\ -lmpiseq\ -lmumps_common\ -lpord
        --with-mumps-cflags=-I${CMAKE_INSTALL_PREFIX}/include
        CPPFLAGS=-I${CMAKE_INSTALL_PREFIX}/include/mumps_seq
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

extract_license(ipopt
    LICENSE
)
