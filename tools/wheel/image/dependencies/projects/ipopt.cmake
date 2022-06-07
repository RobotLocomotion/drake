if(APPLE)
    set(ipopt_extra_dependencies mumps)
    set(ipopt_mumps_configure_args
        --with-mumps-lib=-ldmumps\ -lmumps_common\ -lpord\ -lmpiseq
        --with-mumps-incdir=${CMAKE_INSTALL_PREFIX}/include
        CPPFLAGS=-I${CMAKE_INSTALL_PREFIX}/include/mumps_seq
    )
else()
    set(ipopt_extra_dependencies)
    set(ipopt_mumps_configure_args
        --with-mumps-lib=-ldmumps_seq
        --with-mumps-incdir=/usr/include
        CPPFLAGS=-I/usr/include/mumps_seq
    )
endif()

ExternalProject_Add(ipopt
    URL ${ipopt_url}
    URL_MD5 ${ipopt_md5}
    DOWNLOAD_NAME ${ipopt_dlname}
    DEPENDS lapack ${ipopt_extra_dependencies}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ./configure
        --prefix=${CMAKE_INSTALL_PREFIX}
        --disable-shared
        FFLAGS=-fPIC
        CFLAGS=-fPIC
        CXXFLAGS=-fPIC
        LDFLAGS=-L${CMAKE_INSTALL_PREFIX}/lib
        ${ipopt_mumps_configure_args}
    BUILD_COMMAND make
    INSTALL_COMMAND make install
    )

extract_license(ipopt
    Ipopt/LICENSE
)
