ExternalProject_Add(lapack
    URL ${lapack_url}
    URL_MD5 ${lapack_md5}
    DOWNLOAD_NAME ${lapack_dlname}
    ${COMMON_EP_ARGS}
    CMAKE_GENERATOR "Unix Makefiles"
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
        -DCBLAS=ON
)

extract_license(lapack
    LICENSE
)
