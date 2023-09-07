ExternalProject_Add(xz
    URL ${xz_url}
    URL_MD5 ${xz_md5}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
    )

# Note: Although various utilities are under [L]GPL, we only use liblzma, which
# is Public Domain.
