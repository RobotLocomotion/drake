ExternalProject_Add(double-conversion
    URL ${double-conversion_url}
    URL_MD5 ${double-conversion_md5}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
    )

# Note: double-conversion ships both a COPYING and a LICENSE, but they are
# identical, so there is no need for us to ship both.
extract_license(double-conversion
    COPYING
)
