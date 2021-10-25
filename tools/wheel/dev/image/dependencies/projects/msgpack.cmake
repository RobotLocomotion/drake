ExternalProject_Add(msgpack
    URL ${msgpack_url}
    URL_MD5 ${msgpack_md5}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
)

extract_license(msgpack
    LICENSE_1_0.txt
    COPYING
    NOTICE
)
