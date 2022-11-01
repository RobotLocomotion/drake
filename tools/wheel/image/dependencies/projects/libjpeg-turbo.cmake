ExternalProject_Add(libjpeg-turbo
    URL ${libjpeg-turbo_url}
    URL_MD5 ${libjpeg-turbo_md5}
    ${COMMON_EP_ARGS}
    ${COMMON_CMAKE_EP_ARGS}
    CMAKE_ARGS
        ${COMMON_CMAKE_ARGS}
        -DCMAKE_ASM_NASM_COMPILER=yasm
)

extract_license(libjpeg-turbo
    LICENSE.md
)
