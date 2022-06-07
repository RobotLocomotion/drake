if(NOT APPLE)
    message(FATAL_ERROR "mumps should only be built on macOS")
endif()

ExternalProject_Add(mumps
    URL ${mumps_url}
    URL_MD5 ${mumps_md5}
    DOWNLOAD_NAME ${mumps_dlname}
    ${COMMON_EP_ARGS}
    BUILD_IN_SOURCE 1
    PATCH_COMMAND ${CMAKE_COMMAND}
        -Dmumps_patch=${CMAKE_SOURCE_DIR}/patches/mumps
        -Dmumps_source=${CMAKE_BINARY_DIR}/src/mumps
        -P ${CMAKE_SOURCE_DIR}/patches/mumps/patch.cmake
    CONFIGURE_COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_BINARY_DIR}/src/mumps/Make.inc/Makefile.macos.SEQ
        ${CMAKE_BINARY_DIR}/src/mumps/Makefile.inc
    BUILD_COMMAND make d
    INSTALL_COMMAND make PREFIX=${CMAKE_INSTALL_PREFIX} install
    )

extract_license(mumps
    LICENSE
)
