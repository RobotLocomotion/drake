if(APPLE)
    execute_process(
        COMMAND gfortran --print-file-name=libgfortran.dylib
        OUTPUT_VARIABLE _gfortran_lib
        RESULT_VARIABLE _gfortran_result
        )

    if(NOT _gfortran_result EQUAL 0)
        message(FATAL_ERROR "gfortran was not found")
    endif()

    get_filename_component(_gfortran_lib "${_gfortran_lib}" REALPATH)
    get_filename_component(_gfortran_libdir "${_gfortran_lib}" DIRECTORY)

    set(LIBRT)
    set(LDFLAGS_GFORTRAN "-L${_gfortran_libdir} -lgfortran")
else()
    set(LIBRT "-lrt")
    set(LDFLAGS_GFORTRAN "-lgfortran")
endif()

file(
    RENAME
    ${suitesparse_source}/SuiteSparse_config/SuiteSparse_config.mk
    ${suitesparse_source}/SuiteSparse_config/SuiteSparse_config.mk.in
    )

execute_process(
    COMMAND git apply ${suitesparse_patch}/patch.diff
    WORKING_DIRECTORY ${suitesparse_source}
    )

configure_file(
    ${suitesparse_source}/SuiteSparse_config/SuiteSparse_config.mk.in
    ${suitesparse_source}/SuiteSparse_config/SuiteSparse_config.mk
    @ONLY
    )
