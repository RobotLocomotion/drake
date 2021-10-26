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
