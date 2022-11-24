execute_process(
    COMMAND git apply ${zlib_patch}/patch.diff
    WORKING_DIRECTORY ${zlib_source}
    )
