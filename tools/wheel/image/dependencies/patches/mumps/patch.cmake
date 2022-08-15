execute_process(
    COMMAND git apply ${mumps_patch}/patch.diff
    WORKING_DIRECTORY ${mumps_source}
    )
