set(GIT_REVISION HEAD)
set(BUILD_IDENTIFIER unknown)

if(DEFINED DRAKE_GIT_SHA_OVERRIDE)
  set(GIT_REVISION "${DRAKE_GIT_SHA_OVERRIDE}")
else()
  if(GIT_EXECUTABLE AND EXISTS "${GIT_DIR}")
    execute_process(COMMAND
      "${GIT_EXECUTABLE}" "--git-dir=${GIT_DIR}" rev-parse HEAD
      RESULT_VARIABLE GIT_REV_PARSE_RESULT_VARIABLE
      OUTPUT_VARIABLE GIT_REV_PARSE_OUTPUT_VARIABLE
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    if(GIT_REV_PARSE_RESULT_VARIABLE EQUAL 0)
      set(GIT_REVISION "${GIT_REV_PARSE_OUTPUT_VARIABLE}")
      string(SUBSTRING ${GIT_REVISION} 0 8 GIT_REVISION_SHORT)
      set(BUILD_IDENTIFIER git${GIT_REVISION_SHORT})
    endif()
  endif()
endif()

if(DEFINED DRAKE_VERSION_OVERRIDE)
  set(DRAKE_VERSION "${DRAKE_VERSION_OVERRIDE}")
else()
  string(TIMESTAMP BUILD_TIMESTAMP "%Y%m%d.%H%M%S")
  string(REGEX REPLACE "[.]0+([0-9])" ".\\1"
    DRAKE_VERSION "0.0.${BUILD_TIMESTAMP}+${BUILD_IDENTIFIER}")
endif()

configure_file(${INPUT_FILE} ${OUTPUT_FILE} @ONLY)
