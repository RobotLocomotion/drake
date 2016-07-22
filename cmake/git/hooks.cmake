function(drake_setup_git_hooks)
  set(_git_dir "${PROJECT_SOURCE_DIR}/.git")
  if(NOT IS_DIRECTORY "${_git_dir}")
    return()
  endif()

  # Use a comment to hold the launcher version number.
  set(_hook_version_regex "Drake pre-commit hook launcher version '([0-9.]+)'")
  set(_in "${PROJECT_SOURCE_DIR}/cmake/git/pre-commit.in")
  set(_out "${_git_dir}/hooks/pre-commit")

  # Extract version of our hook launcher.
  file(STRINGS "${_in}" _in_version_line REGEX "${_hook_version_regex}" LIMIT_INPUT 1024)
  if(_in_version_line MATCHES "${_hook_version_regex}")
    set(_in_version "${CMAKE_MATCH_1}")
  else()
    message(FATAL_ERROR "Failed to extract version from '${_in}'")
  endif()

  # Extract version of hook launcher currently installed.
  if(EXISTS "${_out}")
    file(STRINGS "${_out}" _out_version_line REGEX "${_hook_version_regex}" LIMIT_INPUT 1024)
    if(_out_version_line MATCHES "${_hook_version_regex}")
      set(_out_version "${CMAKE_MATCH_1}")
    else()
      set(_out_version 1000000) # hook has unexpected content, do not replace it
    endif()
  else()
    set(_out_version 0)
  endif()

  # Install the hook launcher, but replace an existing launcher
  # only if it is an older version of our launcher.
  if(_in_version VERSION_GREATER _out_version)
    configure_file("${_in}" "${_out}" @ONLY)
  endif()
endfunction()
