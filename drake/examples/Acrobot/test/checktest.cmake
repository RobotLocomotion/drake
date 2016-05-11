message("Checking ${targetfile}")

function(checkit path)
  if(EXISTS ${path})
    message("${path} exists")
    execute_process(COMMAND ${path}
      RESULT_VARIABLE res
      OUTPUT_VARIABLE out
      ERROR_VARIABLE err)
    message("run res out err [${res}] [${out}] [${err}]")
  else()
    message("${path} does not exist")
  endif()
endfunction()


checkit(${targetfile})
