# write_procman(absolute_filename [GROUP groupname] [NAME commandname COMMAND command] [ENDGROUP] ...)
#
# Helper script for generating a procman via cmake (which is useful, when you are installing executables in lots of random places)
#
# see examples/Pendulum/CMakeLists.txt for an example
function (write_procman filename)
  message(STATUS "Writing ${filename} procman script")
  set(pmd "# This file was generated automatically by cmake.  Edit at your own risk\n\n")
  set(argnum 0)
  list(LENGTH ARGV numargs)

  set(tab_whitespace "")

  macro(write_commands)
    list(GET ARGV ${argnum} arg)
    if (arg STREQUAL "ENDGROUP")
      math(EXPR argnum ${argnum}+1)
      break()
    elseif (arg STREQUAL "GROUP")
      math(EXPR argnum ${argnum}+1)
      list(GET ARGV ${argnum} name)
      list(APPEND pmd "group \"${name}\" {\n")

      set(tab_whitespace_backup "${tab_whitespace}")
      set(tab_whitespace "${tab_whitespace}  ")

      math(EXPR argnum ${argnum}+1)
      write_commands()

      set(tab_whitespace "${tab_whitespace_backup}")

      list(APPEND pmd "}\n")
    elseif (arg STREQUAL "NAME")
      math(EXPR argnum ${argnum}+1)
      list(GET ARGV ${argnum} name)

      math(EXPR argnum ${argnum}+1)
      list(GET ARGV ${argnum} command)
      if (NOT command STREQUAL "COMMAND")
        message(FATAL_ERROR "syntax is NAME name COMMAND command")  # just for the sake of readiability
      endif()
      math(EXPR argnum ${argnum}+1)
      list(GET ARGV ${argnum} command)

      list(APPEND pmd
          "${tab_whitespace}cmd \"${name}\" {\n"
          "${tab_whitespace}  exec = \"${command}\"\;\n"
          "${tab_whitespace}  host = \"localhost\"\;\n"  # todo: consider taking this as part of the input
          "${tab_whitespace}}\n")
    endif()
    math(EXPR argnum ${argnum}+1)
  endmacro()

  while (argnum LESS numargs)
    write_commands()
  endwhile()

  file(WRITE ${filename} ${pmd})

endfunction()
