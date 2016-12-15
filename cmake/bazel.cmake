#------------------------------------------------------------------------------
# Create a CMake library target named ${OUTPUT_LIB}Bazel, where OUTPUT_LIB.pic.a
# is a library that Bazel builds in the CMAKE_CURRENT_SOURCE_DIR.
#------------------------------------------------------------------------------
macro(drake_add_bazel_library OUTPUT_LIB)
  get_filename_component(_superbuild_dir ${PROJECT_SOURCE_DIR} DIRECTORY)
  set(_bazel_bin ${_superbuild_dir}/bazel-bin)
  set(_current_dir ${CMAKE_CURRENT_SOURCE_DIR})
  file(RELATIVE_PATH _current_relpath ${_superbuild_dir} ${_current_dir})
  set(_file ${_bazel_bin}/${_current_relpath}/${OUTPUT_LIB}.pic.a)

  add_custom_target(
      _${OUTPUT_LIB}_target
      COMMAND bazel build ...
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Shelling out to bazel in ${_current_relpath}"
  )

  set(_lib_name ${OUTPUT_LIB}Bazel)
  add_library(${_lib_name} STATIC IMPORTED)
  add_dependencies(${_lib_name} _${OUTPUT_LIB}_target)
  set_target_properties(${_lib_name} PROPERTIES IMPORTED_LOCATION ${_file})
endmacro()
