# Find glib-2.0 and optional related components

include(FindPackageHandleStandardArgs)

#------------------------------------------------------------------------------
function(_glib2_find_include VAR HEADER)
  list(APPEND CMAKE_PREFIX_PATH $ENV{GLIB_PATH})

  set(_paths)
  foreach(_lib ${ARGN})
    get_filename_component(_libpath ${GLIB2_${_lib}_LIBRARY} DIRECTORY)
    list(APPEND _paths ${_libpath})
  endforeach()

  find_path(GLIB2_${VAR}_INCLUDE_DIR ${HEADER}
    PATHS ${_paths}
    PATH_SUFFIXES glib-2.0 glib-2.0/include
  )
  mark_as_advanced(GLIB2_${VAR}_INCLUDE_DIR)
endfunction()

#------------------------------------------------------------------------------
function(_glib2_find_library VAR LIB)
  list(APPEND CMAKE_PREFIX_PATH $ENV{GLIB_PATH})

  find_library(GLIB2_${VAR}_LIBRARY NAMES ${LIB}-2.0 ${LIB})
  mark_as_advanced(GLIB2_${VAR}_LIBRARY)

  if(WIN32)
    find_program(GLIB2_${VAR}_RUNTIME NAMES lib${LIB}-2.0-0.dll)
    mark_as_advanced(GLIB2_${VAR}_RUNTIME)
  endif()
endfunction()

#------------------------------------------------------------------------------
function(_glib2_add_target TARGET LIBRARY)
  set(GLIB2_${TARGET}_FIND_QUIETLY TRUE)
  set(_deps GLIB2_${LIBRARY}_LIBRARY)
  foreach(_include ${ARGN})
    list(APPEND _deps GLIB2_${_include}_INCLUDE_DIR)
  endforeach()

  find_package_handle_standard_args(GLib2_${TARGET}
    FOUND_VAR GLib2_${TARGET}_FOUND
    REQUIRED_VARS ${_deps}
  )

  if(GLib2_${TARGET}_FOUND)
    set(GLib2_${TARGET}_FOUND TRUE PARENT_SCOPE)

    set(_target GLib2::${TARGET})
    add_library(${_target} UNKNOWN IMPORTED)
    set_property(TARGET ${_target} APPEND PROPERTY
      IMPORTED_LOCATION ${GLIB2_${LIBRARY}_LIBRARY}
    )
    foreach(_include ${ARGN})
      set_property(TARGET ${_target} APPEND PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES ${GLIB2_${_include}_INCLUDE_DIR}
      )
    endforeach()
  endif()
endfunction()

###############################################################################

_glib2_find_library(GLIB glib)
_glib2_find_include(GLIB glib.h)
_glib2_find_include(GLIBCONFIG glibconfig.h GLIB)

_glib2_add_target(glib GLIB GLIB GLIBCONFIG)

if(WIN32 AND TARGET GLib2::glib)
  set_property(TARGET GLib2::glib APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES ws2_32 winmm
  )
endif()

foreach(_glib2_component ${GLib2_FIND_COMPONENTS})

  if(_glib2_component STREQUAL "gio")

    _glib2_find_include(GIO gio/gio.h)
    _glib2_find_library(GIO gio)

    _glib2_add_target(gio GIO GIO GMODULE GOBJECT GLIB GLIBCONFIG)

  elseif(_glib2_component STREQUAL "gmodule")

    _glib2_find_include(GMODULE gmodule.h)
    _glib2_find_library(GMODULE gmodule)

    _glib2_add_target(gmodule GMODULE GMODULE GLIB GLIBCONFIG)

  elseif(_glib2_component STREQUAL "gobject")

    _glib2_find_include(GOBJECT glib-object.h)
    _glib2_find_library(GOBJECT gobject)

    _glib2_add_target(gobject GOBJECT GOBJECT GLIB GLIBCONFIG)

  elseif(_glib2_component STREQUAL "gthread")

    _glib2_find_library(GTHREAD gthread)

    _glib2_add_target(gthread GTHREAD GLIB GLIBCONFIG)

  endif()

endforeach()

list(APPEND GLib2_FIND_COMPONENTS glib)
set(GLib2_FIND_REQUIRED_glib TRUE)

find_package_handle_standard_args(GLib2
  REQUIRED_VARS GLIB2_GLIB_LIBRARY
  HANDLE_COMPONENTS
)
