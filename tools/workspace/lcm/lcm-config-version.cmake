# Drake's install of LCM alongside Drake (both as part of our CMake install,
# and in our binary packages) is deprecated and will be removed on 2025-05-01.
#
# If you still need LCM tools in your own project, add LCM as a CMake external
# to your project.

set(PACKAGE_VERSION "1.5.1")
set(PACKAGE_COMPAT_VERSION "1.5.1")

if(PACKAGE_VERSION VERSION_LESS PACKAGE_FIND_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
elseif(PACKAGE_VERSION VERSION_LESS COMPAT_VERSION)
  set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
  set(PACKAGE_VERSION_COMPATIBLE TRUE)
  if(PACKAGE_VERSION STREQUAL PACKAGE_FIND_VERSION)
    set(PACKAGE_VERSION_EXACT TRUE)
  endif()
endif()

unset(PACKAGE_COMPAT_VERSION)

