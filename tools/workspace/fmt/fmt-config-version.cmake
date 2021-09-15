# This file is only used in cases when we need to build and/or install fmt from
# source, i.e., when we are not using pkg-config.  See repository.bzl for the
# logic to select when that occurs.

set(PACKAGE_VERSION "6.1.2")
set(PACKAGE_COMPAT_VERSION "6.1.2")

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

