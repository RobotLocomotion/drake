set(PACKAGE_VERSION "3.3.2")
set(PACKAGE_COMPAT_VERSION "3.3.2")

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

