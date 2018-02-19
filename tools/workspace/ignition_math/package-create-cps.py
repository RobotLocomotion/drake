#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs

def_re = "#define IGNITION_MATH_VERSION_FULL[\s]\"([0-9]+).([0-9]+).([0-9]+).*\""
defs = read_version_defs(def_re)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ignition-math4",
  "Description": "Math classes and functions for robot applications",
  "License": "Apache-2.0",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [":ignition-math4"],
  "Components": {
    "ignition-math4": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libignition_math.so",
      "Includes": ["@prefix@/include/ignition-math4"]
    }
  },
  "X-CMake-Variables": {
    "IGNITION-MATH_INCLUDE_DIRS": "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/include",
    "IGNITION-MATH_LINK_DIRS": "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib",
    "IGNITION-MATH_LIBRARY_DIRS": "${${CMAKE_FIND_PACKAGE_NAME}_IMPORT_PREFIX}/lib",
    "IGNITION-MATH_LIBRARIES": "ignition_math"
  }
}
""" % defs

print(content[1:])
