#!/usr/bin/env python

from cpsutils import read_defs

defs = read_defs("#define LCM_(VERSION[^\s]+)\s+([^\s]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "lcm",
  "Description": "Lightweight Communications and Marshaling library",
  "License": "LGPL-2.1+",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [ ":lcm" ],
  "Components": {
    "lcm-coretypes": {
      "Type": "interface",
      "Includes": [ "@prefix@/include" ]
    },
    "lcm": {
      "Type": "dylib",
      "Location": "@prefix@/lib/liblcm.so",
      "Requires": [ ":lcm-coretypes" ]
    },
    "lcm-gen": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-gen"
    },
    "lcm-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcm.jar"
    }
  },
  "X-CMake-Variables": {
    "LCM_NAMESPACE": "lcm::",
    "LCM_VERSION": "${lcm_VERSION}",
    "LCM_USE_FILE": "${CMAKE_CURRENT_LIST_DIR}/lcmUtilities.cmake"
  }
}
""" % defs

print(content[1:])
