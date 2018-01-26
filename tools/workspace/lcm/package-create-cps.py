#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs

defs = read_defs("#define LCM_(VERSION[^\s]+)\s+([^\s]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "lcm",
  "Description": "Lightweight Communications and Marshaling library",
  "License": "LGPL-2.1+",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Requires": {
    "jchart2d": {
      "Hints": ["@prefix@/lib/cmake/jchart2d"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":lcm"],
  "Components": {
    "lcm-coretypes": {
      "Type": "interface",
      "Includes": ["@prefix@/include"]
    },
    "lcm": {
      "Type": "dylib",
      "Includes": ["@prefix@/include"],
      "Location": "@prefix@/lib/liblcm.so",
      "Requires": [":lcm-coretypes"]
    },
    "lcm-gen": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-gen"
    },
    "lcm-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcm.jar",
      "Requires": ["jchart2d:jchart2d"]
    },
    "lcm-logger": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-logger"
    },
    "lcm-logplayer": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-logplayer"
    },
    "lcm-logplayer-gui": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-logplayer-gui"
    },
    "lcm-spy": {
      "Type": "exe",
      "Location": "@prefix@/bin/lcm-spy"
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
