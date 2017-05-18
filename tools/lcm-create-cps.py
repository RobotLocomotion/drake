#!/usr/bin/env python

import re
import sys

def_re = re.compile("#define LCM_(VERSION[^\s]+)\s+([^\s]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

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
