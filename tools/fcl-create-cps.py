#!/usr/bin/env python

import re
import sys

def_re = re.compile("set\(FCL_(\w+_VERSION)\s+([0-9]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "fcl",
  "Description": "Flexible Collision Library",
  "License": ["BSD-3-Clause"],
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Compat-Version": "%(MAJOR_VERSION)s.0.0",
  "Requires": {
    "ccd": {
      "Version": "2.0",
      "Hints": ["@prefix@/lib/cmake/ccd"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "Eigen3": {
      "Version": "3.3.3",
      "Hints": ["@prefix@/lib/cmake/eigen3"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "octomap": {
      "Version": "1.8.0",
      "Hints": ["@prefix@/lib/cmake/octomap"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":fcl" ],
  "Components": {
    "fcl": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libfcl.so",
      "Includes": [ "@prefix@/include/fcl" ],
      "Requires": [ "ccd:ccd", "Eigen3:Eigen", "octomap:octomap" ]
    }
  }
}
""" % defs

print(content[1:])
