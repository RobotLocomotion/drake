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

# Keep Components.config.Definitions in sync with config.defines in
# fcl.BUILD.
content = """
{
  "Cps-Version": "0.8.0",
  "Name": "fcl",
  "Description": "Flexible Collision Library",
  "License": ["BSD-3-Clause-Clear"],
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Compat-Version": "%(MAJOR_VERSION)s.0.0",
  "Requires": {
    "ccd": {
      "Version": "2.0",
      "Hints": ["@prefix@/lib/cmake/ccd"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "eigen": {
      "Version": "3.3.3",
      "Hints": ["@prefix@/lib/cmake/eigen"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "octomap": {
      "Version": "1.8.0",
      "Hints": ["@prefix@/lib/cmake/octomap"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":config", ":fcl_h_genrule", ":fcl" ],
  "Components": {
    "fcl_h_genrule": {
      "Type": "interface",
      "Includes": [ "@prefix@/include/fcl" ]
    },
    "config": {
      "Type": "interface",
      "Definitions": ["FCL_HAVE_OCTOMAP"],
      "Includes": [ "@prefix@/include/fcl" ]
    },
    "fcl": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libfcl.so",
      "Requires": [ ":fcl_h_genrule", ":config", "ccd:ccd", "Eigen3:Eigen", "octomap:octomap" ]
    }
  }
}
""" % defs

print(content[1:])
