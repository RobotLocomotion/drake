#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs, read_requires

defs = read_defs("set\(FCL_(\w+_VERSION)\s+([0-9]+)")
defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "fcl",
  "Description": "Flexible Collision Library",
  "License": ["BSD-3-Clause"],
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Requires": {
    "ccd": {
      "Version": "%(ccd_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ccd"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "Eigen3": {
      "Version": "%(Eigen3_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/eigen3"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    },
    "octomap": {
      "Version": "%(octomap_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/octomap"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":fcl" ],
  "Components": {
    "fcl": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libfcl.so",
      "Includes": [ "@prefix@/include" ],
      "Requires": [ "ccd:ccd", "Eigen3:Eigen", "octomap:octomap" ]
    }
  }
}
""" % defs

print(content[1:])
