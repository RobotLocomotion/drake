#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs

defs = read_defs("set\(OCTOMAP_(\w+_VERSION)\s([0-9]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "octomap",
  "Description": "An efficient probabilistic 3D mapping framework based on octrees.",
  "License": "BSD-3-Clause",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Default-Components": [ ":octomap", ":octomath" ],
  "Components": {
    "octomath": {
      "Type": "dylib",
      "Location": "@prefix@/lib/liboctomath.so",
      "Includes": [ "@prefix@/include" ]
    },
    "octomap": {
      "Type": "dylib",
      "Location": "@prefix@/lib/liboctomap.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs

print(content[1:])
