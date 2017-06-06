#!/usr/bin/env python

from cpsutils import read_defs, read_requires

defs = read_defs("set[\s]\(PROJECT_(.*_VERSION)\s+([^\s]+)\)")

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ignition_rndf",
  "Description": "Ignition RNDF is a portable C++ library for parsing RNDF road network files",
  "License": "Apache-2.0",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Requires": {
    "ignition_math": {
      "Version": "%(ignition_math_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ignition_math"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":ignition_rndf" ],
  "Components": {
    "ignition_rndf": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libignition_rndf.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs

print(content[1:])
