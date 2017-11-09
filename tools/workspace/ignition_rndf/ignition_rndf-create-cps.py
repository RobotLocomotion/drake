#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs, read_requires

defs = read_defs("set[\s]\(PROJECT_(.*_VERSION)\s+([^\s]+)\)")

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ignition-rndf0",
  "Description": "Ignition RNDF is a portable C++ library for parsing RNDF road network files",
  "License": "Apache-2.0",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Requires": {
    "ignition-math3": {
      "Version": "%(ignition-math3_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ignition-math3"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":ignition-rndf0" ],
  "Components": {
    "ignition-rndf0": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libignition_rndf.so",
      "Includes": [ "@prefix@/include" ],
      "Requires": [ "ignition-math3:ignition-math3" ]
    }
  }
}
""" % defs

print(content[1:])
