#!/usr/bin/env python

from cpsutils import read_defs, read_requires

defs = read_defs("set\s\(SDF_(.*VERSION)\s+([0-9]+)\)")

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "SDFormat",
  "Description": "Tiny but powerful single file wavefront obj loader",
  "License": "MIT",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Compat-Version": "%(MAJOR_VERSION)s.0.0",
  "Requires": {
    "ignition-math3": {
      "Version": "%(ignition-math3_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ignition-math3"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":sdformat" ],
  "Components": {
    "sdformat": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libsdformat.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs


print(content[1:])
