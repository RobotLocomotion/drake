#!/usr/bin/env python

from cpsutils import read_defs, read_requires

defs = read_defs("set\s\(SDF_(.*VERSION)\s+([0-9]+)\)")

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "sdformat",
  "Description": "Tiny but powerful single file wavefront obj loader",
  "License": ["MIT"],
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Requires": {
    "ignition_math": {
      "Version": "%(ignition_math_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ignition_math"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Compat-Version": "%(MAJOR_VERSION)s.0.0",
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
