#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs, read_requires

defs = read_defs("set\s\(SDF_(.*VERSION)\s+([0-9]+)\)")

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "SDFormat",
  "Description": "SDF is an XML file format that describes environments, objects, and robots in a manner suitable for robotic applications",
  "License": "Apache-2.0",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
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
      "Includes": [ "@prefix@/include" ],
      "Link-Flags": [ "-ltinyxml" ],
      "Requires": [ "ignition-math3:ignition-math3" ]
    }
  }
}
""" % defs


print(content[1:])
