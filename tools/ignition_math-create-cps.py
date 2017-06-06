#!/usr/bin/env python

from cpsutils import read_version_defs

def_re = "#define IGNITION_MATH_VERSION_FULL[\s]\"([0-9]+).([0-9]+).([0-9]+)\""
defs = read_version_defs(def_re)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ignition-math3",
  "Description": "Math classes and functions for robot applications",
  "License": "Apache-2.0",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [ ":ignition-math3" ],
  "Components": {
    "ignition-math3": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libignition_math.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs

print(content[1:])
