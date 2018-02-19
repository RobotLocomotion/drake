#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs, read_requires

def_re = "#define IGNITION_RNDF_VERSION_FULL[\s]\"([0-9]+).([0-9]+).([0-9]+).*\""
defs = read_version_defs(def_re)

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ignition-rndf0",
  "Description": "Ignition RNDF is a portable C++ library for parsing RNDF road network files",
  "License": "Apache-2.0",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Requires": {
    "ignition-math4": {
      "Version": "%(ignition-math4_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/ignition-math4"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":ignition-rndf0"],
  "Components": {
    "ignition-rndf0": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libignition_rndf.so",
      "Includes": ["@prefix@/include/ignition-rndf0"],
      "Requires": ["ignition-math4:ignition-math4"]
    }
  }
}
""" % defs

print(content[1:])
