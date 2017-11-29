#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs

defs = read_defs("#define\s+EIGEN_(\w+_VERSION)\s+([0-9]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "Eigen3",
  "Description": "Lightweight C++ template library for vector and matrix math",
  "License": ["MPL-2.0", "LGPL-2.1+", "BSD-3-Clause"],
  "Version": "%(WORLD_VERSION)s.%(MAJOR_VERSION)s.%(MINOR_VERSION)s",
  "Default-Components": [ ":Eigen" ],
  "Components": {
    "Eigen": {
      "Type": "interface",
      "Includes": [ "@prefix@/include/eigen3" ]
    }
  },
  "X-CMake-Variables": {
    "EIGEN_FOUND": "ON",
    "EIGEN_INCLUDE_DIRS": "${CMAKE_CURRENT_LIST_DIR}/../../../include/eigen3"
  }
}
""" % defs

print(content[1:])
