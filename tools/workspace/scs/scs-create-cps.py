#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs

defs = read_version_defs("\W+\(\"([0-9]+).([0-9]+).([0-9]+)\"\)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "scs",
  "Description": "Conic optimization via operator splitting.",
  "License": [
    "LGPL-2.1+",
    "MIT"
  ],
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [":scsdir"],
  "Components": {
    "scsdir": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libscsdir.so",
      "Definitions": ["LAPACK_LIB_FOUND=1"],
      "Includes": ["@prefix@/include/scs"]
    },
    "scsindir": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libscsindir.so",
      "Definitions": ["LAPACK_LIB_FOUND=1"],
      "Includes": ["@prefix@/include/scs"]
    }
  }
}
""" % defs

print(content[1:])
