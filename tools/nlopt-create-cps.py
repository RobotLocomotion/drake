#!/usr/bin/env python

from cpsutils import read_defs

defs = read_defs("#define ([^\s]+_VERSION)\s+([^\s]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "NLopt",
  "Description": "Nonlinear Optimizer",
  "License": [
    "BSD-2-Clause",
    "BSD-3-Clause",
    "LGPL-2.1+",
    "MIT"
  ],
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(BUGFIX_VERSION)s",
  "Default-Components": [":nlopt"],
  "Components": {
    "nlopt": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libnlopt.so",
      "Includes": ["@prefix@/include/nlopt"]
    }
  }
}
""" % defs

print(content[1:])
