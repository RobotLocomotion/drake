#!/usr/bin/env python

import re
import sys

def_re = re.compile("#define\s+EIGEN_(\w+_VERSION)\s+([0-9]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "Eigen3",
  "Description": "Lightweight C++ template library for vector and matrix math",
  "License": ["MPL-2.0", "LGPL-2.1+", "BSD-3-Clause"],
  "Version": "%(WORLD_VERSION)s.%(MAJOR_VERSION)s.%(MINOR_VERSION)s",
  "Compat-Version": "%(WORLD_VERSION)s.0.0",
  "Default-Components": [ ":Eigen" ],
  "Components": {
    "Eigen": {
      "Type": "interface",
      "Includes": [ "@prefix@/include/eigen3" ]
    }
  }
}
""" % defs

print(content[1:])
