#!/usr/bin/env python

import re
import sys

def_re = re.compile("#define PYBIND11_(VERSION[^\s]+)\s+([^\s]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "pybind11",
  "Description": "Seamless operability between C++11 and Python",
  "License": "BSD-3-Clause",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [":module"],
  "Components": {
    "module": {
      "Type": "interface",
      "Includes": ["@prefix@/include"],
      "Compile-Features": ["c++11"]
    }
  },
  "X-CMake-Includes": ["${CMAKE_CURRENT_LIST_DIR}/pybind11Tools.cmake"]
}
""" % defs

print(content[1:])
