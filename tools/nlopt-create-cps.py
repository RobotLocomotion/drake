#!/usr/bin/env python

import re
import sys

def_re = re.compile("#define ([^\s]+_VERSION)\s+([^\s]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

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
