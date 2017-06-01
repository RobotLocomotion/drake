#!/usr/bin/env python

import re
import sys

def_re = re.compile("#define IPOPT_(VERSION[^\s]+)\s+([^\s]+)")

defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "IPOPT",
  "Description": "Interior Point Optimizer",
  "License": [
    "EPL-1.0",
    "METIS"
  ],
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_RELEASE)s",
  "Default-Components": [":ipopt"],
  "Components": {
    "ipopt": {
      "Type": "archive",
      "Location": "@prefix@/lib/libipopt.a",
      "Includes": ["@prefix@/include/ipopt"]
    }
  }
}
""" % defs

print(content[1:])
