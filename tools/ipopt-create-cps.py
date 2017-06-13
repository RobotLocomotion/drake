#!/usr/bin/env python

from cpsutils import read_defs

defs = read_defs("#define IPOPT_(VERSION[^\s]+)\s+([^\s]+)")

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
