#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs
import pprint

defs = read_defs("set\(CCD_VERSION_(MAJOR|MINOR)\s+([0-9]+)\)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ccd",
  "Description": "Library for collision detection between two convex shapes",
  "License": "BSD-3-Clause",
  "Version": "%(MAJOR)s.%(MINOR)s",
  "Default-Components": [":ccd"],
  "Components": {
    "ccd": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libccd.so",
      "Includes": ["@prefix@/include"]
    }
  }
}
""" % defs

print(content[1:])
