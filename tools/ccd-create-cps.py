#!/usr/bin/env python

import re
import sys

def_re = re.compile("set\(CCD_(VERSION_\w+)\s([0-9]+)")
defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

# Keep Components.ccd.Definitions in sync with config.defines in
# ccd.BUILD.
content = """
{
  "Cps-Version": "0.8.0",
  "Name": "ccd",
  "Description": "Library for collision detection between two convex shapes",
  "License": "BSD-3-Clause-Clear",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s",
  "Default-Components": [ ":ccd" ],
  "Components": {
    "ccd": {
      "Type": "dylib",
      "Definitions": ["CCD_DOUBLE"],
      "Location": "@prefix@/lib/libccd.so"
    }
  }
}
""" % defs

print(content[1:])
