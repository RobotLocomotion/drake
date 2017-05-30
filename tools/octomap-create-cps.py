#!/usr/bin/env python

import re
import sys

def_re = re.compile("set\(OCTOMAP_(\w+_VERSION)\s([0-9]+)")
defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs[m.group(1)] = m.group(2)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "octomap",
  "Description": "An efficient probabilistic 3D mapping framework based on octrees.",
  "License": "BSD-3-Clause",
  "Version": "%(MAJOR_VERSION)s.%(MINOR_VERSION)s.%(PATCH_VERSION)s",
  "Default-Components": [ ":octomap" ],
  "Components": {
    "octomap": {
      "Type": "dylib",
      "Location": "@prefix@/lib/liboctomap.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs

print(content[1:])
