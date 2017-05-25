#!/usr/bin/env python

import re
import sys

def_re = re.compile("set\(FMT_VERSION\s([0-9]+).([0-9]+).([0-9]+)")
defs = {}
with open(sys.argv[1]) as h:
    for l in h:
        m = def_re.match(l)
        if m is not None:
            defs['VERSION_MAJOR'] = m.group(1)
            defs['VERSION_MINOR'] = m.group(2)
            defs['VERSION_PATCH'] = m.group(3)

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "fmt",
  "Description": "Small, safe and fast formatting library",
  "License": "BSD-2-Clause",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [ ":fmt" ],
  "Components": {
    "fmt-header-only": {
      "Type": "interface",
      "Includes": [ "@prefix@/include" ]
    },
    "fmt": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libfmt.so",
      "Requires": [ ":fmt-header-only" ]
    }
  }
}
""" % defs

print(content[1:])
