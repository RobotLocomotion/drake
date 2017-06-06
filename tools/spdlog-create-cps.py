#!/usr/bin/env python

import re
import sys

def_re = re.compile("project\(spdlog\sVERSION\s([0-9]+).([0-9]+).([0-9]+)")
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
  "Name": "spdlog",
  "Description": "Super fast C++ logging library",
  "License": "MIT",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Requires": {
    "fmt": {
      "Version": "3.0.1",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": [ "CONFIG" ]
    }
  },
  "Default-Components": [ ":spdlog" ],
  "Components": {
    "spdlog": {
      "Type": "interface",
      "Includes": [ "@prefix@/include" ],
      "Definitions": ["SPDLOG_FMT_EXTERNAL", "HAVE_SPDLOG"],
      "Requires": [ "fmt:fmt" ]
    }
  }
}
""" % defs

print(content[1:])
