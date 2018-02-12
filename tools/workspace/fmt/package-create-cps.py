#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs

defs = read_version_defs("set\(FMT_VERSION\s([0-9]+).([0-9]+).([0-9]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "fmt",
  "Description": "Small, safe and fast formatting library",
  "License": "BSD-2-Clause",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [":fmt"],
  "Components": {
    "fmt-header-only": {
      "Type": "interface",
      "Definitions": ["FMT_HEADER_ONLY=1"],
      "Includes": ["@prefix@/include/fmt"]
    }
  }
}
""" % defs

print(content[1:])
