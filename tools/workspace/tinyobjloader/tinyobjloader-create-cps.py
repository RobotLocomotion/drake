#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs

defs = read_version_defs("set\(TINYOBJLOADER_VERSION\s+([0-9]+).([0-9]+).([0-9]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "tinyobjloader",
  "Description": "Tiny but powerful single file wavefront obj loader",
  "License": "MIT",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [ ":tinyobjloader" ],
  "Components": {
    "tinyobjloader": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libtinyobjloader.so",
      "Includes": [ "@prefix@/include/tinyobjloader" ]
    }
  }
}
""" % defs

print(content[1:])
