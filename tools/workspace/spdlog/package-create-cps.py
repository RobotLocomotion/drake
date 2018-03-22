#!/usr/bin/env python2

from drake.tools.install.cpsutils import read_version_defs, read_requires

def_re = "project\(spdlog\sVERSION\s([0-9]+).([0-9]+).([0-9]+)"
defs = read_version_defs(def_re)

defs.update(read_requires())

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "spdlog",
  "Description": "Super fast C++ logging library",
  "License": "MIT",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Requires": {
    "fmt": {
      "Version": "%(fmt_VERSION)s",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":spdlog"],
  "Components": {
    "spdlog": {
      "Type": "interface",
      "Includes": ["@prefix@/include/spdlog"],
      "Definitions": [
        "HAVE_SPDLOG",
        "SPDLOG_FMT_EXTERNAL"
      ],
      "Requires": ["fmt:fmt-header-only"]
    }
  }
}
""" % defs

print(content[1:])
