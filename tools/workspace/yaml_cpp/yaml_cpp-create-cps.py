#!/usr/bin/env python

from drake.tools.install.cpsutils import read_defs

defs = read_defs("set\(YAML_CPP_(VERSION_.*)\s+\"([0-9]+)\"")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "yaml-cpp",
  "Description": "A YAML parser and emitter in C++",
  "License": "MIT",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_MINOR)s",
  "Default-Components": [ ":yaml-cpp" ],
  "Components": {
    "yaml-cpp": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libyaml_cpp.so",
      "Includes": [ "@prefix@/include" ]
    }
  }
}
""" % defs

print(content[1:])
