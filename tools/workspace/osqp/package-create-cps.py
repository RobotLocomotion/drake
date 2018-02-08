#!/usr/bin/env python

from drake.tools.install.cpsutils import read_version_defs

defs = read_version_defs("set\(OSQP_VERSION\s([0-9]+).([0-9]+).([0-9]+)")

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "osqp",
  "Description": "The Operator Splitting QP Solver",
  "License": "Apache-2.0",
  "Version": "%s",
  "Default-Components": [":osqp"],
  "Components": {
    "osqp-header-only": {
      "Type": "interface",
      "Includes": ["@prefix@/include/osqp"]
    },
    "osqp": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libosqp.so",
      "Requires": [":osqp-header-only"]
    }
  }
}
""" % defs

print(content[1:])
