#!/usr/bin/env python2

from drake.tools.install.cpsutils import read_version_defs

defs = read_version_defs("#define JSON_VERSION ([0-9]{1,2})([0-9]{2})([0-9]{2})$")

# Skip leading zeros if any.
print defs
assert len(defs) == 3
defs = dict([(k, int(v)) for k, v in defs.iteritems()])

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "json",
  "Description": "JSON for Modern C++",
  "License": "MIT",
  "Version": "%(VERSION_MAJOR)s.%(VERSION_MINOR)s.%(VERSION_PATCH)s",
  "Default-Components": [":json"],
  "Components": {
    "json-header-only": {
      "Type": "interface",
      "Definitions": ["JSON_HEADER_ONLY=1"],
      "Includes": ["@prefix@/include/json"]
    }
  }
}
""" % defs

print(content[1:])
