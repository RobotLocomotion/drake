#!/usr/bin/env python

import sys

with open(sys.argv[1]) as h:
    version = h.readline().strip()

content = """
{
  "Cps-Version": "0.8.0",
  "Name": "Bullet",
  "Description": "Real-time collision detection and multi-physics simulation",
  "License": ["Zlib"],
  "Version": "%s",
  "Default-Components": [
      ":BulletCollision",
      ":LinearMath"
  ],
  "Components": {
    "BulletCollision": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libBulletCollision.so",
      "Includes": ["@prefix@/include/bullet"],
      "Requires": [":LinearMath"]
    },
    "LinearMath": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libLinearMath.so",
      "Includes": ["@prefix@/include/bullet"]
    }
  }
}
""" % version

print(content[1:])
