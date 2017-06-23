#!/usr/bin/env python

import sys

with open(sys.argv[1]) as h:
    version = h.readline().strip()

# Keep Components.LinearMath.Definitions in sync with LinearMath.defines in
# bullet.BUILD.
content = """
{
  "Cps-Version": "0.8.0",
  "Name": "Bullet",
  "Description": "Real-time collision detection and multi-physics simulation",
  "License": "Zlib",
  "Version": "%s",
  "Default-Components": [
    ":BulletCollision",
    ":BulletDynamics"
  ],
  "Components": {
    "BulletCollision": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libBulletCollision.so",
      "Includes": ["@prefix@/include/bullet"],
      "Requires": [":LinearMath"]
    },
    "BulletDynamics": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libBulletDynamics.so",
      "Includes": ["@prefix@/include/bullet"],
      "Requires": [
        ":BulletCollision",
        ":LinearMath"
      ]
    },
    "LinearMath": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libLinearMath.so",
      "Definitions": ["BT_USE_DOUBLE_PRECISION"],
      "Includes": ["@prefix@/include/bullet"]
    },
    "BulletSoftBody": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libBulletSoftBody.so",
      "Includes": ["@prefix@/include/bullet"],
      "Requires": [
        ":BulletCollision",
        ":BulletDynamics",
        ":LinearMath"
      ]
    }
  }
}
""" % version

print(content[1:])
