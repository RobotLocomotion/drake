{
  "Cps-Version": "0.8.0",
  "Name": "libbot",
  "Description": "Libraries, tools, and algorithms for robotics research",
  "License": [
    "GPL-2.0+ WITH Vincent-Roca",
    "LGPL-2.1+ WITH Vincent-Roca",
    "LGPL-3.0+",
    "Radford-M-Neal"
  ],
  "Requires": {
    "bot2-core-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/bot2-core-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "lcm": {
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":bot2-core"],
  "Components": {
    "bot2-core": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/libbot"],
      "Location": "@prefix@/lib/libbot2_core.so",
      "Requires": [
        "bot2-core-lcmtypes:lcmtypes_bot2-core",
        "lcm:lcm"
      ]
    },
    "bot-lcm-logfilter": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-lcm-logfilter"
    },
    "bot-lcm-logsplice": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-lcm-logsplice"
    },
    "bot-lcm-tunnel": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-lcm-tunnel"
    },
    "bot-lcm-who": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-lcm-who"
    },
    "bot-spy": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-spy"
    }
  }
}
