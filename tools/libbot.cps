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
      "Link-Flags": ["-lglib-2.0"],
      "Location": "@prefix@/lib/libbot2_core.so",
      "Requires": [
        "bot2-core-lcmtypes:lcmtypes_bot2-core",
        "lcm:lcm"
      ]
    },
    "lcmtypes_bot2-param": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Location": "@prefix@/lib/liblcmtypes_bot2_param_c.so",
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-param-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-param-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_bot2_param.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "bot2-param-client": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/libbot"],
      "Link-Flags": ["-lglib-2.0", "-lgthread-2.0"],
      "Location": "@prefix@/lib/libbot2_param_client.so",
      "Requires": [
        ":bot2-core",
        ":lcmtypes_bot2-param",
        "lcm:lcm"
      ]
    },
    "lcmtypes_bot2-frames": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Location": "@prefix@/lib/liblcmtypes_bot2_frames_c.so",
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-frames-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-frames-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_bot2_frames.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "bot2-frames": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/libbot"],
      "Link-Flags": ["-lglib-2.0"],
      "Location": "@prefix@/lib/libbot2_frames.so",
      "Requires": [
        ":bot2-core",
        ":bot2-param-client",
        ":lcmtypes_bot2-frames",
        "lcm:lcm"
      ]
    },
    "lcmtypes_bot2-lcmgl": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Location": "@prefix@/lib/liblcmtypes_bot2_lcmgl_c.so",
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-lcmgl-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-lcmgl-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_bot2_lcmgl.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "bot2-lcmgl-client": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/libbot"],
      "Link-Flags": ["-lglib-2.0"],
      "Location": "@prefix@/lib/libbot2_lcmgl_client.so",
      "Requires": [
        ":lcmtypes_bot2-lcmgl",
        "lcm:lcm"
      ]
    },
    "bot2-lcmgl-render": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/libbot"],
      "Location": "@prefix@/lib/libbot2_lcmgl_client.so",
      "Requires": [
        ":bot2-lcmgl-client",
        ":lcmtypes_bot2-lcmgl",
        "lcm:lcm"
      ]
    },
    "bot2-lcmgl-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/bot2_lcmgl.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "lcmtypes_bot2-procman": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Location": "@prefix@/lib/liblcmtypes_bot2_procman_c.so",
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-procman-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-procman-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_bot2_procman.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "bot-procman-deputy": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-procman-deputy"
    },
    "bot-procman-sheriff": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-procman-sheriff"
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
    "bot-param-dump": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-param-dump"
    },
    "bot-param-server": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-param-server"
    },
    "bot-param-tool": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-param-tool"
    },
    "bot-spy": {
      "Type": "exe",
      "Location": "@prefix@/bin/bot-spy"
    }
  }
}
