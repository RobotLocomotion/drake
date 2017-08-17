{
  "Cps-Version": "0.8.0",
  "Name": "robotlocomotion-lcmtypes",
  "Description": "LCM types for common sensor signals",
  "License": "BSD-3-Clause",
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
  "Default-Components": [":robotlocomotion-lcmtypes-cpp"],
  "Components": {
    "robotlocomotion-lcmtypes": {
      "Type": "dylib",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Location": "@prefix@/lib/liblcmtypes_robotlocomotion_c.so",
      "Requires": [
        "bot2-core-lcmtypes:lcmtypes_bot2-core",
        "lcm:lcm-coretypes"
      ]
    },
    "robotlocomotion-lcmtypes-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": [
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "lcm:lcm-coretypes"
      ]
    },
    "robotlocomotion-lcmtypes-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_robotlocomotion.jar",
      "Requires": [
        "bot2-core-lcmtypes:lcmtypes_bot2-core-java",
        "lcm:lcm-java"
      ]
    }
  }
}
