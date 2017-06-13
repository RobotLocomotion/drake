{
  "Cps-Version": "0.8.0",
  "Name": "bot2-core-lcmtypes",
  "Description": "LCM types for common sensor signals",
  "License": "BSD-3-Clause",
  "Requires": {
    "lcm": {
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":lcmtypes_bot2-core-cpp"],
  "Components": {
    "lcmtypes_bot2-core-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "lcmtypes_bot2-core-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_bot2-core-java.jar",
      "Requires": ["lcm:lcm-java"]
    }
  }
}
