{
  "Cps-Version": "0.8.0",
  "Name": "optitrack",
  "Description": "Translates data streamed from the Optitrack Motive software (a.k.a. NatNet) into LCM messages",
  "License": "BSD-3-Clause",
  "Requires": {
    "lcm": {
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":lcmtypes_optitrack-cpp"],
  "Components": {
    "lcmtypes_optitrack-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/lcmtypes"]
    },
    "lcmtypes_optitrack-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/liblcmtypes_optitrack.jar",
      "Requires": ["lcm:lcm-java"]
    }
  }
}
