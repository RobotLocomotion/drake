{
  "Cps-Version": "0.8.0",
  "Name": "bot2-core",
  "Description": "????",
  "License": "BSD-3-Clause",
  "Default-Components": [":lcmtypes_bot2-core-cpp"],
  "Components": {
    "lcmtypes_bot2-core-cpp": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libbot_core_lcmtypes.so",
      "Includes": ["@prefix@/include/bot2-core"]
    }
  }
}
