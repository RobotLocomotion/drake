{
  "Cps-Version": "0.8.0",
  "Name": "robotlocomotion-lcmtypes",
  "Description": "????",
  "License": "BSD-3-Clause",
  "Default-Components": [":robotlocomotion-lcmtypes-cpp"],
  "Components": {
    "robotlocomotion-lcmtypes-cpp": {
      "Type": "dylib",
      "Location": "@prefix@/lib/librobotlocomotion_lcmtypes.so",
      "Includes": ["@prefix@/include/robotlocomotion-lcmtypes"]
    }
  }
}
