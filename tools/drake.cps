{
  "Meta-Comment": "Common Package Specification for Drake",
  "Meta-Schema": "https://mwoehlke.github.io/cps/",
  "X-Purpose": "Used to generate drake-config.cmake via cps2cmake",
  "X-See-Also": "https://github.com/mwoehlke/pycps",
  "Cps-Version": "0.8.0",
  "Name": "drake",
  "Website": "http://drake.mit.edu/",
  "Requires": {
    "Eigen3": {
      "Version": "3.3.3",
      "Hints": ["@prefix@/lib/cmake/eigen3"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "lcm": {
      "Version": "1.3.95",
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "bot2-core-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/bot2-core-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "robotlocomotion-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/robotlocomotion-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "spdlog": {
      "Version": "1.0.0",
      "Hints": ["@prefix@/lib/cmake/spdlog"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Components": {
    "drake": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake.so",
      "Includes": [
        "@prefix@/include"
      ],
      "Compile-Features": ["c++14"],
      "Requires": [
        "Eigen3:Eigen",
        "lcm:lcm",
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "robotlocomotion-lcmtypes:robotlocomotion-lcmtypes-cpp",
        "spdlog:spdlog"
      ]
    }
  }
}
