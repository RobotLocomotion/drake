{
  "Meta-Comment": "Common Package Specification for Drake",
  "Meta-Schema": "https://mwoehlke.github.io/cps/",
  "X-Purpose": "Used to generate drake-config.cmake via cps2cmake",
  "X-See-Also": "https://github.com/mwoehlke/pycps",
  "Cps-Version": "0.8.0",
  "Name": "drake",
  "Website": "https://drake.mit.edu/",
  "Requires": {
    "bot2-core-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/bot2-core-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "Eigen3": {
      "Version": "3.3.4",
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "fmt": {
      "Version": "6.0",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "ignition-math6": {
      "Version": "6.4",
      "Hints": ["@prefix@/lib/cmake/ignition-math6"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "lcm": {
      "Version": "1.4",
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "optitrack": {
      "Hints": ["@prefix@/lib/cmake/optitrack"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "robotlocomotion-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/robotlocomotion-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "spdlog": {
      "Version": "1.5",
      "Hints": ["@prefix@/lib/cmake/spdlog"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "TinyXML2": {
      "Version": "2.2",
      "X-CMake-Find-Args": ["MODULE"]
    }
  },
  "Default-Components": [":drake"],
  "Components": {
    "drake": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake.so",
      "Includes": ["@prefix@/include"],
      "Compile-Features": ["c++17"],
      "Requires": [
        ":drake-lcmtypes-cpp",
        ":drake-marker",
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "Eigen3:Eigen",
        "fmt:fmt-header-only",
        "ignition-math6:ignition-math6",
        "lcm:lcm",
        "optitrack:optitrack-lcmtypes-cpp",
        "robotlocomotion-lcmtypes:robotlocomotion-lcmtypes-cpp",
        "spdlog:spdlog",
        "tinyxml2:tinyxml2",
        "yaml-cpp"
      ]
    },
    "drake-lcmtypes-cpp": {
      "Type": "interface",
      "Includes": ["@prefix@/include/drake_lcmtypes"],
      "Requires": ["lcm:lcm-coretypes"]
    },
    "drake-lcmtypes-java": {
      "Type": "jar",
      "Location": "@prefix@/share/java/lcmtypes_drake.jar",
      "Requires": ["lcm:lcm-java"]
    },
    "drake-marker": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake_marker.so"
    }
  },
  "X-CMake-Variables-Init": {
    "CMAKE_MODULE_PATH": "${CMAKE_CURRENT_LIST_DIR}/modules;${CMAKE_MODULE_PATH}"
  }
}
