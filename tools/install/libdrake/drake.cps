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
      "Version": "3.3.3",
      "Hints": ["@prefix@/lib/cmake/eigen3"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "fmt": {
      "Version": "6.0",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "GFlags": {
      "Version": "2.1",
      "X-CMake-Find-Args": [
          "MODULE",
          "COMPONENTS",
          "shared"
      ]
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
    "Protobuf": {
      "Version": "2.6",
      "X-CMake-Find-Args": ["MODULE"]
    },
    "robotlocomotion-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/robotlocomotion-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "spdlog": {
      "Version": "1.3",
      "Hints": ["@prefix@/lib/cmake/spdlog"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "stx": {
      "Hints": ["@prefix@/lib/cmake/stx"],
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
      "Compile-Features": ["c++14"],
      "Requires": [
        ":drake-lcmtypes-cpp",
        ":drake-marker",
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "Eigen3:Eigen",
        "fmt:fmt-header-only",
        "ignition-math6:ignition-math6",
        "lcm:lcm",
        "optitrack:optitrack-lcmtypes-cpp",
        "protobuf:libprotobuf",
        "robotlocomotion-lcmtypes:robotlocomotion-lcmtypes-cpp",
        "spdlog:spdlog",
        "stx:stx",
        "tinyxml2:tinyxml2",
        "yaml-cpp"
      ]
    },
    "drake-common-text-logging-gflags": {
      "Type": "interface",
      "Includes": ["@prefix@/include"],
      "Compile-Features": ["c++14"],
      "Requires": [
          ":drake",
          "gflags:gflags_shared"
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
    "CMAKE_MODULE_PATH": "${CMAKE_CURRENT_LIST_DIR}/modules;${CMAKE_CURRENT_LIST_DIR}/modules/3.10;${CMAKE_MODULE_PATH}"
  }
}
