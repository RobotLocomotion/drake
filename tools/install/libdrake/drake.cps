{
  "Meta-Comment": "Common Package Specification for Drake",
  "Meta-Schema": "https://mwoehlke.github.io/cps/",
  "X-Purpose": "Used to generate drake-config.cmake via cps2cmake",
  "X-See-Also": "https://github.com/mwoehlke/pycps",
  "Cps-Version": "0.8.0",
  "Name": "drake",
  "Website": "http://drake.mit.edu/",
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
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "ignition-math4": {
      "Version": "4.0.0",
      "Hints": ["@prefix@/lib/cmake/ignition-math4"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "ignition-rndf0": {
      "Version": "0.1.5",
      "Hints": ["@prefix@/lib/cmake/ignition-rndf0"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "lcm": {
      "Version": "1.3.95",
      "Hints": ["@prefix@/lib/cmake/lcm"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "optitrack": {
      "Hints": ["@prefix@/lib/cmake/optitrack"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "Protobuf": {
      "Version": "2.6.1",
      "X-CMake-Find-Args": ["MODULE"]
    },
    "robotlocomotion-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/robotlocomotion-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "spdlog": {
      "Version": "0.16.3",
      "Hints": ["@prefix@/lib/cmake/spdlog"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "stx": {
      "Hints": ["@prefix@/lib/cmake/stx"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":drake"],
  "Components": {
    "drake": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake.so",
      "Includes": ["@prefix@/include"],
      "Compile-Features": ["c++14"],
      "Link-Flags": ["-ltinyxml2"],
      "Requires": [
        ":drake-lcmtypes-cpp",
        ":drake-marker",
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "Eigen3:Eigen",
        "fmt:fmt-header-only",
        "ignition-math4:ignition-math4",
        "ignition-rndf0:ignition-rndf0",
        "lcm:lcm",
        "optitrack:optitrack-lcmtypes-cpp",
        "protobuf:libprotobuf",
        "robotlocomotion-lcmtypes:robotlocomotion-lcmtypes-cpp",
        "spdlog:spdlog",
        "stx:stx"
      ]
    },
    "drake-common-text-logging-gflags": {
      "Type": "interface",
      "Includes": ["@prefix@/include"],
      "Link-Flags": ["-lgflags"],
      "Requires": [":drake"]
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
  "X-CMake-Variables": {
    "drake_PYTHON_DIRS": "${CMAKE_CURRENT_LIST_DIR}/../../python2.7/site-packages",
    "drake_RESOURCE_ROOT": "${CMAKE_CURRENT_LIST_DIR}/../../../share/drake"
  },
  "X-CMake-Variables-Init": {
    "_Boost_IMPORTED_TARGETS": 1,
    "CMAKE_MODULE_PATH": "${CMAKE_CURRENT_LIST_DIR}/modules/3.10;${CMAKE_MODULE_PATH}"
  }
}
