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
    "Bullet": {
      "Version": "2.86",
      "Hints": ["@prefix@/lib/cmake/bullet"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "Eigen3": {
      "Version": "3.3.3",
      "Hints": ["@prefix@/lib/cmake/eigen3"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "fcl": {
      "Version": "0.6.0",
      "Hints": ["@prefix@/lib/cmake/fcl"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "fmt": {
      "Version": "3.0.1",
      "Hints": ["@prefix@/lib/cmake/fmt"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "ignition-math3": {
      "Version": "3.2.0",
      "Hints": ["@prefix@/lib/cmake/ignition-math3"],
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
    "protobuf": {
      "Version": "3.1.0",
      "Hints": ["@prefix@/lib/cmake/protobuf"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "robotlocomotion-lcmtypes": {
      "Hints": ["@prefix@/lib/cmake/robotlocomotion-lcmtypes"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "scs": {
      "Version": "1.2.6",
      "Hints": ["@prefix@/lib/cmake/scs"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "SDFormat": {
      "Version": "6.0.0",
      "Hints": ["@prefix@/lib/cmake/sdformat"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "spdlog": {
      "Version": "1.0.0",
      "Hints": ["@prefix@/lib/cmake/spdlog"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "stx": {
      "Hints": ["@prefix@/lib/cmake/stx"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "tinyobjloader": {
      "Version": "1.0.4",
      "Hints": ["@prefix@/lib/cmake/tinyobjloader"],
      "X-CMake-Find-Args": ["CONFIG"]
    },
    "yaml-cpp": {
      "Version": "0.5.5",
      "Hints": ["@prefix@/lib/cmake/yaml-cpp"],
      "X-CMake-Find-Args": ["CONFIG"]
    }
  },
  "Default-Components": [":drake"],
  "Components": {
    "drake": {
      "Type": "dylib",
      "Location": "@prefix@/lib/libdrake.so",
      "Includes": [
        "@prefix@/include"
      ],
      "Compile-Features": ["c++14"],
      "Link-Flags": ["-ltinyxml2"],
      "Link-Requires": [
        "fmt:fmt",
        "scs:scsdir",
        "SDFormat:sdformat",
        "tinyobjloader:tinyobjloader"
      ],
      "Requires": [
        ":drake-lcmtypes-cpp",
        "bot2-core-lcmtypes:lcmtypes_bot2-core-cpp",
        "Bullet:BulletCollision",
        "Eigen3:Eigen",
        "fcl:fcl",
        "ignition-math3:ignition-math3",
        "ignition-rndf0:ignition-rndf0",
        "lcm:lcm",
        "optitrack:lcmtypes_optitrack-cpp",
        "protobuf:protobuf",
        "robotlocomotion-lcmtypes:robotlocomotion-lcmtypes-cpp",
        "spdlog:spdlog",
        "stx:stx",
        "yaml-cpp:yaml-cpp"
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
    }
  },
  "X-CMake-Variables": {
    "drake_RESOURCE_ROOT": "${CMAKE_CURRENT_LIST_DIR}/../../../share/drake"
  }
}
