# -*- python -*-

# Lets other packages inspect the CMake code, e.g., for the version number.
filegroup(
    name = "cmakelists_with_version",
    srcs = ["octomap/CMakeLists.txt"],
    visibility = ["//visibility:public"],
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources.  The globbed hdrs= matches upstream's
# explicitly globbed headers.
cc_library(
    name = "lib",
    srcs = [
        "octomap/src/AbstractOcTree.cpp",
        "octomap/src/AbstractOccupancyOcTree.cpp",
        "octomap/src/ColorOcTree.cpp",
        "octomap/src/CountingOcTree.cpp",
        "octomap/src/OcTree.cpp",
        "octomap/src/OcTreeNode.cpp",
        "octomap/src/OcTreeStamped.cpp",
        "octomap/src/Pointcloud.cpp",
        "octomap/src/ScanGraph.cpp",
        "octomap/src/math/Pose6D.cpp",
        "octomap/src/math/Quaternion.cpp",
        "octomap/src/math/Vector3.cpp",
    ],
    hdrs = glob([
        "octomap/include/octomap/*.h*",
        "octomap/include/octomap/math/*.h*",
    ]),
    includes = ["octomap/include"],
    visibility = ["//visibility:public"],
)
