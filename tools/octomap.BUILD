# -*- python -*-

load(
    "@drake//tools:install.bzl",
    "cmake_config",
    "install",
    "install_cmake_config",
)

package(default_visibility = ["//visibility:public"])

# Lets other packages inspect the CMake code, e.g., for the version number.
filegroup(
    name = "cmakelists_with_version",
    srcs = ["octomap/CMakeLists.txt"],
)

# Generates octomath library
cc_library(
    name = "octomath",
    srcs = [
        "octomap/src/math/Pose6D.cpp",
        "octomap/src/math/Quaternion.cpp",
        "octomap/src/math/Vector3.cpp",
    ],
    hdrs = glob([
        "octomap/include/octomap/math/*.h*",
    ]),
    includes = ["octomap/include"],
)

# Generates the library exported to users.  The explicitly listed srcs= matches
# upstream's explicitly listed sources.  The globbed hdrs= matches upstream's
# explicitly globbed headers.
cc_library(
    name = "octomap",
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
    ],
    hdrs = glob([
        "octomap/include/octomap/*.h*",
    ]),
    includes = ["octomap/include"],
    deps = [":octomath"],
)

cmake_config(
    package = "octomap",
    script = "@drake//tools:octomap-create-cps.py",
    version_file = ":cmakelists_with_version",
)

# Creates rule :install_cmake_config.
install_cmake_config(package = "octomap")

install(
    name = "install",
    doc_dest = "share/doc",
    guess_hdrs = "PACKAGE",
    hdr_dest = "include",
    hdr_strip_prefix = ["octomap/include"],
    license_docs = ["octomap/LICENSE.txt"],
    targets = [
        ":octomap",
        ":octomath",
    ],
    deps = [":install_cmake_config"],
)
