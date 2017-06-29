# -*- python -*-

load("//tools:install.bzl", "cmake_config", "install", "install_cmake_config")

package(default_visibility = ["//visibility:public"])

CMAKE_PACKAGE = "optitrack"

cmake_config(
    cps_file_name = CMAKE_PACKAGE + ".cps",
    package = CMAKE_PACKAGE,
)

install_cmake_config(
    package = CMAKE_PACKAGE,
    versioned = 0,
)

OPTITRACK_LICENSE_DOCS = ["@optitrack_driver//:LICENSE"]

OPTITRACK_TARGETS = [
    "@optitrack_driver//lcmtypes:lcmtypes_optitrack",
    "@optitrack_driver//lcmtypes:optitrack_lcmtypes",
    "@optitrack_driver//lcmtypes:py_optitrack_lcmtypes",
    "@optitrack_driver//src:optitrack_client",
]

install(
    name = "install",
    workspace = CMAKE_PACKAGE,
    targets = OPTITRACK_TARGETS,
    java_strip_prefix = ["**/"],
    py_dest = "lib/python2.7/site-packages/optitrack",
    py_strip_prefix = ["**/"],
    hdr_dest = "include/lcmtypes",
    guess_hdrs = "PACKAGE",
    docs = OPTITRACK_LICENSE_DOCS,
    doc_strip_prefix = ["**/"],
    rename = {
        "share/java/liblcmtypes_optitrack.jar": "lcmtypes_optitrack.jar",
    },
    allowed_externals = OPTITRACK_LICENSE_DOCS + OPTITRACK_TARGETS,
    visibility = ["//:__pkg__"],
    deps = [":install_cmake_config"],
)
