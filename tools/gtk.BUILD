# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

package(default_visibility = ["//visibility:public"])

glib_include_list = [
    "include/glib-2.0",
    "lib/x86_64-linux-gnu/glib-2.0/include",
]

cc_library(
    name = "glib",
    hdrs = glob([x + "/**" for x in glib_include_list]),
    includes = glib_include_list,
    linkopts = [
        "-lglib-2.0",
        ],
)
