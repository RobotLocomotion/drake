# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

# Used by //drake/bindings:pydrake_urdf_parser_test.
filegroup(
    name = "models",
    srcs = glob([
        "pr2_description/meshes/**/*.obj",
        "pr2_description/meshes/**/*.stl",
        "pr2.urdf",
    ]),
)
