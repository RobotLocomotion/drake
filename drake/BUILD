# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.
 
 
package(default_visibility = ["//visibility:public"])

load("//tools:drake.bzl", "drake_header_tar")

# This list is obviously incomplete.
# TODO(david-german-tri, jamiesnape): Add everything Director needs, and
# eventually everything any consumer of Drake would need.
_LIBDRAKE_COMPONENTS = [
    "//drake/multibody/parsers",
    "//drake/systems/framework",
    "//drake/systems/primitives",
]

# The Drake binary package. libdrake.so contains all the symbols from all the
# _LIBDRAKE_COMPONENTS and all the Drake externals. We use linkstatic=1 so
# that the binary package will not contain any references to shared libraries
# inside the build tree.
cc_binary(
     name = "libdrake.so",
     deps = _LIBDRAKE_COMPONENTS,
     linkstatic = 1,
     linkshared = 1,
)

# All the headers mentioned in "hdrs" attributes of the transitive closure of
# dependencies of the _LIBDRAKE_COMPONENTS. This is a superset of the headers
# actually required to operate Drake.
drake_header_tar(
    name = "libdrake_headers",
    deps = _LIBDRAKE_COMPONENTS,
)
