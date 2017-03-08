# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.
 
load("//tools:drake.bzl", "header_tar")

# This list is obviously incomplete.
# TODO(david-german-tri, jamiesnape): Add everything Director needs, and
# eventually everything any consumer of Drake would need.
_LIBDRAKE_COMPONENTS = [
    "//drake/multibody/parsers",
    "//drake/systems/framework",
]

# The Drake binary package. libdrake.so contains all the symbols from all the
# _LIBDRAKE_COMPONENTS, but links Drake's WORKSPACE externals dynamically.
cc_binary(
     name = "libdrake.so",
     deps = _LIBDRAKE_COMPONENTS,
     linkstatic = 0,
     linkshared = 1,
)

# All the headers mentioned in "hdrs" attributes of the transitive closure of
# dependencies of the _LIBDRAKE_COMPONENTS. This is a superset of the headers
# actually required to operate Drake.
header_tar(
    name = "libdrake_headers",
    deps = _LIBDRAKE_COMPONENTS,
)
