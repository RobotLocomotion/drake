# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("//tools:install.bzl", "install")
load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

# This list is obviously incomplete.
# TODO(david-german-tri, jamiesnape): Add everything Director needs, and
# eventually everything any consumer of Drake would need.
# When adding new components to the package, please also add the licenses for
# any new external dependencies to :external_licenses.
_LIBDRAKE_COMPONENTS = [
    "//drake/lcm",
    "//drake/multibody:inverse_kinematics",
    "//drake/multibody:rigid_body_tree",
    "//drake/multibody/parsers",
    "//drake/solvers:mathematical_program",
    "//drake/systems/framework",
    "//drake/systems/primitives",
    "//drake/util:convex_hull",
]

# The Drake binary package. libdrake.so contains all the symbols from all the
# _LIBDRAKE_COMPONENTS and all the Drake externals. We use linkstatic=1 so
# that the binary package will not contain any references to shared libraries
# inside the build tree.
cc_binary(
    name = "libdrake.so",
    linkshared = 1,
    linkstatic = 1,
    deps = _LIBDRAKE_COMPONENTS,
)

# Install libdrake.so along with all transitive headers in the same workspace
# (i.e. in Drake itself; not externals).
install(
    name = "install",
    doc_dest = "share/doc/drake",
    guess_hdrs = "WORKSPACE",
    hdr_dest = "include/drake",
    targets = ["libdrake.so"],
    deps = ["//drake/bindings:install"],
)
