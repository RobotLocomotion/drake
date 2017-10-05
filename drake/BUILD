# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

package(default_visibility = ["//visibility:public"])

load("@drake//tools:install.bzl", "install")
load("//tools:lint.bzl", "add_lint_tests")
load("//tools:transitive_hdrs.bzl", "transitive_hdrs_library")
load(":build_components.bzl", "LIBDRAKE_COMPONENTS")

# The Drake binary package. libdrake.so contains all the symbols from all the
# LIBDRAKE_COMPONENTS and all the Drake externals. We use linkstatic=1 so
# that the binary package will not contain any references to shared libraries
# inside the build tree.
cc_binary(
    name = "libdrake.so",
    linkshared = 1,
    linkstatic = 1,
    deps = LIBDRAKE_COMPONENTS,
)

# Install libdrake.so along with all transitive headers in the same workspace
# (i.e. in Drake itself; not externals).
install(
    name = "install",
    targets = ["libdrake.so"],
    hdr_dest = "include/drake",
    guess_hdrs = "WORKSPACE",
)

# Gather all of libdrake.so's dependent headers.
transitive_hdrs_library(
    name = "libdrake_headers",
    visibility = [],
    deps = LIBDRAKE_COMPONENTS,
)

# Depend on Gurobi's shared library iff Gurobi is enabled.
cc_library(
    name = "gurobi_deps",
    visibility = [],
    deps = select({
        "//tools:with_gurobi": ["@gurobi"],
        "//conditions:default": [],
    }),
)

# Depend on Mosek's shared library iff Mosek is enabled.
cc_library(
    name = "mosek_deps",
    visibility = [],
    deps = select({
        "//tools:with_mosek": ["@mosek"],
        "//conditions:default": [],
    }),
)

# Depend on the subset of VTK's shared libraries that Drake uses.
cc_library(
    name = "vtk_deps",
    visibility = [],
    deps = [
        # TODO(jwnimmer-tri) This duplicates the list of VTK libraries needed
        # by //drake/sensors.  We should find a way for ":drake_shared_library"
        # to be declared without having to repeat this list here.
        "@vtk//:vtkCommonCore",
        "@vtk//:vtkCommonDataModel",
        "@vtk//:vtkCommonTransforms",
        "@vtk//:vtkFiltersGeneral",
        "@vtk//:vtkFiltersSources",
        "@vtk//:vtkIOGeometry",
        "@vtk//:vtkIOImage",
        "@vtk//:vtkRenderingCore",
        "@vtk//:vtkRenderingOpenGL2",
    ],
)

# Provide a cc_library target that provides libdrake.so, its headers, and its
# required *.so's that are WORKSPACE downloads (such as VTK, Gurobi, etc).
#
# TODO(jwnimmer-tri) Ideally, Bazel should be able to handle the depended-on
# *.so files (Gurobi, Mosek, VTK) for us, without us having to know up-front
# here which dependencies are coming from the WORKSPACE in the form of *.so.
cc_library(
    name = "drake_shared_library",
    visibility = ["//drake/bindings:__pkg__"],
    deps = [
        ":gurobi_deps",
        ":libdrake_headers",
        ":mosek_deps",
        ":vtk_deps",
    ],
)

add_lint_tests()
