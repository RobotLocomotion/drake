# -*- python -*-

load("//tools:drake.bzl", "drake_cc_binary")

def pybind_cc_binary(name, srcs=[], copts=[],
                     deps=None, linkshared=1, linkstatic=1,
                     **kwargs):
    """Declare a pybind11 shared library with the given name and srcs.

    The deps, linkshared, and linkstatic parameters cannot be set by the
    caller; this rule must fully specify their values.  The libdrake.so library
    and its headers are already automatically depended-on by this rule.

    """

    # Disallow `linkshared` and `linkstatic` because Bazel requires them to be
    # set to 1 when making a ".so" library.
    #
    # Disallow `deps` because we _must not_ deps on any given object code more
    # than once or else we risk linking in multiple copies of it into different
    # _pybind_foo.so files, which breaks C++ global variables.  All object code
    # must come in through libdrake.so.  (Conceivably a header-only library
    # could be allowed in deps, but we can fix that when we need it.)
    for key in ["deps", "linkshared", "linkstatic"]:
        if key in kwargs:
            fail("%s cannot be set by the caller" % key)

    drake_cc_binary(
        name = name,
        # This is how you tell Bazel to link in a shared library.
        srcs = srcs + ["//drake:libdrake.so"],
        # These copts are per pybind11 deficiencies.
        copts = [
            "-Wno-#warnings",
            "-Wno-cpp",
            "-Wno-unknown-warning-option",
        ] + copts,
        # This is how you tell Bazel to create a shared library.
        linkshared = 1,
        linkstatic = 1,
        # Always provide Drake headers and pybind headers.
        deps = [
            "//drake:libdrake_headers",
            "@pybind11",
            # TODO(jwnimmer-tri) This duplicates the list of VTK libraries
            # needed by //drake/sensors:sensors.  Without these, we end up
            # missing these libraries at runtime (as shown by our unit tests).
            # We should find a way to solve this without repeating it here.
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
        **kwargs
    )
