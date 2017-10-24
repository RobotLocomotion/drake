# -*- python -*-

load("//tools:drake.bzl", "drake_cc_binary")

def drake_pybind_cc_binary(name, srcs = [], copts = [], **kwargs):
    """Declare a pybind11 shared library with the given name and srcs.  The
    libdrake.so library and its headers are already automatically depended-on
    by this rule.

    The deps, linkshared, and linkstatic parameters cannot be set by the
    caller; this rule must fully control their values.
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
        srcs = srcs + ["//tools/install/libdrake:libdrake.so"],
        # These copts are per pybind11 deficiencies.
        copts = [
            "-Wno-#warnings",
            "-Wno-cpp",
            "-Wno-unknown-warning-option",
        ] + copts,
        # This is how you tell Bazel to create a shared library.
        linkshared = 1,
        linkstatic = 1,
        # For all pydrake_foo.so, always link to Drake and pybind11.
        deps = [
            # Even though "libdrake.so" appears in srcs above, we have to list
            # :drake_shared_library here in order to get its headers onto the
            # include path, and its prerequisite *.so's onto LD_LIBRARY_PATH.
            "//tools/install/libdrake:drake_shared_library",
            "@pybind11",
            # TODO(jwnimmer-tri) We should be getting stx header path from
            # :drake_shared_library, but that isn't working yet.
            "@stx",
        ],
        **kwargs
    )
