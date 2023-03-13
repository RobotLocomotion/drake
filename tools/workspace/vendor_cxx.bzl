def cc_library_vendored(
        name,
        hdrs = None,
        hdrs_vendored = None,
        edit_include = None,
        srcs = None,
        srcs_vendored = None,
        **kwargs):
    """
    Compiles a third-party C++ library using altered include paths and
    namespaces so that it will not interfere with co-habitating builds
    of the same library by others.

    The lists of hdrs and hdrs_vendored paths must be equal in length and
    correspond as elementwise pairs. The hdrs gives the list of library
    header file paths as found in the third-party source layout; the
    hdrs_vendored gives the list of header file paths to use for Drake's
    vendored build. Typically we will prefix "drake_vendor/" to the path.

    The edit_include mapping provides #include patterns that should be
    rewritten in all of the source files (both hdrs and srcs). The patterns
    are implicitly anchored at the start of the #include statements.
    For example, {"yaml-cpp/": "drake_vendor/yaml-cpp/"} would edit this line:
      #include <yaml-cpp/node.h>
    into this line instead:
      #include <drake_vendor/yaml-cpp/node.h>

    The lists of srcs and srcs_vendored paths must be equal in length and
    correspond as elementwise pairs. The srcs gives the list of library
    source file paths as found in the third-party source layout; the
    srcs_vendored gives the list of source file paths to use for Drake's
    vendored build.
    """
    hdrs = hdrs or []
    hdrs_vendored = hdrs_vendored or []
    edit_include = edit_include or {}
    srcs = srcs or []
    srcs_vendored = srcs_vendored or []
    if len(hdrs) != len(hdrs_vendored):
        fail("The hdrs= and hdrs_vendored= list lengths must match")
    if len(srcs) != len(srcs_vendored):
        fail("The srcs= and srcs_vendored= list lengths must match")
    native.genrule(
        name = "_{}_vendoring".format(name),
        srcs = hdrs + srcs,
        outs = hdrs_vendored + srcs_vendored,
        cmd = " ".join([
            "$(execpath @drake//tools/workspace:vendor_cxx)",
        ] + [
            "'--edit-include={}:{}'".format(k, v)
            for k, v in edit_include.items()
        ] + [
            "$(execpath {}):$(execpath {})".format(old, new)
            for old, new in (zip(hdrs, hdrs_vendored) +
                             zip(srcs, srcs_vendored))
        ]),
        tools = ["@drake//tools/workspace:vendor_cxx"],
        tags = ["manual"],
        visibility = ["//visibility:private"],
    )
    native.cc_library(
        name = name,
        hdrs = hdrs_vendored,
        srcs = srcs_vendored,
        **kwargs
    )
