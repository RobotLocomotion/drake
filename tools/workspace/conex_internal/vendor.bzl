load("//tools/workspace:vendor_cxx.bzl", "cc_library_vendored")

def conex_cc_library(
        *,
        name,
        srcs = None,
        hdrs = None,
        copts = None,
        deps = None,
        visibility = None):
    """Declares a cc_library in the usual sense, except that we rewrite
    the source code so that it uses a `drake_vendor` namespace wrapper.
    """

    # Only vendor the one library we care about. Otherwise, just emit an empty
    # library to placate Bazel.
    if name != "supernodal_solver":
        native.cc_library(name = name, visibility = visibility)
        return

    # Switch one of the `deps` to be sources files instead, for simplicity.
    srcs = list(srcs)
    hdrs = list(hdrs)
    deps = list(deps)
    deps.remove("tree_utils")
    srcs.append("tree_utils.cc")
    hdrs.append("tree_utils.h")

    # Fix some hdrs that are mis-identified as srcs upstream.
    for hdr in [x for x in srcs if x.endswith(".h")]:
        srcs.remove(hdr)
        if hdr not in hdrs:
            hdrs.append(hdr)

    # Choose the rewritten paths.
    # Include paths like "conex/foo.h" will become "drake_vendor/conex/foo.h".
    hdrs_vendored = [
        "drake_vendor/conex/" + x
        for x in hdrs or []
    ]
    srcs_vendored = [
        "drake_vendor/conex/" + x
        for x in srcs or []
    ]
    edit_include = {
        "conex/": "drake_vendor/conex/",
    }
    strip_include_prefix = "/conex"

    # Compile the static library.
    cc_library_vendored(
        name = name,
        srcs = srcs,
        srcs_vendored = srcs_vendored,
        hdrs = hdrs,
        hdrs_vendored = hdrs_vendored,
        edit_include = edit_include,
        strip_include_prefix = strip_include_prefix,
        copts = copts,
        deps = deps,
        visibility = visibility,
        linkstatic = True,
    )
