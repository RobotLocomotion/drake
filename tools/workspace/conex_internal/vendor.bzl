load("//tools/skylark:cc.bzl", "cc_library")
load("//tools/workspace:vendor_cxx.bzl", "cc_library_vendored")

def conex_cc_library(
        *,
        name,
        srcs = None,
        hdrs = None,
        copts = None,
        deps = None,
        visibility = None):
    """For anything not named "supernodal_solver", creates an empty cc_library
    with the given name (to placate Bazel).

    For "supernodal_solver", declares a cc_library in the usual sense, except
    rewrites the source code so that it uses a `drake_vendor` inline hidden
    namespace wrapper so that the library's linker symbols are private. For
    this, we can delegate to Drake's existing `cc_library_vendored` rule, after
    adjusting for a few BUILD file typos upstream.
    """

    # Only vendor the one library we care about. Otherwise, just emit an empty
    # library to placate Bazel.
    if name != "supernodal_solver":
        cc_library(name = name, visibility = visibility)
        return

    # Switch one of the `deps` to be source files instead, for simplicity.
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

    # Choose where the vendored files will live.
    hdrs_vendored = [
        "drake_vendor/conex/" + x
        for x in hdrs or []
    ]
    srcs_vendored = [
        "drake_vendor/conex/" + x
        for x in srcs or []
    ]
    strip_include_prefix = "drake_vendor"

    # Mac clang is angry about this.
    copts = copts or []
    if "-Wno-unused-but-set-variable" not in copts:
        copts.append("-Wno-unused-but-set-variable")

    # Compile the static library.
    cc_library_vendored(
        name = name,
        srcs = srcs,
        srcs_vendored = srcs_vendored,
        hdrs = hdrs,
        hdrs_vendored = hdrs_vendored,
        strip_include_prefix = strip_include_prefix,
        copts = copts,
        deps = deps,
        visibility = visibility,
        linkstatic = True,
    )
