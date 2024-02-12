load("@drake//tools/skylark:cc.bzl", "cc_library")
load("@drake//tools/workspace/openusd_internal:lock/files.bzl", "FILES")

def pxr_library(
        name,
        *,
        subdir):
    """Defines a cc_library in the spirit of OpenUSD's pxr_library CMake macro.

    The srcs, hdrs, and deps are not passed as an argument to this function.
    Instead, they are loaded from the `lock/files.bzl` database.

    Args:
        name: Matches the upstream name (the first argument in CMake).
        subdir: The subdirectory under `OpenUSD/pxr` (e.g. "base/arch").
    """
    attrs = FILES[subdir]
    srcs = [
        subdir + "/" + x + ".cpp"
        for x in attrs["PUBLIC_CLASSES"] + attrs["PRIVATE_CLASSES"]
    ] + [
        subdir + "/" + x
        for x in attrs["CPPFILES"]
    ]
    hdrs = [
        subdir + "/" + x + ".h"
        for x in attrs["PUBLIC_CLASSES"] + attrs["PRIVATE_CLASSES"]
    ] + [
        subdir + "/" + x
        for x in attrs["PUBLIC_HEADERS"] + attrs["PRIVATE_HEADERS"]
    ]
    defines = [
        # In Drake we use the oneAPI flavor of TBB, which is not the default
        # in OpenUSD, so we need to opt-in.
        "PXR_ONETBB_SUPPORT_ENABLED",
        # OpenUSD still has calls to deprecated TBB functions, so we need to
        # opt-in to some vestigial parts of TBB.
        "TBB_ALLOCATOR_TRAITS_BROKEN",
    ]
    deps = attrs["LIBRARIES"] + [
        ":pxr_h",
        "@onetbb_internal//:tbb",
        # TODO(jwnimmer-tri) We also need to list some @boost here.
    ]
    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        defines = defines,
        copts = ["-w"],
        deps = deps,
    )
