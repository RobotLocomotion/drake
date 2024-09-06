load("@drake//tools/skylark:cc.bzl", "cc_library")
load("@drake//tools/workspace/openusd_internal:lock/files.bzl", "FILES")

def pxr_library(
        *,
        subdir):
    """Defines a cc_library in the spirit of OpenUSD's pxr_library CMake macro.

    The srcs, hdrs, and deps are not passed as an argument to this function.
    Instead, they are loaded from the `lock/files.bzl` database.

    Args:
        subdir: The subdirectory under `OpenUSD` (e.g. "pxr/base/arch").
    """
    attrs = FILES[subdir]
    name = attrs["NAME"]
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
    ]

    # TODO(jwnimmer-tri) The plugInfo files will need to be pseudo-installed.
    data = native.glob([subdir + "/plugInfo.json"], allow_empty = True)

    # OpenUSD uses `__attribute__((constructor))` in anger, so we must mark
    # all of its code as "alwayslink" (aka "whole archive").
    alwayslink = True

    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        defines = defines,
        copts = [
            # Special treatment for pxr/base/gf/nanocolor.{h,c}.
            "-DNCAPI='__attribute__((visibility(\"hidden\")))'",
            "-w",
        ],
        alwayslink = alwayslink,
        linkstatic = True,
        data = data,
        deps = deps,
    )
