load("@drake//tools/skylark:cc.bzl", "cc_binary", "cc_library")
load("@drake//tools/workspace/openusd_internal:files.bzl", "FILES")

def usd_library(
        name,
        *,
        subdir):
    """
    subdir: e.g. "base/arch"
    """
    attrs = FILES[subdir]
    srcs = [
        "pxr/" + subdir + "/" + x + ".cpp"
        for x in attrs["PUBLIC_CLASSES"] + attrs["PRIVATE_CLASSES"]
    ] + [
        "pxr/" + subdir + "/" + x
        for x in attrs["CPPFILES"]
    ]
    hdrs = [
        "pxr/" + subdir + "/" + x + ".h"
        for x in attrs["PUBLIC_CLASSES"] + attrs["PRIVATE_CLASSES"]
    ] + [
        "pxr/" + subdir + "/" + x
        for x in attrs["PUBLIC_HEADERS"] + attrs["PRIVATE_HEADERS"]
    ]
    defines = [
        # In Drake we use the oneAPI flavor of TBB, which is not the
        # default in OpenUSD, so we need to opt-in.
        "PXR_ONETBB_SUPPORT_ENABLED",
        # OpenUSD still has calls to deprecated TBB functions, so we need
        # to opt-in to some vestigial parts of TBB.
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
