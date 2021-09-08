# -*- python -*-

# This is a separate file from drake_cc.bzl because the dependency on
# @drake_detected_os is somewhat brittle and might present challenges
# for users exploring novel platforms.

# TODO(jwnimmer-tri) For the moment, these functions are named "ubuntu",
# but also are armed on "manylinux".  We should probably rework the names
# and/or documentation to be more clear on this point.

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_package_library",
)
load(
    "@drake_detected_os//:os.bzl",
    "DISTRIBUTION",
)

def drake_cc_googletest_ubuntu_only(
        name,
        visibility = ["//visibility:private"],
        **kwargs):
    """Declares a drake_cc_googletest iff we are building on Ubuntu.
    Otherwise, does nothing.

    Because this test is not cross-platform, the visibility defaults to
    private.
    """
    if DISTRIBUTION in ["ubuntu", "manylinux"]:
        drake_cc_googletest(
            name = name,
            visibility = visibility,
            **kwargs
        )

def drake_cc_library_ubuntu_only(
        name,
        hdrs = [],
        visibility = ["//visibility:private"],
        **kwargs):
    """Declares a drake_cc_library iff we are building on Ubuntu.
    Otherwise, does nothing.

    Because this library is not cross-platform, the visibility defaults to
    private and the headers are excluded from the installation.
    """
    if DISTRIBUTION in ["ubuntu", "manylinux"]:
        drake_cc_library(
            name = name,
            hdrs = hdrs,
            visibility = visibility,
            install_hdrs_exclude = hdrs,
            **kwargs
        )

def drake_cc_package_library_per_os(
        macos_deps = [],
        ubuntu_deps = [],
        **kwargs):
    """Declares a drake_cc_package_library, where the deps of the library are
    conditioned on whether we are building on macOS or Ubuntu.
    """
    if DISTRIBUTION == "macos":
        drake_cc_package_library(deps = macos_deps, **kwargs)
    elif DISTRIBUTION in ["ubuntu", "manylinux"]:
        drake_cc_package_library(deps = ubuntu_deps, **kwargs)
