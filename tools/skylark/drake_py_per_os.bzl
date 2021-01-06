# -*- python -*-

# This is a separate file from drake_py.bzl because the dependency on
# @drake_detected_os is somewhat brittle and might present challenges
# for users exploring novel platforms.

load(
    "@drake//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_unittest",
)
load(
    "@drake_detected_os//:os.bzl",
    "DISTRIBUTION",
)

def drake_py_binary_ubuntu_only(
        name,
        visibility = ["//visibility:private"],
        **kwargs):
    """Declares a drake_py_binary iff we are building on Ubuntu.
    Otherwise, does nothing.

    The visibility defaults to private because this binary is not
    cross-platform.
    """
    if DISTRIBUTION == "ubuntu":
        drake_py_binary(
            name = name,
            visibility = visibility,
            **kwargs
        )

def drake_py_unittest_ubuntu_only(
        name,
        visibility = ["//visibility:private"],
        **kwargs):
    """Declares a drake_py_unittest iff we are building on Ubuntu.
    Otherwise, does nothing.

    The visibility defaults to private because this binary is not
    cross-platform.
    """
    if DISTRIBUTION == "ubuntu":
        drake_py_unittest(
            name = name,
            visibility = visibility,
            **kwargs
        )
