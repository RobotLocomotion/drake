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

def drake_cc_googletest_gl_ubuntu_only(**kwargs):
    if DISTRIBUTION == "ubuntu":
        drake_cc_googletest(**kwargs)

def drake_cc_library_gl_ubuntu_only(name, hdrs = [], **kwargs):
    if DISTRIBUTION == "ubuntu":
        # Because this library is not cross-platform, we must use default
        # visibility (i.e., private) and not install its private headers.
        drake_cc_library(
            name = name,
            hdrs = hdrs,
            install_hdrs_exclude = hdrs,
            visibility = None,
            **kwargs
        )

def drake_cc_package_library_gl_per_os(
        macos_deps = [],
        ubuntu_deps = [],
        **kwargs):
    if DISTRIBUTION == "macos":
        drake_cc_package_library(deps = macos_deps, **kwargs)
    elif DISTRIBUTION == "ubuntu":
        drake_cc_package_library(deps = ubuntu_deps, **kwargs)
