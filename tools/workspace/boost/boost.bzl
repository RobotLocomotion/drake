# -*- mode: python -*-
# vi: set ft=python :

"""
Makes Boost headers available to be used as a C/C++ dependency.

Example:
    WORKSPACE:
        load("//tools/workspace/boost:boost.bzl", "boost_repository")
        boost_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:boost_headers"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

HDRS_PATTERNS = [
    "boost/**/*.h",
    "boost/**/*.hpp",
    "boost/**/*.ipp",
]

LINUX_PREFIX = "/usr"

MAC_OS_X_PREFIX = "/usr/local/opt/boost"

def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        prefix = MAC_OS_X_PREFIX

        repository_ctx.file("empty.cc", executable = False)
        srcs = ["empty.cc"]

    elif repository_ctx.os.name == "linux":
        prefix = LINUX_PREFIX

        srcs = []

    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    repository_ctx.symlink("{}/include/boost".format(prefix), "boost")

    file_content = """
cc_library(
    name = "boost_headers",
    srcs = {},
    hdrs = glob({}),
    includes = ["."],
    visibility = ["//visibility:public"],
)
    """.format(srcs, HDRS_PATTERNS)

    repository_ctx.file("BUILD", content = file_content, executable = False)

boost_repository = repository_rule(implementation = _impl)
