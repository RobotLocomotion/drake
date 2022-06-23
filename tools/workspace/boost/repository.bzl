# -*- mode: python -*-
# vi: set ft=python :

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

"""
Makes Boost headers available to be used as a C/C++ dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/boost:repository.bzl", "boost_repository")  # noqa
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

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)
    elif os_result.is_macos or os_result.is_macos_wheel:
        prefix = "{}/opt/boost".format(os_result.homebrew_prefix)
    elif os_result.is_ubuntu or os_result.is_manylinux:
        prefix = "/usr"
    else:
        fail(
            "Operating system is NOT supported",
            attr = repository_ctx.os.name,
        )

    repository_ctx.symlink("{}/include/boost".format(prefix), "boost")

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by boost_repository()

licenses(["notice"])  # BSL-1.0

cc_library(
    name = "boost_headers",
    hdrs = glob({}),
    includes = ["."],
    visibility = ["//visibility:public"],
    deprecation = "DRAKE DEPRECATED: The @boost external is deprecated and will be removed from Drake on or after 2022-10-01.",  # noqa
)
    """.format(HDRS_PATTERNS)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

boost_repository = repository_rule(
    local = True,
    implementation = _impl,
)
