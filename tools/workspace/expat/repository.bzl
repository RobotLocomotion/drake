# -*- mode: python -*-
# vi: set ft=python :

"""
Makes a system-installed Expat XML parser headers and library available to be
used as a C/C++ dependency. On Ubuntu Xenial, pkg-config is used to locate the
Expat headers and library. On macOS and OS X, no pkg-config expat.pc file is
installed, but the Expat headers and library are always located in /usr/include
and /usr/lib, respectively.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/expat:repository.bzl", "expat_repository")  # noqa
        expat_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:expat"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load(
    "@drake//tools/workspace:pkg_config.bzl",
    "setup_pkg_config_repository",
)
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.file("empty.cc", executable = False)

        repository_ctx.symlink("/usr/include/expat.h", "include/expat.h")
        repository_ctx.symlink("/usr/include/expat_external.h",
                               "include/expat_external.h")

        file_content = """
cc_library(
    name = "expat",
    srcs = ["empty.cc"],
    hdrs = [
        "include/expat_external.h",
        "include/expat.h",
    ],
    includes = ["include"],
    linkopts = ["-lexpat"],
    visibility = ["//visibility:public"],
)
"""

        repository_ctx.file("BUILD", content = file_content,
                            executable = False)
    else:
        error = setup_pkg_config_repository(repository_ctx).error

        if error != None:
            fail(error)

expat_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "expat"),
    },
    local = True,
    implementation = _impl,
)
