# -*- mode: python -*-
# vi: set ft=python :

"""
Makes system-installed gflags headers and library available to be used as a
C/C++ dependency. On macOS, pkg-config is used to locate the gflags headers and
library. On Ubuntu Xenial, no pkg-config gflags.pc file is installed, but the
gflags headers and library are always located in /usr/include and
/usr/lib/x86_64-linux-gnu, respectively.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/gflags:repository.bzl", "gflags_repository")  # noqa
        gflags_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:gflags"],
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

    if os_result.ubuntu_release == "16.04":
        repository_ctx.symlink("/usr/include/gflags", "include/gflags")

        file_content = """
cc_library(
    name = "gflags",
    hdrs = [
      "include/gflags/gflags.h",
      "include/gflags/gflags_completions.h",
      "include/gflags/gflags_declare.h",
      "include/gflags/gflags_gflags.h",
    ],
    includes = ["include"],
    linkopts = ["-lgflags"],
    visibility = ["//visibility:public"],
)
"""

        repository_ctx.file("BUILD", content = file_content,
                            executable = False)
    else:
        error = setup_pkg_config_repository(repository_ctx).error

        if error != None:
            fail(error)

gflags_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "gflags"),
    },
    local = True,
    implementation = _impl,
)
