# -*- mode: python -*-
# vi: set ft=python :

"""
Makes a system-installed zlib image compression library headers and library
available to be used as a C/C++ dependency. On Ubuntu Xenial,  pkg-config is
used to locate the zlib headers and library. On macOS and OS X, no pkg-config
zlib.pc file is installed, but the zlib headers and library are always located
in /usr/include and /usr/lib, respectively.

Example:
    WORKSPACE:
        load("//tools/workspace/zlib:zlib.bzl", "zlib_repository")
        zlib_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:zlib"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load(
    "@kythe//tools/build_rules/config:pkg_config.bzl",
    "setup_pkg_config_package",
)
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.file("empty.cc", executable = False)

        repository_ctx.symlink("/usr/include/zlib.h", "include/zlib.h")
        repository_ctx.symlink("/usr/include/zconf.h", "include/zconf.h")

        file_content = """
cc_library(
    name = "zlib",
    srcs = ["empty.cc"],
    hdrs = [
      "include/zconf.h",
      "include/zlib.h",
    ],
    includes = ["include"],
    linkopts = ["-lz"],
    visibility = ["//visibility:public"],
)
"""

        repository_ctx.file("BUILD", content = file_content,
                            executable = False)
    else:
        error = setup_pkg_config_package(repository_ctx).error

        if error != None:
            fail(error)

zlib_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "zlib"),
        "build_file_template": attr.label(
            default = Label("@kythe//tools/build_rules/config:BUILD.tpl"),
            single_file = True,
            allow_files = True,
        ),
    },
    local = True,
    implementation = _impl,
)
