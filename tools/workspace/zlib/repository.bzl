"""
Makes a system-installed zlib image compression library headers and library
available to be used as a C/C++ dependency. On Ubuntu, pkg-config is used to
locate the zlib headers and library. On macOS, no pkg-config zlib.pc file is
installed, but the zlib headers are included in the macOS SDK and the library
is always located at /usr/lib.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/zlib:repository.bzl", "zlib_repository")
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

load("//tools/workspace:execute.bzl", "execute_or_fail")
load("//tools/skylark:pathutils.bzl", "join_paths")
load("//tools/workspace:pkg_config.bzl", "setup_pkg_config_repository")
load("//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        result = execute_or_fail(repository_ctx, ["xcrun", "--show-sdk-path"])
        include = join_paths(result.stdout.strip(), "usr/include")
        repository_ctx.symlink(
            join_paths(include, "zlib.h"),
            "include/zlib.h",
        )
        repository_ctx.symlink(
            join_paths(include, "zconf.h"),
            "include/zconf.h",
        )

        file_content = """# DO NOT EDIT: generated by zlib_repository()

licenses(["notice"])  # Zlib

cc_library(
    name = "zlib",
    hdrs = [
      "include/zconf.h",
      "include/zlib.h",
    ],
    includes = ["include"],
    linkopts = ["-lz"],
    visibility = ["//visibility:public"],
)
"""

        repository_ctx.file(
            "BUILD.bazel",
            content = file_content,
            executable = False,
        )
    else:
        error = setup_pkg_config_repository(repository_ctx).error

        if error != None:
            fail(error)

zlib_repository = repository_rule(
    # TODO(jamiesnape): Pass down licenses to setup_pkg_config_repository.
    attrs = {
        "modname": attr.string(default = "zlib"),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
