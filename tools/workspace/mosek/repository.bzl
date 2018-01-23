# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a MOSEK archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")  # noqa
        mosek_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:mosek"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

def _impl(repository_ctx):
    mosek_major_version = 7
    mosek_minor_version = 1

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "26c5bc0be667c92d1a5f81d2a1f7694de1fb0a3e9e9064c17f98e425db0a3c64"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "9b2bfcba7bcdd24b7e87ecdcccc11222302ced7b3d2a2af7090bdf625ab7cfae"  # noqa
    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    url = "http://download.mosek.com/stable/{}/mosektools{}.tar.bz2".format(
        mosek_major_version, mosek_platform)
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}".format(mosek_major_version)

    repository_ctx.download_and_extract(
        url, root_path, sha256 = sha256, stripPrefix = strip_prefix)

    platform_prefix = "tools/platform/{}".format(mosek_platform)

    if repository_ctx.os.name == "mac os x":
        install_name_tool = repository_ctx.which("install_name_tool")

        files = [
            "bin/libiomp5.dylib",
            "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
                                                mosek_minor_version),
        ]

        for file in files:
            file_path = repository_ctx.path(
                "{}/{}".format(platform_prefix, file)
            )

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                file_path,
                file_path,
            ])

            if result.return_code != 0:
                fail("Could NOT change shared library identification name",
                     attr = result.stderr)

        repository_ctx.file("empty.cc", executable = False)

        srcs = ["empty.cc"]

        bin_path = repository_ctx.path("{}/bin".format(platform_prefix))

        linkopts = [
            "-L{}".format(bin_path),
            "-liomp5",
            "-lmosek64",
        ]
    else:
        files = [
            "bin/libiomp5.so",
            "bin/libmosek64.so.{}.{}".format(mosek_major_version,
                                             mosek_minor_version),
        ]

        linkopts = []
        srcs = ["{}/{}".format(platform_prefix, file) for file in files]

    hdrs = ["{}/h/mosek.h".format(platform_prefix)]
    includes = ["{}/h".format(platform_prefix)]
    files = ["{}/{}".format(platform_prefix, file) for file in files]
    libraries_strip_prefix = ["{}/bin".format(platform_prefix)]

    file_content = """# -*- python -*-

load("@drake//tools/install:install.bzl", "install", "install_files")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mosek",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
)

install_files(
    name = "install_libraries",
    dest = "lib",
    files = {},
    strip_prefix = {},
    visibility = ["//visibility:private"],
)

install(
   name = "install",
   docs = ["license.pdf"],
   deps = [":install_libraries"],
)
    """.format(srcs, hdrs, includes, linkopts, files, libraries_strip_prefix)

    repository_ctx.file("BUILD", content = file_content, executable = False)

mosek_repository = repository_rule(implementation = _impl)
