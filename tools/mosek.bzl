# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a MOSEK archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("//tools:mosek.bzl", "mosek_repository")
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
        sha256 = "26c5bc0be667c92d1a5f81d2a1f7694de1fb0a3e9e9064c17f98e425db0a3c64"
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "9b2bfcba7bcdd24b7e87ecdcccc11222302ced7b3d2a2af7090bdf625ab7cfae"
    else:
        fail("Operating system is NOT supported", attr=repository_ctx.os.name)

    url = "http://download.mosek.com/stable/{}/mosektools{}.tar.bz2".format(
        mosek_major_version, mosek_platform)
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}/tools/platform/{}".format(mosek_major_version,
                                                       mosek_platform)

    repository_ctx.download_and_extract(
        url, root_path, sha256=sha256, stripPrefix=strip_prefix)

    if repository_ctx.os.name == "mac os x":
        install_name_tool = repository_ctx.which("install_name_tool")

        libraries = [
            "bin/libiomp5.dylib",
            "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
                                                mosek_minor_version),
        ]

        for library in libraries:
            library_path = repository_ctx.path(library)

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                library_path,
                library_path,
            ])

            if result.return_code != 0:
                fail("Could NOT change shared library identification name",
                     attr=result.stderr)

            repository_ctx.file("empty.cc", executable=False)

            srcs = ["empty.cc"]

            bin_path = repository_ctx.path("bin")

            linkopts = [
                "-L{}".format(bin_path),
                "-liomp5",
                "-lmosek64",
            ]
    else:
        srcs = [
            "bin/libiomp5.so",
            "bin/libmosek64.so.{}.{}".format(mosek_major_version,
                                             mosek_minor_version),
        ]

        linkopts = []

    file_content = """
cc_library(
    name = "mosek",
    srcs = {},
    hdrs = ["h/mosek.h"],
    includes = ["h"],
    linkopts = {},
    visibility = ["//visibility:public"],
)
    """.format(srcs, linkopts)

    repository_ctx.file("BUILD", content=file_content, executable=False)

mosek_repository = repository_rule(implementation = _impl)
