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

load("@drake//tools/workspace:execute.bzl", "which")

def _impl(repository_ctx):
    mosek_major_version = 8
    mosek_minor_version = 1
    mosek_patch_version = 0
    mosek_tweak_version = 51

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "00aed5ca62acca6689b579503ad19aedb100b4a37dfd6b95e52f52dee414520e"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "ab2f39c1668105acbdfbe6835f59f547dd8b076378d9943ab40839c70e1141a2"  # noqa
    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    # TODO(jwnimmer-tri) Port to use mirrors.bzl.
    url = "http://download.mosek.com/stable/{}.{}.{}.{}/mosektools{}.tar.bz2".format(  # noqa
        mosek_major_version, mosek_minor_version, mosek_patch_version,
        mosek_tweak_version, mosek_platform)
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}".format(mosek_major_version)

    repository_ctx.download_and_extract(
        url, root_path, sha256 = sha256, stripPrefix = strip_prefix)

    platform_prefix = "tools/platform/{}".format(mosek_platform)

    if repository_ctx.os.name == "mac os x":
        install_name_tool = which(repository_ctx, "install_name_tool")

        files = [
            "bin/libcilkrts.5.dylib",
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

        srcs = []

        bin_path = repository_ctx.path("{}/bin".format(platform_prefix))

        linkopts = [
            "-L{}".format(bin_path),
            "-liomp5",
            "-lmosek64",
        ]
    else:
        files = [
            "bin/libcilkrts.so.5",
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

# DO NOT EDIT: generated by mosek_repository()

load("@drake//tools/install:install.bzl", "install", "install_files")

licenses([
    "by_exception_only",  # MOSEK
    "notice",  # fplib AND Zlib
])

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
   docs = ["mosek-eula.pdf"],
   deps = [":install_libraries"],
)
    """.format(srcs, hdrs, includes, linkopts, files, libraries_strip_prefix)

    repository_ctx.file("BUILD.bazel", content = file_content,
                        executable = False)

mosek_repository = repository_rule(implementation = _impl)
